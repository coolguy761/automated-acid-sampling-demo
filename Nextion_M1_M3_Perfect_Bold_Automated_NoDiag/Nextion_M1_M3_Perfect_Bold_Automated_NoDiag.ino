/*
  Nextion Motor Command Test - Uno + M1/M3 + RTC
  ----------------------------------------------
  No-diagnostics test build based on the working automated M1/M3 sketch.

  Purpose:
  - Run the automated M1 + M3 sequence from the Nextion HMI
  - Show RTC time on the HMI and timestamp short log messages
  - Support STOP / RESET / manual jog commands
  - Repeat automatically after a one-minute countdown
  - Skip diagnostics-page support for a leaner showcase build

  Notes for operators:
  - This build keeps the stable logic and lighter log behavior.
  - Diagnostics-page handling has been removed so only the main and logs pages
    remain in the live UI flow.
*/

#include <SoftwareSerial.h>
#include <Wire.h>
#include <avr/pgmspace.h>
#include <ctype.h>
#include <stdio.h>
#include <string.h>

// Motor 1 uses a PUL / DIR / ENA style driver.
const uint8_t PIN_M1_PUL      = 2;
const uint8_t PIN_M1_DIR      = 3;
const uint8_t PIN_M1_ENA      = 8;

// Motor 3 uses the shared enable line and its own step / direction pins.
const uint8_t PIN_M3_STEP     = 6;
const uint8_t PIN_M3_DIR      = 7;
const uint8_t PIN_SHARED_ENA  = 8;

// Nextion serial link and DS3231 RTC bus address.
const uint8_t PIN_NX_RX       = 10; // Arduino RX <- Nextion TX
const uint8_t PIN_NX_TX       = 11; // Arduino TX -> Nextion RX
const uint8_t RTC_I2C_ADDR    = 0x68;

SoftwareSerial nxt(PIN_NX_RX, PIN_NX_TX);

// Electrical levels expected by the motor drivers.
const uint8_t DIR_FWD_LEVEL             = HIGH;
const uint8_t DIR_REV_LEVEL             = LOW;
const uint8_t M1_ENA_ENABLED_LEVEL      = LOW;
const uint8_t M1_ENA_DISABLED_LEVEL     = HIGH;
const uint8_t SHARED_ENA_ENABLED_LEVEL  = LOW;
const uint8_t SHARED_ENA_DISABLED_LEVEL = HIGH;

// If true, every upload/boot sets the RTC to the sketch build time.
const bool RTC_SET_TIME_ON_BOOT = true;

// Flip these during bench tuning instead of changing logic below.
const bool M1_FORWARD_IS_DOWN     = true;
const bool M3_FORWARD_IS_TILT_OUT = true;

// Per-motor pulse timing and commanded step distances.
// Smaller STEP_INTERVAL values = faster motion.
const unsigned long STEP_INTERVAL_M1_DOWN_US     = 1200UL;
const unsigned long STEP_INTERVAL_M1_UP_US       = 2000UL;
const unsigned long STEP_INTERVAL_M3_US          = 2800UL;
const unsigned long STEP_PULSE_US                = 10UL;
const unsigned long STEP_01_M1_DOWN_STEPS        = 6500UL;
const unsigned long STEP_04_M1_UP_STEPS          = 6500UL;
const unsigned long STEP_06_M3_TILT_OUT_STEPS    = 2650UL;
const unsigned long STEP_07_M3_TILT_HOME_STEPS   = 2650UL;
// Timing used between steps, between automated cycles, and for RTC refresh.
const unsigned long SETTLE_MS                 = 300UL;
const unsigned long REPEAT_DELAY_MS           = 60000UL;
const unsigned long RTC_UPDATE_MS             = 1000UL;

// Nextion 16-bit color values used for status boxes.
const uint16_t NX_GRAY  = 33840;
const uint16_t NX_GREEN = 2016;
const uint16_t NX_BLUE  = 31775;

// Short rolling log buffer shown on the log page.
// This keeps the same total SRAM footprint as 3 x 32-byte entries, but trades
// that space for 4 shorter entries so the log page can show more cycle stages.
const uint8_t LOG_ENTRY_COUNT = 4;
const uint8_t LOG_ENTRY_LEN   = 24;

enum Axis {
  AXIS_NONE = 0,
  AXIS_M1,
  AXIS_M3
};

enum Direction {
  MOVE_NONE = 0,
  MOVE_FWD,
  MOVE_REV
};

enum DemoState {
  ST_IDLE = 0,
  ST_AUTO_RUN,     // Currently executing one automatic sequence step
  ST_AUTO_WAIT,    // Settling between automatic steps
  ST_REPEAT_WAIT,  // Waiting for the one-minute auto-repeat countdown
  ST_RESET_RUN,    // Executing a reset / return-to-idle move
  ST_RESET_WAIT,   // Settling between reset moves
  ST_DONE
};

struct AutoStep {
  Axis axis;              // Which motor this step drives
  Direction dir;          // Which direction that motor should move
  unsigned long stepCount; // Exact number of step pulses for this move
};

// Full automated cycle for this build:
// 1. M1 down
// 2. M1 up
// 3. M3 tilt out
// 4. M3 tilt home
const AutoStep FULL_CYCLE[] = {
  { AXIS_M1, MOVE_FWD, STEP_01_M1_DOWN_STEPS },
  { AXIS_M1, MOVE_REV, STEP_04_M1_UP_STEPS },
  { AXIS_M3, MOVE_FWD, STEP_06_M3_TILT_OUT_STEPS },
  { AXIS_M3, MOVE_REV, STEP_07_M3_TILT_HOME_STEPS }
};

const uint8_t FULL_CYCLE_COUNT = sizeof(FULL_CYCLE) / sizeof(FULL_CYCLE[0]);

// Active motion state.
Axis activeAxis = AXIS_NONE;           // Which motor is currently stepping
Direction activeDir = MOVE_NONE;       // Current move direction
DemoState demoState = ST_IDLE;         // High-level sequence state machine
bool stepHigh = false;                 // Tracks whether the step pin is currently high

// UI / logging state.
bool logDirty = false;                 // True when the log page needs a refresh
bool currentPageIsLog = false;         // Tracks whether the HMI is currently on the log page
bool rtcOnline = false;                // True if the DS3231 read succeeds

// Logical mechanism state used for HMI and reset behavior.
bool m1Done = false;                   // M1 has completed at least one commanded cycle step
bool m3Done = false;                   // M3 has completed at least one commanded cycle step
bool m1IsDown = false;                 // Sketch belief: M1 is in its down/extended position
bool m3IsOut = false;                  // Sketch belief: M3 is in its out/tilted position

// Saved partial return distances used when STOP is pressed mid-motion.
unsigned long m1ResetUpSteps = 0;      // Remaining or equivalent home distance for M1
unsigned long m3ResetHomeSteps = 0;    // Remaining or equivalent home distance for M3

// Timing and step-count bookkeeping.
unsigned long lastStepUs = 0;          // Last microsecond timestamp used by the step pulse engine
unsigned long demoStateStartMs = 0;    // Millisecond timestamp for the current state
unsigned long lastRtcUpdateMs = 0;     // Last time the RTC/HMI clock was refreshed
unsigned long cycleStartMs = 0;        // Start time of the current cycle or reset sequence
unsigned long lastUiRefreshMs = 0;     // Last once-per-second UI refresh timestamp
unsigned long targetStepCount = 0;     // Number of pulses the current move should execute
unsigned long completedStepCount = 0;  // Number of pulses already completed in the current move

// Sequence indexing.
uint8_t autoStepIndex = 0;             // Current index into FULL_CYCLE
AutoStep resetCycle[2];                // Small dynamic reset sequence built from current machine state
uint8_t resetStepCount = 0;            // Number of valid reset steps built into resetCycle
uint8_t resetStepIndex = 0;            // Current index inside resetCycle during RESET

// Serial and RTC text buffers.
char currentTimeText[9] = "--:--:--";  // HH:MM:SS RTC string
char serialLine[32];                   // Incoming command buffer from USB serial
uint8_t serialLen = 0;                 // Current length of serialLine
char cycleSummary[64] = "NO CYCLE YET"; // Compact per-cycle progression shown in tFrame

// Log storage shown on the Nextion log page.
char logEntries[LOG_ENTRY_COUNT][LOG_ENTRY_LEN];
uint8_t logHead = 0;                   // Oldest entry in the ring buffer
uint8_t logCount = 0;                  // Number of valid log entries stored

void setupPins();
void stopAllMotion();
void startAxis(Axis axis, Direction dir, unsigned long stepCount);
void serviceMotion();
void serviceSequence();
void beginAutoCycle();
void startAutoStep(uint8_t index);
void beginResetSequence();
void startResetStep(uint8_t index);
uint8_t stepPinFor(Axis axis);
uint8_t dirPinFor(Axis axis);
uint8_t enablePinFor(Axis axis);
uint8_t enabledLevelFor(Axis axis);
uint8_t dirLevelFor(Axis axis, Direction dir);
void handleCommand(char *cmd);
void readNextion();
void nxCmd(const char *cmd);
void nxSetTxt(const char *obj, const char *txt);
void nxSetPageTxt(const char *page, const char *obj, const char *txt);
void nxSetBco(const char *obj, uint16_t color);
void nxSetPco(const char *obj, uint16_t color);
void nxWriteTerminator();
void pushClockText();
void updateHmi();
void refreshLogPage();
void updateRtc(bool force);
bool readRtcTime(char *timeText, size_t timeTextLen);
uint8_t bcdToDec(uint8_t value);
uint8_t decToBcd(uint8_t value);
void syncRtcToBuildTime();
bool parseBuildDateTime(uint8_t &year, uint8_t &month, uint8_t &day, uint8_t &hour, uint8_t &minute, uint8_t &second);
uint8_t monthFromBuildDate(const char *dateText);
void writeRtcDateTime(uint8_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t second);
void appendLogRam(const char *msg);
void appendStateLogRam(const char *msg);
void appendLogP(PGM_P msg);
void appendStateLogP(PGM_P msg);
void appendCycleSummaryToken(const char *token);
void clearLog();
void copyStepLabel(uint8_t index, bool completeText, char *out, size_t outLen);
void fillAxisStateText(char *out, size_t outLen);
void fillDemoStateText(char *out, size_t outLen);
void fillSeqActiveText(char *out, size_t outLen);
void fillDoneText(bool done, const char *prefix, char *out, size_t outLen);
void fillActiveText(bool active, const char *prefix, char *out, size_t outLen);
void fillCycleElapsedText(char *out, size_t outLen);
void fillDriverText(char *out, size_t outLen);
void fillLogStateText(char *out, size_t outLen);
void fillRepeatCountdownText(char *out, size_t outLen);
void resetSequenceFlags();
void markStepComplete(uint8_t index);
void buildResetCycle();
void captureActiveMotionState();
unsigned long currentPlannedStepCount();
void setDemoState(DemoState s);

// Arduino startup:
// - bring up serial, Nextion, and I2C
// - set safe pin defaults
// - initialize the HMI
// - initialize/sync the RTC
// - log boot status
void setup() {
  Serial.begin(115200);
  nxt.begin(9600);
  Wire.begin();

  setupPins();
  stopAllMotion();

  nxCmd("bkcmd=0");
  nxCmd("sendxy=0");
  nxCmd("sendme");

  syncRtcToBuildTime();
  updateRtc(true);

  Serial.println(F("NEXTION MOTOR COMMAND TEST READY"));
  Serial.println(F("Commands: M1F M1R M3F M3R START RESET STOP STATUS"));

  appendLogP(PSTR("BOOT"));
  appendLogP(rtcOnline ? PSTR("RTCOK") : PSTR("RTCER"));
  appendLogP(PSTR("READY"));
  updateHmi();
}

// Main scheduler loop:
// - consume serial commands
// - consume Nextion commands/page changes
// - refresh the RTC clock text
// - advance the state machine
// - generate step pulses for the active motor
void loop() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (serialLen) {
        serialLine[serialLen] = '\0';
        handleCommand(serialLine);
        serialLen = 0;
      }
    } else if (serialLen < sizeof(serialLine) - 1 && c >= 32 && c <= 126) {
      serialLine[serialLen++] = c;
    }
  }

  readNextion();
  updateRtc(false);
  serviceSequence();
  serviceMotion();

  if (demoState != ST_IDLE && millis() - lastUiRefreshMs >= 1000UL) {
    lastUiRefreshMs = millis();
    if (demoState == ST_REPEAT_WAIT) {
      char repeatBuf[18];
      char faultBuf[28];
      fillRepeatCountdownText(repeatBuf, sizeof(repeatBuf));
      fillDemoStateText(faultBuf, sizeof(faultBuf));
      nxSetPageTxt("page0", "tMotorrev", repeatBuf);
      nxSetPageTxt("page0", "tFault", faultBuf);
    } else {
      char cycleBuf[22];
      fillCycleElapsedText(cycleBuf, sizeof(cycleBuf));
      nxSetPageTxt("page0", "tCycle", cycleBuf);
    }
  }
}

// Configure all motor control outputs and place them in a safe idle state.
void setupPins() {
  pinMode(PIN_M1_PUL, OUTPUT);
  pinMode(PIN_M1_DIR, OUTPUT);
  pinMode(PIN_M3_STEP, OUTPUT);
  pinMode(PIN_M3_DIR, OUTPUT);
  pinMode(PIN_M1_ENA, OUTPUT);
  pinMode(PIN_SHARED_ENA, OUTPUT);

  digitalWrite(PIN_M1_PUL, LOW);
  digitalWrite(PIN_M3_STEP, LOW);
  digitalWrite(PIN_M1_DIR, DIR_FWD_LEVEL);
  digitalWrite(PIN_M3_DIR, DIR_FWD_LEVEL);
  digitalWrite(PIN_M1_ENA, M1_ENA_DISABLED_LEVEL);
  digitalWrite(PIN_SHARED_ENA, SHARED_ENA_DISABLED_LEVEL);
}

// Hard stop for both motors.
// This drops enable lines, clears the active move, and resets pulse counters.
void stopAllMotion() {
  activeAxis = AXIS_NONE;
  activeDir = MOVE_NONE;
  stepHigh = false;
  targetStepCount = 0;
  completedStepCount = 0;

  digitalWrite(PIN_M1_PUL, LOW);
  digitalWrite(PIN_M3_STEP, LOW);
  digitalWrite(PIN_M1_ENA, M1_ENA_DISABLED_LEVEL);
  digitalWrite(PIN_SHARED_ENA, SHARED_ENA_DISABLED_LEVEL);
}

// Begin one motor move using exact step counts.
// This is the core entry point for manual jogs, auto steps, and reset steps.
void startAxis(Axis axis, Direction dir, unsigned long stepCount) {
  stopAllMotion();

  activeAxis = axis;
  activeDir = dir;
  targetStepCount = stepCount;
  completedStepCount = 0;
  digitalWrite(dirPinFor(axis), dirLevelFor(axis, dir));
  digitalWrite(enablePinFor(axis), enabledLevelFor(axis));

  lastStepUs = micros();
  stepHigh = false;

  Serial.print(F("RUN "));
  char stateBuf[12];
  fillAxisStateText(stateBuf, sizeof(stateBuf));
  Serial.println(stateBuf);
  updateHmi();
}

// Non-blocking pulse generator.
// Each loop pass decides whether to raise or lower the current step pin and
// increments completedStepCount after each full low-going pulse edge.
void serviceMotion() {
  if (activeAxis == AXIS_NONE || activeDir == MOVE_NONE) return;

  uint8_t stepPin = stepPinFor(activeAxis);
  unsigned long stepIntervalUs = STEP_INTERVAL_M1_DOWN_US;
  if (activeAxis == AXIS_M1) {
    stepIntervalUs = (activeDir == MOVE_FWD) ? STEP_INTERVAL_M1_DOWN_US : STEP_INTERVAL_M1_UP_US;
  } else if (activeAxis == AXIS_M3) {
    stepIntervalUs = STEP_INTERVAL_M3_US;
  }
  unsigned long now = micros();

  if (!stepHigh) {
    if ((unsigned long)(now - lastStepUs) >= stepIntervalUs) {
      digitalWrite(stepPin, HIGH);
      stepHigh = true;
      lastStepUs = now;
    }
  } else if ((unsigned long)(now - lastStepUs) >= STEP_PULSE_US) {
    digitalWrite(stepPin, LOW);
    stepHigh = false;
    lastStepUs = now;
    if (completedStepCount < targetStepCount) completedStepCount++;
  }
}

// High-level sequence state machine.
// This decides when moves finish, when to advance to the next step, when to
// wait between cycles, and when to start the next automatic cycle.
void serviceSequence() {
  unsigned long now = millis();

  switch (demoState) {
    case ST_IDLE:
      return;

    case ST_AUTO_RUN:
      if (completedStepCount >= targetStepCount) {
        stopAllMotion();
        markStepComplete(autoStepIndex);
        char msgBuf[28];
        copyStepLabel(autoStepIndex, true, msgBuf, sizeof(msgBuf));
        appendCycleSummaryToken(msgBuf);
        appendStateLogRam(msgBuf);
        setDemoState(ST_AUTO_WAIT);
      }
      break;

    case ST_AUTO_WAIT:
      if (now - demoStateStartMs >= SETTLE_MS) {
        autoStepIndex++;
        if (autoStepIndex >= FULL_CYCLE_COUNT) {
          appendCycleSummaryToken("CYDN");
          appendStateLogP(PSTR("CYDN"));
          setDemoState(ST_REPEAT_WAIT);
        } else {
          startAutoStep(autoStepIndex);
        }
      }
      break;

    case ST_REPEAT_WAIT:
      if (now - demoStateStartMs >= REPEAT_DELAY_MS) {
        appendStateLogP(PSTR("RNEXT"));
        beginAutoCycle();
      }
      break;

    case ST_RESET_RUN:
      if (completedStepCount >= targetStepCount) {
        stopAllMotion();
        switch (resetCycle[resetStepIndex].axis) {
          case AXIS_M1: m1IsDown = false; m1ResetUpSteps = 0; break;
          case AXIS_M3: m3IsOut = false; m3ResetHomeSteps = 0; break;
          default: break;
        }
        appendCycleSummaryToken("RSDN");
        appendStateLogP(PSTR("RSDN"));
        setDemoState(ST_RESET_WAIT);
      }
      break;

    case ST_RESET_WAIT:
      if (now - demoStateStartMs >= SETTLE_MS) {
        resetStepIndex++;
        if (resetStepIndex >= resetStepCount) {
          appendCycleSummaryToken("RSTHM");
          appendStateLogP(PSTR("RSTHM"));
          setDemoState(ST_IDLE);
        } else {
          startResetStep(resetStepIndex);
        }
      }
      break;

    case ST_DONE:
      setDemoState(ST_IDLE);
      break;
  }
}

// Reset the automatic sequence back to step 0 and start a fresh cycle.
void beginAutoCycle() {
  autoStepIndex = 0;
  resetSequenceFlags();
  resetStepCount = 0;
  resetStepIndex = 0;
  cycleStartMs = millis();
  logHead = 0;
  logCount = 0;
  strcpy(cycleSummary, "START");
  appendStateLogP(PSTR("START"));
  startAutoStep(autoStepIndex);
  updateHmi();
}

// Start one step from the FULL_CYCLE table and log its label.
void startAutoStep(uint8_t index) {
  char msgBuf[28];
  copyStepLabel(index, false, msgBuf, sizeof(msgBuf));
  appendCycleSummaryToken(msgBuf);
  appendStateLogRam(msgBuf);
  startAxis(FULL_CYCLE[index].axis, FULL_CYCLE[index].dir, FULL_CYCLE[index].stepCount);
  setDemoState(ST_AUTO_RUN);
}

// Build and run a return-to-idle sequence after STOP or RESET.
// The reset path uses the saved partial step counts so it can return only the
// remaining or equivalent home distance instead of always running a full move.
void beginResetSequence() {
  captureActiveMotionState();
  buildResetCycle();
  strcpy(cycleSummary, "RESET");

  if (!resetStepCount) {
    appendStateLogP(PSTR("RSTHM"));
    appendCycleSummaryToken("RSTHM");
    setDemoState(ST_IDLE);
    stopAllMotion();
    updateHmi();
    return;
  }

  resetStepIndex = 0;
  cycleStartMs = millis();
  appendStateLogP(PSTR("RSTST"));
  appendCycleSummaryToken("RSTST");
  startResetStep(resetStepIndex);
  updateHmi();
}

// Start one step from the resetCycle table.
void startResetStep(uint8_t index) {
  startAxis(resetCycle[index].axis, resetCycle[index].dir, resetCycle[index].stepCount);
  setDemoState(ST_RESET_RUN);
}

// Resolve which STEP/PUL pin belongs to the selected axis.
uint8_t stepPinFor(Axis axis) {
  switch (axis) {
    case AXIS_M1: return PIN_M1_PUL;
    case AXIS_M3: return PIN_M3_STEP;
    default:      return PIN_M1_PUL;
  }
}

// Resolve which DIR pin belongs to the selected axis.
uint8_t dirPinFor(Axis axis) {
  switch (axis) {
    case AXIS_M1: return PIN_M1_DIR;
    case AXIS_M3: return PIN_M3_DIR;
    default:      return PIN_M1_DIR;
  }
}

// Resolve which enable pin should be toggled for the selected axis.
uint8_t enablePinFor(Axis axis) {
  return axis == AXIS_M1 ? PIN_M1_ENA : PIN_SHARED_ENA;
}

// Return the active electrical enable level for the selected axis.
uint8_t enabledLevelFor(Axis axis) {
  return axis == AXIS_M1 ? M1_ENA_ENABLED_LEVEL : SHARED_ENA_ENABLED_LEVEL;
}

// Convert logical MOVE_FWD / MOVE_REV into the real driver level needed on DIR.
// The M1_FORWARD_IS_DOWN and M3_FORWARD_IS_TILT_OUT flags let operators flip
// the mechanism direction without rewriting sequence logic.
uint8_t dirLevelFor(Axis axis, Direction dir) {
  bool forwardUsesFwdLevel = true;

  if (axis == AXIS_M1) forwardUsesFwdLevel = M1_FORWARD_IS_DOWN;
  else if (axis == AXIS_M3) forwardUsesFwdLevel = M3_FORWARD_IS_TILT_OUT;

  if (dir == MOVE_FWD) return forwardUsesFwdLevel ? DIR_FWD_LEVEL : DIR_REV_LEVEL;
  return forwardUsesFwdLevel ? DIR_REV_LEVEL : DIR_FWD_LEVEL;
}

// Central command parser.
// Accepts text commands from either USB serial or the Nextion display.
void handleCommand(char *cmd) {
  for (char *p = cmd; *p; ++p) *p = (char)toupper((unsigned char)*p);

  Serial.print(F("CMD: "));
  Serial.println(cmd);

  if (!strcmp(cmd, "M1F")) {
    setDemoState(ST_IDLE);
    strcpy(cycleSummary, "MAN>M1EXT");
    appendStateLogP(PSTR("M1EXT"));
    m1IsDown = true;
    startAxis(AXIS_M1, MOVE_FWD, STEP_01_M1_DOWN_STEPS);
  } else if (!strcmp(cmd, "M1R")) {
    setDemoState(ST_IDLE);
    strcpy(cycleSummary, "MAN>M1RET");
    appendStateLogP(PSTR("M1RET"));
    m1IsDown = false;
    startAxis(AXIS_M1, MOVE_REV, STEP_04_M1_UP_STEPS);
  } else if (!strcmp(cmd, "M3F")) {
    setDemoState(ST_IDLE);
    strcpy(cycleSummary, "MAN>M3OUT");
    appendStateLogP(PSTR("M3OUT"));
    m3IsOut = true;
    startAxis(AXIS_M3, MOVE_FWD, STEP_06_M3_TILT_OUT_STEPS);
  } else if (!strcmp(cmd, "M3R")) {
    setDemoState(ST_IDLE);
    strcpy(cycleSummary, "MAN>M3HOM");
    appendStateLogP(PSTR("M3HOM"));
    m3IsOut = false;
    startAxis(AXIS_M3, MOVE_REV, STEP_07_M3_TILT_HOME_STEPS);
  } else if (!strcmp(cmd, "START")) {
    beginAutoCycle();
  } else if (!strcmp(cmd, "RESET")) {
    beginResetSequence();
  } else if (!strcmp(cmd, "STOP")) {
    appendCycleSummaryToken("STOP");
    appendLogP(PSTR("STOP"));
    captureActiveMotionState();
    setDemoState(ST_IDLE);
    stopAllMotion();
    updateHmi();
  } else if (!strcmp(cmd, "STATUS")) {
    char stateBuf[28];
    fillDemoStateText(stateBuf, sizeof(stateBuf));
    appendLogRam(stateBuf);
    updateHmi();
  } else if (!strcmp(cmd, "PAGE:MAIN")) {
    currentPageIsLog = false;
    updateHmi();
  } else if (!strcmp(cmd, "PAGE:LOG") || !strcmp(cmd, "PAGE:LOGS")) {
    currentPageIsLog = true;
    refreshLogPage();
  } else if (!strcmp(cmd, "CLEARLOG")) {
    clearLog();
    appendLogP(PSTR("LCLR"));
    refreshLogPage();
  } else {
    appendLogP(PSTR("BADCMD"));
  }
}

// Parse raw serial bytes from the Nextion.
// This handles both:
// - page ID responses from `sendme`
// - text commands sent from button press events
void readNextion() {
  static char line[32];
  static uint8_t len = 0;
  static bool awaitPageId = false;
  static bool awaitFF = false;
  static uint8_t ffCount = 0;

  while (nxt.available()) {
    uint8_t b = (uint8_t)nxt.read();

    if (awaitPageId) {
      currentPageIsLog = (b == 1);
      if (currentPageIsLog) refreshLogPage();
      else updateHmi();
      awaitPageId = false;
      awaitFF = true;
      ffCount = 0;
      continue;
    }

    if (awaitFF) {
      if (b == 0xFF) {
        ffCount++;
        if (ffCount >= 3) awaitFF = false;
      } else {
        awaitFF = false;
      }
      continue;
    }

    if (b == 0x66) {
      awaitPageId = true;
      len = 0;
      continue;
    }

    char c = (char)b;
    if (c == '\n') {
      if (len) {
        line[len] = '\0';
        handleCommand(line);
        len = 0;
      }
    } else if (c != '\r' && c >= 32 && c <= 126 && len < sizeof(line) - 1) {
      line[len++] = c;
    }
  }
}

// Send a raw command string to the Nextion and append the required terminator.
void nxCmd(const char *cmd) {
  nxt.print(cmd);
  nxWriteTerminator();
}

// Set a text object value by object name.
void nxSetTxt(const char *obj, const char *txt) {
  nxt.print(obj);
  nxt.print(F(".txt=\""));
  nxt.print(txt);
  nxt.print('"');
  nxWriteTerminator();
}

// Set a text object value using a fully qualified page + object target.
void nxSetPageTxt(const char *page, const char *obj, const char *txt) {
  nxt.print(page);
  nxt.print('.');
  nxt.print(obj);
  nxt.print(F(".txt=\""));
  nxt.print(txt);
  nxt.print('"');
  nxWriteTerminator();
}

// Set the background color (`bco`) of a Nextion object.
void nxSetBco(const char *obj, uint16_t color) {
  nxt.print(obj);
  nxt.print(F(".bco="));
  nxt.print(color);
  nxWriteTerminator();
}

// Set the text color (`pco`) of a Nextion object.
void nxSetPco(const char *obj, uint16_t color) {
  nxt.print(obj);
  nxt.print(F(".pco="));
  nxt.print(color);
  nxWriteTerminator();
}

// Every Nextion command must end with three 0xFF bytes.
void nxWriteTerminator() {
  nxt.write(0xFF);
  nxt.write(0xFF);
  nxt.write(0xFF);
}

// Push the current RTC clock text to all page clock fields.
void pushClockText() {
  char clockBuf[15];
  if (rtcOnline) snprintf(clockBuf, sizeof(clockBuf), "TIME: %s", currentTimeText);
  else strcpy(clockBuf, "TIME: RTC ERR");

  nxSetPageTxt("page0", "tClock", clockBuf);
  nxSetPageTxt("page1", "tClock1", clockBuf);
}

// Refresh the main-page HMI fields from the sketch's current state.
// This is the main "operator view" update function.
void updateHmi() {
  char stateBuf[12];
  char faultBuf[28];
  char pressBuf[12];
  char revsBuf[18];
  char estopBuf[18];
  char driverBuf[18];
  char cycleBuf[22];
  char lsRetBuf[16];
  char lsExtBuf[16];
  char encBuf[16];
  char motorBuf[18];
  char motorFwdBuf[18];
  char motorRevBuf[18];

  updateRtc(false);
  fillAxisStateText(stateBuf, sizeof(stateBuf));
  fillDemoStateText(faultBuf, sizeof(faultBuf));
  strcpy(pressBuf, "CMD LINK OK");
  fillDemoStateText(revsBuf, sizeof(revsBuf));
  fillSeqActiveText(estopBuf, sizeof(estopBuf));
  fillDriverText(driverBuf, sizeof(driverBuf));
  fillCycleElapsedText(cycleBuf, sizeof(cycleBuf));
  fillDoneText(m1Done, "M1 DONE:", lsRetBuf, sizeof(lsRetBuf));
  snprintf(lsExtBuf, sizeof(lsExtBuf), "M1 POS: %s", m1IsDown ? "DOWN" : "UP");
  fillDoneText(m3Done, "M3 DONE:", encBuf, sizeof(encBuf));
  fillActiveText(activeAxis == AXIS_M3, "M3 ACTIVE:", motorBuf, sizeof(motorBuf));
  fillActiveText(activeAxis == AXIS_M1, "M1 ACTIVE:", motorFwdBuf, sizeof(motorFwdBuf));
  if (demoState == ST_REPEAT_WAIT) {
    fillRepeatCountdownText(motorRevBuf, sizeof(motorRevBuf));
  } else {
    snprintf(motorRevBuf, sizeof(motorRevBuf), "RESET ACT: %s",
             (demoState == ST_RESET_RUN || demoState == ST_RESET_WAIT) ? "YES" : "NO");
  }

  nxSetPageTxt("page0", "tState", stateBuf);
  nxSetPageTxt("page0", "tFault", faultBuf);
  nxSetPageTxt("page0", "tPress", pressBuf);
  nxSetPageTxt("page0", "tRevs", revsBuf);
  nxSetPageTxt("page0", "tEstop", estopBuf);
  nxSetPageTxt("page0", "tDriver", driverBuf);
  nxSetPageTxt("page0", "tCycle", cycleBuf);
  nxSetPageTxt("page0", "tLsRet", lsRetBuf);
  nxSetPageTxt("page0", "tLsExt", lsExtBuf);
  nxSetPageTxt("page0", "tEnc", encBuf);
  nxSetPageTxt("page0", "tMotor", motorBuf);
  nxSetPageTxt("page0", "tMotorfwd", motorFwdBuf);
  nxSetPageTxt("page0", "tMotorrev", motorRevBuf);

  // Keep the original color-coded state handling.
  nxSetBco("page0.tState", activeAxis == AXIS_NONE ? NX_GRAY : NX_BLUE);
  nxSetBco("page0.tFault", activeAxis == AXIS_NONE ? NX_GREEN : NX_BLUE);
  nxSetBco("page0.tPress", NX_GREEN);
  nxSetPco("page0.tState", 0);
  nxSetPco("page0.tFault", 0);
  nxSetPco("page0.tPress", 0);
  nxSetPco("page0.tRevs", 0);
  nxSetPco("page0.tEstop", 0);
  nxSetPco("page0.tDriver", 0);
  nxSetPco("page0.tCycle", 0);
  nxSetPco("page0.tLsRet", 0);
  nxSetPco("page0.tLsExt", 0);
  nxSetPco("page0.tEnc", 0);
  nxSetPco("page0.tMotor", 0);
  nxSetPco("page0.tMotorfwd", 0);
  nxSetPco("page0.tMotorrev", 0);

  if (logDirty && currentPageIsLog) refreshLogPage();
}

// Rebuild the short log string shown on the log page.
// The log buffer is intentionally small to fit Uno memory limits.
void refreshLogPage() {
  pushClockText();
  nxt.print(F("page1.tFrame.txt=\""));
  nxt.print(cycleSummary);
  nxt.print('"');
  nxWriteTerminator();
  logDirty = false;
}

// Refresh the DS3231 time string at a limited rate and push it to the HMI
// only when it changes or when a forced refresh is requested.
void updateRtc(bool force) {
  static char lastTimeText[9] = "";

  if (!force && millis() - lastRtcUpdateMs < RTC_UPDATE_MS) return;
  lastRtcUpdateMs = millis();

  if (readRtcTime(currentTimeText, sizeof(currentTimeText))) rtcOnline = true;
  else {
    rtcOnline = false;
    strcpy(currentTimeText, "--:--:--");
  }

  if (force || strcmp(lastTimeText, currentTimeText)) {
    pushClockText();
    strcpy(lastTimeText, currentTimeText);
  }
}

// Read HH:MM:SS from the DS3231.
// Returns false if the RTC does not respond correctly.
bool readRtcTime(char *timeText, size_t timeTextLen) {
  Wire.beginTransmission(RTC_I2C_ADDR);
  Wire.write((uint8_t)0x00);
  if (Wire.endTransmission() != 0) return false;

  const uint8_t bytesWanted = 3;
  uint8_t bytesRead = (uint8_t)Wire.requestFrom((int)RTC_I2C_ADDR, (int)bytesWanted);
  if (bytesRead < bytesWanted) return false;

  uint8_t sec = Wire.read();
  uint8_t min = Wire.read();
  uint8_t hour = Wire.read();

  uint8_t seconds = bcdToDec(sec & 0x7F);
  uint8_t minutes = bcdToDec(min & 0x7F);
  uint8_t hours = bcdToDec(hour & 0x3F);

  snprintf(timeText, timeTextLen, "%02u:%02u:%02u", hours, minutes, seconds);
  return true;
}

// Convert RTC BCD format into normal decimal.
uint8_t bcdToDec(uint8_t value) {
  return (uint8_t)(((value >> 4) * 10U) + (value & 0x0F));
}

// Convert normal decimal into RTC BCD format.
uint8_t decToBcd(uint8_t value) {
  return (uint8_t)(((value / 10U) << 4) | (value % 10U));
}

// Optionally overwrite the RTC with the sketch build time.
// Useful for quick demo uploads, but normally disabled once the RTC is set.
void syncRtcToBuildTime() {
  if (!RTC_SET_TIME_ON_BOOT) return;

  uint8_t year, month, day, hour, minute, second;
  if (!parseBuildDateTime(year, month, day, hour, minute, second)) return;
  writeRtcDateTime(year, month, day, hour, minute, second);
}

// Parse the Arduino build date/time macros into RTC-ready numeric values.
bool parseBuildDateTime(uint8_t &year, uint8_t &month, uint8_t &day, uint8_t &hour, uint8_t &minute, uint8_t &second) {
  month = monthFromBuildDate(__DATE__);
  if (!month) return false;

  day = (uint8_t)(((__DATE__[4] == ' ') ? 0 : (__DATE__[4] - '0')) * 10 + (__DATE__[5] - '0'));
  year = (uint8_t)(((__DATE__[9] - '0') * 10) + (__DATE__[10] - '0'));
  hour = (uint8_t)(((__TIME__[0] - '0') * 10) + (__TIME__[1] - '0'));
  minute = (uint8_t)(((__TIME__[3] - '0') * 10) + (__TIME__[4] - '0'));
  second = (uint8_t)(((__TIME__[6] - '0') * 10) + (__TIME__[7] - '0'));
  return true;
}

// Translate the month abbreviation from __DATE__ into a numeric month.
uint8_t monthFromBuildDate(const char *dateText) {
  if (!strncmp(dateText, "Jan", 3)) return 1;
  if (!strncmp(dateText, "Feb", 3)) return 2;
  if (!strncmp(dateText, "Mar", 3)) return 3;
  if (!strncmp(dateText, "Apr", 3)) return 4;
  if (!strncmp(dateText, "May", 3)) return 5;
  if (!strncmp(dateText, "Jun", 3)) return 6;
  if (!strncmp(dateText, "Jul", 3)) return 7;
  if (!strncmp(dateText, "Aug", 3)) return 8;
  if (!strncmp(dateText, "Sep", 3)) return 9;
  if (!strncmp(dateText, "Oct", 3)) return 10;
  if (!strncmp(dateText, "Nov", 3)) return 11;
  if (!strncmp(dateText, "Dec", 3)) return 12;
  return 0;
}

// Write a complete date/time value into the DS3231 register set.
void writeRtcDateTime(uint8_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t second) {
  Wire.beginTransmission(RTC_I2C_ADDR);
  Wire.write((uint8_t)0x00);
  Wire.write(decToBcd(second));
  Wire.write(decToBcd(minute));
  Wire.write(decToBcd(hour));
  Wire.write((uint8_t)0x01);
  Wire.write(decToBcd(day));
  Wire.write(decToBcd(month));
  Wire.write(decToBcd(year));
  Wire.endTransmission();
}

// Add one timestamped message to the rolling log buffer.
// New entries overwrite the oldest entry once the buffer is full.
void appendLogRam(const char *msg) {
  uint8_t idx;
  if (logCount < LOG_ENTRY_COUNT) {
    idx = (uint8_t)((logHead + logCount) % LOG_ENTRY_COUNT);
    logCount++;
  } else {
    idx = logHead;
    logHead = (uint8_t)((logHead + 1) % LOG_ENTRY_COUNT);
  }

  snprintf(logEntries[idx], LOG_ENTRY_LEN, "%s %s", currentTimeText, msg);
  logEntries[idx][LOG_ENTRY_LEN - 1] = '\0';
  logDirty = true;
}

// Add one compact timestamped log line that also includes the current machine
// position snapshot. This gives each entry more cycle context without needing
// a much larger ring buffer.
void appendStateLogRam(const char *msg) {
  char stateBuf[8];
  fillLogStateText(stateBuf, sizeof(stateBuf));

  uint8_t idx;
  if (logCount < LOG_ENTRY_COUNT) {
    idx = (uint8_t)((logHead + logCount) % LOG_ENTRY_COUNT);
    logCount++;
  } else {
    idx = logHead;
    logHead = (uint8_t)((logHead + 1) % LOG_ENTRY_COUNT);
  }

  snprintf(logEntries[idx], LOG_ENTRY_LEN, "%s %s %s", currentTimeText, msg, stateBuf);
  logEntries[idx][LOG_ENTRY_LEN - 1] = '\0';
  logDirty = true;
}

// Convenience wrapper for log strings stored in flash / PROGMEM.
void appendLogP(PGM_P msg) {
  char msgBuf[28];
  strncpy_P(msgBuf, msg, sizeof(msgBuf) - 1);
  msgBuf[sizeof(msgBuf) - 1] = '\0';
  appendLogRam(msgBuf);
}

// PROGMEM-backed wrapper for the compact state-aware log format.
void appendStateLogP(PGM_P msg) {
  char msgBuf[12];
  strncpy_P(msgBuf, msg, sizeof(msgBuf) - 1);
  msgBuf[sizeof(msgBuf) - 1] = '\0';
  appendStateLogRam(msgBuf);
}

// Build a compact multiline cycle-progress string for the main log text area.
// Each line is prefixed with a compact MM:SS timestamp from the RTC clock text.
// We store the literal "\r" escape sequence instead of a raw carriage return
// so the entire Nextion command string stays intact over serial.
void appendCycleSummaryToken(const char *token) {
  if (!token || !token[0]) return;

  char entryBuf[18];
  const char *stamp = "??:??";
  if (strlen(currentTimeText) >= 8) stamp = currentTimeText + 3; // MM:SS from HH:MM:SS
  snprintf(entryBuf, sizeof(entryBuf), "%5.5s %s", stamp, token);
  entryBuf[sizeof(entryBuf) - 1] = '\0';

  const size_t entryLen = strlen(entryBuf);
  size_t used = strlen(cycleSummary);
  if (used == 0) {
    strncpy(cycleSummary, entryBuf, sizeof(cycleSummary) - 1);
    cycleSummary[sizeof(cycleSummary) - 1] = '\0';
    return;
  }

  const size_t needed = used + 2 + entryLen;
  if (needed >= sizeof(cycleSummary)) {
    const char *overflowText = "...";
    const size_t overflowLen = 3;
    if (used + 2 + overflowLen < sizeof(cycleSummary)) {
      cycleSummary[used++] = '\\';
      cycleSummary[used++] = 'r';
      memcpy(&cycleSummary[used], overflowText, overflowLen);
      cycleSummary[used + overflowLen] = '\0';
    }
    return;
  }

  cycleSummary[used++] = '\\';
  cycleSummary[used++] = 'r';
  memcpy(&cycleSummary[used], entryBuf, entryLen);
  cycleSummary[used + entryLen] = '\0';
}

// Reset the rolling log buffer shown on the log page.
void clearLog() {
  logHead = 0;
  logCount = 0;
  logDirty = true;
  strcpy(cycleSummary, "NO CYCLE YET");
}

// Clear per-cycle "done" flags before a fresh automatic run starts.
void resetSequenceFlags() {
  m1Done = false;
  m3Done = false;
}

// Update tracked machine-state booleans when an automatic step completes.
// This is what tells the UI and reset logic where the sketch believes the
// mechanism is after each full move.
void markStepComplete(uint8_t index) {
  if (index == 0) {
    m1Done = true;
    m1IsDown = true;
    m1ResetUpSteps = STEP_04_M1_UP_STEPS;
  } else if (index == 1) {
    m1Done = true;
    m1IsDown = false;
    m1ResetUpSteps = 0;
  } else if (index == 2) {
    m3Done = true;
    m3IsOut = true;
    m3ResetHomeSteps = STEP_07_M3_TILT_HOME_STEPS;
  } else if (index == 3) {
    m3Done = true;
    m3IsOut = false;
    m3ResetHomeSteps = 0;
  }
}

// Build the minimal reset path needed to return to idle.
// Only motors believed to be away from home are added to resetCycle.
void buildResetCycle() {
  resetStepCount = 0;

  if (m3IsOut && m3ResetHomeSteps > 0) {
    resetCycle[resetStepCount].axis = AXIS_M3;
    resetCycle[resetStepCount].dir = MOVE_REV;
    resetCycle[resetStepCount].stepCount = m3ResetHomeSteps;
    resetStepCount++;
  }
  if (m1IsDown && m1ResetUpSteps > 0) {
    resetCycle[resetStepCount].axis = AXIS_M1;
    resetCycle[resetStepCount].dir = MOVE_REV;
    resetCycle[resetStepCount].stepCount = m1ResetUpSteps;
    resetStepCount++;
  }
}

// Estimate machine position when STOP is pressed mid-move.
// For forward moves, we save how far the mechanism likely traveled outward.
// For reverse/home moves, we save the remaining distance still needed to home.
void captureActiveMotionState() {
  const unsigned long plannedSteps = currentPlannedStepCount();
  const unsigned long doneSteps = completedStepCount;

  if (activeAxis == AXIS_M1) {
    if (activeDir == MOVE_FWD) {
      m1IsDown = (doneSteps > 0UL);
      m1ResetUpSteps = doneSteps;
    } else if (activeDir == MOVE_REV) {
      m1ResetUpSteps = (plannedSteps > doneSteps) ? (plannedSteps - doneSteps) : 0UL;
      m1IsDown = (m1ResetUpSteps > 0UL);
    }
  } else if (activeAxis == AXIS_M3) {
    if (activeDir == MOVE_FWD) {
      m3IsOut = (doneSteps > 0UL);
      m3ResetHomeSteps = doneSteps;
    } else if (activeDir == MOVE_REV) {
      m3ResetHomeSteps = (plannedSteps > doneSteps) ? (plannedSteps - doneSteps) : 0UL;
      m3IsOut = (m3ResetHomeSteps > 0UL);
    }
  }
}

// Return the target step count for the currently active move.
// Used by STOP-state capture to scale partial reset distances.
unsigned long currentPlannedStepCount() {
  return targetStepCount;
}

// Build a compact step label for logs and state text.
void copyStepLabel(uint8_t index, bool completeText, char *out, size_t outLen) {
  switch (index) {
    case 0: snprintf_P(out, outLen, completeText ? PSTR("S1DN") : PSTR("S1M1DN")); break;
    case 1: snprintf_P(out, outLen, completeText ? PSTR("S2DN") : PSTR("S2M1UP")); break;
    case 2: snprintf_P(out, outLen, completeText ? PSTR("S3DN") : PSTR("S3M3OT")); break;
    case 3: snprintf_P(out, outLen, completeText ? PSTR("S4DN") : PSTR("S4M3HM")); break;
    default: snprintf_P(out, outLen, PSTR("READY")); break;
  }
}

// Build the main state box text from the currently active axis and direction.
void fillAxisStateText(char *out, size_t outLen) {
  if (activeAxis == AXIS_M1) {
    snprintf_P(out, outLen, activeDir == MOVE_FWD ? PSTR("M1 DOWN") : PSTR("M1 UP"));
  } else if (activeAxis == AXIS_M3) {
    snprintf_P(out, outLen, activeDir == MOVE_FWD ? PSTR("TILT OUT") : PSTR("TILT HOME"));
  } else {
    snprintf_P(out, outLen, PSTR("IDLE"));
  }
}

// Build the sequence/status message shown on the HMI.
void fillDemoStateText(char *out, size_t outLen) {
  if (demoState == ST_IDLE) {
    snprintf_P(out, outLen, PSTR("READY"));
  } else if (demoState == ST_AUTO_WAIT) {
    snprintf_P(out, outLen, PSTR("STEP SETTLE"));
  } else if (demoState == ST_REPEAT_WAIT) {
    snprintf_P(out, outLen, PSTR("WAIT NEXT CYCLE"));
  } else if (demoState == ST_DONE) {
    snprintf_P(out, outLen, PSTR("DONE"));
  } else {
    copyStepLabel(autoStepIndex, false, out, outLen);
  }
}

// Build the "sequence active" text shown in the repurposed field.
void fillSeqActiveText(char *out, size_t outLen) {
  snprintf(out, outLen, "SEQ ACTIVE: %s", demoState == ST_IDLE ? "NO" : "YES");
}

// Build a YES/NO text field for completion status.
void fillDoneText(bool done, const char *prefix, char *out, size_t outLen) {
  snprintf(out, outLen, "%s %s", prefix, done ? "YES" : "NO");
}

// Build a YES/NO text field for live motor-active status.
void fillActiveText(bool active, const char *prefix, char *out, size_t outLen) {
  snprintf(out, outLen, "%s %s", prefix, active ? "YES" : "NO");
}

// Build the cycle elapsed timer string for the main page.
void fillCycleElapsedText(char *out, size_t outLen) {
  unsigned long elapsedMs = (demoState == ST_IDLE) ? 0UL : (millis() - cycleStartMs);
  unsigned int totalSeconds = (unsigned int)(elapsedMs / 1000UL);
  unsigned int minutes = totalSeconds / 60U;
  unsigned int seconds = totalSeconds % 60U;
  snprintf(out, outLen, "Cycle Elapsed: %02u:%02u", minutes, seconds);
}

// Build the simple driver/system readiness text.
void fillDriverText(char *out, size_t outLen) {
  snprintf(out, outLen, "System: %s", activeAxis == AXIS_NONE ? "READY" : "ACTIVE");
}

// Build a tiny position snapshot for log lines.
// Example: M1U M3H
void fillLogStateText(char *out, size_t outLen) {
  snprintf(out, outLen, "M1%c M3%c", m1IsDown ? 'D' : 'U', m3IsOut ? 'O' : 'H');
}

// Build the one-minute countdown text shown between automatic cycles.
void fillRepeatCountdownText(char *out, size_t outLen) {
  if (demoState != ST_REPEAT_WAIT) {
    snprintf(out, outLen, "NEXT 01:00");
    return;
  }

  unsigned long elapsedMs = millis() - demoStateStartMs;
  unsigned long remainingMs = (elapsedMs >= REPEAT_DELAY_MS) ? 0UL : (REPEAT_DELAY_MS - elapsedMs);
  unsigned int totalSeconds = (unsigned int)((remainingMs + 999UL) / 1000UL);
  unsigned int minutes = totalSeconds / 60U;
  unsigned int seconds = totalSeconds % 60U;
  snprintf(out, outLen, "NEXT %02u:%02u", minutes, seconds);
}

// State transition helper.
// Updates demoState, stamps the state start time, and refreshes the HMI so
// operators immediately see the new mode.
void setDemoState(DemoState s) {
  demoState = s;
  demoStateStartMs = millis();
  updateHmi();
}
