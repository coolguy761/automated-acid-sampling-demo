/*
  Automated Acid Sampling Demo (Motorized) — Arduino Uno + Nextion + DS3231 + Encoder

  What it does:
  - IDLE: waits for START
  - EXTENDING: drive motor forward until LS_EXT triggers (or timeout)
  - DWELL: wait fixed time (simulate sampling)
  - RETRACTING: drive motor reverse until LS_RET triggers (or timeout)
  - COMPLETE: log + return to IDLE
  - FAULT: de-energize outputs, show fault, require RESET

  Safety/Interlocks:
  - E-STOP: stops actuators immediately (should also physically cut actuator power via E-STOP contact)
  
  Nextion:
  - Updates text values + background colors (bco) for OK/FAULT/ON/OFF.
  - Page1 uses a Scroll Text component named "tLog"; we overwrite its .txt with the whole log buffer.

  Libraries:
  - Uses RTClib (Adafruit) for DS3231. If you don't have it, comment it out and timestamps will be "00:00:00".
*/

#include <SoftwareSerial.h>
#include <Wire.h>

// ---- Optional RTC ----
#define USE_RTC 1
#if USE_RTC
  #include <RTClib.h>
  RTC_DS3231 rtc;
#endif

// =====================
// Pin Mapping (Uno Rev3)
// =====================

// Inputs (wired NC -> GND, use INPUT_PULLUP for fail-safe)
// With NC + pullup: normal/ok = LOW, triggered/fault/broken wire = HIGH
const uint8_t PIN_ENC_A   = 2;   // Encoder A (interrupt) - optional
const uint8_t PIN_START   = 3;   // Start button (optional physical)
const uint8_t PIN_STOP    = 4;   // Stop/Abort button (optional physical)
const uint8_t PIN_RESET   = 5;   // Reset button (optional physical)
const uint8_t PIN_ESTOP   = 6;   // E-STOP status (NC contact to GND)
const uint8_t PIN_LS_EXT  = 8;   // Limit switch extend (NC contact to GND)
const uint8_t PIN_LS_RET  = 9;   // Limit switch retract (NC contact to GND)

// Outputs (drive relay/MOSFET modules; do NOT power loads directly from pins)
const uint8_t PIN_MOTOR_FWD = 12; // Motor driver forward / DIR+ command
const uint8_t PIN_MOTOR_REV = 13; // Motor driver reverse / DIR- command
const uint8_t PIN_MOTOR_EN = A0; // Motor enable (A0 as digital)
const uint8_t PIN_ALARM    = A1; // optional LED/Alarm (A1 as digital)

// Nextion on SoftwareSerial (recommended pins)
const uint8_t NX_RX = 10; // Arduino RX (yellow) <- Nextion TX
const uint8_t NX_TX = 11; // Arduino TX  (-> Nextion RX
SoftwareSerial nxt(NX_RX, NX_TX);

// =====================
// Nextion Color Constants (RGB565)
// =====================
const uint16_t NX_GRAY   = 50712;
const uint16_t NX_WHITE  = 65535;
const uint16_t NX_BLACK  = 0;
const uint16_t NX_GREEN  = 2016;
const uint16_t NX_RED    = 63488;
const uint16_t NX_YELLOW = 65504;

// =====================
// Encoder / Revs
// =====================
volatile long encPulses = 0;
const float PULSES_PER_REV = 20.0f; // set to your encoder resolution (current demo value)

// =====================
// State Machine
// =====================
enum State {
  ST_IDLE,
  ST_EXTENDING,
  ST_DWELL,
  ST_RETRACTING,
  ST_COMPLETE,
  ST_FAULT
};

State state = ST_IDLE;
unsigned long stateStartMs = 0;
unsigned long cycleStartMs = 0;

// Timing (fine tune)
const unsigned long EXTEND_TIMEOUT_MS = 6000;
const unsigned long RETRACT_TIMEOUT_MS = 6000;
const unsigned long DWELL_MS = 2000;

// Fault text
String faultMsg = "None";

// =====================
// Log buffer for Nextion Scroll Text (tLog)
// =====================
String bigLog = "";                 // entire log shown in tLog
const int MAX_LOG_CHARS = 900;      // keep under Nextion text limit
bool logDirty = true;

// Track current page (if needed)
enum Page { PAGE_UNKNOWN, PAGE_MAIN, PAGE_LOG, PAGE_DIAG };
Page currentPage = PAGE_MAIN;

// =====================
// Helpers: input read
// =====================
// NO pushbuttons wired to GND using INPUT_PULLUP
// idle = HIGH, pressed = LOW
bool readNO_Pressed(uint8_t pin) {
  return (digitalRead(pin) == LOW);
}

// NC safety/limit switches wired to GND using INPUT_PULLUP
// normal/healthy = LOW, active/open or broken wire = HIGH
bool readNC_ActiveHigh(uint8_t pin) {
  return (digitalRead(pin) == HIGH);
}

bool startPressed()  { return readNO_Pressed(PIN_START); }
bool stopPressed()   { return readNO_Pressed(PIN_STOP); }
bool resetPressed()  { return readNO_Pressed(PIN_RESET); }

bool estopActive()   { return readNC_ActiveHigh(PIN_ESTOP); }
bool lsExtActive()   { return readNC_ActiveHigh(PIN_LS_EXT); }
bool lsRetActive()   { return readNC_ActiveHigh(PIN_LS_RET); }

// =====================
// Nextion: low-level send
// =====================
void nxtCmd(const String &cmd) {
  nxt.print(cmd);
  nxt.write(0xFF); nxt.write(0xFF); nxt.write(0xFF);
}

// Escape quotes
String nxEscape(String s) {
  s.replace("\"", "\\\"");
  return s;
}

void nxSetTxt(const char* obj, const String &val) {
  nxtCmd(String(obj) + ".txt=\"" + nxEscape(val) + "\"");
}

void nxSetBco(const char* obj, uint16_t color) {
  nxtCmd(String(obj) + ".bco=" + String(color));
}

// =====================
// Time string (RTC if available)
// =====================
String timeHHMMSS() {
#if USE_RTC
  DateTime now = rtc.now();
  char buf[9];
  snprintf(buf, sizeof(buf), "%02d:%02d:%02d", now.hour(), now.minute(), now.second());
  return String(buf);
#else
  return "00:00:00";
#endif
}

// =====================
// Logging
// =====================
void addLog(const String &line) {
  String entry = timeHHMMSS() + " | " + line;

  // Append to big log
  if (bigLog.length() > 0) bigLog += "\r\n";
  bigLog += entry;

  // Trim from the front if too long
  if ((int)bigLog.length() > MAX_LOG_CHARS) {
    int cut = bigLog.length() - MAX_LOG_CHARS;
    // cut to next newline so we don't break mid-line
    int nl = bigLog.indexOf('\n', cut);
    if (nl > 0) bigLog = bigLog.substring(nl + 1);
    else bigLog = bigLog.substring(cut);
  }

  logDirty = true;
}

void pushLogToNextion() {
  // tClock1 is page-local on the log page, so only update it while that page is active.
  nxSetTxt("tClock1", "TIME " + timeHHMMSS());
  // Page1 has tLog (Scroll Text). Only resend the full buffer when it changed;
  // pushing a long string every second can overwhelm SoftwareSerial on an Uno.
  if (logDirty) {
    nxSetTxt("tLog", bigLog);
    logDirty = false;
  }
}

void updateLogPage() {
  pushLogToNextion();
  logDirty = false;
}

// =====================
// Output control
// =====================
void allOutputsOff() {
  digitalWrite(PIN_MOTOR_FWD, LOW);
  digitalWrite(PIN_MOTOR_REV, LOW);
  digitalWrite(PIN_MOTOR_EN, LOW);
  digitalWrite(PIN_ALARM, LOW);
}

// Motor direction helpers. These assume an H-bridge, relay pair, or step/direction
// style interface that has been abstracted into forward/reverse command lines.
// If you later use a dedicated stepper library/driver, replace these helpers only.
void moveForward(bool on) {
  digitalWrite(PIN_MOTOR_FWD, on ? HIGH : LOW);
  if (on) digitalWrite(PIN_MOTOR_REV, LOW); // never command both directions at once
}

void moveReverse(bool on) {
  digitalWrite(PIN_MOTOR_REV, on ? HIGH : LOW);
  if (on) digitalWrite(PIN_MOTOR_FWD, LOW); // never command both directions at once
}

void setMotorEn(bool on){ digitalWrite(PIN_MOTOR_EN, on ? HIGH : LOW); }
void setAlarm(bool on)  { digitalWrite(PIN_ALARM, on ? HIGH : LOW); }

// =====================
// State transitions
// =====================
void enterState(State s) {
  state = s;
  stateStartMs = millis();

  // Log state transitions (optional)
  switch (state) {
    case ST_IDLE:       addLog("STATE | IDLE"); break;
    case ST_EXTENDING:  addLog("STATE | EXTENDING"); break;
    case ST_DWELL:      addLog("STATE | DWELL"); break;
    case ST_RETRACTING: addLog("STATE | RETRACTING"); break;
    case ST_COMPLETE:   addLog("STATE | COMPLETE"); break;
    case ST_FAULT:      addLog("FAULT | " + faultMsg); break;
  }
}

// =====================
// Fault handling
// =====================
void triggerFault(const String &msg) {
  faultMsg = msg;
  allOutputsOff();
  setAlarm(true);
  enterState(ST_FAULT);
}

// =====================
// Encoder ISR
// =====================
void isrEncA() {
  encPulses++;
}

// =====================
// Nextion command parsing
// =====================
// Expecting simple newline-terminated strings:
// START, STOP, RESET, CLEARLOG, PAGE:MAIN, PAGE:LOG, PAGE:DIAG
String rxLine = "";

void setCurrentPageFromId(uint8_t pageId) {
  switch (pageId) {
    case 0:
      currentPage = PAGE_MAIN;
      Serial.println("NX PAGE: MAIN");
      break;
    case 1:
      currentPage = PAGE_LOG;
      logDirty = true;
      Serial.println("NX PAGE: LOG");
      break;
    case 2:
      currentPage = PAGE_DIAG;
      Serial.println("NX PAGE: DIAG");
      break;
    default:
      currentPage = PAGE_UNKNOWN;
      Serial.print("NX PAGE ID: ");
      Serial.println(pageId);
      break;
  }
}

void handleNxLine(const String &lineRaw) {
  String line = lineRaw;
  line.trim();

  if (line == "START") {
    if (state == ST_IDLE) {
      // Check interlocks before starting
      if (estopActive()) {
        triggerFault("E-STOP ACTIVE");
      } else {
        // Start cycle
        cycleStartMs = millis();
        encPulses = 0;
        faultMsg = "None";
        setAlarm(false);
        enterState(ST_EXTENDING);
      }
    }
  }
  else if (line == "STOP") {
    // Immediate stop/abort
    allOutputsOff();
    triggerFault("ABORTED");
  }
  else if (line == "RESET") {
    // Clear fault and return to idle
    faultMsg = "None";
    setAlarm(false);
    allOutputsOff();
    enterState(ST_IDLE);
  }
  else if (line == "CLEARLOG") {
    bigLog = "";
    addLog("INFO  | Log cleared");
    logDirty = true;
  }
  else if (line == "PAGE:MAIN") currentPage = PAGE_MAIN;
  else if (line == "PAGE:LOG")  { currentPage = PAGE_LOG;  logDirty = true; }
  else if (line == "PAGE:DIAG") currentPage = PAGE_DIAG;
}

// Read Nextion lines
void pollNextion() {
  static bool awaitingPageId = false;
  static bool awaitingPageTerminator = false;
  static uint8_t pageId = 0;
  static uint8_t ffCount = 0;

  while (nxt.available()) {
    uint8_t b = (uint8_t)nxt.read();

    if (awaitingPageId) {
      pageId = b;
      awaitingPageId = false;
      awaitingPageTerminator = true;
      ffCount = 0;
      continue;
    }

    if (awaitingPageTerminator) {
      if (b == 0xFF) {
        ffCount++;
        if (ffCount >= 3) {
          setCurrentPageFromId(pageId);
          awaitingPageTerminator = false;
          ffCount = 0;
        }
      } else {
        awaitingPageTerminator = false;
        ffCount = 0;
      }
      continue;
    }

    if (b == 0x66) {
      // Nextion "sendme" current-page packet: 0x66 <pageId> 0xFF 0xFF 0xFF
      rxLine = "";
      awaitingPageId = true;
      ffCount = 0;
      continue;
    }

    char c = (char)b;
    if (c == '\n') {
      handleNxLine(rxLine);
      rxLine = "";
    } else if (c != '\r' && c >= 32 && c <= 126) {
      rxLine += c;
      if (rxLine.length() > 80) rxLine.remove(0, 40); // prevent runaway
    }
  }
}

// =====================
// HMI update helpers
// =====================
String stateName(State s) {
  switch (s) {
    case ST_IDLE: return "IDLE";
    case ST_EXTENDING: return "EXTENDING";
    case ST_DWELL: return "DWELL";
    case ST_RETRACTING: return "RETRACTING";
    case ST_COMPLETE: return "COMPLETE";
    case ST_FAULT: return "FAULT";
  }
  return "UNKNOWN";
}

String mmss(unsigned long ms) {
  unsigned long sec = ms / 1000;
  unsigned long m = sec / 60;
  unsigned long s = sec % 60;
  char buf[6];
  snprintf(buf, sizeof(buf), "%02lu:%02lu", m, s);
  return String(buf);
}

// Update Page 0 (main)
void updateMainPage() {
  // Time
  nxSetTxt("tClock", "TIME " + timeHHMMSS());

  // State
  nxSetTxt("tState", "STATE: " + stateName(state));

  // Cycle elapsed
  unsigned long elapsed = (cycleStartMs == 0) ? 0 : (millis() - cycleStartMs);
  if (state == ST_IDLE) elapsed = 0;
  nxSetTxt("tCycle", "Cycle Elapsed: " + mmss(elapsed));

  // Inputs display
  bool eAct = estopActive();
  bool lExt = lsExtActive();
  bool lRet = lsRetActive();

  // Reuse existing HMI field tPress/tInPress so the screen does not need to be redesigned.
  // Since pneumatics were removed, this field now indicates controller readiness.
  bool driveReady = (state != ST_FAULT) && !eAct;
  nxSetTxt("tPress", String("Drive: ") + (driveReady ? "READY" : "FAULT"));
  nxSetBco("tPress", driveReady ? NX_GREEN : NX_RED);

  nxSetTxt("tEstop", String("E-STOP: ") + (eAct ? "YES" : "NO"));
  nxSetBco("tEstop", eAct ? NX_RED : NX_GRAY);

  nxSetTxt("tLsExt", String("LS_EXT: ") + (lExt ? "ON" : "OFF"));
  nxSetBco("tLsExt", lExt ? NX_GREEN : NX_GRAY);

  nxSetTxt("tLsRet", String("LS_RET: ") + (lRet ? "ON" : "OFF"));
  nxSetBco("tLsRet", lRet ? NX_GREEN : NX_GRAY);

  // Encoder/Revs
  long p = encPulses;
  float revs = p / PULSES_PER_REV;

  nxSetTxt("tEnc", "Encoder: " + String(p));
  nxSetTxt("tRevs", "Motor Revs: " + String(revs, 1));

  // Outputs display
  bool outFwd = (digitalRead(PIN_MOTOR_FWD) == HIGH);
  bool outRev = (digitalRead(PIN_MOTOR_REV) == HIGH);
  bool outMot = (digitalRead(PIN_MOTOR_EN) == HIGH);

  // Reuse existing HMI fields tVext/tVret so the screen does not need to be redesigned.
  nxSetTxt("tVext", String("Drive FWD: ") + (outFwd ? "ON" : "OFF"));
  nxSetBco("tVext", outFwd ? NX_GREEN : NX_GRAY);

  nxSetTxt("tVret", String("Drive REV: ") + (outRev ? "ON" : "OFF"));
  nxSetBco("tVret", outRev ? NX_GREEN : NX_GRAY);

  nxSetTxt("tMotor", String("Motor EN: ") + (outMot ? "ON" : "OFF"));
  nxSetBco("tMotor", outMot ? NX_GREEN : NX_GRAY);

  // Fault banner
  if (state == ST_FAULT) {
    nxSetTxt("tFault", "FAULT / MESSAGE: " + faultMsg);
    nxSetBco("tFault", NX_RED);
  } else {
    nxSetTxt("tFault", "FAULT / MESSAGE: None");
    nxSetBco("tFault", NX_GRAY);
  }
}

// Update Page 2 (diagnostics)
void updateDiagPage() {
  nxSetTxt("tClock2", "TIME " + timeHHMMSS());

  bool eAct = estopActive();
  bool lExt = lsExtActive();
  bool lRet = lsRetActive();

  // Inputs
  bool driveReady = (state != ST_FAULT) && !eAct;
  nxSetTxt("tInPress", String("DRIVE: ") + (driveReady ? "READY" : "FAULT"));
  nxSetBco("tInPress", driveReady ? NX_GREEN : NX_RED);

  nxSetTxt("tInLsExt", String("LS_EXT: ") + (lExt ? "ON" : "OFF"));
  nxSetBco("tInLsExt", lExt ? NX_GREEN : NX_GRAY);

  nxSetTxt("tInLsRet", String("LS_RET: ") + (lRet ? "ON" : "OFF"));
  nxSetBco("tInLsRet", lRet ? NX_GREEN : NX_GRAY);

  nxSetTxt("tInEstop", String("E-STOP: ") + (eAct ? "ON" : "OFF"));
  nxSetBco("tInEstop", eAct ? NX_RED : NX_GRAY);

  // Outputs
  bool outFwd = (digitalRead(PIN_MOTOR_FWD) == HIGH);
  bool outRev = (digitalRead(PIN_MOTOR_REV) == HIGH);
  bool outMot = (digitalRead(PIN_MOTOR_EN) == HIGH);
  bool outAlm = (digitalRead(PIN_ALARM) == HIGH);

  nxSetTxt("tOutExt", String("DRIVE_FWD: ") + (outFwd ? "ON" : "OFF"));
  nxSetBco("tOutExt", outFwd ? NX_GREEN : NX_GRAY);

  nxSetTxt("tOutRet", String("DRIVE_REV: ") + (outRev ? "ON" : "OFF"));
  nxSetBco("tOutRet", outRev ? NX_GREEN : NX_GRAY);

  nxSetTxt("tOutMotor", String("MOTOR_EN: ") + (outMot ? "ON" : "OFF"));
  nxSetBco("tOutMotor", outMot ? NX_GREEN : NX_GRAY);

  nxSetTxt("tOutAlarm", String("ALARM: ") + (outAlm ? "ON" : "OFF"));
  nxSetBco("tOutAlarm", outAlm ? NX_RED : NX_GRAY);

  // Fault line
  if (state == ST_FAULT) {
    nxSetTxt("tFault2", "FAULT: " + faultMsg);
    nxSetBco("tFault2", NX_RED);
  } else {
    nxSetTxt("tFault2", "FAULT: None");
    nxSetBco("tFault2", NX_GRAY);
  }
}

// =====================
// Main control logic
// =====================
void loopStateMachine() {
  // Global interlocks: E-STOP always wins
  if (estopActive() && state != ST_IDLE && state != ST_FAULT) {
    triggerFault("E-STOP ACTIVE");
    return;
  }

  switch (state) {
    case ST_IDLE:
      allOutputsOff();
      // Optionally read physical buttons too:
      if (startPressed()) handleNxLine("START");
      break;

    case ST_EXTENDING: {
      // Drive mechanism forward until the extend limit switch is reached.
      setMotorEn(true);
      moveReverse(false);
      moveForward(true);

      if (lsExtActive()) {
        moveForward(false);
        setMotorEn(false);
        enterState(ST_DWELL);
      } else if (millis() - stateStartMs > EXTEND_TIMEOUT_MS) {
        triggerFault("EXT TIMEOUT");
      }
    } break;

    case ST_DWELL:
      allOutputsOff();
      if (millis() - stateStartMs > DWELL_MS) {
        enterState(ST_RETRACTING);
      }
      break;

    case ST_RETRACTING: {
      // Drive mechanism in reverse until the retract limit switch is reached.
      setMotorEn(true);
      moveForward(false);
      moveReverse(true);

      if (lsRetActive()) {
        moveReverse(false);
        setMotorEn(false);
        enterState(ST_COMPLETE);
      } else if (millis() - stateStartMs > RETRACT_TIMEOUT_MS) {
        triggerFault("RET TIMEOUT");
      }
    } break;

    case ST_COMPLETE:
      allOutputsOff();
      addLog("INFO  | Cycle complete");
      enterState(ST_IDLE);
      break;

    case ST_FAULT:
      allOutputsOff();
      setAlarm(true);
      // Optionally allow physical reset:
      if (resetPressed()) handleNxLine("RESET");
      break;
  }
}

// =====================
// Setup
// =====================
void setup() {
  // IO
  pinMode(PIN_ENC_A, INPUT_PULLUP);
  pinMode(PIN_START, INPUT_PULLUP);
  pinMode(PIN_STOP, INPUT_PULLUP);
  pinMode(PIN_RESET, INPUT_PULLUP);
  pinMode(PIN_ESTOP, INPUT_PULLUP);
  pinMode(PIN_LS_EXT, INPUT_PULLUP);
  pinMode(PIN_LS_RET, INPUT_PULLUP);

  pinMode(PIN_MOTOR_FWD, OUTPUT);
  pinMode(PIN_MOTOR_REV, OUTPUT);
  pinMode(PIN_MOTOR_EN, OUTPUT);
  pinMode(PIN_ALARM, OUTPUT);

  allOutputsOff();

  // Encoder interrupt (optional)
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), isrEncA, RISING);

  // Serial
  Serial.begin(115200);   // USB debug
  nxt.begin(9600);        // common Nextion baud
  nxtCmd("bkcmd=0");
  nxtCmd("sendxy=0");
  nxtCmd("sendme");

  Wire.begin();

#if USE_RTC
  if (!rtc.begin()) {
    // If RTC missing, still run
    Serial.println("RTC not found");
  } else if (rtc.lostPower()) {
      Serial.println("RTC found");
      // Uncomment once if you need to set the RTC from compile time:
     //rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
#endif

  // Boot logs
  addLog("INFO  | Boot");
  enterState(ST_IDLE);
}

// =====================
// Main loop
// =====================
unsigned long lastHmiUpdateMs = 0;
const unsigned long HMI_UPDATE_MS = 1000;
unsigned long lastRtcDebugMs = 0;
unsigned long lastPageQueryMs = 0;
const unsigned long PAGE_QUERY_MS = 500;

void loop() {
  // Read Nextion commands
  pollNextion();

  // Physical STOP button as abort (optional)
  if (stopPressed() && state != ST_IDLE) {
    handleNxLine("STOP");
  }

  // Run control logic
  loopStateMachine();

  if (millis() - lastRtcDebugMs >= 1000) {
    lastRtcDebugMs = millis();
    Serial.println(timeHHMMSS());
  }

  // Ask Nextion for the active page periodically so a missed page-change packet
  // does not leave the Arduino updating the wrong local widgets forever.
  if (millis() - lastPageQueryMs >= PAGE_QUERY_MS) {
    lastPageQueryMs = millis();
    nxtCmd("sendme");
  }

  // Throttle HMI updates so SoftwareSerial does not get overwhelmed.
  if (millis() - lastHmiUpdateMs >= HMI_UPDATE_MS) {
    lastHmiUpdateMs = millis();

    // Nextion clock widgets are page-local, so only update the widgets
    // that exist on the page currently being displayed.
    switch (currentPage) {
      case PAGE_MAIN:
        updateMainPage();
        break;

      case PAGE_LOG:
        updateLogPage();
        break;

      case PAGE_DIAG:
        updateDiagPage();
        break;

      case PAGE_UNKNOWN:
      default:
        // Fall back to the main page until the HMI reports the active page.
        updateMainPage();
        break;
    }
  }
}
