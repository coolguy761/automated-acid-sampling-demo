/*
  Automated Acid Sampling Demo - Failsafe Single-Motor HMI Demo
  -------------------------------------------------------------
  Purpose:
    - Provide a demo path that keeps the HMI and main lift mechanism working
      even if the full multi-actuator parent/child architecture is not ready.
    - Control only the primary lift stepper for extend / retract motion.
    - Use a STEP / DIR / ENA microstep driver with the same Nextion + limit
      switch structure as the parent-style prototype.

  Demo sequence:
    IDLE
    -> EXTENDING_LIFT
    -> HOLD_AT_DEPTH
    -> RETRACTING_LIFT
    -> COMPLETE
    -> IDLE

  Supported HMI / serial commands:
    - START
    - STOP
    - RESET
    - PAGE:MAIN
    - PAGE:LOG
    - PAGE:LOGS
    - PAGE:DIAG
    - CLEARLOG

  Limit switch wiring model:
    - Each end-of-travel switch is SPDT.
    - COM -> GND
    - NO  -> "reached" pin
    - NC  -> "healthy/not reached" pin
    - Inputs use INPUT_PULLUP.
*/

#include <Wire.h>
#include <SoftwareSerial.h>
#include <RTClib.h>

#define USE_RTC 1
#define USE_NEXTION 1
#define USE_ENCODER 1
#define USE_PHYSICAL_BUTTONS 0

// -----------------------------
// Pin map (same lift/HMI wiring as Parent Uno)
// -----------------------------
const uint8_t PIN_ENC_A           = 2;
const uint8_t PIN_START_BTN       = 3;
const uint8_t PIN_STOP_BTN        = 4;
const uint8_t PIN_RESET_BTN       = 5;
const uint8_t PIN_ESTOP_NC        = 6;
const uint8_t PIN_LS_EXT_REACHED  = 7;
const uint8_t PIN_LS_RET_REACHED  = 8;

const uint8_t PIN_NX_RX           = 10;
const uint8_t PIN_NX_TX           = 11;

const uint8_t PIN_STEP_PUL        = 12;  // Driver PUL+
const uint8_t PIN_STEP_DIR        = 13;  // Driver DIR+
const uint8_t PIN_STEP_ENA        = A0;  // Driver ENA+
const uint8_t PIN_ALARM           = A1;
const uint8_t PIN_LS_EXT_HEALTHY  = A2;
const uint8_t PIN_LS_RET_HEALTHY  = A3;

#if USE_NEXTION
SoftwareSerial nxt(PIN_NX_RX, PIN_NX_TX);
#endif

#if USE_RTC
RTC_DS3231 rtc;
bool rtcOk = false;
#endif

volatile long encPulses = 0;
const float PULSES_PER_REV = 20.0f;

const unsigned long EXTEND_TIMEOUT_MS = 15000UL;
const unsigned long HOLD_DWELL_MS     = 3000UL;
const unsigned long RETRACT_TIMEOUT_MS= 15000UL;
const unsigned long PAGE_REFRESH_MS   = 300UL;
const unsigned long PAGE_QUERY_MS     = 500UL;
const unsigned long CLOCK_UPDATE_MS   = 1000UL;
const unsigned long DEBUG_PRINT_MS    = 1000UL;
const unsigned long DEBOUNCE_MS       = 40UL;
const unsigned long STEP_INTERVAL_US  = 1500UL;
const unsigned long STEP_PULSE_US     = 10UL;

const uint8_t DRIVER_ENABLE_LEVEL  = HIGH;
const uint8_t DRIVER_DISABLE_LEVEL = LOW;
const uint8_t DIR_EXTEND_LEVEL     = HIGH;
const uint8_t DIR_RETRACT_LEVEL    = LOW;

const size_t MAX_LOG_LINES = 8;
const size_t MAX_LOG_LINE_CHARS = 72;
char logLines[MAX_LOG_LINES][MAX_LOG_LINE_CHARS];
size_t logCount = 0;
bool logDirty = true;

enum VisiblePage { VP_MAIN, VP_LOGS, VP_DIAG };
VisiblePage visiblePage = VP_MAIN;

struct DebouncedInput {
  uint8_t pin;
  bool stableState;
  bool lastRead;
  unsigned long lastChangeMs;
};

enum LimitPairState {
  LP_NOT_REACHED = 0,
  LP_REACHED,
  LP_FAULT_OPEN,
  LP_FAULT_SHORT
};

enum State {
  ST_IDLE = 0,
  ST_EXTENDING_LIFT,
  ST_HOLD_AT_DEPTH,
  ST_RETRACTING_LIFT,
  ST_COMPLETE,
  ST_FAULT
};

State currentState = ST_IDLE;
bool faultLatched = false;
bool cycleActive = false;
char faultMsg[48] = "";
unsigned long stateStartMs = 0;
unsigned long cycleStartMs = 0;
unsigned long lastPageRefreshMs = 0;
unsigned long lastPageQueryMs = 0;
unsigned long lastClockMs = 0;
unsigned long lastDebugMs = 0;
unsigned long lastStepToggleUs = 0;
bool stepPulseHigh = false;
bool stepperEnabled = false;

enum MotionDirection {
  MD_STOPPED = 0,
  MD_EXTEND,
  MD_RETRACT
};

MotionDirection motionDirection = MD_STOPPED;

#if USE_PHYSICAL_BUTTONS
DebouncedInput dbStart = {PIN_START_BTN, HIGH, HIGH, 0};
DebouncedInput dbStop  = {PIN_STOP_BTN,  HIGH, HIGH, 0};
DebouncedInput dbReset = {PIN_RESET_BTN, HIGH, HIGH, 0};
#endif

void logEvent(const String &msg);
void enterState(State s);
void handleState();
void faultNow(const String &msg);
void allOutputsOff();
void startStepping(MotionDirection direction);
void stopStepping();
void serviceStepper();
void setStepperEnable(bool on);
bool estopActive();
LimitPairState readLimitPair(uint8_t reachedPin, uint8_t healthyPin);
LimitPairState liftExtendState();
LimitPairState liftRetractState();
bool limitPairIsHealthy(LimitPairState state);
bool limitPairIsReached(LimitPairState state);
String limitPairName(LimitPairState state);
void encISR();
long getEncoderPulses();
void pollPhysicalButtons();
bool updateDebounced(DebouncedInput &db);
void onStartPressed();
void onStopPressed();
void onResetPressed();
void updateHMI(bool force = false);
void updateMainPage(bool force = false);
void updateDiagPage(bool force = false);
void updateLogPage(bool force = false);
void updateAllClocks();
void readNextion();
void setCurrentPageFromId(uint8_t pageId);
void handleNxLine(String line);
void nxCmd(const String &cmd);
void nxSetTxt(const String &obj, const String &txt);
String timeHHMMSS();
String stateName(State s);

void setup() {
  Serial.begin(115200);

#if USE_PHYSICAL_BUTTONS
  pinMode(PIN_START_BTN, INPUT_PULLUP);
  pinMode(PIN_STOP_BTN, INPUT_PULLUP);
  pinMode(PIN_RESET_BTN, INPUT_PULLUP);
#endif
  pinMode(PIN_ESTOP_NC, INPUT_PULLUP);
  pinMode(PIN_LS_EXT_REACHED, INPUT_PULLUP);
  pinMode(PIN_LS_RET_REACHED, INPUT_PULLUP);
  pinMode(PIN_LS_EXT_HEALTHY, INPUT_PULLUP);
  pinMode(PIN_LS_RET_HEALTHY, INPUT_PULLUP);

  pinMode(PIN_STEP_PUL, OUTPUT);
  pinMode(PIN_STEP_DIR, OUTPUT);
  pinMode(PIN_STEP_ENA, OUTPUT);
  pinMode(PIN_ALARM, OUTPUT);
  allOutputsOff();

#if USE_NEXTION
  nxt.begin(9600);
  nxCmd("bkcmd=0");
  nxCmd("sendxy=0");
  nxCmd("sendme");
#endif

#if USE_RTC
  if (rtc.begin()) {
    rtcOk = true;
    if (rtc.lostPower()) {
      Serial.println(F("RTC lost power. Set time once using rtc.adjust(...), then comment it back out."));
    }
  } else {
    rtcOk = false;
    Serial.println(F("RTC not found."));
  }
#endif

#if USE_ENCODER
  pinMode(PIN_ENC_A, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), encISR, RISING);
#endif

  logEvent("FAILSAFE DEMO BOOT");
  enterState(ST_IDLE);
}

void loop() {
#if USE_PHYSICAL_BUTTONS
  pollPhysicalButtons();
#endif

  if (estopActive() && currentState != ST_FAULT) {
    faultNow("E-STOP ACTIVE");
  }

#if USE_NEXTION
  readNextion();
#endif

  handleState();
  serviceStepper();

  if (millis() - lastClockMs >= CLOCK_UPDATE_MS) {
    lastClockMs = millis();
    updateAllClocks();
  }

  if (millis() - lastPageQueryMs >= PAGE_QUERY_MS) {
    lastPageQueryMs = millis();
#if USE_NEXTION
    nxCmd("sendme");
#endif
  }

  if (millis() - lastPageRefreshMs >= PAGE_REFRESH_MS) {
    lastPageRefreshMs = millis();
    updateHMI(false);
  }

  if (millis() - lastDebugMs >= DEBUG_PRINT_MS) {
    lastDebugMs = millis();
    Serial.print(F("Time: "));
    Serial.print(timeHHMMSS());
    Serial.print(F(" | State: "));
    Serial.print(stateName(currentState));
    Serial.print(F(" | Ext: "));
    Serial.print(limitPairName(liftExtendState()));
    Serial.print(F(" | Ret: "));
    Serial.println(limitPairName(liftRetractState()));
  }
}

void handleState() {
  unsigned long elapsed = millis() - stateStartMs;

  switch (currentState) {
    case ST_IDLE:
      allOutputsOff();
      break;

    case ST_EXTENDING_LIFT: {
      LimitPairState extend = liftExtendState();
      if (!limitPairIsHealthy(extend)) {
        faultNow("EXT LIMIT FAULT");
        break;
      }

      if (limitPairIsReached(extend)) {
        stopStepping();
        logEvent("LIFT AT DEPTH");
        enterState(ST_HOLD_AT_DEPTH);
        break;
      }

      startStepping(MD_EXTEND);

      if (elapsed > EXTEND_TIMEOUT_MS) {
        faultNow("EXTEND TIMEOUT");
      }
    } break;

    case ST_HOLD_AT_DEPTH:
      allOutputsOff();
      if (elapsed >= HOLD_DWELL_MS) {
        enterState(ST_RETRACTING_LIFT);
      }
      break;

    case ST_RETRACTING_LIFT: {
      LimitPairState retract = liftRetractState();
      if (!limitPairIsHealthy(retract)) {
        faultNow("RET LIMIT FAULT");
        break;
      }

      if (limitPairIsReached(retract)) {
        stopStepping();
        logEvent("LIFT HOME");
        enterState(ST_COMPLETE);
        break;
      }

      startStepping(MD_RETRACT);

      if (elapsed > RETRACT_TIMEOUT_MS) {
        faultNow("RETRACT TIMEOUT");
      }
    } break;

    case ST_COMPLETE:
      cycleActive = false;
      logEvent("DEMO CYCLE COMPLETE");
      enterState(ST_IDLE);
      break;

    case ST_FAULT:
      allOutputsOff();
      break;
  }
}

void enterState(State s) {
  currentState = s;
  stateStartMs = millis();

  if (s == ST_IDLE) {
    cycleActive = false;
    cycleStartMs = 0;
  }

  if (s == ST_EXTENDING_LIFT) {
    cycleActive = true;
    cycleStartMs = millis();
  }

  logEvent("STATE -> " + stateName(s));
  updateHMI(true);
}

void faultNow(const String &msg) {
  faultLatched = true;
  msg.toCharArray(faultMsg, sizeof(faultMsg));
  cycleActive = false;
  allOutputsOff();
  logEvent("FAULT: " + msg);
  currentState = ST_FAULT;
  stateStartMs = millis();
  updateHMI(true);
}

void allOutputsOff() {
  stopStepping();
  digitalWrite(PIN_ALARM, faultLatched ? HIGH : LOW);
}

void startStepping(MotionDirection direction) {
  if (direction == MD_STOPPED) {
    stopStepping();
    return;
  }

  if (motionDirection != direction) {
    motionDirection = direction;
    digitalWrite(PIN_STEP_DIR, direction == MD_EXTEND ? DIR_EXTEND_LEVEL : DIR_RETRACT_LEVEL);
    stepPulseHigh = false;
    digitalWrite(PIN_STEP_PUL, LOW);
    lastStepToggleUs = micros();
  }

  if (!stepperEnabled) {
    setStepperEnable(true);
  }
}

void stopStepping() {
  motionDirection = MD_STOPPED;
  stepPulseHigh = false;
  digitalWrite(PIN_STEP_PUL, LOW);
  setStepperEnable(false);
}

void serviceStepper() {
  if (motionDirection == MD_STOPPED || !stepperEnabled) return;

  unsigned long now = micros();
  if (!stepPulseHigh) {
    if ((unsigned long)(now - lastStepToggleUs) >= STEP_INTERVAL_US) {
      digitalWrite(PIN_STEP_PUL, HIGH);
      stepPulseHigh = true;
      lastStepToggleUs = now;
    }
  } else if ((unsigned long)(now - lastStepToggleUs) >= STEP_PULSE_US) {
    digitalWrite(PIN_STEP_PUL, LOW);
    stepPulseHigh = false;
    lastStepToggleUs = now;
  }
}

void setStepperEnable(bool on) {
  stepperEnabled = on;
  digitalWrite(PIN_STEP_ENA, on ? DRIVER_ENABLE_LEVEL : DRIVER_DISABLE_LEVEL);
}

bool estopActive() {
  return digitalRead(PIN_ESTOP_NC) == HIGH;
}

LimitPairState readLimitPair(uint8_t reachedPin, uint8_t healthyPin) {
  bool noHigh = (digitalRead(reachedPin) == HIGH);
  bool ncHigh = (digitalRead(healthyPin) == HIGH);

  if (noHigh && !ncHigh) return LP_NOT_REACHED;
  if (!noHigh && ncHigh) return LP_REACHED;
  if (noHigh && ncHigh) return LP_FAULT_OPEN;
  return LP_FAULT_SHORT;
}

LimitPairState liftExtendState() {
  return readLimitPair(PIN_LS_EXT_REACHED, PIN_LS_EXT_HEALTHY);
}

LimitPairState liftRetractState() {
  return readLimitPair(PIN_LS_RET_REACHED, PIN_LS_RET_HEALTHY);
}

bool limitPairIsHealthy(LimitPairState state) {
  return state == LP_NOT_REACHED || state == LP_REACHED;
}

bool limitPairIsReached(LimitPairState state) {
  return state == LP_REACHED;
}

String limitPairName(LimitPairState state) {
  switch (state) {
    case LP_NOT_REACHED: return "NR";
    case LP_REACHED:     return "OK";
    case LP_FAULT_OPEN:  return "OPEN";
    case LP_FAULT_SHORT: return "SHORT";
    default:             return "?";
  }
}

void encISR() {
  encPulses++;
}

long getEncoderPulses() {
  noInterrupts();
  long p = encPulses;
  interrupts();
  return p;
}

void pollPhysicalButtons() {
#if !USE_PHYSICAL_BUTTONS
  return;
#else
  if (updateDebounced(dbStart) && dbStart.stableState == LOW) onStartPressed();
  if (updateDebounced(dbStop)  && dbStop.stableState  == LOW) onStopPressed();
  if (updateDebounced(dbReset) && dbReset.stableState == LOW) onResetPressed();
#endif
}

bool updateDebounced(DebouncedInput &db) {
  bool raw = digitalRead(db.pin);
  bool changed = false;

  if (raw != db.lastRead) {
    db.lastRead = raw;
    db.lastChangeMs = millis();
  }

  if ((millis() - db.lastChangeMs) > DEBOUNCE_MS && raw != db.stableState) {
    db.stableState = raw;
    changed = true;
  }

  return changed;
}

void onStartPressed() {
  if (currentState != ST_IDLE || faultLatched) return;

  if (estopActive()) {
    faultNow("E-STOP ACTIVE");
    return;
  }

  if (!limitPairIsHealthy(liftExtendState()) || !limitPairIsHealthy(liftRetractState())) {
    faultNow("LIMITS NOT HEALTHY");
    return;
  }

  logEvent("START DEMO");
  enterState(ST_EXTENDING_LIFT);
}

void onStopPressed() {
  if (currentState != ST_IDLE && currentState != ST_FAULT) {
    faultNow("STOP/ABORT PRESSED");
  }
}

void onResetPressed() {
  if (!(currentState == ST_FAULT || faultLatched)) return;

  if (estopActive()) {
    logEvent("RESET BLOCKED: ESTOP");
    return;
  }

  faultLatched = false;
  faultMsg[0] = '\0';
  digitalWrite(PIN_ALARM, LOW);
  logEvent("RESET TO IDLE");
  enterState(ST_IDLE);
}

void updateHMI(bool force) {
  if (visiblePage == VP_MAIN) {
    updateMainPage(force);
  } else if (visiblePage == VP_DIAG) {
    updateDiagPage(force);
  } else if (visiblePage == VP_LOGS) {
    updateLogPage(force);
  }
}

void updateMainPage(bool force) {
#if USE_NEXTION
  static String lastClock = "";
  static String lastState = "";
  static String lastFault = "";
  static String lastCycle = "";
  static String lastReady = "";
  static String lastExt = "";
  static String lastRet = "";
  static String lastMotor = "";
  static String lastEstop = "";
  static String lastLsExt = "";
  static String lastLsRet = "";
  static String lastEnc = "";
  static String lastRevs = "";

  String clock = "TIME " + timeHHMMSS();
  if (force || clock != lastClock) {
    nxSetTxt("tClock", clock);
    lastClock = clock;
  }

  String state = "STATE: " + stateName(currentState);
  if (force || state != lastState) {
    nxSetTxt("tState", state);
    lastState = state;
  }

  unsigned long elapsed = cycleActive ? (millis() - cycleStartMs) : 0;
  char buf[6];
  snprintf(buf, sizeof(buf), "%02lu:%02lu", (elapsed / 1000UL) / 60UL, (elapsed / 1000UL) % 60UL);
  String cycle = "Cycle Elapsed: " + String(buf);
  if (force || cycle != lastCycle) {
    nxSetTxt("tCycle", cycle);
    lastCycle = cycle;
  }

  String ready = String("Drive: ") + ((currentState != ST_FAULT && !estopActive()) ? "READY" : "FAULT");
  if (force || ready != lastReady) {
    nxSetTxt("tPress", ready);
    lastReady = ready;
  }

  String estop = String("E-STOP: ") + (estopActive() ? "YES" : "NO");
  if (force || estop != lastEstop) {
    nxSetTxt("tEstop", estop);
    lastEstop = estop;
  }

  String lsExt = "LS_EXT: " + limitPairName(liftExtendState());
  if (force || lsExt != lastLsExt) {
    nxSetTxt("tLsExt", lsExt);
    lastLsExt = lsExt;
  }

  String lsRet = "LS_RET: " + limitPairName(liftRetractState());
  if (force || lsRet != lastLsRet) {
    nxSetTxt("tLsRet", lsRet);
    lastLsRet = lsRet;
  }

  String ext = "Drive FWD: " + String(motionDirection == MD_EXTEND ? "ON" : "OFF");
  if (force || ext != lastExt) {
    nxSetTxt("tVext", ext);
    lastExt = ext;
  }

  String ret = "Drive REV: " + String(motionDirection == MD_RETRACT ? "ON" : "OFF");
  if (force || ret != lastRet) {
    nxSetTxt("tVret", ret);
    lastRet = ret;
  }

  String motor = "Motor EN: " + String(stepperEnabled ? "ON" : "OFF");
  if (force || motor != lastMotor) {
    nxSetTxt("tMotor", motor);
    lastMotor = motor;
  }

  long pulses = getEncoderPulses();
  String enc = "Encoder: " + String(pulses);
  if (force || enc != lastEnc) {
    nxSetTxt("tEnc", enc);
    lastEnc = enc;
  }

  String revs = "Motor Revs: " + String(pulses / PULSES_PER_REV, 1);
  if (force || revs != lastRevs) {
    nxSetTxt("tRevs", revs);
    lastRevs = revs;
  }

  String fault = faultLatched ? String("FAULT / MESSAGE: ") + faultMsg : "FAULT / MESSAGE: None";
  if (force || fault != lastFault) {
    nxSetTxt("tFault", fault);
    lastFault = fault;
  }
#endif
}

void updateDiagPage(bool force) {
#if USE_NEXTION
  static String lastClock = "";
  static String lastInPress = "";
  static String lastInLsExt = "";
  static String lastInLsRet = "";
  static String lastInEstop = "";
  static String lastOutExt = "";
  static String lastOutRet = "";
  static String lastOutMotor = "";
  static String lastOutAlarm = "";
  static String lastFault = "";

  String clock = "TIME " + timeHHMMSS();
  if (force || clock != lastClock) {
    nxSetTxt("tClock2", clock);
    lastClock = clock;
  }

  String inPress = String("DRIVE: ") + ((currentState != ST_FAULT && !estopActive()) ? "READY" : "FAULT");
  if (force || inPress != lastInPress) {
    nxSetTxt("tInPress", inPress);
    lastInPress = inPress;
  }

  String inLsExt = "LS_EXT: " + limitPairName(liftExtendState());
  if (force || inLsExt != lastInLsExt) {
    nxSetTxt("tInLsExt", inLsExt);
    lastInLsExt = inLsExt;
  }

  String inLsRet = "LS_RET: " + limitPairName(liftRetractState());
  if (force || inLsRet != lastInLsRet) {
    nxSetTxt("tInLsRet", inLsRet);
    lastInLsRet = inLsRet;
  }

  String inEstop = String("E-STOP: ") + (estopActive() ? "ON" : "OFF");
  if (force || inEstop != lastInEstop) {
    nxSetTxt("tInEstop", inEstop);
    lastInEstop = inEstop;
  }

  String outExt = String("DRIVE_FWD: ") + (motionDirection == MD_EXTEND ? "ON" : "OFF");
  if (force || outExt != lastOutExt) {
    nxSetTxt("tOutExt", outExt);
    lastOutExt = outExt;
  }

  String outRet = String("DRIVE_REV: ") + (motionDirection == MD_RETRACT ? "ON" : "OFF");
  if (force || outRet != lastOutRet) {
    nxSetTxt("tOutRet", outRet);
    lastOutRet = outRet;
  }

  String outMotor = String("MOTOR_EN: ") + (stepperEnabled ? "ON" : "OFF");
  if (force || outMotor != lastOutMotor) {
    nxSetTxt("tOutMotor", outMotor);
    lastOutMotor = outMotor;
  }

  String outAlarm = String("ALARM: ") + (digitalRead(PIN_ALARM) == HIGH ? "ON" : "OFF");
  if (force || outAlarm != lastOutAlarm) {
    nxSetTxt("tOutAlarm", outAlarm);
    lastOutAlarm = outAlarm;
  }

  String fault = faultLatched ? String("FAULT: ") + faultMsg : "FAULT: None";
  if (force || fault != lastFault) {
    nxSetTxt("tFault2", fault);
    lastFault = fault;
  }
#endif
}

void updateLogPage(bool force) {
#if USE_NEXTION
  static String lastClock = "";
  String clock = "TIME " + timeHHMMSS();
  if (force || clock != lastClock) {
    nxSetTxt("tClock1", clock);
    lastClock = clock;
  }

  if (!force && !logDirty) return;

  String block = "";
  for (size_t i = 0; i < logCount; i++) {
    block += String(logLines[i]);
    if (i + 1 < logCount) block += "\r\n";
  }
  nxSetTxt("tLog", block);
  logDirty = false;
#endif
}

void updateAllClocks() {
#if USE_NEXTION
  if (visiblePage == VP_MAIN) {
    nxSetTxt("tClock", "TIME " + timeHHMMSS());
  } else if (visiblePage == VP_LOGS) {
    nxSetTxt("tClock1", "TIME " + timeHHMMSS());
  } else if (visiblePage == VP_DIAG) {
    nxSetTxt("tClock2", "TIME " + timeHHMMSS());
  }
#endif
}

void readNextion() {
#if USE_NEXTION
  static String line = "";
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
      line = "";
      awaitingPageId = true;
      ffCount = 0;
      continue;
    }

    char c = (char)b;
    if (c == '\n') {
      line.trim();
      if (line.length()) handleNxLine(line);
      line = "";
    } else if (c != '\r' && c >= 32 && c <= 126) {
      line += c;
      if (line.length() > 80) line.remove(0, 40);
    }
  }
#endif
}

void setCurrentPageFromId(uint8_t pageId) {
  if (pageId == 0) {
    visiblePage = VP_MAIN;
    Serial.println(F("NX PAGE: MAIN"));
  } else if (pageId == 1) {
    visiblePage = VP_LOGS;
    logDirty = true;
    Serial.println(F("NX PAGE: LOG"));
  } else if (pageId == 2) {
    visiblePage = VP_DIAG;
    Serial.println(F("NX PAGE: DIAG"));
  } else {
    Serial.print(F("NX PAGE ID: "));
    Serial.println(pageId);
    return;
  }

  updateHMI(true);
}

void handleNxLine(String line) {
  line.trim();
  line.toUpperCase();
  if (!line.length()) return;

  Serial.print(F("NX CMD: "));
  Serial.println(line);

  if (line == "START") {
    onStartPressed();
  } else if (line == "STOP") {
    onStopPressed();
  } else if (line == "RESET") {
    onResetPressed();
  } else if (line == "PAGE:MAIN") {
    visiblePage = VP_MAIN;
    updateHMI(true);
  } else if (line == "PAGE:LOG" || line == "PAGE:LOGS") {
    visiblePage = VP_LOGS;
    logDirty = true;
    updateHMI(true);
  } else if (line == "PAGE:DIAG") {
    visiblePage = VP_DIAG;
    updateHMI(true);
  } else if (line == "CLEARLOG") {
    logCount = 0;
    logDirty = true;
    logEvent("INFO | Log cleared");
  }
}

void nxCmd(const String &cmd) {
#if USE_NEXTION
  nxt.print(cmd);
  nxt.write(0xFF);
  nxt.write(0xFF);
  nxt.write(0xFF);
#endif
}

void nxSetTxt(const String &obj, const String &txt) {
  String safe = txt;
  safe.replace("\"", "'");
  nxCmd(obj + ".txt=\"" + safe + "\"");
}

String timeHHMMSS() {
#if USE_RTC
  if (rtcOk) {
    DateTime now = rtc.now();
    char buf[9];
    snprintf(buf, sizeof(buf), "%02d:%02d:%02d", now.hour(), now.minute(), now.second());
    return String(buf);
  }
#endif

  unsigned long s = millis() / 1000UL;
  unsigned long h = (s / 3600UL) % 24UL;
  unsigned long m = (s / 60UL) % 60UL;
  unsigned long sec = s % 60UL;

  char buf[9];
  snprintf(buf, sizeof(buf), "%02lu:%02lu:%02lu", h, m, sec);
  return String(buf);
}

String stateName(State s) {
  switch (s) {
    case ST_IDLE:            return "IDLE";
    case ST_EXTENDING_LIFT:  return "EXTENDING";
    case ST_HOLD_AT_DEPTH:   return "HOLD";
    case ST_RETRACTING_LIFT: return "RETRACTING";
    case ST_COMPLETE:        return "COMPLETE";
    case ST_FAULT:           return "FAULT";
    default:                 return "UNKNOWN";
  }
}

void logEvent(const String &msg) {
  String line = timeHHMMSS() + " " + msg;
  char lineBuf[MAX_LOG_LINE_CHARS];
  line.toCharArray(lineBuf, sizeof(lineBuf));

  if (logCount < MAX_LOG_LINES) {
    strncpy(logLines[logCount], lineBuf, MAX_LOG_LINE_CHARS - 1);
    logLines[logCount][MAX_LOG_LINE_CHARS - 1] = '\0';
    logCount++;
  } else {
    for (size_t i = 1; i < MAX_LOG_LINES; i++) {
      strncpy(logLines[i - 1], logLines[i], MAX_LOG_LINE_CHARS - 1);
      logLines[i - 1][MAX_LOG_LINE_CHARS - 1] = '\0';
    }
    strncpy(logLines[MAX_LOG_LINES - 1], lineBuf, MAX_LOG_LINE_CHARS - 1);
    logLines[MAX_LOG_LINES - 1][MAX_LOG_LINE_CHARS - 1] = '\0';
  }

  Serial.println(line);
  logDirty = true;
}
