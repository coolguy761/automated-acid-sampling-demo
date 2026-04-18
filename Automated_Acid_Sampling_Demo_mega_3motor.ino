/*
  Automated Acid Sampling Demo (Motorized, 3-Motor Revision)
  Target: Arduino Mega 2560 R3 + Nextion + DS3231 RTC

  Why this file exists:
  - The 3-motor design exceeds the practical I/O budget of an Arduino Uno.
  - This scaffold moves the design to an Arduino Mega and keeps the HMI mostly unchanged.
  - The main page can still reuse tState, tCycle, tFault, tClock, tVext, and tVret.

  Intended sequence:
  IDLE
  EXTENDING_LIFT
  OPENING_CUP_AT_DEPTH
  FILL_DWELL
  CLOSING_CUP_AT_DEPTH
  RETRACTING_LIFT
  TILTING_TO_DUMP
  OPENING_CUP_TO_DUMP
  DUMP_DWELL
  CLOSING_CUP_AFTER_DUMP
  UNTILTING_HOME
  COMPLETE
  FAULT
*/

#include <Wire.h>

#define USE_RTC 1
#if USE_RTC
  #include <RTClib.h>
  RTC_DS3231 rtc;
#endif

// =====================
// Pin Mapping (Mega 2560 R3)
// =====================

// Inputs
const uint8_t PIN_ENC_A            = 2;   // optional encoder input
const uint8_t PIN_START            = 37;
const uint8_t PIN_STOP             = 38;
const uint8_t PIN_RESET            = 39;
const uint8_t PIN_ESTOP            = 40;

const uint8_t PIN_LIFT_LS_EXT      = 31;
const uint8_t PIN_LIFT_LS_RET      = 32;
const uint8_t PIN_CUP_LS_OPEN      = 33;
const uint8_t PIN_CUP_LS_CLOSED    = 34;
const uint8_t PIN_TILT_LS_DUMP     = 35;
const uint8_t PIN_TILT_LS_HOME     = 36;

// Outputs
const uint8_t PIN_LIFT_FWD         = 22;
const uint8_t PIN_LIFT_REV         = 23;
const uint8_t PIN_LIFT_EN          = 24;

const uint8_t PIN_CUP_OPEN         = 25;
const uint8_t PIN_CUP_CLOSE        = 26;
const uint8_t PIN_CUP_EN           = 27;

const uint8_t PIN_TILT_DUMP        = 28;
const uint8_t PIN_TILT_HOME        = 29;
const uint8_t PIN_TILT_EN          = 30;

const uint8_t PIN_ALARM            = 41;

// Nextion on Mega hardware serial
// Mega RX1 pin 19 <- Nextion TX
// Mega TX1 pin 18 -> Nextion RX
#define nxt Serial1

// RTC on Mega I2C:
// SDA = pin 20
// SCL = pin 21

// =====================
// Nextion Colors
// =====================
const uint16_t NX_GRAY   = 50712;
const uint16_t NX_GREEN  = 2016;
const uint16_t NX_RED    = 63488;

// =====================
// Encoder / Revs
// =====================
volatile long encPulses = 0;
const float PULSES_PER_REV = 20.0f;

// =====================
// State Machine
// =====================
enum State {
  ST_IDLE,
  ST_EXTENDING_LIFT,
  ST_OPENING_CUP_AT_DEPTH,
  ST_FILL_DWELL,
  ST_CLOSING_CUP_AT_DEPTH,
  ST_RETRACTING_LIFT,
  ST_TILTING_TO_DUMP,
  ST_OPENING_CUP_TO_DUMP,
  ST_DUMP_DWELL,
  ST_CLOSING_CUP_AFTER_DUMP,
  ST_UNTILTING_HOME,
  ST_COMPLETE,
  ST_FAULT
};

State state = ST_IDLE;
unsigned long stateStartMs = 0;
unsigned long cycleStartMs = 0;

// =====================
// Timing
// =====================
const unsigned long LIFT_EXTEND_TIMEOUT_MS     = 8000;
const unsigned long CUP_OPEN_TIMEOUT_MS        = 4000;
const unsigned long FILL_DWELL_MS              = 2000;
const unsigned long CUP_CLOSE_TIMEOUT_MS       = 4000;
const unsigned long LIFT_RETRACT_TIMEOUT_MS    = 8000;
const unsigned long TILT_DUMP_TIMEOUT_MS       = 4000;
const unsigned long DUMP_DWELL_MS              = 1500;
const unsigned long TILT_HOME_TIMEOUT_MS       = 4000;

// =====================
// Log Buffer
// =====================
String faultMsg = "None";
String bigLog = "";
const int MAX_LOG_CHARS = 900;
bool logDirty = true;

enum Page { PAGE_UNKNOWN, PAGE_MAIN, PAGE_LOG, PAGE_DIAG };
Page currentPage = PAGE_MAIN;

// =====================
// Basic Input Helpers
// =====================
bool readNO_Pressed(uint8_t pin) {
  return (digitalRead(pin) == LOW);
}

bool readNC_ActiveHigh(uint8_t pin) {
  return (digitalRead(pin) == HIGH);
}

bool startPressed()         { return readNO_Pressed(PIN_START); }
bool stopPressed()          { return readNO_Pressed(PIN_STOP); }
bool resetPressed()         { return readNO_Pressed(PIN_RESET); }

bool estopActive()          { return readNC_ActiveHigh(PIN_ESTOP); }
bool liftExtActive()        { return readNC_ActiveHigh(PIN_LIFT_LS_EXT); }
bool liftRetActive()        { return readNC_ActiveHigh(PIN_LIFT_LS_RET); }
bool cupOpenActive()        { return readNC_ActiveHigh(PIN_CUP_LS_OPEN); }
bool cupClosedActive()      { return readNC_ActiveHigh(PIN_CUP_LS_CLOSED); }
bool tiltDumpActive()       { return readNC_ActiveHigh(PIN_TILT_LS_DUMP); }
bool tiltHomeActive()       { return readNC_ActiveHigh(PIN_TILT_LS_HOME); }

// =====================
// Nextion Low-Level
// =====================
void nxtCmd(const String &cmd) {
  nxt.print(cmd);
  nxt.write(0xFF); nxt.write(0xFF); nxt.write(0xFF);
}

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
// Time / Logging
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

void addLog(const String &line) {
  String entry = timeHHMMSS() + " | " + line;
  if (bigLog.length() > 0) bigLog += "\r\n";
  bigLog += entry;

  if ((int)bigLog.length() > MAX_LOG_CHARS) {
    int cut = bigLog.length() - MAX_LOG_CHARS;
    int nl = bigLog.indexOf('\n', cut);
    if (nl > 0) bigLog = bigLog.substring(nl + 1);
    else bigLog = bigLog.substring(cut);
  }

  logDirty = true;
}

// =====================
// Motor Output Helpers
// =====================
void allOutputsOff() {
  digitalWrite(PIN_LIFT_FWD, LOW);
  digitalWrite(PIN_LIFT_REV, LOW);
  digitalWrite(PIN_LIFT_EN, LOW);

  digitalWrite(PIN_CUP_OPEN, LOW);
  digitalWrite(PIN_CUP_CLOSE, LOW);
  digitalWrite(PIN_CUP_EN, LOW);

  digitalWrite(PIN_TILT_DUMP, LOW);
  digitalWrite(PIN_TILT_HOME, LOW);
  digitalWrite(PIN_TILT_EN, LOW);

  digitalWrite(PIN_ALARM, LOW);
}

void setAlarm(bool on) {
  digitalWrite(PIN_ALARM, on ? HIGH : LOW);
}

void driveLiftExtend(bool on) {
  digitalWrite(PIN_LIFT_EN, on ? HIGH : LOW);
  digitalWrite(PIN_LIFT_FWD, on ? HIGH : LOW);
  if (on) digitalWrite(PIN_LIFT_REV, LOW);
}

void driveLiftRetract(bool on) {
  digitalWrite(PIN_LIFT_EN, on ? HIGH : LOW);
  digitalWrite(PIN_LIFT_REV, on ? HIGH : LOW);
  if (on) digitalWrite(PIN_LIFT_FWD, LOW);
}

void driveCupOpen(bool on) {
  digitalWrite(PIN_CUP_EN, on ? HIGH : LOW);
  digitalWrite(PIN_CUP_OPEN, on ? HIGH : LOW);
  if (on) digitalWrite(PIN_CUP_CLOSE, LOW);
}

void driveCupClose(bool on) {
  digitalWrite(PIN_CUP_EN, on ? HIGH : LOW);
  digitalWrite(PIN_CUP_CLOSE, on ? HIGH : LOW);
  if (on) digitalWrite(PIN_CUP_OPEN, LOW);
}

void driveTiltDump(bool on) {
  digitalWrite(PIN_TILT_EN, on ? HIGH : LOW);
  digitalWrite(PIN_TILT_DUMP, on ? HIGH : LOW);
  if (on) digitalWrite(PIN_TILT_HOME, LOW);
}

void driveTiltHome(bool on) {
  digitalWrite(PIN_TILT_EN, on ? HIGH : LOW);
  digitalWrite(PIN_TILT_HOME, on ? HIGH : LOW);
  if (on) digitalWrite(PIN_TILT_DUMP, LOW);
}

// =====================
// State Labels
// =====================
String stateName(State s) {
  switch (s) {
    case ST_IDLE:                    return "IDLE";
    case ST_EXTENDING_LIFT:          return "LIFT EXTEND";
    case ST_OPENING_CUP_AT_DEPTH:    return "CUP OPEN DEPTH";
    case ST_FILL_DWELL:              return "FILL DWELL";
    case ST_CLOSING_CUP_AT_DEPTH:    return "CUP CLOSE DEPTH";
    case ST_RETRACTING_LIFT:         return "LIFT RETRACT";
    case ST_TILTING_TO_DUMP:         return "TILT TO DUMP";
    case ST_OPENING_CUP_TO_DUMP:     return "CUP OPEN DUMP";
    case ST_DUMP_DWELL:              return "DUMP DWELL";
    case ST_CLOSING_CUP_AFTER_DUMP:  return "CUP CLOSE AFTER";
    case ST_UNTILTING_HOME:          return "TILT HOME";
    case ST_COMPLETE:                return "COMPLETE";
    case ST_FAULT:                   return "FAULT";
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

// =====================
// State Transitions / Faults
// =====================
void enterState(State s) {
  state = s;
  stateStartMs = millis();
  addLog("STATE | " + stateName(s));
}

void triggerFault(const String &msg) {
  faultMsg = msg;
  allOutputsOff();
  setAlarm(true);
  enterState(ST_FAULT);
}

// =====================
// Nextion Page Handling
// =====================
void setCurrentPageFromId(uint8_t pageId) {
  switch (pageId) {
    case 0: currentPage = PAGE_MAIN; Serial.println("NX PAGE: MAIN"); break;
    case 1: currentPage = PAGE_LOG;  logDirty = true; Serial.println("NX PAGE: LOG"); break;
    case 2: currentPage = PAGE_DIAG; Serial.println("NX PAGE: DIAG"); break;
    default: currentPage = PAGE_UNKNOWN; break;
  }
}

void handleNxLine(const String &lineRaw) {
  String line = lineRaw;
  line.trim();

  if (line == "START") {
    if (state == ST_IDLE) {
      if (estopActive()) {
        triggerFault("E-STOP ACTIVE");
      } else {
        cycleStartMs = millis();
        encPulses = 0;
        faultMsg = "None";
        setAlarm(false);
        enterState(ST_EXTENDING_LIFT);
      }
    }
  } else if (line == "STOP") {
    triggerFault("ABORTED");
  } else if (line == "RESET") {
    faultMsg = "None";
    setAlarm(false);
    allOutputsOff();
    enterState(ST_IDLE);
  } else if (line == "CLEARLOG") {
    bigLog = "";
    addLog("INFO  | Log cleared");
    logDirty = true;
  }
}

void pollNextion() {
  static bool awaitingPageId = false;
  static bool awaitingPageTerminator = false;
  static uint8_t pageId = 0;
  static uint8_t ffCount = 0;
  static String rxLine = "";

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
      if (rxLine.length() > 80) rxLine.remove(0, 40);
    }
  }
}

// =====================
// HMI Update Helpers (do not touch)
// =====================
String activeLiftText() {
  if (digitalRead(PIN_LIFT_FWD) == HIGH) return "LIFT: EXTEND";
  if (digitalRead(PIN_LIFT_REV) == HIGH) return "LIFT: RETRACT";
  return "LIFT: OFF";
}

String activeCupText() {
  if (digitalRead(PIN_CUP_OPEN) == HIGH) return "CUP: OPEN";
  if (digitalRead(PIN_CUP_CLOSE) == HIGH) return "CUP: CLOSE";
  return "CUP: OFF";
}

String activeTiltText() {
  if (digitalRead(PIN_TILT_DUMP) == HIGH) return "TILT: DUMP";
  if (digitalRead(PIN_TILT_HOME) == HIGH) return "TILT: HOME";
  return "TILT: OFF";
}

void updateMainPage() {
  nxSetTxt("tClock", "TIME " + timeHHMMSS());
  nxSetTxt("tState", "STATE: " + stateName(state));

  unsigned long elapsed = (cycleStartMs == 0 || state == ST_IDLE) ? 0 : (millis() - cycleStartMs);
  nxSetTxt("tCycle", "Cycle Elapsed: " + mmss(elapsed));

  bool ready = (state != ST_FAULT) && !estopActive();
  nxSetTxt("tPress", String("System: ") + (ready ? "READY" : "FAULT"));
  nxSetBco("tPress", ready ? NX_GREEN : NX_RED);

  nxSetTxt("tVext", activeLiftText());
  nxSetTxt("tVret", activeCupText());
  nxSetTxt("tMotor", activeTiltText());

  if (state == ST_FAULT) {
    nxSetTxt("tFault", "FAULT / MESSAGE: " + faultMsg);
    nxSetBco("tFault", NX_RED);
  } else {
    nxSetTxt("tFault", "FAULT / MESSAGE: None");
    nxSetBco("tFault", NX_GRAY);
  }
}

void updateLogPage() {
  nxSetTxt("tClock1", "TIME " + timeHHMMSS());
  if (logDirty) {
    nxSetTxt("tLog", bigLog);
    logDirty = false;
  }
}

void updateDiagPage() {
  nxSetTxt("tClock2", "TIME " + timeHHMMSS());

  nxSetTxt("tInPress", String("E-STOP: ") + (estopActive() ? "ON" : "OFF"));
  nxSetBco("tInPress", estopActive() ? NX_RED : NX_GREEN);

  nxSetTxt("tInLsExt", String("LIFT X/R: ") + (liftExtActive() ? "EXT ON" : "EXT OFF"));
  nxSetTxt("tInLsRet", String("LIFT H/R: ") + (liftRetActive() ? "HOME ON" : "HOME OFF"));
  nxSetTxt("tInEstop", String("CUP O/C: ") + (cupOpenActive() ? "OPEN ON" : (cupClosedActive() ? "CLOSE ON" : "OFF")));

  nxSetTxt("tOutExt", activeLiftText());
  nxSetTxt("tOutRet", activeCupText());
  nxSetTxt("tOutMotor", activeTiltText());
  nxSetTxt("tOutAlarm", String("ALARM: ") + ((digitalRead(PIN_ALARM) == HIGH) ? "ON" : "OFF"));

  if (state == ST_FAULT) {
    nxSetTxt("tFault2", "FAULT: " + faultMsg);
    nxSetBco("tFault2", NX_RED);
  } else {
    nxSetTxt("tFault2", String("TILT D/H: ") + (tiltDumpActive() ? "DUMP ON" : (tiltHomeActive() ? "HOME ON" : "OFF")));
    nxSetBco("tFault2", NX_GRAY);
  }
}

// =====================
// Main Control Logic
// =====================
void loopStateMachine() {
  if (estopActive() && state != ST_IDLE && state != ST_FAULT) {
    triggerFault("E-STOP ACTIVE");
    return;
  }

  switch (state) {
    case ST_IDLE:
      allOutputsOff();
      if (startPressed()) handleNxLine("START");
      break;

    case ST_EXTENDING_LIFT:
      driveLiftExtend(true);
      if (liftExtActive()) {
        driveLiftExtend(false);
        enterState(ST_OPENING_CUP_AT_DEPTH);
      } else if (millis() - stateStartMs > LIFT_EXTEND_TIMEOUT_MS) {
        triggerFault("LIFT EXT TIMEOUT");
      }
      break;

    case ST_OPENING_CUP_AT_DEPTH:
      driveCupOpen(true);
      if (cupOpenActive()) {
        driveCupOpen(false);
        enterState(ST_FILL_DWELL);
      } else if (millis() - stateStartMs > CUP_OPEN_TIMEOUT_MS) {
        triggerFault("CUP OPEN TIMEOUT");
      }
      break;

    case ST_FILL_DWELL:
      allOutputsOff();
      if (millis() - stateStartMs > FILL_DWELL_MS) {
        enterState(ST_CLOSING_CUP_AT_DEPTH);
      }
      break;

    case ST_CLOSING_CUP_AT_DEPTH:
      driveCupClose(true);
      if (cupClosedActive()) {
        driveCupClose(false);
        enterState(ST_RETRACTING_LIFT);
      } else if (millis() - stateStartMs > CUP_CLOSE_TIMEOUT_MS) {
        triggerFault("CUP CLOSE TIMEOUT");
      }
      break;

    case ST_RETRACTING_LIFT:
      driveLiftRetract(true);
      if (liftRetActive()) {
        driveLiftRetract(false);
        enterState(ST_TILTING_TO_DUMP);
      } else if (millis() - stateStartMs > LIFT_RETRACT_TIMEOUT_MS) {
        triggerFault("LIFT RET TIMEOUT");
      }
      break;

    case ST_TILTING_TO_DUMP:
      driveTiltDump(true);
      if (tiltDumpActive()) {
        driveTiltDump(false);
        enterState(ST_OPENING_CUP_TO_DUMP);
      } else if (millis() - stateStartMs > TILT_DUMP_TIMEOUT_MS) {
        triggerFault("TILT DUMP TIMEOUT");
      }
      break;

    case ST_OPENING_CUP_TO_DUMP:
      driveCupOpen(true);
      if (cupOpenActive()) {
        driveCupOpen(false);
        enterState(ST_DUMP_DWELL);
      } else if (millis() - stateStartMs > CUP_OPEN_TIMEOUT_MS) {
        triggerFault("CUP REOPEN TIMEOUT");
      }
      break;

    case ST_DUMP_DWELL:
      allOutputsOff();
      if (millis() - stateStartMs > DUMP_DWELL_MS) {
        enterState(ST_CLOSING_CUP_AFTER_DUMP);
      }
      break;

    case ST_CLOSING_CUP_AFTER_DUMP:
      driveCupClose(true);
      if (cupClosedActive()) {
        driveCupClose(false);
        enterState(ST_UNTILTING_HOME);
      } else if (millis() - stateStartMs > CUP_CLOSE_TIMEOUT_MS) {
        triggerFault("CUP FINAL CLOSE TO");
      }
      break;

    case ST_UNTILTING_HOME:
      driveTiltHome(true);
      if (tiltHomeActive()) {
        driveTiltHome(false);
        enterState(ST_COMPLETE);
      } else if (millis() - stateStartMs > TILT_HOME_TIMEOUT_MS) {
        triggerFault("TILT HOME TIMEOUT");
      }
      break;

    case ST_COMPLETE:
      allOutputsOff();
      addLog("INFO  | Cycle complete");
      enterState(ST_IDLE);
      break;

    case ST_FAULT:
      allOutputsOff();
      setAlarm(true);
      if (resetPressed()) handleNxLine("RESET");
      break;
  }
}

// =====================
// Encoder ISR
// =====================
void isrEncA() {
  encPulses++;
}

// =====================
// Setup / Loop
// =====================
unsigned long lastHmiUpdateMs = 0;
const unsigned long HMI_UPDATE_MS = 1000;
unsigned long lastRtcDebugMs = 0;
unsigned long lastPageQueryMs = 0;
const unsigned long PAGE_QUERY_MS = 500;

void setup() {
  pinMode(PIN_ENC_A, INPUT_PULLUP);
  pinMode(PIN_START, INPUT_PULLUP);
  pinMode(PIN_STOP, INPUT_PULLUP);
  pinMode(PIN_RESET, INPUT_PULLUP);
  pinMode(PIN_ESTOP, INPUT_PULLUP);

  pinMode(PIN_LIFT_LS_EXT, INPUT_PULLUP);
  pinMode(PIN_LIFT_LS_RET, INPUT_PULLUP);
  pinMode(PIN_CUP_LS_OPEN, INPUT_PULLUP);
  pinMode(PIN_CUP_LS_CLOSED, INPUT_PULLUP);
  pinMode(PIN_TILT_LS_DUMP, INPUT_PULLUP);
  pinMode(PIN_TILT_LS_HOME, INPUT_PULLUP);

  pinMode(PIN_LIFT_FWD, OUTPUT);
  pinMode(PIN_LIFT_REV, OUTPUT);
  pinMode(PIN_LIFT_EN, OUTPUT);
  pinMode(PIN_CUP_OPEN, OUTPUT);
  pinMode(PIN_CUP_CLOSE, OUTPUT);
  pinMode(PIN_CUP_EN, OUTPUT);
  pinMode(PIN_TILT_DUMP, OUTPUT);
  pinMode(PIN_TILT_HOME, OUTPUT);
  pinMode(PIN_TILT_EN, OUTPUT);
  pinMode(PIN_ALARM, OUTPUT);

  allOutputsOff();
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), isrEncA, RISING);

  Serial.begin(115200);
  nxt.begin(9600);
  nxtCmd("bkcmd=0");
  nxtCmd("sendxy=0");
  nxtCmd("sendme");

  Wire.begin();

#if USE_RTC
  if (!rtc.begin()) {
    Serial.println("RTC not found");
  } else {
    Serial.println("RTC found");
  }
#endif

  addLog("INFO  | Boot (Mega 3-motor scaffold)");
  enterState(ST_IDLE);
}

void loop() {
  pollNextion();

  if (stopPressed() && state != ST_IDLE) {
    handleNxLine("STOP");
  }

  loopStateMachine();

  if (millis() - lastRtcDebugMs >= 1000) {
    lastRtcDebugMs = millis();
    Serial.println(timeHHMMSS());
  }

  if (millis() - lastPageQueryMs >= PAGE_QUERY_MS) {
    lastPageQueryMs = millis();
    nxtCmd("sendme");
  }

  if (millis() - lastHmiUpdateMs >= HMI_UPDATE_MS) {
    lastHmiUpdateMs = millis();

    switch (currentPage) {
      case PAGE_MAIN: updateMainPage(); break;
      case PAGE_LOG:  updateLogPage();  break;
      case PAGE_DIAG: updateDiagPage(); break;
      case PAGE_UNKNOWN:
      default:
        updateMainPage();
        break;
    }
  }
}
