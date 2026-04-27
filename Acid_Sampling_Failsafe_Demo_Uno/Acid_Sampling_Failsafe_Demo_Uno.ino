/*
  Lean Failsafe Demo - Uno + 3 DRV8825 + Nextion + RTC
  ----------------------------------------------------
  Current setup:
  - 1 Arduino Uno
  - 3 DRV8825 drivers
  - 3 NEMA stepper motors
  - 1 Nextion HMI
  - 1 I2C RTC module on A4/A5
  - optional lift home / extend limit switches

  Full cycle on START:
  1. Motor 1 extend down
  2. Motor 2 open lid
  3. Motor 2 close lid
  4. Motor 1 retract up
  5. Motor 2 open lid
  6. Motor 3 tilt out
  7. Motor 3 tilt back to default
  8. Motor 2 close lid

  Commands from Serial or Nextion:
  START, STOP, RESET
  M1F, M1R, M2F, M2R, M3F, M3R
*/

#include <SoftwareSerial.h>
#include <Wire.h>
#include <ctype.h>
#include <stdio.h>
#include <string.h>

#define USE_NEXTION 1
#define USE_LIFT_LIMIT_SWITCHES 0

const uint8_t PIN_M1_STEP      = 2;
const uint8_t PIN_M1_DIR       = 3;
const uint8_t PIN_M2_STEP      = 4;
const uint8_t PIN_M2_DIR       = 5;
const uint8_t PIN_M3_STEP      = 6;
const uint8_t PIN_M3_DIR       = 7;
const uint8_t PIN_DRV_ENABLE   = 8;
const uint8_t PIN_NX_RX        = 10;
const uint8_t PIN_NX_TX        = 11;
const uint8_t PIN_LIFT_HOME_SW = 12;
const uint8_t PIN_LIFT_EXT_SW  = 13;
const uint8_t PIN_ESTOP_NC     = A0;
const uint8_t PIN_ALARM        = A1;
const uint8_t RTC_I2C_ADDR     = 0x68;

#if USE_NEXTION
SoftwareSerial nxt(PIN_NX_RX, PIN_NX_TX);
#endif

const uint8_t DRIVER_EN_ON  = LOW;
const uint8_t DRIVER_EN_OFF = HIGH;
const uint8_t DIR_FWD       = HIGH;
const uint8_t DIR_REV       = LOW;

// Flip these three flags during bench tuning instead of rewriting the sequence.
const bool M1_FORWARD_IS_DOWN     = true;
const bool M2_FORWARD_IS_OPEN     = true;
const bool M3_FORWARD_IS_TILT_OUT = true;

const unsigned long STEP_INTERVAL_US = 1200UL;
const unsigned long STEP_PULSE_US    = 10UL;
const unsigned long STEP_01_M1_DOWN_MS        = 3000UL;
const unsigned long STEP_02_M2_OPEN_MS        = 1500UL;
const unsigned long STEP_03_M2_CLOSE_MS       = 1500UL;
const unsigned long STEP_04_M1_UP_MS          = 3000UL;
const unsigned long STEP_05_M2_OPEN_AFTER_MS  = 1500UL;
const unsigned long STEP_06_M3_TILT_OUT_MS    = 1800UL;
const unsigned long STEP_07_M3_TILT_HOME_MS   = 1800UL;
const unsigned long STEP_08_M2_FINAL_CLOSE_MS = 1500UL;
const unsigned long SETTLE_MS        = 300UL;
const unsigned long AUTO_TIMEOUT_MS  = 12000UL;
const unsigned long HMI_UPDATE_MS    = 250UL;
const unsigned long PAGE_QUERY_MS    = 800UL;
const unsigned long RTC_UPDATE_MS    = 1000UL;

enum Axis { AX_NONE = 0, AX_LIFT, AX_CUP, AX_TILT };
enum Dir  { DIR_NONE = 0, D_FWD, D_REV };
enum State {
  ST_IDLE = 0,
  ST_AUTO_RUN,
  ST_AUTO_WAIT,
  ST_MANUAL,
  ST_FAULT
};

struct AutoStep {
  Axis axis;
  Dir dir;
  unsigned long runMs;
  const char *label;
};

const AutoStep FULL_CYCLE[] = {
  { AX_LIFT, D_FWD, STEP_01_M1_DOWN_MS,        "STEP 1 M1 EXTEND DOWN" },
  { AX_CUP,  D_FWD, STEP_02_M2_OPEN_MS,        "STEP 2 M2 OPEN LID" },
  { AX_CUP,  D_REV, STEP_03_M2_CLOSE_MS,       "STEP 3 M2 CLOSE LID" },
  { AX_LIFT, D_REV, STEP_04_M1_UP_MS,          "STEP 4 M1 RETRACT UP" },
  { AX_CUP,  D_FWD, STEP_05_M2_OPEN_AFTER_MS,  "STEP 5 M2 OPEN AFTER LIFT" },
  { AX_TILT, D_FWD, STEP_06_M3_TILT_OUT_MS,    "STEP 6 M3 TILT OUT" },
  { AX_TILT, D_REV, STEP_07_M3_TILT_HOME_MS,   "STEP 7 M3 TILT HOME" },
  { AX_CUP,  D_REV, STEP_08_M2_FINAL_CLOSE_MS, "STEP 8 M2 FINAL CLOSE" }
};

const uint8_t FULL_CYCLE_COUNT = sizeof(FULL_CYCLE) / sizeof(FULL_CYCLE[0]);

State state = ST_IDLE;
Axis activeAxis = AX_NONE;
Dir activeDir = DIR_NONE;
bool driversEnabled = false;
bool stepHigh = false;
bool faultLatched = false;
bool currentPageIsLog = false;
bool logDirty = false;
bool rtcOnline = false;
char faultMsg[24] = "NONE";
char currentTimeText[9] = "--:--:--";
char currentClockText[15] = "TIME: --:--:--";
char logText[420] = "BOOT: READY";
uint8_t autoStepIndex = 0;

unsigned long stateStartMs = 0;
unsigned long lastHmiMs = 0;
unsigned long lastPageQueryMs = 0;
unsigned long lastStepUs = 0;
unsigned long lastRtcUpdateMs = 0;

char serialBuf[32];
uint8_t serialLen = 0;

void enterState(State s);
void handleState();
void beginAutoCycle();
void startAutoStep(uint8_t index);
void startMotion(Axis axis, Dir dir);
void stopMotion();
void serviceStepper();
void setDriversEnabled(bool on);
bool estopActive();
bool liftHomeHit();
bool liftExtendHit();
bool currentStepComplete(unsigned long elapsed);
void faultNow(const char *msg);
void clearFault();
void handleCommand(char *cmd);
void readNextion();
void nxCmd(const char *cmd);
void nxSetTxt(const char *obj, const char *txt);
void nxSetPageTxt(const char *page, const char *obj, const char *txt);
void updateHmi(bool force);
void updateRtc(bool force);
bool readRtcTime(char *timeText, size_t timeTextLen);
uint8_t bcdToDec(uint8_t value);
void appendLog(const char *msg);
uint8_t dirLevelFor(Axis axis, Dir dir);
const char* stateName(State s);
const char* axisDirName(Axis axis, Dir dir);
uint8_t stepPinFor(Axis axis);
uint8_t dirPinFor(Axis axis);

void setup() {
  Serial.begin(115200);
  Wire.begin();

  pinMode(PIN_M1_STEP, OUTPUT);
  pinMode(PIN_M1_DIR, OUTPUT);
  pinMode(PIN_M2_STEP, OUTPUT);
  pinMode(PIN_M2_DIR, OUTPUT);
  pinMode(PIN_M3_STEP, OUTPUT);
  pinMode(PIN_M3_DIR, OUTPUT);
  pinMode(PIN_DRV_ENABLE, OUTPUT);
  pinMode(PIN_ALARM, OUTPUT);
  pinMode(PIN_ESTOP_NC, INPUT_PULLUP);
  pinMode(PIN_LIFT_HOME_SW, INPUT_PULLUP);
  pinMode(PIN_LIFT_EXT_SW, INPUT_PULLUP);

  digitalWrite(PIN_M1_STEP, LOW);
  digitalWrite(PIN_M2_STEP, LOW);
  digitalWrite(PIN_M3_STEP, LOW);
  digitalWrite(PIN_ALARM, LOW);
  setDriversEnabled(false);

#if USE_NEXTION
  nxt.begin(9600);
  nxCmd("bkcmd=0");
  nxCmd("sendxy=0");
  nxCmd("sendme");
#endif

  updateRtc(true);
  enterState(ST_IDLE);
  appendLog(rtcOnline ? "RTC ONLINE" : "RTC NOT DETECTED");
  appendLog("FAILSAFE READY");
  Serial.println(F("LEAN FAILSAFE READY"));
}

void loop() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (serialLen) {
        serialBuf[serialLen] = '\0';
        handleCommand(serialBuf);
        serialLen = 0;
      }
    } else if (serialLen < sizeof(serialBuf) - 1 && c >= 32 && c <= 126) {
      serialBuf[serialLen++] = c;
    }
  }

  if (estopActive() && state != ST_FAULT) {
    faultNow("E-STOP");
  }

#if USE_NEXTION
  readNextion();
#endif

  updateRtc(false);
  handleState();
  serviceStepper();

  if (millis() - lastHmiMs >= HMI_UPDATE_MS) {
    lastHmiMs = millis();
    updateHmi(false);
  }

#if USE_NEXTION
  if (millis() - lastPageQueryMs >= PAGE_QUERY_MS) {
    lastPageQueryMs = millis();
    nxCmd("sendme");
  }
#endif
}

void enterState(State s) {
  state = s;
  stateStartMs = millis();
  updateHmi(true);
}

void handleState() {
  unsigned long elapsed = millis() - stateStartMs;

  switch (state) {
    case ST_IDLE:
      stopMotion();
      break;

    case ST_AUTO_RUN:
      if (currentStepComplete(elapsed)) {
        stopMotion();
        appendLog(FULL_CYCLE[autoStepIndex].label);
        enterState(ST_AUTO_WAIT);
        break;
      }
      if (elapsed > AUTO_TIMEOUT_MS) faultNow("AUTO TO");
      break;

    case ST_AUTO_WAIT:
      stopMotion();
      if (elapsed >= SETTLE_MS) {
        autoStepIndex++;
        if (autoStepIndex >= FULL_CYCLE_COUNT) {
          appendLog("FULL CYCLE COMPLETE");
          enterState(ST_IDLE);
        } else {
          startAutoStep(autoStepIndex);
        }
      }
      break;

    case ST_MANUAL:
      if (activeAxis == AX_NONE || activeDir == DIR_NONE) enterState(ST_IDLE);
      break;

    case ST_FAULT:
      stopMotion();
      break;
  }
}

void beginAutoCycle() {
  autoStepIndex = 0;
  appendLog("START FULL CYCLE");
  startAutoStep(autoStepIndex);
}

void startAutoStep(uint8_t index) {
  appendLog(FULL_CYCLE[index].label);
  startMotion(FULL_CYCLE[index].axis, FULL_CYCLE[index].dir);
  enterState(ST_AUTO_RUN);
}

void startMotion(Axis axis, Dir dir) {
  if (axis == AX_NONE || dir == DIR_NONE) return;

  if (activeAxis != axis || activeDir != dir) {
    activeAxis = axis;
    activeDir = dir;
    digitalWrite(dirPinFor(axis), dirLevelFor(axis, dir));
    digitalWrite(stepPinFor(axis), LOW);
    stepHigh = false;
    lastStepUs = micros();
  }
  if (!driversEnabled) setDriversEnabled(true);
}

void stopMotion() {
  digitalWrite(PIN_M1_STEP, LOW);
  digitalWrite(PIN_M2_STEP, LOW);
  digitalWrite(PIN_M3_STEP, LOW);
  activeAxis = AX_NONE;
  activeDir = DIR_NONE;
  stepHigh = false;
  setDriversEnabled(false);
}

void serviceStepper() {
  if (activeAxis == AX_NONE || activeDir == DIR_NONE || !driversEnabled) return;

  uint8_t stepPin = stepPinFor(activeAxis);
  unsigned long now = micros();

  if (!stepHigh) {
    if ((unsigned long)(now - lastStepUs) >= STEP_INTERVAL_US) {
      digitalWrite(stepPin, HIGH);
      stepHigh = true;
      lastStepUs = now;
    }
  } else if ((unsigned long)(now - lastStepUs) >= STEP_PULSE_US) {
    digitalWrite(stepPin, LOW);
    stepHigh = false;
    lastStepUs = now;
  }
}

void setDriversEnabled(bool on) {
  driversEnabled = on;
  digitalWrite(PIN_DRV_ENABLE, on ? DRIVER_EN_ON : DRIVER_EN_OFF);
}

bool estopActive() {
  return digitalRead(PIN_ESTOP_NC) == HIGH;
}

bool liftHomeHit() {
  return digitalRead(PIN_LIFT_HOME_SW) == HIGH;
}

bool liftExtendHit() {
  return digitalRead(PIN_LIFT_EXT_SW) == HIGH;
}

bool currentStepComplete(unsigned long elapsed) {
  const AutoStep &step = FULL_CYCLE[autoStepIndex];

#if USE_LIFT_LIMIT_SWITCHES
  if (step.axis == AX_LIFT && step.dir == D_FWD && liftExtendHit()) return true;
  if (step.axis == AX_LIFT && step.dir == D_REV && liftHomeHit()) return true;
#endif

  return elapsed >= step.runMs;
}

void faultNow(const char *msg) {
  faultLatched = true;
  strncpy(faultMsg, msg, sizeof(faultMsg) - 1);
  faultMsg[sizeof(faultMsg) - 1] = '\0';
  digitalWrite(PIN_ALARM, HIGH);
  appendLog(msg);
  enterState(ST_FAULT);
}

void clearFault() {
  faultLatched = false;
  strcpy(faultMsg, "NONE");
  digitalWrite(PIN_ALARM, LOW);
  appendLog("FAULT CLEARED");
  enterState(ST_IDLE);
}

void handleCommand(char *cmd) {
  for (char *p = cmd; *p; ++p) *p = (char)toupper((unsigned char)*p);
  Serial.print(F("CMD: "));
  Serial.println(cmd);

  if (!strcmp(cmd, "START")) {
    if (!faultLatched) beginAutoCycle();
  } else if (!strcmp(cmd, "STOP")) {
    faultNow("STOP");
  } else if (!strcmp(cmd, "RESET")) {
    clearFault();
  } else if (!strcmp(cmd, "M1F")) {
    if (!faultLatched) { appendLog("MANUAL M1 EXTEND"); startMotion(AX_LIFT, D_FWD); enterState(ST_MANUAL); }
  } else if (!strcmp(cmd, "M1R")) {
    if (!faultLatched) { appendLog("MANUAL M1 RETRACT"); startMotion(AX_LIFT, D_REV); enterState(ST_MANUAL); }
  } else if (!strcmp(cmd, "M2F")) {
    if (!faultLatched) { appendLog("MANUAL M2 OPEN LID"); startMotion(AX_CUP, D_FWD); enterState(ST_MANUAL); }
  } else if (!strcmp(cmd, "M2R")) {
    if (!faultLatched) { appendLog("MANUAL M2 CLOSE LID"); startMotion(AX_CUP, D_REV); enterState(ST_MANUAL); }
  } else if (!strcmp(cmd, "M3F")) {
    if (!faultLatched) { appendLog("MANUAL M3 TILT OUT"); startMotion(AX_TILT, D_FWD); enterState(ST_MANUAL); }
  } else if (!strcmp(cmd, "M3R")) {
    if (!faultLatched) { appendLog("MANUAL M3 RETURN DEFAULT"); startMotion(AX_TILT, D_REV); enterState(ST_MANUAL); }
  } else if (!strcmp(cmd, "PAGE:MAIN")) {
    currentPageIsLog = false;
  } else if (!strcmp(cmd, "PAGE:LOG") || !strcmp(cmd, "PAGE:LOGS")) {
    currentPageIsLog = true;
    logDirty = true;
  } else if (!strcmp(cmd, "PAGE:DIAG")) {
    currentPageIsLog = false;
  } else if (!strcmp(cmd, "CLEARLOG")) {
    logText[0] = '\0';
    appendLog("LOG CLEARED");
  }

  updateHmi(true);
}

void readNextion() {
#if USE_NEXTION
  static char line[32];
  static uint8_t len = 0;
  static bool awaitPageId = false;
  static bool awaitFF = false;
  static uint8_t ffCount = 0;

  while (nxt.available()) {
    uint8_t b = (uint8_t)nxt.read();

    if (awaitPageId) {
      currentPageIsLog = (b == 1);
      logDirty = currentPageIsLog;
      updateRtc(true);
      updateHmi(true);
      awaitPageId = false;
      awaitFF = true;
      ffCount = 0;
      continue;
    }
    if (awaitFF) {
      if (b == 0xFF) {
        ffCount++;
        if (ffCount >= 3) {
          awaitFF = false;
        }
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
#endif
}

void nxCmd(const char *cmd) {
#if USE_NEXTION
  nxt.print(cmd);
  nxt.write(0xFF);
  nxt.write(0xFF);
  nxt.write(0xFF);
#endif
}

void nxSetTxt(const char *obj, const char *txt) {
#if USE_NEXTION
  nxt.print(obj);
  nxt.print(".txt=\"");
  nxt.print(txt);
  nxt.print("\"");
  nxt.write(0xFF);
  nxt.write(0xFF);
  nxt.write(0xFF);
#endif
}

void nxSetPageTxt(const char *page, const char *obj, const char *txt) {
#if USE_NEXTION
  nxt.print(page);
  nxt.print(".");
  nxt.print(obj);
  nxt.print(".txt=\"");
  nxt.print(txt);
  nxt.print("\"");
  nxt.write(0xFF);
  nxt.write(0xFF);
  nxt.write(0xFF);
#endif
}

void updateHmi(bool force) {
#if USE_NEXTION
  static char lastState[24] = "";
  static char lastFault[24] = "";
  static char lastPress[24] = "";

  char curState[24];
  char curFault[24];
  char curPress[24];

  updateRtc(false);

  strncpy(curState, stateName(state), sizeof(curState) - 1);
  curState[sizeof(curState) - 1] = '\0';

  if (faultLatched) {
    strncpy(curFault, faultMsg, sizeof(curFault) - 1);
    curFault[sizeof(curFault) - 1] = '\0';
  } else if (USE_LIFT_LIMIT_SWITCHES) {
    snprintf(curFault, sizeof(curFault), "H:%d E:%d", liftHomeHit() ? 1 : 0, liftExtendHit() ? 1 : 0);
  } else {
    strcpy(curFault, "NO LIMITS");
  }

  if (state == ST_AUTO_RUN || state == ST_AUTO_WAIT) {
    strncpy(curPress, FULL_CYCLE[autoStepIndex].label, sizeof(curPress) - 1);
    curPress[sizeof(curPress) - 1] = '\0';
  } else {
    strncpy(curPress, axisDirName(activeAxis, activeDir), sizeof(curPress) - 1);
    curPress[sizeof(curPress) - 1] = '\0';
  }

  nxSetPageTxt("page0", "tClock", currentClockText);
  nxSetPageTxt("page1", "tClock1", currentClockText);
  nxSetPageTxt("page2", "tClock2", currentClockText);

  if (force || strcmp(curState, lastState)) {
    nxSetPageTxt("page0", "tState", curState);
    strcpy(lastState, curState);
  }
  if (force || strcmp(curFault, lastFault)) {
    nxSetPageTxt("page0", "tFault", curFault);
    nxSetPageTxt("page2", "tFault2", curFault);
    strcpy(lastFault, curFault);
  }
  if (force || strcmp(curPress, lastPress)) {
    nxSetPageTxt("page0", "tPress", curPress);
    strcpy(lastPress, curPress);
  }
  if (force || logDirty || currentPageIsLog) {
    nxSetPageTxt("page1", "tFrame", logText[0] ? logText : "NO EVENTS YET");
    logDirty = false;
  }
#endif
}

void updateRtc(bool force) {
  static char lastClockText[15] = "";

  if (!force && millis() - lastRtcUpdateMs < RTC_UPDATE_MS) return;
  lastRtcUpdateMs = millis();

  if (readRtcTime(currentTimeText, sizeof(currentTimeText))) {
    rtcOnline = true;
    snprintf(currentClockText, sizeof(currentClockText), "TIME: %s", currentTimeText);
  } else {
    rtcOnline = false;
    strcpy(currentTimeText, "--:--:--");
    strcpy(currentClockText, "TIME: RTC ERR");
  }

#if USE_NEXTION
  if (force || strcmp(lastClockText, currentClockText)) {
    nxSetPageTxt("page0", "tClock", currentClockText);
    nxSetPageTxt("page1", "tClock1", currentClockText);
    nxSetPageTxt("page2", "tClock2", currentClockText);
    strncpy(lastClockText, currentClockText, sizeof(lastClockText) - 1);
    lastClockText[sizeof(lastClockText) - 1] = '\0';
  }
#endif
}

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

uint8_t bcdToDec(uint8_t value) {
  return (uint8_t)(((value >> 4) * 10U) + (value & 0x0F));
}

uint8_t dirLevelFor(Axis axis, Dir dir) {
  bool forwardUsesFwdLevel = true;

  if (axis == AX_LIFT) forwardUsesFwdLevel = M1_FORWARD_IS_DOWN;
  else if (axis == AX_CUP) forwardUsesFwdLevel = M2_FORWARD_IS_OPEN;
  else if (axis == AX_TILT) forwardUsesFwdLevel = M3_FORWARD_IS_TILT_OUT;

  if (dir == D_FWD) return forwardUsesFwdLevel ? DIR_FWD : DIR_REV;
  return forwardUsesFwdLevel ? DIR_REV : DIR_FWD;
}

void appendLog(const char *msg) {
  char newLog[420];
  char lineWithTime[72];

  strncpy(newLog, logText, sizeof(newLog) - 1);
  newLog[sizeof(newLog) - 1] = '\0';
  snprintf(lineWithTime, sizeof(lineWithTime), "%s | %s", currentTimeText, msg);

  size_t used = strlen(newLog);
  size_t msgLen = strlen(lineWithTime);
  size_t needed = used + (used ? 2 : 0) + msgLen + 1;

  if (needed >= sizeof(newLog)) {
    size_t trim = needed - sizeof(newLog) + 1;
    if (trim >= used) {
      newLog[0] = '\0';
      used = 0;
    } else {
      memmove(newLog, newLog + trim, used - trim + 1);
      used = strlen(newLog);
    }
  }

  if (used) {
    strncat(newLog, "\r\n", sizeof(newLog) - strlen(newLog) - 1);
  }
  strncat(newLog, lineWithTime, sizeof(newLog) - strlen(newLog) - 1);

  strncpy(logText, newLog, sizeof(logText) - 1);
  logText[sizeof(logText) - 1] = '\0';
  logDirty = true;
}

const char* stateName(State s) {
  switch (s) {
    case ST_IDLE:      return "IDLE";
    case ST_AUTO_RUN:  return "AUTO RUN";
    case ST_AUTO_WAIT: return "AUTO WAIT";
    case ST_MANUAL:    return "MANUAL";
    case ST_FAULT:     return "FAULT";
    default:           return "?";
  }
}

const char* axisDirName(Axis axis, Dir dir) {
  if (axis == AX_NONE || dir == DIR_NONE) return "READY";
  if (axis == AX_LIFT) return dir == D_FWD ? "M1 DOWN" : "M1 UP";
  if (axis == AX_CUP)  return dir == D_FWD ? "LID OPEN" : "LID CLOSE";
  return dir == D_FWD ? "TILT OUT" : "TILT HOME";
}

uint8_t stepPinFor(Axis axis) {
  switch (axis) {
    case AX_LIFT: return PIN_M1_STEP;
    case AX_CUP:  return PIN_M2_STEP;
    case AX_TILT: return PIN_M3_STEP;
    default:      return PIN_M1_STEP;
  }
}

uint8_t dirPinFor(Axis axis) {
  switch (axis) {
    case AX_LIFT: return PIN_M1_DIR;
    case AX_CUP:  return PIN_M2_DIR;
    case AX_TILT: return PIN_M3_DIR;
    default:      return PIN_M1_DIR;
  }
}
