/*
  Nextion Motor Command Test - AccelStepper Version
  -------------------------------------------------
  Separate comparison sketch that keeps the existing Nextion flow,
  RTC clock updates, and main-page status fields, but uses the
  AccelStepper library for motion control.
*/

#include <AccelStepper.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <avr/pgmspace.h>
#include <ctype.h>
#include <stdio.h>
#include <string.h>

const uint8_t PIN_M1_PUL      = 2;
const uint8_t PIN_M1_DIR      = 3;
const uint8_t PIN_M2_STEP     = 4;
const uint8_t PIN_M2_DIR      = 5;
const uint8_t PIN_M3_STEP     = 6;
const uint8_t PIN_M3_DIR      = 7;
const uint8_t PIN_M1_ENA      = 8;
const uint8_t PIN_SHARED_ENA  = 8;
const uint8_t PIN_NX_RX       = 10;
const uint8_t PIN_NX_TX       = 11;
const uint8_t RTC_I2C_ADDR    = 0x68;

const uint8_t M1_ENA_ENABLED_LEVEL      = LOW;
const uint8_t M1_ENA_DISABLED_LEVEL     = HIGH;
const uint8_t SHARED_ENA_ENABLED_LEVEL  = LOW;
const uint8_t SHARED_ENA_DISABLED_LEVEL = HIGH;

const bool RTC_SET_TIME_ON_BOOT = false;
const bool M1_FORWARD_IS_DOWN     = true;
const bool M2_FORWARD_IS_OPEN     = true;
const bool M3_FORWARD_IS_TILT_OUT = true;

const float M1_MAX_SPEED = 400.0f;
const float M2_MAX_SPEED = 400.0f;
const float M3_MAX_SPEED = 400.0f;
const float M1_ACCEL     = 250.0f;
const float M2_ACCEL     = 300.0f;
const float M3_ACCEL     = 300.0f;

const long STEP_01_M1_DOWN_STEPS = 3500L;
const long STEP_02_M2_OPEN_STEPS = 480L;
const long STEP_03_M2_CLOSE_STEPS = 480L;
const long STEP_04_M1_UP_STEPS = 3500L;
const long STEP_05_M2_OPEN_AFTER_STEPS = 480L;
const long STEP_06_M3_TILT_OUT_STEPS = 720L;
const long STEP_07_M3_TILT_HOME_STEPS = 720L;
const long STEP_08_M2_FINAL_CLOSE_STEPS = 480L;
const long MANUAL_JOG_STEPS = 200000L;

const unsigned long SETTLE_MS     = 300UL;
const unsigned long RTC_UPDATE_MS = 1000UL;
const unsigned long UI_TICK_MS    = 500UL;

const uint16_t NX_BLACK = 0;

SoftwareSerial nxt(PIN_NX_RX, PIN_NX_TX);
AccelStepper stepperM1(AccelStepper::DRIVER, PIN_M1_PUL, PIN_M1_DIR);
AccelStepper stepperM2(AccelStepper::DRIVER, PIN_M2_STEP, PIN_M2_DIR);
AccelStepper stepperM3(AccelStepper::DRIVER, PIN_M3_STEP, PIN_M3_DIR);

enum Axis {
  AXIS_NONE = 0,
  AXIS_M1,
  AXIS_M2,
  AXIS_M3
};

enum Direction {
  MOVE_NONE = 0,
  MOVE_FWD,
  MOVE_REV
};

enum DemoState {
  ST_IDLE = 0,
  ST_AUTO_RUN,
  ST_AUTO_WAIT,
  ST_DONE
};

struct AutoStep {
  Axis axis;
  Direction dir;
  long steps;
};

const AutoStep FULL_CYCLE[] = {
  { AXIS_M1, MOVE_FWD, STEP_01_M1_DOWN_STEPS },
  { AXIS_M2, MOVE_FWD, STEP_02_M2_OPEN_STEPS },
  { AXIS_M2, MOVE_REV, STEP_03_M2_CLOSE_STEPS },
  { AXIS_M1, MOVE_REV, STEP_04_M1_UP_STEPS },
  { AXIS_M2, MOVE_FWD, STEP_05_M2_OPEN_AFTER_STEPS },
  { AXIS_M3, MOVE_FWD, STEP_06_M3_TILT_OUT_STEPS },
  { AXIS_M3, MOVE_REV, STEP_07_M3_TILT_HOME_STEPS },
  { AXIS_M2, MOVE_REV, STEP_08_M2_FINAL_CLOSE_STEPS }
};

const uint8_t FULL_CYCLE_COUNT = sizeof(FULL_CYCLE) / sizeof(FULL_CYCLE[0]);

Axis activeAxis = AXIS_NONE;
Direction activeDir = MOVE_NONE;
DemoState demoState = ST_IDLE;
bool currentPageIsLog = false;
bool rtcOnline = false;
bool sequenceActive = false;
bool continuousMode = false;
bool m1Done = false;
bool m2Done = false;
bool m3Done = false;
unsigned long demoStateStartMs = 0;
unsigned long cycleStartMs = 0;
unsigned long lastRtcUpdateMs = 0;
unsigned long lastUiTickMs = 0;
uint8_t autoStepIndex = 0;
char currentTimeText[9] = "--:--:--";
char serialLine[32];
uint8_t serialLen = 0;
char latestLogLine[36] = "NO EVENTS";

void setupPins();
void setupSteppers();
void stopAllMotion();
void startAxis(Axis axis, Direction dir);
void startAutoStep(uint8_t index);
void beginAutoCycle();
void prepareCycleStart();
void serviceMotion();
void serviceSequence();
void serviceUiTick();
void resetSequenceFlags();
void markStepComplete(uint8_t index);
AccelStepper &stepperFor(Axis axis);
uint8_t dirPinFor(Axis axis);
uint8_t enablePinFor(Axis axis);
uint8_t enabledLevelFor(Axis axis);
uint8_t disabledLevelFor(Axis axis);
bool directionInvertFor(Axis axis);
void handleCommand(char *cmd);
void readNextion();
void nxCmd(const char *cmd);
void nxSetPageTxt(const char *page, const char *obj, const char *txt);
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
void appendLogP(PGM_P msg);
void copyStepLabel(uint8_t index, bool completeText, char *out, size_t outLen);
void fillAxisStateText(char *out, size_t outLen);
void fillDemoStateText(char *out, size_t outLen);
void fillSeqActiveText(char *out, size_t outLen);
void fillDoneText(bool done, const char *prefix, char *out, size_t outLen);
void fillActiveText(bool active, const char *prefix, char *out, size_t outLen);
void fillCycleElapsedText(char *out, size_t outLen);
void fillDriverText(char *out, size_t outLen);
void setDemoState(DemoState s);

void setup() {
  Serial.begin(115200);
  nxt.begin(9600);
  Wire.begin();

  setupPins();
  setupSteppers();
  stopAllMotion();

  nxCmd("bkcmd=0");
  nxCmd("sendxy=0");
  nxCmd("sendme");

  syncRtcToBuildTime();
  updateRtc(true);

  appendLogP(PSTR("BOOT"));
  appendLogP(rtcOnline ? PSTR("RTC OK") : PSTR("RTC ERR"));
  appendLogP(PSTR("ACCEL READY"));
  updateHmi();
}

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
  serviceMotion();
  serviceSequence();
  serviceUiTick();

  if (activeAxis == AXIS_NONE) updateRtc(false);
}

void setupPins() {
  pinMode(PIN_M1_PUL, OUTPUT);
  pinMode(PIN_M1_DIR, OUTPUT);
  pinMode(PIN_M2_STEP, OUTPUT);
  pinMode(PIN_M2_DIR, OUTPUT);
  pinMode(PIN_M3_STEP, OUTPUT);
  pinMode(PIN_M3_DIR, OUTPUT);
  pinMode(PIN_M1_ENA, OUTPUT);
  pinMode(PIN_SHARED_ENA, OUTPUT);

  digitalWrite(PIN_M1_PUL, LOW);
  digitalWrite(PIN_M2_STEP, LOW);
  digitalWrite(PIN_M3_STEP, LOW);
  digitalWrite(PIN_M1_ENA, M1_ENA_DISABLED_LEVEL);
  digitalWrite(PIN_SHARED_ENA, SHARED_ENA_DISABLED_LEVEL);
}

void setupSteppers() {
  stepperM1.setEnablePin(PIN_M1_ENA);
  stepperM1.setPinsInverted(directionInvertFor(AXIS_M1), false, M1_ENA_ENABLED_LEVEL == HIGH);
  stepperM1.setMaxSpeed(M1_MAX_SPEED);
  stepperM1.setAcceleration(M1_ACCEL);
  stepperM1.disableOutputs();

  stepperM2.setEnablePin(PIN_SHARED_ENA);
  stepperM2.setPinsInverted(directionInvertFor(AXIS_M2), false, SHARED_ENA_ENABLED_LEVEL == HIGH);
  stepperM2.setMaxSpeed(M2_MAX_SPEED);
  stepperM2.setAcceleration(M2_ACCEL);
  stepperM2.disableOutputs();

  stepperM3.setEnablePin(PIN_SHARED_ENA);
  stepperM3.setPinsInverted(directionInvertFor(AXIS_M3), false, SHARED_ENA_ENABLED_LEVEL == HIGH);
  stepperM3.setMaxSpeed(M3_MAX_SPEED);
  stepperM3.setAcceleration(M3_ACCEL);
  stepperM3.disableOutputs();
}

void stopAllMotion() {
  stepperM1.stop();
  stepperM2.stop();
  stepperM3.stop();

  stepperM1.moveTo(stepperM1.currentPosition());
  stepperM2.moveTo(stepperM2.currentPosition());
  stepperM3.moveTo(stepperM3.currentPosition());

  stepperM1.disableOutputs();
  stepperM2.disableOutputs();
  stepperM3.disableOutputs();

  digitalWrite(PIN_M1_ENA, M1_ENA_DISABLED_LEVEL);
  digitalWrite(PIN_SHARED_ENA, SHARED_ENA_DISABLED_LEVEL);

  activeAxis = AXIS_NONE;
  activeDir = MOVE_NONE;
  continuousMode = false;
}

void startAxis(Axis axis, Direction dir) {
  stopAllMotion();

  activeAxis = axis;
  activeDir = dir;
  continuousMode = true;

  AccelStepper &stepper = stepperFor(axis);
  digitalWrite(enablePinFor(axis), enabledLevelFor(axis));
  stepper.enableOutputs();
  stepper.move((dir == MOVE_FWD) ? MANUAL_JOG_STEPS : -MANUAL_JOG_STEPS);

  updateHmi();
}

void startAutoStep(uint8_t index) {
  stopAllMotion();

  char msgBuf[20];
  copyStepLabel(index, false, msgBuf, sizeof(msgBuf));
  appendLogRam(msgBuf);

  activeAxis = FULL_CYCLE[index].axis;
  activeDir = FULL_CYCLE[index].dir;
  continuousMode = false;

  AccelStepper &stepper = stepperFor(activeAxis);
  digitalWrite(enablePinFor(activeAxis), enabledLevelFor(activeAxis));
  stepper.enableOutputs();
  stepper.move((activeDir == MOVE_FWD) ? FULL_CYCLE[index].steps : -FULL_CYCLE[index].steps);

  setDemoState(ST_AUTO_RUN);
}

void beginAutoCycle() {
  sequenceActive = true;
  prepareCycleStart();
  appendLogP(PSTR("START"));
  startAutoStep(autoStepIndex);
}

void prepareCycleStart() {
  autoStepIndex = 0;
  resetSequenceFlags();
  cycleStartMs = millis();
}

void serviceMotion() {
  stepperM1.run();
  stepperM2.run();
  stepperM3.run();
}

void serviceSequence() {
  unsigned long now = millis();

  switch (demoState) {
    case ST_IDLE:
      return;

    case ST_AUTO_RUN:
      if (!continuousMode && activeAxis != AXIS_NONE && stepperFor(activeAxis).distanceToGo() == 0) {
        markStepComplete(autoStepIndex);
        char msgBuf[20];
        copyStepLabel(autoStepIndex, true, msgBuf, sizeof(msgBuf));
        appendLogRam(msgBuf);
        stopAllMotion();
        setDemoState(ST_AUTO_WAIT);
      }
      break;

    case ST_AUTO_WAIT:
      if (now - demoStateStartMs >= SETTLE_MS) {
        autoStepIndex++;
        if (autoStepIndex >= FULL_CYCLE_COUNT) {
          appendLogP(PSTR("CYCLE DONE"));
          sequenceActive = false;
          setDemoState(ST_DONE);
          setDemoState(ST_IDLE);
        } else {
          startAutoStep(autoStepIndex);
        }
      }
      break;

    case ST_DONE:
      setDemoState(ST_IDLE);
      break;
  }
}

void serviceUiTick() {
  unsigned long now = millis();
  if (now - lastUiTickMs < UI_TICK_MS) return;
  lastUiTickMs = now;

  if (sequenceActive || activeAxis != AXIS_NONE) updateHmi();
}

void resetSequenceFlags() {
  m1Done = false;
  m2Done = false;
  m3Done = false;
}

void markStepComplete(uint8_t index) {
  if (index == 0 || index == 3) m1Done = true;
  else if (index == 1 || index == 2 || index == 4 || index == 7) m2Done = true;
  else if (index == 5 || index == 6) m3Done = true;
}

AccelStepper &stepperFor(Axis axis) {
  if (axis == AXIS_M2) return stepperM2;
  if (axis == AXIS_M3) return stepperM3;
  return stepperM1;
}

uint8_t dirPinFor(Axis axis) {
  if (axis == AXIS_M2) return PIN_M2_DIR;
  if (axis == AXIS_M3) return PIN_M3_DIR;
  return PIN_M1_DIR;
}

uint8_t enablePinFor(Axis axis) {
  return axis == AXIS_M1 ? PIN_M1_ENA : PIN_SHARED_ENA;
}

uint8_t enabledLevelFor(Axis axis) {
  return axis == AXIS_M1 ? M1_ENA_ENABLED_LEVEL : SHARED_ENA_ENABLED_LEVEL;
}

uint8_t disabledLevelFor(Axis axis) {
  return axis == AXIS_M1 ? M1_ENA_DISABLED_LEVEL : SHARED_ENA_DISABLED_LEVEL;
}

bool directionInvertFor(Axis axis) {
  if (axis == AXIS_M1) return !M1_FORWARD_IS_DOWN;
  if (axis == AXIS_M2) return !M2_FORWARD_IS_OPEN;
  if (axis == AXIS_M3) return !M3_FORWARD_IS_TILT_OUT;
  return false;
}

void handleCommand(char *cmd) {
  for (char *p = cmd; *p; ++p) *p = (char)toupper((unsigned char)*p);

  if (!strcmp(cmd, "M1F")) {
    sequenceActive = false;
    setDemoState(ST_IDLE);
    appendLogP(PSTR("M1 EXT"));
    startAxis(AXIS_M1, MOVE_FWD);
  } else if (!strcmp(cmd, "M1R")) {
    sequenceActive = false;
    setDemoState(ST_IDLE);
    appendLogP(PSTR("M1 RET"));
    startAxis(AXIS_M1, MOVE_REV);
  } else if (!strcmp(cmd, "M2F")) {
    sequenceActive = false;
    setDemoState(ST_IDLE);
    appendLogP(PSTR("M2 OPEN"));
    startAxis(AXIS_M2, MOVE_FWD);
  } else if (!strcmp(cmd, "M2R")) {
    sequenceActive = false;
    setDemoState(ST_IDLE);
    appendLogP(PSTR("M2 CLS"));
    startAxis(AXIS_M2, MOVE_REV);
  } else if (!strcmp(cmd, "M3F")) {
    sequenceActive = false;
    setDemoState(ST_IDLE);
    appendLogP(PSTR("M3 OUT"));
    startAxis(AXIS_M3, MOVE_FWD);
  } else if (!strcmp(cmd, "M3R")) {
    sequenceActive = false;
    setDemoState(ST_IDLE);
    appendLogP(PSTR("M3 HOM"));
    startAxis(AXIS_M3, MOVE_REV);
  } else if (!strcmp(cmd, "START")) {
    beginAutoCycle();
    updateHmi();
  } else if (!strcmp(cmd, "RESET")) {
    appendLogP(PSTR("RESET"));
    sequenceActive = false;
    resetSequenceFlags();
    setDemoState(ST_IDLE);
    stopAllMotion();
    updateHmi();
  } else if (!strcmp(cmd, "STOP")) {
    appendLogP(PSTR("STOP"));
    sequenceActive = false;
    setDemoState(ST_IDLE);
    stopAllMotion();
    updateHmi();
  } else if (!strcmp(cmd, "STATUS")) {
    char stateBuf[18];
    fillDemoStateText(stateBuf, sizeof(stateBuf));
    appendLogRam(stateBuf);
    updateHmi();
  } else if (!strcmp(cmd, "PAGE:MAIN")) {
    currentPageIsLog = false;
    updateHmi();
  } else if (!strcmp(cmd, "PAGE:LOG") || !strcmp(cmd, "PAGE:LOGS")) {
    currentPageIsLog = true;
    refreshLogPage();
  } else if (!strcmp(cmd, "PAGE:DIAG")) {
    currentPageIsLog = false;
  } else if (!strcmp(cmd, "CLEARLOG")) {
    strcpy(latestLogLine, "NO EVENTS");
    refreshLogPage();
  } else {
    appendLogP(PSTR("BADCMD"));
  }
}

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

void nxCmd(const char *cmd) {
  nxt.print(cmd);
  nxWriteTerminator();
}

void nxSetPageTxt(const char *page, const char *obj, const char *txt) {
  nxt.print(page);
  nxt.print('.');
  nxt.print(obj);
  nxt.print(F(".txt=\""));
  nxt.print(txt);
  nxt.print('"');
  nxWriteTerminator();
}

void nxSetPco(const char *obj, uint16_t color) {
  nxt.print(obj);
  nxt.print(F(".pco="));
  nxt.print(color);
  nxWriteTerminator();
}

void nxWriteTerminator() {
  nxt.write(0xFF);
  nxt.write(0xFF);
  nxt.write(0xFF);
}

void pushClockText() {
  char clockBuf[15];
  if (rtcOnline) snprintf(clockBuf, sizeof(clockBuf), "TIME: %s", currentTimeText);
  else strcpy(clockBuf, "TIME: RTC ERR");
  nxSetPageTxt("page0", "tClock", clockBuf);
  nxSetPageTxt("page1", "tClock1", clockBuf);
  nxSetPageTxt("page2", "tClock2", clockBuf);
}

void updateHmi() {
  static char lastState[12] = "";
  static char lastFault[18] = "";
  static char lastPress[12] = "";
  static char lastRevs[18] = "";
  static char lastEstop[18] = "";
  static char lastDriver[18] = "";
  static char lastCycle[22] = "";
  static char lastLsRet[16] = "";
  static char lastLsExt[16] = "";
  static char lastEnc[16] = "";
  static char lastMotor[18] = "";
  static char lastMotorFwd[18] = "";
  static char lastMotorRev[18] = "";

  char stateBuf[12];
  char faultBuf[18];
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

  fillAxisStateText(stateBuf, sizeof(stateBuf));
  fillDemoStateText(faultBuf, sizeof(faultBuf));
  strcpy(pressBuf, "CMD LINK OK");
  fillDemoStateText(revsBuf, sizeof(revsBuf));
  fillSeqActiveText(estopBuf, sizeof(estopBuf));
  fillDriverText(driverBuf, sizeof(driverBuf));
  fillCycleElapsedText(cycleBuf, sizeof(cycleBuf));
  fillDoneText(m1Done, "M1 DONE:", lsRetBuf, sizeof(lsRetBuf));
  fillDoneText(m2Done, "M2 DONE:", lsExtBuf, sizeof(lsExtBuf));
  fillDoneText(m3Done, "M3 DONE:", encBuf, sizeof(encBuf));
  fillActiveText(activeAxis == AXIS_M3, "M3 ACTIVE:", motorBuf, sizeof(motorBuf));
  fillActiveText(activeAxis == AXIS_M1, "M1 ACTIVE:", motorFwdBuf, sizeof(motorFwdBuf));
  fillActiveText(activeAxis == AXIS_M2, "M2 ACTIVE:", motorRevBuf, sizeof(motorRevBuf));

  if (strcmp(stateBuf, lastState)) {
    nxSetPageTxt("page0", "tState", stateBuf);
    strcpy(lastState, stateBuf);
  }
  if (strcmp(faultBuf, lastFault)) {
    nxSetPageTxt("page0", "tFault", faultBuf);
    strcpy(lastFault, faultBuf);
  }
  if (strcmp(pressBuf, lastPress)) {
    nxSetPageTxt("page0", "tPress", pressBuf);
    strcpy(lastPress, pressBuf);
  }
  if (strcmp(revsBuf, lastRevs)) {
    nxSetPageTxt("page0", "tRevs", revsBuf);
    strcpy(lastRevs, revsBuf);
  }
  if (strcmp(estopBuf, lastEstop)) {
    nxSetPageTxt("page0", "tEstop", estopBuf);
    strcpy(lastEstop, estopBuf);
  }
  if (strcmp(driverBuf, lastDriver)) {
    nxSetPageTxt("page0", "tDriver", driverBuf);
    strcpy(lastDriver, driverBuf);
  }
  if (strcmp(cycleBuf, lastCycle)) {
    nxSetPageTxt("page0", "tCycle", cycleBuf);
    strcpy(lastCycle, cycleBuf);
  }
  if (strcmp(lsRetBuf, lastLsRet)) {
    nxSetPageTxt("page0", "tLsRet", lsRetBuf);
    strcpy(lastLsRet, lsRetBuf);
  }
  if (strcmp(lsExtBuf, lastLsExt)) {
    nxSetPageTxt("page0", "tLsExt", lsExtBuf);
    strcpy(lastLsExt, lsExtBuf);
  }
  if (strcmp(encBuf, lastEnc)) {
    nxSetPageTxt("page0", "tEnc", encBuf);
    strcpy(lastEnc, encBuf);
  }
  if (strcmp(motorBuf, lastMotor)) {
    nxSetPageTxt("page0", "tMotor", motorBuf);
    strcpy(lastMotor, motorBuf);
  }
  if (strcmp(motorFwdBuf, lastMotorFwd)) {
    nxSetPageTxt("page0", "tMotorfwd", motorFwdBuf);
    strcpy(lastMotorFwd, motorFwdBuf);
  }
  if (strcmp(motorRevBuf, lastMotorRev)) {
    nxSetPageTxt("page0", "tMotorrev", motorRevBuf);
    strcpy(lastMotorRev, motorRevBuf);
  }

  nxSetPco("page0.tState", NX_BLACK);
  nxSetPco("page0.tFault", NX_BLACK);
  nxSetPco("page0.tPress", NX_BLACK);
  nxSetPco("page0.tRevs", NX_BLACK);
  nxSetPco("page0.tEstop", NX_BLACK);
  nxSetPco("page0.tDriver", NX_BLACK);
  nxSetPco("page0.tCycle", NX_BLACK);
  nxSetPco("page0.tLsRet", NX_BLACK);
  nxSetPco("page0.tLsExt", NX_BLACK);
  nxSetPco("page0.tEnc", NX_BLACK);
  nxSetPco("page0.tMotor", NX_BLACK);
  nxSetPco("page0.tMotorfwd", NX_BLACK);
  nxSetPco("page0.tMotorrev", NX_BLACK);

  if (currentPageIsLog) refreshLogPage();
}

void refreshLogPage() {
  pushClockText();
  nxSetPageTxt("page1", "tLog", latestLogLine);
  nxSetPageTxt("page1", "tFrame", latestLogLine);
}

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

uint8_t decToBcd(uint8_t value) {
  return (uint8_t)(((value / 10U) << 4) | (value % 10U));
}

void syncRtcToBuildTime() {
  if (!RTC_SET_TIME_ON_BOOT) return;

  uint8_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;

  if (!parseBuildDateTime(year, month, day, hour, minute, second)) return;
  writeRtcDateTime(year, month, day, hour, minute, second);
}

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

void appendLogRam(const char *msg) {
  snprintf(latestLogLine, sizeof(latestLogLine), "%s|%s", currentTimeText, msg);
  latestLogLine[sizeof(latestLogLine) - 1] = '\0';
}

void appendLogP(PGM_P msg) {
  char msgBuf[20];
  strncpy_P(msgBuf, msg, sizeof(msgBuf) - 1);
  msgBuf[sizeof(msgBuf) - 1] = '\0';
  appendLogRam(msgBuf);
}

void copyStepLabel(uint8_t index, bool completeText, char *out, size_t outLen) {
  switch (index) {
    case 0: snprintf_P(out, outLen, completeText ? PSTR("S1 DONE") : PSTR("S1 M1 DOWN")); break;
    case 1: snprintf_P(out, outLen, completeText ? PSTR("S2 DONE") : PSTR("S2 M2 OPEN")); break;
    case 2: snprintf_P(out, outLen, completeText ? PSTR("S3 DONE") : PSTR("S3 M2 CLS")); break;
    case 3: snprintf_P(out, outLen, completeText ? PSTR("S4 DONE") : PSTR("S4 M1 UP")); break;
    case 4: snprintf_P(out, outLen, completeText ? PSTR("S5 DONE") : PSTR("S5 M2 OPEN")); break;
    case 5: snprintf_P(out, outLen, completeText ? PSTR("S6 DONE") : PSTR("S6 M3 OUT")); break;
    case 6: snprintf_P(out, outLen, completeText ? PSTR("S7 DONE") : PSTR("S7 M3 HOM")); break;
    case 7: snprintf_P(out, outLen, completeText ? PSTR("S8 DONE") : PSTR("S8 M2 CLS")); break;
    default: snprintf_P(out, outLen, PSTR("READY")); break;
  }
}

void fillAxisStateText(char *out, size_t outLen) {
  if (activeAxis == AXIS_M1) snprintf_P(out, outLen, activeDir == MOVE_FWD ? PSTR("M1 DOWN") : PSTR("M1 UP"));
  else if (activeAxis == AXIS_M2) snprintf_P(out, outLen, activeDir == MOVE_FWD ? PSTR("M2 OPEN") : PSTR("M2 CLOSE"));
  else if (activeAxis == AXIS_M3) snprintf_P(out, outLen, activeDir == MOVE_FWD ? PSTR("TILT OUT") : PSTR("TILT HOME"));
  else snprintf_P(out, outLen, PSTR("IDLE"));
}

void fillDemoStateText(char *out, size_t outLen) {
  if (demoState == ST_IDLE) snprintf_P(out, outLen, PSTR("READY"));
  else if (demoState == ST_AUTO_WAIT) snprintf_P(out, outLen, PSTR("STEP SET"));
  else if (demoState == ST_DONE) snprintf_P(out, outLen, PSTR("DONE"));
  else copyStepLabel(autoStepIndex, false, out, outLen);
}

void fillSeqActiveText(char *out, size_t outLen) {
  snprintf(out, outLen, "SEQ ACTIVE: %s", sequenceActive ? "YES" : "NO");
}

void fillDoneText(bool done, const char *prefix, char *out, size_t outLen) {
  snprintf(out, outLen, "%s %s", prefix, done ? "YES" : "NO");
}

void fillActiveText(bool active, const char *prefix, char *out, size_t outLen) {
  snprintf(out, outLen, "%s %s", prefix, active ? "YES" : "NO");
}

void fillCycleElapsedText(char *out, size_t outLen) {
  unsigned long elapsedMs = sequenceActive ? (millis() - cycleStartMs) : 0UL;
  unsigned int totalSeconds = (unsigned int)(elapsedMs / 1000UL);
  unsigned int minutes = totalSeconds / 60U;
  unsigned int seconds = totalSeconds % 60U;
  snprintf(out, outLen, "Cycle Elapsed: %02u:%02u", minutes, seconds);
}

void fillDriverText(char *out, size_t outLen) {
  snprintf(out, outLen, "System: %s", activeAxis != AXIS_NONE ? "ACTIVE" : "READY");
}

void setDemoState(DemoState s) {
  demoState = s;
  demoStateStartMs = millis();
}
