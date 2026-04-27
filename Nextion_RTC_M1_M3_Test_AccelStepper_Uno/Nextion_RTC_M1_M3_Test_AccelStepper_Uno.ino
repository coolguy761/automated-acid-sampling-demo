/*
  Nextion + RTC + M1/M3 Test - AccelStepper Version
  -------------------------------------------------
  Purpose:
  - Validate only Motor 1 and Motor 3
  - Keep Nextion main/log pages active
  - Keep RTC clock updates
  - Use AccelStepper for motion control
*/

#include <AccelStepper.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <avr/pgmspace.h>
#include <ctype.h>
#include <stdio.h>
#include <string.h>

const uint8_t PIN_M1_PUL     = 2;
const uint8_t PIN_M1_DIR     = 3;
const uint8_t PIN_M1_ENA     = 8;
const uint8_t PIN_M3_STEP    = 6;
const uint8_t PIN_M3_DIR     = 7;
const uint8_t PIN_M3_ENA     = 8;
const uint8_t PIN_NX_RX      = 10;
const uint8_t PIN_NX_TX      = 11;
const uint8_t RTC_I2C_ADDR   = 0x68;

SoftwareSerial nxt(PIN_NX_RX, PIN_NX_TX);
AccelStepper stepperM1(AccelStepper::DRIVER, PIN_M1_PUL, PIN_M1_DIR);
AccelStepper stepperM3(AccelStepper::DRIVER, PIN_M3_STEP, PIN_M3_DIR);

const uint8_t M1_ENA_ENABLED_LEVEL  = LOW;
const uint8_t M1_ENA_DISABLED_LEVEL = HIGH;
const uint8_t M3_ENA_ENABLED_LEVEL  = LOW;
const uint8_t M3_ENA_DISABLED_LEVEL = HIGH;

const bool RTC_SET_TIME_ON_BOOT   = false;
const bool M1_FORWARD_IS_DOWN     = true;
const bool M3_FORWARD_IS_TILT_OUT = true;

const float M1_MAX_SPEED = 180.0f;
const float M1_ACCEL     = 90.0f;
const float M3_MAX_SPEED = 90.0f;
const float M3_ACCEL     = 45.0f;

const long STEP_01_M1_DOWN_STEPS = 3500L;
const long STEP_02_M1_UP_STEPS   = 3500L;
const long STEP_03_M3_OUT_STEPS  = 480L;
const long STEP_04_M3_HOME_STEPS = 480L;
const long MANUAL_JOG_STEPS      = 200000L;

const unsigned long SETTLE_MS     = 300UL;
const unsigned long RTC_UPDATE_MS = 1000UL;
const unsigned long UI_TICK_MS    = 500UL;

const uint16_t NX_BLACK = 0;
const uint8_t LOG_ENTRY_COUNT = 2;
const uint8_t LOG_ENTRY_LEN   = 28;

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
  ST_AUTO_RUN,
  ST_AUTO_WAIT,
  ST_DONE
};

struct AutoStep {
  Axis axis;
  Direction dir;
  long steps;
};

const AutoStep VALIDATION_CYCLE[] = {
  { AXIS_M1, MOVE_FWD, STEP_01_M1_DOWN_STEPS },
  { AXIS_M1, MOVE_REV, STEP_02_M1_UP_STEPS },
  { AXIS_M3, MOVE_FWD, STEP_03_M3_OUT_STEPS },
  { AXIS_M3, MOVE_REV, STEP_04_M3_HOME_STEPS }
};

const uint8_t VALIDATION_STEP_COUNT = sizeof(VALIDATION_CYCLE) / sizeof(VALIDATION_CYCLE[0]);

Axis activeAxis = AXIS_NONE;
Direction activeDir = MOVE_NONE;
DemoState demoState = ST_IDLE;
bool currentPageIsLog = false;
bool rtcOnline = false;
bool sequenceActive = false;
bool continuousMode = false;
bool m1Done = false;
bool m3Done = false;
bool logDirty = false;
unsigned long demoStateStartMs = 0;
unsigned long cycleStartMs = 0;
unsigned long lastRtcUpdateMs = 0;
unsigned long lastUiTickMs = 0;
uint8_t autoStepIndex = 0;
char currentTimeText[9] = "--:--:--";
char serialLine[32];
uint8_t serialLen = 0;
char latestLogLine[LOG_ENTRY_LEN] = "NO EVENTS";
char logEntries[LOG_ENTRY_COUNT][LOG_ENTRY_LEN];
uint8_t logHead = 0;
uint8_t logCount = 0;

void stopAllMotion();
void startAxis(Axis axis, Direction dir);
void startAutoStep(uint8_t index);
void beginAutoCycle();
void serviceMotion();
void serviceSequence();
void serviceUiTick();
void resetSequenceFlags();
void markStepComplete(uint8_t index);
AccelStepper &stepperFor(Axis axis);
bool invertDirFor(Axis axis);
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

  pinMode(PIN_M1_ENA, OUTPUT);
  pinMode(PIN_M3_ENA, OUTPUT);
  digitalWrite(PIN_M1_ENA, M1_ENA_DISABLED_LEVEL);
  digitalWrite(PIN_M3_ENA, M3_ENA_DISABLED_LEVEL);

  stepperM1.setEnablePin(PIN_M1_ENA);
  stepperM1.setPinsInverted(invertDirFor(AXIS_M1), false, M1_ENA_ENABLED_LEVEL == HIGH);
  stepperM1.setMaxSpeed(M1_MAX_SPEED);
  stepperM1.setAcceleration(M1_ACCEL);
  stepperM1.disableOutputs();

  stepperM3.setEnablePin(PIN_M3_ENA);
  stepperM3.setPinsInverted(invertDirFor(AXIS_M3), false, M3_ENA_ENABLED_LEVEL == HIGH);
  stepperM3.setMaxSpeed(M3_MAX_SPEED);
  stepperM3.setAcceleration(M3_ACCEL);
  stepperM3.disableOutputs();

  nxCmd("bkcmd=0");
  nxCmd("sendxy=0");
  nxCmd("sendme");

  syncRtcToBuildTime();
  updateRtc(true);

  appendLogP(PSTR("BOOT"));
  appendLogP(rtcOnline ? PSTR("RTCOK") : PSTR("RTCERR"));
  appendLogP(PSTR("READY"));
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

void stopAllMotion() {
  stepperM1.stop();
  stepperM3.stop();
  stepperM1.moveTo(stepperM1.currentPosition());
  stepperM3.moveTo(stepperM3.currentPosition());
  stepperM1.disableOutputs();
  stepperM3.disableOutputs();
  digitalWrite(PIN_M1_ENA, M1_ENA_DISABLED_LEVEL);
  digitalWrite(PIN_M3_ENA, M3_ENA_DISABLED_LEVEL);
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
  stepper.enableOutputs();
  digitalWrite(axis == AXIS_M1 ? PIN_M1_ENA : PIN_M3_ENA, axis == AXIS_M1 ? M1_ENA_ENABLED_LEVEL : M3_ENA_ENABLED_LEVEL);
  stepper.move(dir == MOVE_FWD ? MANUAL_JOG_STEPS : -MANUAL_JOG_STEPS);
  updateHmi();
}

void startAutoStep(uint8_t index) {
  stopAllMotion();
  char msgBuf[14];
  copyStepLabel(index, false, msgBuf, sizeof(msgBuf));
  appendLogRam(msgBuf);
  activeAxis = VALIDATION_CYCLE[index].axis;
  activeDir = VALIDATION_CYCLE[index].dir;
  continuousMode = false;
  AccelStepper &stepper = stepperFor(activeAxis);
  stepper.enableOutputs();
  digitalWrite(activeAxis == AXIS_M1 ? PIN_M1_ENA : PIN_M3_ENA, activeAxis == AXIS_M1 ? M1_ENA_ENABLED_LEVEL : M3_ENA_ENABLED_LEVEL);
  stepper.move(activeDir == MOVE_FWD ? VALIDATION_CYCLE[index].steps : -VALIDATION_CYCLE[index].steps);
  setDemoState(ST_AUTO_RUN);
}

void beginAutoCycle() {
  autoStepIndex = 0;
  resetSequenceFlags();
  sequenceActive = true;
  cycleStartMs = millis();
  appendLogP(PSTR("START"));
  startAutoStep(autoStepIndex);
}

void serviceMotion() {
  stepperM1.run();
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
        char msgBuf[14];
        copyStepLabel(autoStepIndex, true, msgBuf, sizeof(msgBuf));
        appendLogRam(msgBuf);
        stopAllMotion();
        setDemoState(ST_AUTO_WAIT);
      }
      break;
    case ST_AUTO_WAIT:
      if (now - demoStateStartMs >= SETTLE_MS) {
        autoStepIndex++;
        if (autoStepIndex >= VALIDATION_STEP_COUNT) {
          sequenceActive = false;
          appendLogP(PSTR("DONE"));
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
  m3Done = false;
}

void markStepComplete(uint8_t index) {
  if (index == 0 || index == 1) m1Done = true;
  else if (index == 2 || index == 3) m3Done = true;
}

AccelStepper &stepperFor(Axis axis) {
  return axis == AXIS_M3 ? stepperM3 : stepperM1;
}

bool invertDirFor(Axis axis) {
  return axis == AXIS_M3 ? !M3_FORWARD_IS_TILT_OUT : !M1_FORWARD_IS_DOWN;
}

void handleCommand(char *cmd) {
  for (char *p = cmd; *p; ++p) *p = (char)toupper((unsigned char)*p);

  if (!strcmp(cmd, "M1F")) {
    sequenceActive = false;
    setDemoState(ST_IDLE);
    appendLogP(PSTR("M1EXT"));
    startAxis(AXIS_M1, MOVE_FWD);
  } else if (!strcmp(cmd, "M1R")) {
    sequenceActive = false;
    setDemoState(ST_IDLE);
    appendLogP(PSTR("M1RET"));
    startAxis(AXIS_M1, MOVE_REV);
  } else if (!strcmp(cmd, "M3F")) {
    sequenceActive = false;
    setDemoState(ST_IDLE);
    appendLogP(PSTR("M3OUT"));
    startAxis(AXIS_M3, MOVE_FWD);
  } else if (!strcmp(cmd, "M3R")) {
    sequenceActive = false;
    setDemoState(ST_IDLE);
    appendLogP(PSTR("M3HOM"));
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
    appendLogP(PSTR("CLR"));
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

void nxCmd(const char *cmd) { nxt.print(cmd); nxWriteTerminator(); }

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
  static char lastEnc[16] = "";
  static char lastMotor[16] = "";
  static char lastMotorFwd[18] = "";

  char stateBuf[12];
  char faultBuf[18];
  char pressBuf[12];
  char revsBuf[18];
  char estopBuf[18];
  char driverBuf[18];
  char cycleBuf[22];
  char lsRetBuf[16];
  char encBuf[16];
  char motorBuf[16];
  char motorFwdBuf[18];

  if (activeAxis == AXIS_NONE) updateRtc(false);

  fillAxisStateText(stateBuf, sizeof(stateBuf));
  fillDemoStateText(faultBuf, sizeof(faultBuf));
  strcpy(pressBuf, "CMD LINK OK");
  fillDemoStateText(revsBuf, sizeof(revsBuf));
  fillSeqActiveText(estopBuf, sizeof(estopBuf));
  fillDriverText(driverBuf, sizeof(driverBuf));
  fillCycleElapsedText(cycleBuf, sizeof(cycleBuf));
  fillDoneText(m1Done, "M1 DONE:", lsRetBuf, sizeof(lsRetBuf));
  fillDoneText(m3Done, "M3 DONE:", encBuf, sizeof(encBuf));
  fillActiveText(activeAxis == AXIS_M3, "M3 ACTIVE:", motorBuf, sizeof(motorBuf));
  fillActiveText(activeAxis == AXIS_M1, "M1 ACTIVE:", motorFwdBuf, sizeof(motorFwdBuf));

  if (strcmp(stateBuf, lastState)) { nxSetPageTxt("page0", "tState", stateBuf); strcpy(lastState, stateBuf); }
  if (strcmp(faultBuf, lastFault)) { nxSetPageTxt("page0", "tFault", faultBuf); strcpy(lastFault, faultBuf); }
  if (strcmp(pressBuf, lastPress)) { nxSetPageTxt("page0", "tPress", pressBuf); strcpy(lastPress, pressBuf); }
  if (strcmp(revsBuf, lastRevs)) { nxSetPageTxt("page0", "tRevs", revsBuf); strcpy(lastRevs, revsBuf); }
  if (strcmp(estopBuf, lastEstop)) { nxSetPageTxt("page0", "tEstop", estopBuf); strcpy(lastEstop, estopBuf); }
  if (strcmp(driverBuf, lastDriver)) { nxSetPageTxt("page0", "tDriver", driverBuf); strcpy(lastDriver, driverBuf); }
  if (strcmp(cycleBuf, lastCycle)) { nxSetPageTxt("page0", "tCycle", cycleBuf); strcpy(lastCycle, cycleBuf); }
  if (strcmp(lsRetBuf, lastLsRet)) { nxSetPageTxt("page0", "tLsRet", lsRetBuf); strcpy(lastLsRet, lsRetBuf); }
  if (strcmp(encBuf, lastEnc)) { nxSetPageTxt("page0", "tEnc", encBuf); strcpy(lastEnc, encBuf); }
  if (strcmp(motorBuf, lastMotor)) { nxSetPageTxt("page0", "tMotor", motorBuf); strcpy(lastMotor, motorBuf); }
  if (strcmp(motorFwdBuf, lastMotorFwd)) { nxSetPageTxt("page0", "tMotorfwd", motorFwdBuf); strcpy(lastMotorFwd, motorFwdBuf); }

  nxSetPco("page0.tState", NX_BLACK);
  nxSetPco("page0.tFault", NX_BLACK);
  nxSetPco("page0.tPress", NX_BLACK);
  nxSetPco("page0.tRevs", NX_BLACK);
  nxSetPco("page0.tEstop", NX_BLACK);
  nxSetPco("page0.tDriver", NX_BLACK);
  nxSetPco("page0.tCycle", NX_BLACK);
  nxSetPco("page0.tLsRet", NX_BLACK);
  nxSetPco("page0.tEnc", NX_BLACK);
  nxSetPco("page0.tMotor", NX_BLACK);
  nxSetPco("page0.tMotorfwd", NX_BLACK);

  if (logDirty && currentPageIsLog && activeAxis == AXIS_NONE) refreshLogPage();
}

void refreshLogPage() {
  pushClockText();
  nxt.print(F("page1.tFrame.txt=\""));
  if (!logCount) {
    nxt.print(F("NO EVENTS"));
  } else {
    for (uint8_t i = 0; i < logCount; ++i) {
      uint8_t idx = (uint8_t)((logHead + i) % LOG_ENTRY_COUNT);
      nxt.print(logEntries[idx]);
      if (i + 1 < logCount) nxt.print(F(" | "));
    }
  }
  nxt.print('"');
  nxWriteTerminator();

  nxt.print(F("page1.tLog.txt=\""));
  nxt.print(latestLogLine);
  nxt.print('"');
  nxWriteTerminator();
  logDirty = false;
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

uint8_t bcdToDec(uint8_t value) { return (uint8_t)(((value >> 4) * 10U) + (value & 0x0F)); }
uint8_t decToBcd(uint8_t value) { return (uint8_t)(((value / 10U) << 4) | (value % 10U)); }

void syncRtcToBuildTime() {
  if (!RTC_SET_TIME_ON_BOOT) return;
  uint8_t year, month, day, hour, minute, second;
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

  uint8_t idx;
  if (logCount < LOG_ENTRY_COUNT) {
    idx = (uint8_t)((logHead + logCount) % LOG_ENTRY_COUNT);
    logCount++;
  } else {
    idx = logHead;
    logHead = (uint8_t)((logHead + 1) % LOG_ENTRY_COUNT);
  }

  strncpy(logEntries[idx], latestLogLine, LOG_ENTRY_LEN - 1);
  logEntries[idx][LOG_ENTRY_LEN - 1] = '\0';
  logDirty = true;
}

void appendLogP(PGM_P msg) {
  char msgBuf[14];
  strncpy_P(msgBuf, msg, sizeof(msgBuf) - 1);
  msgBuf[sizeof(msgBuf) - 1] = '\0';
  appendLogRam(msgBuf);
}

void copyStepLabel(uint8_t index, bool completeText, char *out, size_t outLen) {
  switch (index) {
    case 0: snprintf_P(out, outLen, completeText ? PSTR("S1DN") : PSTR("S1M1DN")); break;
    case 1: snprintf_P(out, outLen, completeText ? PSTR("S2DN") : PSTR("S2M1UP")); break;
    case 2: snprintf_P(out, outLen, completeText ? PSTR("S3DN") : PSTR("S3M3OT")); break;
    case 3: snprintf_P(out, outLen, completeText ? PSTR("S4DN") : PSTR("S4M3HM")); break;
    default: snprintf_P(out, outLen, PSTR("READY")); break;
  }
}

void fillAxisStateText(char *out, size_t outLen) {
  if (activeAxis == AXIS_M1) snprintf_P(out, outLen, activeDir == MOVE_FWD ? PSTR("M1 DOWN") : PSTR("M1 UP"));
  else if (activeAxis == AXIS_M3) snprintf_P(out, outLen, activeDir == MOVE_FWD ? PSTR("TILT OUT") : PSTR("TILT HOME"));
  else snprintf_P(out, outLen, PSTR("IDLE"));
}

void fillDemoStateText(char *out, size_t outLen) {
  if (demoState == ST_IDLE) snprintf_P(out, outLen, PSTR("READY"));
  else if (demoState == ST_AUTO_WAIT) snprintf_P(out, outLen, PSTR("STEP SET"));
  else if (demoState == ST_DONE) snprintf_P(out, outLen, PSTR("DONE"));
  else copyStepLabel(autoStepIndex, false, out, outLen);
}

void fillSeqActiveText(char *out, size_t outLen) { snprintf(out, outLen, "SEQ ACTIVE: %s", sequenceActive ? "YES" : "NO"); }
void fillDoneText(bool done, const char *prefix, char *out, size_t outLen) { snprintf(out, outLen, "%s %s", prefix, done ? "YES" : "NO"); }
void fillActiveText(bool active, const char *prefix, char *out, size_t outLen) { snprintf(out, outLen, "%s %s", prefix, active ? "YES" : "NO"); }

void fillCycleElapsedText(char *out, size_t outLen) {
  unsigned long elapsedMs = sequenceActive ? (millis() - cycleStartMs) : 0UL;
  unsigned int totalSeconds = (unsigned int)(elapsedMs / 1000UL);
  unsigned int minutes = totalSeconds / 60U;
  unsigned int seconds = totalSeconds % 60U;
  snprintf(out, outLen, "Cycle Elapsed: %02u:%02u", minutes, seconds);
}

void fillDriverText(char *out, size_t outLen) { snprintf(out, outLen, "System: %s", activeAxis != AXIS_NONE ? "ACTIVE" : "READY"); }

void setDemoState(DemoState s) {
  demoState = s;
  demoStateStartMs = millis();
}
