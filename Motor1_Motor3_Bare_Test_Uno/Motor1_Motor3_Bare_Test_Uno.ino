/*
  Motor 1 / Motor 3 Bare Test - Uno
  ---------------------------------
  Purpose:
  - Test only Motor 1 and Motor 3
  - Remove Nextion and RTC from the equation
  - Use Serial Monitor commands at 115200 baud

  Commands:
  - M1F   : Motor 1 forward / extend down
  - M1R   : Motor 1 reverse / retract up
  - M3F   : Motor 3 forward / tilt out
  - M3R   : Motor 3 reverse / tilt home
  - STOP  : Stop all motion
  - START : Run one short validation cycle
  - HELP  : Print commands again
*/

#include <ctype.h>
#include <string.h>

const uint8_t PIN_M1_PUL  = 2;
const uint8_t PIN_M1_DIR  = 3;
const uint8_t PIN_M1_ENA  = 8;
const uint8_t PIN_M3_STEP = 6;
const uint8_t PIN_M3_DIR  = 7;
const uint8_t PIN_M3_ENA  = 8;

const uint8_t DIR_FWD_LEVEL         = HIGH;
const uint8_t DIR_REV_LEVEL         = LOW;
const uint8_t M1_ENA_ENABLED_LEVEL  = LOW;
const uint8_t M1_ENA_DISABLED_LEVEL = HIGH;
const uint8_t M3_ENA_ENABLED_LEVEL  = LOW;
const uint8_t M3_ENA_DISABLED_LEVEL = HIGH;

const bool M1_FORWARD_IS_DOWN     = true;
const bool M3_FORWARD_IS_TILT_OUT = true;

const unsigned long STEP_INTERVAL_US   = 8000UL;
const unsigned long STEP_PULSE_US      = 10UL;
const unsigned long STEP_01_M1_DOWN_MS = 8750UL;
const unsigned long STEP_02_M1_UP_MS   = 8750UL;
const unsigned long STEP_03_M3_OUT_MS  = 1800UL;
const unsigned long STEP_04_M3_HOME_MS = 1800UL;
const unsigned long SETTLE_MS          = 300UL;

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

enum RunState {
  ST_IDLE = 0,
  ST_AUTO_RUN,
  ST_AUTO_WAIT
};

struct AutoStep {
  Axis axis;
  Direction dir;
  unsigned long runMs;
};

const AutoStep TEST_CYCLE[] = {
  { AXIS_M1, MOVE_FWD, STEP_01_M1_DOWN_MS },
  { AXIS_M1, MOVE_REV, STEP_02_M1_UP_MS },
  { AXIS_M3, MOVE_FWD, STEP_03_M3_OUT_MS },
  { AXIS_M3, MOVE_REV, STEP_04_M3_HOME_MS }
};

const uint8_t TEST_STEP_COUNT = sizeof(TEST_CYCLE) / sizeof(TEST_CYCLE[0]);

Axis activeAxis = AXIS_NONE;
Direction activeDir = MOVE_NONE;
RunState runState = ST_IDLE;
bool stepHigh = false;
unsigned long lastStepUs = 0;
unsigned long stateStartMs = 0;
uint8_t autoStepIndex = 0;
char serialLine[24];
uint8_t serialLen = 0;

void printHelp();
void stopAllMotion();
void startAxis(Axis axis, Direction dir);
void serviceMotion();
void serviceAutoCycle();
void beginAutoCycle();
void startAutoStep(uint8_t index);
uint8_t stepPinFor(Axis axis);
uint8_t dirPinFor(Axis axis);
uint8_t enablePinFor(Axis axis);
uint8_t enabledLevelFor(Axis axis);
uint8_t disabledLevelFor(Axis axis);
uint8_t dirLevelFor(Axis axis, Direction dir);
void handleCommand(char *cmd);

void setup() {
  Serial.begin(115200);

  pinMode(PIN_M1_PUL, OUTPUT);
  pinMode(PIN_M1_DIR, OUTPUT);
  pinMode(PIN_M1_ENA, OUTPUT);
  pinMode(PIN_M3_STEP, OUTPUT);
  pinMode(PIN_M3_DIR, OUTPUT);
  pinMode(PIN_M3_ENA, OUTPUT);

  digitalWrite(PIN_M1_PUL, LOW);
  digitalWrite(PIN_M3_STEP, LOW);
  digitalWrite(PIN_M1_DIR, DIR_FWD_LEVEL);
  digitalWrite(PIN_M3_DIR, DIR_FWD_LEVEL);
  digitalWrite(PIN_M1_ENA, M1_ENA_DISABLED_LEVEL);
  digitalWrite(PIN_M3_ENA, M3_ENA_DISABLED_LEVEL);

  Serial.println(F("M1/M3 BARE TEST READY"));
  printHelp();
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

  serviceAutoCycle();
  serviceMotion();
}

void printHelp() {
  Serial.println(F("Commands: M1F M1R M3F M3R START STOP HELP"));
}

void stopAllMotion() {
  activeAxis = AXIS_NONE;
  activeDir = MOVE_NONE;
  stepHigh = false;
  runState = ST_IDLE;

  digitalWrite(PIN_M1_PUL, LOW);
  digitalWrite(PIN_M3_STEP, LOW);
  digitalWrite(PIN_M1_ENA, M1_ENA_DISABLED_LEVEL);
  digitalWrite(PIN_M3_ENA, M3_ENA_DISABLED_LEVEL);
}

void startAxis(Axis axis, Direction dir) {
  stopAllMotion();
  activeAxis = axis;
  activeDir = dir;
  digitalWrite(dirPinFor(axis), dirLevelFor(axis, dir));
  digitalWrite(enablePinFor(axis), enabledLevelFor(axis));
  lastStepUs = micros();
  stepHigh = false;

  if (axis == AXIS_M1) {
    Serial.println(dir == MOVE_FWD ? F("M1 DOWN") : F("M1 UP"));
  } else if (axis == AXIS_M3) {
    Serial.println(dir == MOVE_FWD ? F("M3 OUT") : F("M3 HOME"));
  }
}

void serviceMotion() {
  if (activeAxis == AXIS_NONE || activeDir == MOVE_NONE) return;

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

void serviceAutoCycle() {
  unsigned long now = millis();

  if (runState == ST_AUTO_RUN) {
    if (now - stateStartMs >= TEST_CYCLE[autoStepIndex].runMs) {
      stopAllMotion();
      runState = ST_AUTO_WAIT;
      stateStartMs = now;
      Serial.print(F("STEP "));
      Serial.print(autoStepIndex + 1);
      Serial.println(F(" DONE"));
    }
  } else if (runState == ST_AUTO_WAIT) {
    if (now - stateStartMs >= SETTLE_MS) {
      autoStepIndex++;
      if (autoStepIndex >= TEST_STEP_COUNT) {
        runState = ST_IDLE;
        Serial.println(F("CYCLE DONE"));
      } else {
        startAutoStep(autoStepIndex);
      }
    }
  }
}

void beginAutoCycle() {
  autoStepIndex = 0;
  Serial.println(F("START CYCLE"));
  startAutoStep(autoStepIndex);
}

void startAutoStep(uint8_t index) {
  startAxis(TEST_CYCLE[index].axis, TEST_CYCLE[index].dir);
  runState = ST_AUTO_RUN;
  stateStartMs = millis();
}

uint8_t stepPinFor(Axis axis) {
  return axis == AXIS_M3 ? PIN_M3_STEP : PIN_M1_PUL;
}

uint8_t dirPinFor(Axis axis) {
  return axis == AXIS_M3 ? PIN_M3_DIR : PIN_M1_DIR;
}

uint8_t enablePinFor(Axis axis) {
  return axis == AXIS_M3 ? PIN_M3_ENA : PIN_M1_ENA;
}

uint8_t enabledLevelFor(Axis axis) {
  return axis == AXIS_M3 ? M3_ENA_ENABLED_LEVEL : M1_ENA_ENABLED_LEVEL;
}

uint8_t disabledLevelFor(Axis axis) {
  return axis == AXIS_M3 ? M3_ENA_DISABLED_LEVEL : M1_ENA_DISABLED_LEVEL;
}

uint8_t dirLevelFor(Axis axis, Direction dir) {
  bool forwardUsesFwdLevel = (axis == AXIS_M3) ? M3_FORWARD_IS_TILT_OUT : M1_FORWARD_IS_DOWN;
  if (dir == MOVE_FWD) return forwardUsesFwdLevel ? DIR_FWD_LEVEL : DIR_REV_LEVEL;
  return forwardUsesFwdLevel ? DIR_REV_LEVEL : DIR_FWD_LEVEL;
}

void handleCommand(char *cmd) {
  for (char *p = cmd; *p; ++p) *p = (char)toupper((unsigned char)*p);

  if (!strcmp(cmd, "M1F")) {
    startAxis(AXIS_M1, MOVE_FWD);
  } else if (!strcmp(cmd, "M1R")) {
    startAxis(AXIS_M1, MOVE_REV);
  } else if (!strcmp(cmd, "M3F")) {
    startAxis(AXIS_M3, MOVE_FWD);
  } else if (!strcmp(cmd, "M3R")) {
    startAxis(AXIS_M3, MOVE_REV);
  } else if (!strcmp(cmd, "START")) {
    beginAutoCycle();
  } else if (!strcmp(cmd, "STOP")) {
    stopAllMotion();
    Serial.println(F("STOPPED"));
  } else if (!strcmp(cmd, "HELP")) {
    printHelp();
  } else {
    Serial.println(F("UNKNOWN COMMAND"));
    printHelp();
  }
}
