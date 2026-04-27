/*
  Motor 1 / Motor 3 Bare Test - AccelStepper Version
  --------------------------------------------------
  Purpose:
  - Test only Motor 1 and Motor 3
  - Remove Nextion and RTC from the equation
  - Use AccelStepper for gentler starts/stops
  - Use Serial Monitor commands at 115200 baud

  Commands:
  - Auto-runs one validation cycle after boot delay
  - Optional Serial commands still exist if available:
    M1F, M1R, M3F, M3R, STOP, START, HELP
*/

#include <AccelStepper.h>
#include <ctype.h>
#include <string.h>

const uint8_t PIN_M1_PUL  = 2;
const uint8_t PIN_M1_DIR  = 3;
const uint8_t PIN_M1_ENA  = 8;
const uint8_t PIN_M3_STEP = 6;
const uint8_t PIN_M3_DIR  = 7;
const uint8_t PIN_M3_ENA  = 8;

const uint8_t M1_ENA_ENABLED_LEVEL  = LOW;
const uint8_t M1_ENA_DISABLED_LEVEL = HIGH;
const uint8_t M3_ENA_ENABLED_LEVEL  = LOW;
const uint8_t M3_ENA_DISABLED_LEVEL = HIGH;

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
const unsigned long SETTLE_MS    = 300UL;
const unsigned long AUTO_START_DELAY_MS = 3000UL;

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
  long steps;
};

const AutoStep TEST_CYCLE[] = {
  { AXIS_M1, MOVE_FWD, STEP_01_M1_DOWN_STEPS },
  { AXIS_M1, MOVE_REV, STEP_02_M1_UP_STEPS },
  { AXIS_M3, MOVE_FWD, STEP_03_M3_OUT_STEPS },
  { AXIS_M3, MOVE_REV, STEP_04_M3_HOME_STEPS }
};

const uint8_t TEST_STEP_COUNT = sizeof(TEST_CYCLE) / sizeof(TEST_CYCLE[0]);

AccelStepper stepperM1(AccelStepper::DRIVER, PIN_M1_PUL, PIN_M1_DIR);
AccelStepper stepperM3(AccelStepper::DRIVER, PIN_M3_STEP, PIN_M3_DIR);

Axis activeAxis = AXIS_NONE;
Direction activeDir = MOVE_NONE;
RunState runState = ST_IDLE;
unsigned long stateStartMs = 0;
uint8_t autoStepIndex = 0;
char serialLine[24];
uint8_t serialLen = 0;
bool autoStartPending = true;
unsigned long bootMs = 0;

void printHelp();
void setupSteppers();
void stopAllMotion();
void startAxis(Axis axis, Direction dir);
void serviceMotion();
void serviceAutoCycle();
void beginAutoCycle();
void startAutoStep(uint8_t index);
AccelStepper &stepperFor(Axis axis);
bool invertDirFor(Axis axis);
const __FlashStringHelper *labelFor(Axis axis, Direction dir);
void handleCommand(char *cmd);

void setup() {
  Serial.begin(115200);

  pinMode(PIN_M1_ENA, OUTPUT);
  pinMode(PIN_M3_ENA, OUTPUT);
  digitalWrite(PIN_M1_ENA, M1_ENA_DISABLED_LEVEL);
  digitalWrite(PIN_M3_ENA, M3_ENA_DISABLED_LEVEL);

  setupSteppers();
  stopAllMotion();
  bootMs = millis();

  Serial.println(F("M1/M3 ACCELSTEPPER TEST READY"));
  Serial.println(F("Auto cycle starts after boot delay."));
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

  if (autoStartPending && millis() - bootMs >= AUTO_START_DELAY_MS) {
    autoStartPending = false;
    beginAutoCycle();
  }

  serviceMotion();
  serviceAutoCycle();
}

void printHelp() {
  Serial.println(F("Commands: M1F M1R M3F M3R START STOP HELP"));
  Serial.println(F("Tune M1 with M1_MAX_SPEED and M1_ACCEL near the top."));
}

void setupSteppers() {
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
  runState = ST_IDLE;
}

void startAxis(Axis axis, Direction dir) {
  stopAllMotion();
  activeAxis = axis;
  activeDir = dir;

  AccelStepper &stepper = stepperFor(axis);
  stepper.enableOutputs();
  if (axis == AXIS_M1) digitalWrite(PIN_M1_ENA, M1_ENA_ENABLED_LEVEL);
  else digitalWrite(PIN_M3_ENA, M3_ENA_ENABLED_LEVEL);

  stepper.move((dir == MOVE_FWD) ? MANUAL_JOG_STEPS : -MANUAL_JOG_STEPS);
  Serial.println(labelFor(axis, dir));
}

void serviceMotion() {
  stepperM1.run();
  stepperM3.run();
}

void serviceAutoCycle() {
  unsigned long now = millis();

  if (runState == ST_AUTO_RUN) {
    if (activeAxis != AXIS_NONE && stepperFor(activeAxis).distanceToGo() == 0) {
      Serial.print(F("STEP "));
      Serial.print(autoStepIndex + 1);
      Serial.println(F(" DONE"));

      stopAllMotion();
      runState = ST_AUTO_WAIT;
      stateStartMs = now;
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
  stopAllMotion();
  activeAxis = TEST_CYCLE[index].axis;
  activeDir = TEST_CYCLE[index].dir;

  AccelStepper &stepper = stepperFor(activeAxis);
  stepper.enableOutputs();
  if (activeAxis == AXIS_M1) digitalWrite(PIN_M1_ENA, M1_ENA_ENABLED_LEVEL);
  else digitalWrite(PIN_M3_ENA, M3_ENA_ENABLED_LEVEL);

  stepper.move((activeDir == MOVE_FWD) ? TEST_CYCLE[index].steps : -TEST_CYCLE[index].steps);
  runState = ST_AUTO_RUN;
  stateStartMs = millis();

  Serial.println(labelFor(activeAxis, activeDir));
}

AccelStepper &stepperFor(Axis axis) {
  return axis == AXIS_M3 ? stepperM3 : stepperM1;
}

bool invertDirFor(Axis axis) {
  return axis == AXIS_M3 ? !M3_FORWARD_IS_TILT_OUT : !M1_FORWARD_IS_DOWN;
}

const __FlashStringHelper *labelFor(Axis axis, Direction dir) {
  if (axis == AXIS_M1) return dir == MOVE_FWD ? F("M1 DOWN") : F("M1 UP");
  return dir == MOVE_FWD ? F("M3 OUT") : F("M3 HOME");
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
