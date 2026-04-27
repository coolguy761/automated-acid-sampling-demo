/*
  Motor 3 Bare Test - AccelStepper Version
  ----------------------------------------
  Purpose:
  - Test only Motor 3
  - No Nextion
  - No RTC
  - Auto-runs on power-up after a short delay

  Optional Serial commands at 115200:
  - M3F
  - M3R
  - START
  - STOP
  - HELP
*/

#include <AccelStepper.h>
#include <ctype.h>
#include <string.h>

const uint8_t PIN_M3_STEP = 6;
const uint8_t PIN_M3_DIR  = 7;
const uint8_t PIN_M3_ENA  = 8;

const uint8_t M3_ENA_ENABLED_LEVEL  = LOW;
const uint8_t M3_ENA_DISABLED_LEVEL = HIGH;
const bool M3_FORWARD_IS_TILT_OUT   = true;

const float M3_MAX_SPEED = 40.0f;
const float M3_ACCEL     = 20.0f;

const long STEP_M3_OUT_STEPS  = 360L;
const long STEP_M3_HOME_STEPS = 360L;
const long MANUAL_JOG_STEPS   = 200000L;
const unsigned long SETTLE_MS = 500UL;
const unsigned long AUTO_START_DELAY_MS = 3000UL;

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
  Direction dir;
  long steps;
};

const AutoStep TEST_CYCLE[] = {
  { MOVE_FWD, STEP_M3_OUT_STEPS },
  { MOVE_REV, STEP_M3_HOME_STEPS }
};

const uint8_t TEST_STEP_COUNT = sizeof(TEST_CYCLE) / sizeof(TEST_CYCLE[0]);

AccelStepper stepperM3(AccelStepper::DRIVER, PIN_M3_STEP, PIN_M3_DIR);

Direction activeDir = MOVE_NONE;
RunState runState = ST_IDLE;
unsigned long stateStartMs = 0;
unsigned long bootMs = 0;
uint8_t autoStepIndex = 0;
bool autoStartPending = true;
char serialLine[24];
uint8_t serialLen = 0;

void printHelp();
void setupStepper();
void stopMotion();
void startDirection(Direction dir);
void serviceMotion();
void serviceAutoCycle();
void beginAutoCycle();
void startAutoStep(uint8_t index);
bool invertDir();
const __FlashStringHelper *labelFor(Direction dir);
void handleCommand(char *cmd);

void setup() {
  Serial.begin(115200);

  pinMode(PIN_M3_ENA, OUTPUT);
  digitalWrite(PIN_M3_ENA, M3_ENA_DISABLED_LEVEL);

  setupStepper();
  stopMotion();
  bootMs = millis();

  Serial.println(F("MOTOR 3 ACCELSTEPPER TEST READY"));
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
  Serial.println(F("Commands: M3F M3R START STOP HELP"));
  Serial.println(F("Tune M3 with M3_MAX_SPEED and M3_ACCEL near the top."));
}

void setupStepper() {
  stepperM3.setEnablePin(PIN_M3_ENA);
  stepperM3.setPinsInverted(invertDir(), false, M3_ENA_ENABLED_LEVEL == HIGH);
  stepperM3.setMaxSpeed(M3_MAX_SPEED);
  stepperM3.setAcceleration(M3_ACCEL);
  stepperM3.disableOutputs();
}

void stopMotion() {
  stepperM3.stop();
  stepperM3.moveTo(stepperM3.currentPosition());
  stepperM3.disableOutputs();
  digitalWrite(PIN_M3_ENA, M3_ENA_DISABLED_LEVEL);
  activeDir = MOVE_NONE;
  runState = ST_IDLE;
}

void startDirection(Direction dir) {
  stopMotion();
  activeDir = dir;
  stepperM3.enableOutputs();
  digitalWrite(PIN_M3_ENA, M3_ENA_ENABLED_LEVEL);
  stepperM3.move(dir == MOVE_FWD ? MANUAL_JOG_STEPS : -MANUAL_JOG_STEPS);
  Serial.println(labelFor(dir));
}

void serviceMotion() {
  stepperM3.run();
}

void serviceAutoCycle() {
  unsigned long now = millis();

  if (runState == ST_AUTO_RUN) {
    if (stepperM3.distanceToGo() == 0) {
      Serial.print(F("STEP "));
      Serial.print(autoStepIndex + 1);
      Serial.println(F(" DONE"));
      stopMotion();
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
  stopMotion();
  activeDir = TEST_CYCLE[index].dir;
  stepperM3.enableOutputs();
  digitalWrite(PIN_M3_ENA, M3_ENA_ENABLED_LEVEL);
  stepperM3.move(activeDir == MOVE_FWD ? TEST_CYCLE[index].steps : -TEST_CYCLE[index].steps);
  runState = ST_AUTO_RUN;
  stateStartMs = millis();
  Serial.println(labelFor(activeDir));
}

bool invertDir() {
  return !M3_FORWARD_IS_TILT_OUT;
}

const __FlashStringHelper *labelFor(Direction dir) {
  return dir == MOVE_FWD ? F("M3 OUT") : F("M3 HOME");
}

void handleCommand(char *cmd) {
  for (char *p = cmd; *p; ++p) *p = (char)toupper((unsigned char)*p);

  if (!strcmp(cmd, "M3F")) {
    startDirection(MOVE_FWD);
  } else if (!strcmp(cmd, "M3R")) {
    startDirection(MOVE_REV);
  } else if (!strcmp(cmd, "START")) {
    beginAutoCycle();
  } else if (!strcmp(cmd, "STOP")) {
    stopMotion();
    Serial.println(F("STOPPED"));
  } else if (!strcmp(cmd, "HELP")) {
    printHelp();
  } else {
    Serial.println(F("UNKNOWN COMMAND"));
    printHelp();
  }
}
