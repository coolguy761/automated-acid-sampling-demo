/*
  M1 Half Retract -> M3 Slack -> M1 Finish Retract
  ------------------------------------------------
  Purpose:
  - Bench-test a staged retract sequence without Nextion or RTC
  - Start from the extended/down position
  - Retract Motor 1 halfway
  - Run Motor 3 to pull slack out
  - Finish retracting Motor 1

  Behavior:
  - Auto-runs once, 3 seconds after power-up
  - Optional Serial Monitor output at 115200 for status only
*/

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

const unsigned long BOOT_DELAY_MS       = 3000UL;
const unsigned long STEP_INTERVAL_M1_US = 800UL;
const unsigned long STEP_INTERVAL_M3_US = 800UL;
const unsigned long STEP_PULSE_US       = 10UL;
const unsigned long STEP_M1_UP_TOTAL_MS = 8550UL;
const unsigned long STEP_M3_SLACK_MS    = 1800UL;
const unsigned long SETTLE_MS           = 300UL;

const unsigned long STEP_M1_UP_HALF_MS = STEP_M1_UP_TOTAL_MS / 2UL;
const unsigned long STEP_M1_UP_REST_MS = STEP_M1_UP_TOTAL_MS - STEP_M1_UP_HALF_MS;

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
  ST_BOOT_WAIT = 0,
  ST_AUTO_RUN,
  ST_AUTO_WAIT,
  ST_DONE
};

struct MotionStep {
  Axis axis;
  Direction dir;
  unsigned long runMs;
};

const MotionStep TEST_CYCLE[] = {
  { AXIS_M1, MOVE_REV, STEP_M1_UP_HALF_MS },
  { AXIS_M3, MOVE_REV, STEP_M3_SLACK_MS   },
  { AXIS_M1, MOVE_REV, STEP_M1_UP_REST_MS }
};

const uint8_t TEST_STEP_COUNT = sizeof(TEST_CYCLE) / sizeof(TEST_CYCLE[0]);

Axis activeAxis = AXIS_NONE;
Direction activeDir = MOVE_NONE;
RunState runState = ST_BOOT_WAIT;
bool stepHigh = false;
bool autoStarted = false;
unsigned long bootMs = 0;
unsigned long stateStartMs = 0;
unsigned long lastStepUs = 0;
uint8_t autoStepIndex = 0;

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
unsigned long intervalUsFor(Axis axis);
void printStepName(uint8_t index);

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

  bootMs = millis();

  Serial.println(F("M1 HALF RETRACT / M3 SLACK TEST"));
  Serial.print(F("M1 HALF UP MS: "));
  Serial.println(STEP_M1_UP_HALF_MS);
  Serial.print(F("M3 SLACK MS: "));
  Serial.println(STEP_M3_SLACK_MS);
  Serial.print(F("M1 FINAL UP MS: "));
  Serial.println(STEP_M1_UP_REST_MS);
}

void loop() {
  if (!autoStarted && (millis() - bootMs >= BOOT_DELAY_MS)) {
    beginAutoCycle();
    autoStarted = true;
  }

  serviceAutoCycle();
  serviceMotion();
}

void stopAllMotion() {
  activeAxis = AXIS_NONE;
  activeDir = MOVE_NONE;
  stepHigh = false;

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
}

void serviceMotion() {
  if (activeAxis == AXIS_NONE || activeDir == MOVE_NONE) return;

  const uint8_t stepPin = stepPinFor(activeAxis);
  const unsigned long intervalUs = intervalUsFor(activeAxis);
  const unsigned long now = micros();

  if (!stepHigh) {
    if ((unsigned long)(now - lastStepUs) >= intervalUs) {
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
  const unsigned long now = millis();

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
        runState = ST_DONE;
        stopAllMotion();
        Serial.println(F("TEST DONE"));
      } else {
        startAutoStep(autoStepIndex);
      }
    }
  }
}

void beginAutoCycle() {
  autoStepIndex = 0;
  Serial.println(F("START TEST"));
  startAutoStep(autoStepIndex);
}

void startAutoStep(uint8_t index) {
  printStepName(index);
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
  const bool forwardUsesFwdLevel = (axis == AXIS_M3) ? M3_FORWARD_IS_TILT_OUT : M1_FORWARD_IS_DOWN;
  if (dir == MOVE_FWD) return forwardUsesFwdLevel ? DIR_FWD_LEVEL : DIR_REV_LEVEL;
  return forwardUsesFwdLevel ? DIR_REV_LEVEL : DIR_FWD_LEVEL;
}

unsigned long intervalUsFor(Axis axis) {
  return axis == AXIS_M3 ? STEP_INTERVAL_M3_US : STEP_INTERVAL_M1_US;
}

void printStepName(uint8_t index) {
  if (index == 0) {
    Serial.println(F("STEP 1: M1 HALF UP"));
  } else if (index == 1) {
    Serial.println(F("STEP 2: M3 SLACK HOME"));
  } else {
    Serial.println(F("STEP 3: M1 FINISH UP"));
  }
}
