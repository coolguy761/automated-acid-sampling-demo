/*
  Motor 2 Half-Rotation Test - Uno
  --------------------------------
  Purpose:
  - Test only Motor 2
  - Move exactly half a revolution using counted pulses
  - Auto-run on power-up after a short delay

  Notes:
  - Set STEPS_PER_REV to match your motor + driver microstepping.
  - If your driver is set to 1/8 microstepping and the motor is 200 full
    steps/rev, then STEPS_PER_REV should be 1600.
*/

const uint8_t PIN_M2_STEP = 4;
const uint8_t PIN_M2_DIR  = 5;
const uint8_t PIN_M2_ENA  = 8;

const uint8_t DIR_FWD_LEVEL         = HIGH;
const uint8_t DIR_REV_LEVEL         = LOW;
const uint8_t M2_ENA_ENABLED_LEVEL  = LOW;
const uint8_t M2_ENA_DISABLED_LEVEL = HIGH;

const bool M2_FORWARD_IS_OPEN = true;
const bool RETURN_HOME_AFTER_TEST = true;

const unsigned int STEPS_PER_REV = 200;
const unsigned int HALF_ROTATION_STEPS = STEPS_PER_REV / 2;

const unsigned long STEP_INTERVAL_US = 2500UL;
const unsigned long STEP_PULSE_US    = 10UL;
const unsigned long AUTO_START_DELAY_MS = 3000UL;
const unsigned long SETTLE_MS = 1000UL;

enum MoveState {
  ST_BOOT_WAIT = 0,
  ST_MOVE_FWD,
  ST_SETTLE,
  ST_MOVE_REV,
  ST_DONE
};

MoveState moveState = ST_BOOT_WAIT;
bool stepHigh = false;
bool returnLegDone = false;
unsigned long bootMs = 0;
unsigned long stateStartMs = 0;
unsigned long lastStepUs = 0;
unsigned int stepsTaken = 0;

void stopMotor();
void startMove(bool forward);
void serviceMotion();
void advanceStateAfterMove();
uint8_t dirLevelFor(bool forward);

void setup() {
  pinMode(PIN_M2_STEP, OUTPUT);
  pinMode(PIN_M2_DIR, OUTPUT);
  pinMode(PIN_M2_ENA, OUTPUT);

  digitalWrite(PIN_M2_STEP, LOW);
  digitalWrite(PIN_M2_DIR, DIR_FWD_LEVEL);
  digitalWrite(PIN_M2_ENA, M2_ENA_DISABLED_LEVEL);

  bootMs = millis();
}

void loop() {
  unsigned long nowMs = millis();

  if (moveState == ST_BOOT_WAIT && nowMs - bootMs >= AUTO_START_DELAY_MS) {
    startMove(true);
  }

  serviceMotion();

  if (moveState == ST_SETTLE && nowMs - stateStartMs >= SETTLE_MS) {
    if (RETURN_HOME_AFTER_TEST && !returnLegDone) {
      returnLegDone = true;
      startMove(false);
    } else {
      moveState = ST_DONE;
      stopMotor();
    }
  }
}

void stopMotor() {
  digitalWrite(PIN_M2_STEP, LOW);
  digitalWrite(PIN_M2_ENA, M2_ENA_DISABLED_LEVEL);
  stepHigh = false;
}

void startMove(bool forward) {
  digitalWrite(PIN_M2_DIR, dirLevelFor(forward));
  digitalWrite(PIN_M2_ENA, M2_ENA_ENABLED_LEVEL);
  stepHigh = false;
  stepsTaken = 0;
  lastStepUs = micros();
  moveState = forward ? ST_MOVE_FWD : ST_MOVE_REV;
}

void serviceMotion() {
  if (moveState != ST_MOVE_FWD && moveState != ST_MOVE_REV) return;

  unsigned long nowUs = micros();

  if (!stepHigh) {
    if ((unsigned long)(nowUs - lastStepUs) >= STEP_INTERVAL_US) {
      digitalWrite(PIN_M2_STEP, HIGH);
      stepHigh = true;
      lastStepUs = nowUs;
    }
  } else if ((unsigned long)(nowUs - lastStepUs) >= STEP_PULSE_US) {
    digitalWrite(PIN_M2_STEP, LOW);
    stepHigh = false;
    lastStepUs = nowUs;
    stepsTaken++;

    if (stepsTaken >= HALF_ROTATION_STEPS) {
      advanceStateAfterMove();
    }
  }
}

void advanceStateAfterMove() {
  stopMotor();
  moveState = ST_SETTLE;
  stateStartMs = millis();
}

uint8_t dirLevelFor(bool forward) {
  if (forward) return M2_FORWARD_IS_OPEN ? DIR_FWD_LEVEL : DIR_REV_LEVEL;
  return M2_FORWARD_IS_OPEN ? DIR_REV_LEVEL : DIR_FWD_LEVEL;
}
