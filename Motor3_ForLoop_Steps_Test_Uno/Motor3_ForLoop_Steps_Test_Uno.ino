/*
  Motor 3 For-Loop Steps Test - Uno
  ---------------------------------
  Purpose:
  - Demonstrate fixed-position tilt moves using an explicit for loop
  - Send the same number of step pulses every time
  - Make the move easy to explain and show to the team

  Behavior:
  - Waits 3 seconds after power-up
  - Moves Motor 3 out by a fixed step count
  - Waits briefly
  - Moves Motor 3 home by a fixed step count
  - Stops
*/

const uint8_t PIN_M3_STEP    = 6;
const uint8_t PIN_M3_DIR     = 7;
const uint8_t PIN_SHARED_ENA = 8;

const uint8_t DIR_FWD_LEVEL             = HIGH;
const uint8_t DIR_REV_LEVEL             = LOW;
const uint8_t SHARED_ENA_ENABLED_LEVEL  = LOW;
const uint8_t SHARED_ENA_DISABLED_LEVEL = HIGH;

const bool M3_FORWARD_IS_TILT_OUT = true;

const unsigned long BOOT_DELAY_MS      = 3000UL;
const unsigned long STEP_INTERVAL_US   = 2800UL;
const unsigned long STEP_PULSE_US      = 10UL;
const unsigned long TILT_OUT_STEPS     = 2300UL;
const unsigned long TILT_HOME_STEPS    = 2300UL;
const unsigned long SETTLE_MS          = 500UL;

uint8_t dirLevelFor(bool forwardMove);
void runStepMove(bool forwardMove, unsigned long stepCount);

void setup() {
  Serial.begin(115200);

  pinMode(PIN_M3_STEP, OUTPUT);
  pinMode(PIN_M3_DIR, OUTPUT);
  pinMode(PIN_SHARED_ENA, OUTPUT);

  digitalWrite(PIN_M3_STEP, LOW);
  digitalWrite(PIN_M3_DIR, DIR_FWD_LEVEL);
  digitalWrite(PIN_SHARED_ENA, SHARED_ENA_DISABLED_LEVEL);

  Serial.println(F("M3 FOR-LOOP STEPS TEST"));
  Serial.print(F("STEP INTERVAL US: "));
  Serial.println(STEP_INTERVAL_US);
  Serial.print(F("TILT OUT STEPS: "));
  Serial.println(TILT_OUT_STEPS);
  Serial.print(F("TILT HOME STEPS: "));
  Serial.println(TILT_HOME_STEPS);

  delay(BOOT_DELAY_MS);

  Serial.println(F("RUN M3 OUT"));
  runStepMove(true, TILT_OUT_STEPS);

  delay(SETTLE_MS);

  Serial.println(F("RUN M3 HOME"));
  runStepMove(false, TILT_HOME_STEPS);

  Serial.println(F("TEST DONE"));
}

void loop() {
}

uint8_t dirLevelFor(bool forwardMove) {
  if (forwardMove) return M3_FORWARD_IS_TILT_OUT ? DIR_FWD_LEVEL : DIR_REV_LEVEL;
  return M3_FORWARD_IS_TILT_OUT ? DIR_REV_LEVEL : DIR_FWD_LEVEL;
}

void runStepMove(bool forwardMove, unsigned long stepCount) {
  digitalWrite(PIN_SHARED_ENA, SHARED_ENA_ENABLED_LEVEL);
  digitalWrite(PIN_M3_DIR, dirLevelFor(forwardMove));
  delay(2);

  for (unsigned long i = 0; i < stepCount; ++i) {
    digitalWrite(PIN_M3_STEP, HIGH);
    delayMicroseconds(STEP_PULSE_US);
    digitalWrite(PIN_M3_STEP, LOW);
    delayMicroseconds(STEP_INTERVAL_US);
  }

  digitalWrite(PIN_SHARED_ENA, SHARED_ENA_DISABLED_LEVEL);
}
