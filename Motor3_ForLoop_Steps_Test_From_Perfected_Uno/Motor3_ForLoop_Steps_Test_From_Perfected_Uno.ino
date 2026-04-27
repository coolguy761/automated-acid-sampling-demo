/*
  Motor 3 For-Loop Steps Test From Perfected Setup - Uno
  ------------------------------------------------------
  Purpose:
  - Standalone M3-only test using the same for-loop idea as the
    Nextion perfected M3-for-loop version
  - No Nextion, no RTC, no extra motion logic
  - Fixed step count for repeatable tilt out and tilt home
*/

const uint8_t PIN_M3_STEP    = 6;
const uint8_t PIN_M3_DIR     = 7;
const uint8_t PIN_SHARED_ENA = 8;

const uint8_t DIR_FWD_LEVEL             = HIGH;
const uint8_t DIR_REV_LEVEL             = LOW;
const uint8_t SHARED_ENA_ENABLED_LEVEL  = LOW;
const uint8_t SHARED_ENA_DISABLED_LEVEL = HIGH;

const bool M3_FORWARD_IS_TILT_OUT = true;

const unsigned long BOOT_DELAY_MS        = 3000UL;
const unsigned long STEP_INTERVAL_M3_US  = 2800UL;
const unsigned long STEP_PULSE_US        = 10UL;
const unsigned long M3_TILT_OUT_STEPS    = 2300UL;
const unsigned long M3_TILT_HOME_STEPS   = 2300UL;
const unsigned long SETTLE_MS            = 500UL;

uint8_t dirLevelFor(bool forwardMove);
void runM3ForLoop(bool forwardMove, unsigned long stepCount);

void setup() {
  Serial.begin(115200);

  pinMode(PIN_M3_STEP, OUTPUT);
  pinMode(PIN_M3_DIR, OUTPUT);
  pinMode(PIN_SHARED_ENA, OUTPUT);

  digitalWrite(PIN_M3_STEP, LOW);
  digitalWrite(PIN_M3_DIR, DIR_FWD_LEVEL);
  digitalWrite(PIN_SHARED_ENA, SHARED_ENA_DISABLED_LEVEL);

  Serial.println(F("M3 FOR-LOOP STANDALONE TEST"));
  Serial.print(F("STEP INTERVAL US: "));
  Serial.println(STEP_INTERVAL_M3_US);
  Serial.print(F("TILT OUT STEPS: "));
  Serial.println(M3_TILT_OUT_STEPS);
  Serial.print(F("TILT HOME STEPS: "));
  Serial.println(M3_TILT_HOME_STEPS);

  delay(BOOT_DELAY_MS);

  Serial.println(F("RUN M3 OUT"));
  runM3ForLoop(true, M3_TILT_OUT_STEPS);

  delay(SETTLE_MS);

  Serial.println(F("RUN M3 HOME"));
  runM3ForLoop(false, M3_TILT_HOME_STEPS);

  Serial.println(F("TEST DONE"));
}

void loop() {
}

uint8_t dirLevelFor(bool forwardMove) {
  if (forwardMove) return M3_FORWARD_IS_TILT_OUT ? DIR_FWD_LEVEL : DIR_REV_LEVEL;
  return M3_FORWARD_IS_TILT_OUT ? DIR_REV_LEVEL : DIR_FWD_LEVEL;
}

void runM3ForLoop(bool forwardMove, unsigned long stepCount) {
  digitalWrite(PIN_SHARED_ENA, SHARED_ENA_ENABLED_LEVEL);
  digitalWrite(PIN_M3_DIR, dirLevelFor(forwardMove));
  delay(2);

  for (unsigned long i = 0; i < stepCount; ++i) {
    digitalWrite(PIN_M3_STEP, HIGH);
    delayMicroseconds(STEP_PULSE_US);
    digitalWrite(PIN_M3_STEP, LOW);
    delayMicroseconds(STEP_INTERVAL_M3_US);
  }

  digitalWrite(PIN_SHARED_ENA, SHARED_ENA_DISABLED_LEVEL);
}
