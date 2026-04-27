/*
  Automated Acid Sampling Demo - Two Arduino Version
  --------------------------------------------------
  CHILD UNO

  Responsibilities:
    - Cup actuator control (open / close)
    - Tilt actuator control (dump / home)
    - I2C slave interface for Parent Uno commands + status
    - Limit health validation using SPDT switches
    - Local fault latching and recovery to a safe home position

  Wiring model for motion limits:
    - Each end-of-travel switch is an SPDT switch.
    - COM -> GND
    - NO  -> "reached" pin
    - NC  -> "healthy/not reached" pin
    - All inputs use INPUT_PULLUP.

  Valid combinations:
    - NO=HIGH, NC=LOW  -> NOT_REACHED
    - NO=LOW,  NC=HIGH -> REACHED
    - NO=HIGH, NC=HIGH -> FAULT_OPEN
    - NO=LOW,  NC=LOW  -> FAULT_SHORT
*/

#include <Wire.h>

// -----------------------------
// Child Uno pin map
// -----------------------------
const uint8_t PIN_PARENT_RESET_REQ  = 2;   // Parent holds HIGH, drives LOW to request recovery

const uint8_t PIN_CUP_OPEN_CMD      = 3;   // Cup actuator open direction
const uint8_t PIN_CUP_CLOSE_CMD     = 4;   // Cup actuator close direction
const uint8_t PIN_CUP_EN            = 5;   // Cup actuator enable
const uint8_t PIN_TILT_DUMP_CMD     = 6;   // Tilt actuator dump direction
const uint8_t PIN_TILT_HOME_CMD     = 7;   // Tilt actuator home direction
const uint8_t PIN_TILT_EN           = 8;   // Tilt actuator enable

const uint8_t PIN_CUP_OPEN_REACHED  = 9;   // Cup open limit NO contact
const uint8_t PIN_CUP_OPEN_HEALTHY  = 10;  // Cup open limit NC contact
const uint8_t PIN_CUP_CLS_REACHED   = 11;  // Cup closed limit NO contact
const uint8_t PIN_CUP_CLS_HEALTHY   = 12;  // Cup closed limit NC contact
const uint8_t PIN_TILT_DMP_REACHED  = 13;  // Tilt dump limit NO contact
const uint8_t PIN_TILT_DMP_HEALTHY  = A0;  // Tilt dump limit NC contact
const uint8_t PIN_TILT_HOM_REACHED  = A1;  // Tilt home limit NO contact
const uint8_t PIN_TILT_HOM_HEALTHY  = A2;  // Tilt home limit NC contact

// Child Uno I2C address
const uint8_t CHILD_ADDR = 0x08;

// -----------------------------
// Timing configuration
// -----------------------------
const unsigned long CUP_MOVE_TIMEOUT_MS   = 5000UL;
const unsigned long TILT_MOVE_TIMEOUT_MS  = 5000UL;
const unsigned long RECOVERY_TIMEOUT_MS   = 9000UL;
const unsigned long DEBUG_PRINT_MS        = 1000UL;

// -----------------------------
// Parent <-> Child contract
// -----------------------------
enum ChildAction {
  CA_NONE = 0,
  CA_OPEN_CUP,
  CA_CLOSE_CUP,
  CA_TILT_DUMP,
  CA_UNTILT_HOME,
  CA_STOP_ALL
};

enum ChildState {
  CS_IDLE = 0,
  CS_OPENING,
  CS_CLOSING,
  CS_TILTING,
  CS_UNTILTING,
  CS_RECOVER_CLOSING,
  CS_RECOVER_UNTILTING,
  CS_FAULT
};

struct ChildStatus {
  uint8_t seq;
  uint8_t busy;
  uint8_t fault;
  uint8_t state;
  uint8_t cupOpen;
  uint8_t cupClosed;
  uint8_t tiltDump;
  uint8_t tiltHome;
  uint8_t limitsValid;
};

// Keep the byte layout aligned with the parent sketch.
ChildStatus statusPacket = {0, 0, 0, 0, 0, 1, 0, 1, 0};

enum LimitPairState {
  LP_NOT_REACHED = 0,
  LP_REACHED,
  LP_FAULT_OPEN,
  LP_FAULT_SHORT
};

ChildState currentState = CS_IDLE;
bool faultLatched = false;
char faultMsg[40] = "OK";
unsigned long stateStartMs = 0;
unsigned long lastDebugMs = 0;

volatile uint8_t pendingCommand = CA_NONE;
volatile bool commandPending = false;

bool lastResetReqLow = false;

// -----------------------------
// Function declarations
// -----------------------------
void setState(ChildState nextState);
void serviceStateMachine();
void allOutputsOff();
void driveCupOpen(bool on);
void driveCupClose(bool on);
void driveTiltDump(bool on);
void driveTiltHome(bool on);

LimitPairState readLimitPair(uint8_t reachedPin, uint8_t healthyPin);
LimitPairState cupOpenState();
LimitPairState cupClosedState();
LimitPairState tiltDumpState();
LimitPairState tiltHomeState();
bool limitPairIsHealthy(LimitPairState state);
bool limitPairIsReached(LimitPairState state);
bool limitsAreConsistent();
bool readyForStart();
bool safeHomeReached();

void refreshStatusPacket();
void bumpStatusSeq();
void acknowledgeStatus();
void faultNow(const __FlashStringHelper *msg);
void faultNow(const char *msg);
void clearFault();
void startRecovery();
void processPendingCommand();
void receiveEvent(int count);
void requestEvent();
const __FlashStringHelper* stateName(ChildState state);

// -----------------------------
// Setup
// -----------------------------
void setup() {
  Serial.begin(115200);

  pinMode(PIN_PARENT_RESET_REQ, INPUT_PULLUP);

  pinMode(PIN_CUP_OPEN_CMD, OUTPUT);
  pinMode(PIN_CUP_CLOSE_CMD, OUTPUT);
  pinMode(PIN_CUP_EN, OUTPUT);
  pinMode(PIN_TILT_DUMP_CMD, OUTPUT);
  pinMode(PIN_TILT_HOME_CMD, OUTPUT);
  pinMode(PIN_TILT_EN, OUTPUT);

  pinMode(PIN_CUP_OPEN_REACHED, INPUT_PULLUP);
  pinMode(PIN_CUP_OPEN_HEALTHY, INPUT_PULLUP);
  pinMode(PIN_CUP_CLS_REACHED, INPUT_PULLUP);
  pinMode(PIN_CUP_CLS_HEALTHY, INPUT_PULLUP);
  pinMode(PIN_TILT_DMP_REACHED, INPUT_PULLUP);
  pinMode(PIN_TILT_DMP_HEALTHY, INPUT_PULLUP);
  pinMode(PIN_TILT_HOM_REACHED, INPUT_PULLUP);
  pinMode(PIN_TILT_HOM_HEALTHY, INPUT_PULLUP);

  allOutputsOff();
  refreshStatusPacket();

  Wire.begin(CHILD_ADDR);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);

  setState(CS_IDLE);
  refreshStatusPacket();
  bumpStatusSeq();

  Serial.println(F("CHILD BOOT"));
}

// -----------------------------
// Main loop
// -----------------------------
void loop() {
  bool resetLow = (digitalRead(PIN_PARENT_RESET_REQ) == LOW);
  if (resetLow && !lastResetReqLow) {
    startRecovery();
  }
  lastResetReqLow = resetLow;

  processPendingCommand();
  serviceStateMachine();
  refreshStatusPacket();

  if (millis() - lastDebugMs >= DEBUG_PRINT_MS) {
    lastDebugMs = millis();
    Serial.print(F("State: "));
    Serial.print(stateName(currentState));
    Serial.print(F(" | Fault: "));
    Serial.print(faultLatched ? faultMsg : "NO");
    Serial.print(F(" | Limits: "));
    Serial.println(statusPacket.limitsValid ? F("OK") : F("BAD"));
  }
}

// -----------------------------
// Core state handling
// -----------------------------
void setState(ChildState nextState) {
  if (currentState == nextState) return;
  currentState = nextState;
  stateStartMs = millis();
  refreshStatusPacket();
  bumpStatusSeq();
}

void serviceStateMachine() {
  unsigned long elapsed = millis() - stateStartMs;

  if (!limitsAreConsistent()) {
    faultNow(F("LIMIT FAULT"));
    return;
  }

  switch (currentState) {
    case CS_IDLE:
      allOutputsOff();
      break;

    case CS_OPENING:
      if (limitPairIsReached(cupOpenState())) {
        driveCupOpen(false);
        setState(CS_IDLE);
      } else if (elapsed > CUP_MOVE_TIMEOUT_MS) {
        faultNow(F("OPEN TIMEOUT"));
      } else {
        driveCupOpen(true);
      }
      break;

    case CS_CLOSING:
      if (limitPairIsReached(cupClosedState())) {
        driveCupClose(false);
        setState(CS_IDLE);
      } else if (elapsed > CUP_MOVE_TIMEOUT_MS) {
        faultNow(F("CLOSE TIMEOUT"));
      } else {
        driveCupClose(true);
      }
      break;

    case CS_TILTING:
      if (limitPairIsReached(tiltDumpState())) {
        driveTiltDump(false);
        setState(CS_IDLE);
      } else if (elapsed > TILT_MOVE_TIMEOUT_MS) {
        faultNow(F("TILT TIMEOUT"));
      } else {
        driveTiltDump(true);
      }
      break;

    case CS_UNTILTING:
      if (limitPairIsReached(tiltHomeState())) {
        driveTiltHome(false);
        setState(CS_IDLE);
      } else if (elapsed > TILT_MOVE_TIMEOUT_MS) {
        faultNow(F("HOME TIMEOUT"));
      } else {
        driveTiltHome(true);
      }
      break;

    case CS_RECOVER_CLOSING:
      if (safeHomeReached()) {
        clearFault();
        setState(CS_IDLE);
      } else if (limitPairIsReached(cupClosedState())) {
        driveCupClose(false);
        setState(CS_RECOVER_UNTILTING);
      } else if (elapsed > RECOVERY_TIMEOUT_MS) {
        faultNow(F("RECLOSE TIMEOUT"));
      } else {
        driveCupClose(true);
      }
      break;

    case CS_RECOVER_UNTILTING:
      if (safeHomeReached()) {
        clearFault();
        setState(CS_IDLE);
      } else if (limitPairIsReached(tiltHomeState())) {
        driveTiltHome(false);
        clearFault();
        setState(CS_IDLE);
      } else if (elapsed > RECOVERY_TIMEOUT_MS) {
        faultNow(F("RHOME TIMEOUT"));
      } else {
        driveTiltHome(true);
      }
      break;

    case CS_FAULT:
      allOutputsOff();
      break;
  }
}

void processPendingCommand() {
  if (!commandPending) return;

  noInterrupts();
  uint8_t action = pendingCommand;
  pendingCommand = CA_NONE;
  commandPending = false;
  interrupts();

  if (action == CA_STOP_ALL) {
    allOutputsOff();
    if (faultLatched) {
      setState(CS_FAULT);
    } else {
      setState(CS_IDLE);
      acknowledgeStatus();
    }
    return;
  }

  if (!limitsAreConsistent()) {
    faultNow(F("LIMIT FAULT"));
    return;
  }

  if (faultLatched || currentState == CS_RECOVER_CLOSING || currentState == CS_RECOVER_UNTILTING) {
    return;
  }

  switch (action) {
    case CA_OPEN_CUP:
      if (limitPairIsReached(cupOpenState())) {
        acknowledgeStatus();
      } else {
        setState(CS_OPENING);
      }
      break;

    case CA_CLOSE_CUP:
      if (limitPairIsReached(cupClosedState())) {
        acknowledgeStatus();
      } else {
        setState(CS_CLOSING);
      }
      break;

    case CA_TILT_DUMP:
      if (limitPairIsReached(tiltDumpState())) {
        acknowledgeStatus();
      } else {
        setState(CS_TILTING);
      }
      break;

    case CA_UNTILT_HOME:
      if (limitPairIsReached(tiltHomeState())) {
        acknowledgeStatus();
      } else {
        setState(CS_UNTILTING);
      }
      break;

    default:
      break;
  }
}

// -----------------------------
// Fault / recovery helpers
// -----------------------------
void faultNow(const __FlashStringHelper *msg) {
  char buf[40];
  strncpy_P(buf, (PGM_P)msg, sizeof(buf) - 1);
  buf[sizeof(buf) - 1] = '\0';
  faultNow(buf);
}

void faultNow(const char *msg) {
  allOutputsOff();
  faultLatched = true;
  strncpy(faultMsg, msg, sizeof(faultMsg) - 1);
  faultMsg[sizeof(faultMsg) - 1] = '\0';
  setState(CS_FAULT);
}

void clearFault() {
  faultLatched = false;
  strncpy(faultMsg, "OK", sizeof(faultMsg) - 1);
  faultMsg[sizeof(faultMsg) - 1] = '\0';
}

void startRecovery() {
  allOutputsOff();
  if (!limitsAreConsistent()) {
    faultNow(F("LIMIT FAULT"));
    return;
  }

  if (safeHomeReached()) {
    clearFault();
    setState(CS_IDLE);
    return;
  }

  if (!limitPairIsReached(cupClosedState())) {
    setState(CS_RECOVER_CLOSING);
  } else {
    setState(CS_RECOVER_UNTILTING);
  }
}

// -----------------------------
// Output helpers
// -----------------------------
void allOutputsOff() {
  digitalWrite(PIN_CUP_OPEN_CMD, LOW);
  digitalWrite(PIN_CUP_CLOSE_CMD, LOW);
  digitalWrite(PIN_CUP_EN, LOW);
  digitalWrite(PIN_TILT_DUMP_CMD, LOW);
  digitalWrite(PIN_TILT_HOME_CMD, LOW);
  digitalWrite(PIN_TILT_EN, LOW);
}

void driveCupOpen(bool on) {
  digitalWrite(PIN_CUP_CLOSE_CMD, LOW);
  digitalWrite(PIN_TILT_DUMP_CMD, LOW);
  digitalWrite(PIN_TILT_HOME_CMD, LOW);
  digitalWrite(PIN_TILT_EN, LOW);
  digitalWrite(PIN_CUP_OPEN_CMD, on ? HIGH : LOW);
  digitalWrite(PIN_CUP_EN, on ? HIGH : LOW);
}

void driveCupClose(bool on) {
  digitalWrite(PIN_CUP_OPEN_CMD, LOW);
  digitalWrite(PIN_TILT_DUMP_CMD, LOW);
  digitalWrite(PIN_TILT_HOME_CMD, LOW);
  digitalWrite(PIN_TILT_EN, LOW);
  digitalWrite(PIN_CUP_CLOSE_CMD, on ? HIGH : LOW);
  digitalWrite(PIN_CUP_EN, on ? HIGH : LOW);
}

void driveTiltDump(bool on) {
  digitalWrite(PIN_CUP_OPEN_CMD, LOW);
  digitalWrite(PIN_CUP_CLOSE_CMD, LOW);
  digitalWrite(PIN_CUP_EN, LOW);
  digitalWrite(PIN_TILT_HOME_CMD, LOW);
  digitalWrite(PIN_TILT_DUMP_CMD, on ? HIGH : LOW);
  digitalWrite(PIN_TILT_EN, on ? HIGH : LOW);
}

void driveTiltHome(bool on) {
  digitalWrite(PIN_CUP_OPEN_CMD, LOW);
  digitalWrite(PIN_CUP_CLOSE_CMD, LOW);
  digitalWrite(PIN_CUP_EN, LOW);
  digitalWrite(PIN_TILT_DUMP_CMD, LOW);
  digitalWrite(PIN_TILT_HOME_CMD, on ? HIGH : LOW);
  digitalWrite(PIN_TILT_EN, on ? HIGH : LOW);
}

// -----------------------------
// Limit helpers
// -----------------------------
LimitPairState readLimitPair(uint8_t reachedPin, uint8_t healthyPin) {
  bool noHigh = (digitalRead(reachedPin) == HIGH);
  bool ncHigh = (digitalRead(healthyPin) == HIGH);

  if (noHigh && !ncHigh) return LP_NOT_REACHED;
  if (!noHigh && ncHigh) return LP_REACHED;
  if (noHigh && ncHigh) return LP_FAULT_OPEN;
  return LP_FAULT_SHORT;
}

LimitPairState cupOpenState() {
  return readLimitPair(PIN_CUP_OPEN_REACHED, PIN_CUP_OPEN_HEALTHY);
}

LimitPairState cupClosedState() {
  return readLimitPair(PIN_CUP_CLS_REACHED, PIN_CUP_CLS_HEALTHY);
}

LimitPairState tiltDumpState() {
  return readLimitPair(PIN_TILT_DMP_REACHED, PIN_TILT_DMP_HEALTHY);
}

LimitPairState tiltHomeState() {
  return readLimitPair(PIN_TILT_HOM_REACHED, PIN_TILT_HOM_HEALTHY);
}

bool limitPairIsHealthy(LimitPairState state) {
  return state == LP_NOT_REACHED || state == LP_REACHED;
}

bool limitPairIsReached(LimitPairState state) {
  return state == LP_REACHED;
}

bool limitsAreConsistent() {
  LimitPairState cOpen = cupOpenState();
  LimitPairState cClosed = cupClosedState();
  LimitPairState tDump = tiltDumpState();
  LimitPairState tHome = tiltHomeState();

  if (!limitPairIsHealthy(cOpen) ||
      !limitPairIsHealthy(cClosed) ||
      !limitPairIsHealthy(tDump) ||
      !limitPairIsHealthy(tHome)) {
    return false;
  }

  if (limitPairIsReached(cOpen) && limitPairIsReached(cClosed)) return false;
  if (limitPairIsReached(tDump) && limitPairIsReached(tHome)) return false;
  return true;
}

bool readyForStart() {
  return !faultLatched &&
         currentState == CS_IDLE &&
         limitsAreConsistent() &&
         limitPairIsReached(cupClosedState()) &&
         limitPairIsReached(tiltHomeState());
}

bool safeHomeReached() {
  return limitsAreConsistent() &&
         limitPairIsReached(cupClosedState()) &&
         limitPairIsReached(tiltHomeState());
}

// -----------------------------
// I2C callbacks
// -----------------------------
void receiveEvent(int count) {
  while (Wire.available() > 0) {
    uint8_t action = Wire.read();
    pendingCommand = action;
    commandPending = true;
  }
}

void requestEvent() {
  refreshStatusPacket();
  Wire.write((const uint8_t*)&statusPacket, sizeof(statusPacket));
}

// -----------------------------
// Status helpers
// -----------------------------
void refreshStatusPacket() {
  bool limitsValid = limitsAreConsistent();
  bool busy = !(currentState == CS_IDLE || currentState == CS_FAULT);

  statusPacket.busy       = busy ? 1 : 0;
  statusPacket.fault      = faultLatched ? 1 : 0;
  statusPacket.state      = (uint8_t)currentState;
  statusPacket.cupOpen    = limitPairIsReached(cupOpenState()) ? 1 : 0;
  statusPacket.cupClosed  = limitPairIsReached(cupClosedState()) ? 1 : 0;
  statusPacket.tiltDump   = limitPairIsReached(tiltDumpState()) ? 1 : 0;
  statusPacket.tiltHome   = limitPairIsReached(tiltHomeState()) ? 1 : 0;
  statusPacket.limitsValid = limitsValid ? 1 : 0;

  if (!limitsValid && !faultLatched) {
    statusPacket.fault = 1;
  }

  if (readyForStart()) {
    statusPacket.busy = 0;
    statusPacket.fault = 0;
  }
}

void bumpStatusSeq() {
  statusPacket.seq++;
}

void acknowledgeStatus() {
  refreshStatusPacket();
  bumpStatusSeq();
}

// -----------------------------
// Utility / debug helpers
// -----------------------------
const __FlashStringHelper* stateName(ChildState state) {
  switch (state) {
    case CS_IDLE:             return F("IDLE");
    case CS_OPENING:          return F("OPENING");
    case CS_CLOSING:          return F("CLOSING");
    case CS_TILTING:          return F("TILTING");
    case CS_UNTILTING:        return F("UNTILTING");
    case CS_RECOVER_CLOSING:  return F("RCLOSE");
    case CS_RECOVER_UNTILTING:return F("RUNTILT");
    case CS_FAULT:            return F("FAULT");
    default:                  return F("UNKNOWN");
  }
}
