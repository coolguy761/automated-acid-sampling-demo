/*
  Automated Acid Sampling Demo - Two Arduino Version
  --------------------------------------------------
  PARENT UNO

  Responsibilities:
    - Main lift motor control (extend/retract)
    - HMI / Nextion serial communication
    - RTC time display
    - Encoder pulse counting
    - Start / Stop / Reset / E-    - Each end-of-travel switch is an SPDT switch.
Stop handling
    - Main state machine for the sampling cycle
    - Sends commands to the Child Uno over I2C

  Wiring model for motion limits:
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
#include <SoftwareSerial.h>
#include <RTClib.h>

#define USE_RTC 1
#define USE_NEXTION 0
#define USE_ENCODER 1
#define USE_PHYSICAL_BUTTONS 0

// -----------------------------
// Parent Uno pin map
// -----------------------------
const uint8_t PIN_ENC_A              = 2;   // Encoder pulse input (interrupt)
const uint8_t PIN_START_BTN          = 3;   // Physical start button
const uint8_t PIN_STOP_BTN           = 4;   // Physical stop / abort button
const uint8_t PIN_RESET_BTN          = 5;   // Physical reset button
const uint8_t PIN_ESTOP_NC           = 6;   // E-stop input (NC wiring assumption)
const uint8_t PIN_LS_EXT_REACHED     = 7;   // Lift extended limit NO contact
const uint8_t PIN_LS_RET_REACHED     = 8;   // Lift retracted/home limit NO contact
const uint8_t PIN_CHILD_RESET_REQ    = 9;   // Parent holds HIGH, pulses LOW to request child recovery

const uint8_t PIN_NX_RX              = 10;  // Arduino RX  <- Nextion TX
const uint8_t PIN_NX_TX              = 11;  // Arduino TX  -> Nextion RX

const uint8_t PIN_LIFT_FWD           = 12;  // Lift motor forward/extend direction
const uint8_t PIN_LIFT_REV           = 13;  // Lift motor reverse/retract direction
const uint8_t PIN_LIFT_EN            = A0;  // Lift motor enable
const uint8_t PIN_ALARM              = A1;  // Alarm / fault output
const uint8_t PIN_LS_EXT_HEALTHY     = A2;  // Lift extended limit NC contact
const uint8_t PIN_LS_RET_HEALTHY     = A3;  // Lift retracted/home limit NC contact

// Child Uno I2C address
const uint8_t CHILD_ADDR = 0x08;

// -----------------------------
// Timing configuration
// -----------------------------
const unsigned long EXTEND_TIMEOUT_MS        = 15000UL;
const unsigned long RETRACT_TIMEOUT_MS       = 15000UL;
const unsigned long FILL_DWELL_MS            = 3000UL;
const unsigned long DUMP_DWELL_MS            = 2000UL;
const unsigned long CHILD_ACTION_TIMEOUT_MS  = 6000UL;
const unsigned long HEARTBEAT_MS             = 250UL;
const unsigned long CLOCK_UPDATE_MS          = 1000UL;
const unsigned long PAGE_REFRESH_MS          = 300UL;
const unsigned long DEBUG_PRINT_MS           = 1000UL;
const unsigned long PAGE_QUERY_MS            = 500UL;
const unsigned long DEBOUNCE_MS              = 40UL;
const unsigned long CHILD_STATUS_STALE_MS    = 1000UL;
const unsigned long CHILD_RESET_PULSE_MS     = 100UL;

#if USE_NEXTION
SoftwareSerial nxt(PIN_NX_RX, PIN_NX_TX);
#endif

#if USE_RTC
RTC_DS3231 rtc;
bool rtcOk = false;
#endif

// Encoder pulse counter
volatile long encPulses = 0;
const long PULSES_PER_REV = 20;

// Simple in-memory rolling event log
const size_t MAX_LOG_LINES = 2;
const size_t MAX_LOG_LINE_CHARS = 40;
char logLines[MAX_LOG_LINES][MAX_LOG_LINE_CHARS];
size_t logCount = 0;
bool logDirty = true;

// -----------------------------
// Child status packet
// -----------------------------
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

// -----------------------------
// Debounced button tracking
// -----------------------------
struct DebouncedInput {
  uint8_t pin;
  bool stableState;
  bool lastRead;
  unsigned long lastChangeMs;
};

enum LimitPairState {
  LP_NOT_REACHED = 0,
  LP_REACHED,
  LP_FAULT_OPEN,
  LP_FAULT_SHORT
};

// -----------------------------
// Parent state machine
// -----------------------------
enum State {
  ST_IDLE = 0,
  ST_EXTENDING_LIFT,
  ST_OPENING_CUP_AT_DEPTH,
  ST_FILL_DWELL,
  ST_CLOSING_CUP_AT_DEPTH,
  ST_RETRACTING_LIFT,
  ST_TILTING_TO_DUMP,
  ST_OPENING_CUP_TO_DUMP,
  ST_DUMP_DWELL,
  ST_CLOSING_CUP_AFTER_DUMP,
  ST_UNTILTING_HOME,
  ST_COMPLETE,
  ST_RECOVERING_CHILD,
  ST_RECOVERING_LIFT,
  ST_FAULT
};

// Commands sent from Parent -> Child
enum ChildAction {
  CA_NONE = 0,
  CA_OPEN_CUP,
  CA_CLOSE_CUP,
  CA_TILT_DUMP,
  CA_UNTILT_HOME,
  CA_STOP_ALL
};

State currentState = ST_IDLE;
ChildAction pendingChildAction = CA_NONE;
ChildStatus childStatus = {0, 0, 0, 0, 0, 1, 0, 1, 1};

bool childStatusValid = false;
bool childSeqSeen = false;
uint8_t lastChildSeq = 0;
unsigned long lastChildGoodPollMs = 0;
bool pendingChildFresh = false;

#if USE_PHYSICAL_BUTTONS
DebouncedInput dbStart = {PIN_START_BTN, HIGH, HIGH, 0};
DebouncedInput dbStop  = {PIN_STOP_BTN,  HIGH, HIGH, 0};
DebouncedInput dbReset = {PIN_RESET_BTN, HIGH, HIGH, 0};
#endif

unsigned long stateStartMs = 0;
unsigned long cycleStartMs = 0;
unsigned long lastHeartbeatMs = 0;
unsigned long lastClockMs = 0;
unsigned long lastPageRefreshMs = 0;
unsigned long lastPageQueryMs = 0;
unsigned long lastDebugMs = 0;
unsigned long childResetPulseStartMs = 0;

bool childResetPulseActive = false;
bool cycleActive = false;
bool faultLatched = false;
char faultMsg[32] = "";
enum VisiblePage { VP_MAIN, VP_LOGS, VP_DIAG };
VisiblePage visiblePage = VP_MAIN;

// -----------------------------
// Function declarations
// -----------------------------
void logEvent(const String &msg);
void enterState(State s);
void handleState();
void faultNow(const String &msg);
void completeRecovery();
void allOutputsOff();
void updateHMI(bool force = false);
void updateMainPage(bool force = false);
void updateDiagPage(bool force = false);
void updateLogPage(bool force = false);
void updateAllClocks();
void readNextion();
void setCurrentPageFromId(uint8_t pageId);
void handleNxLine(String line);
void nxCmd(const String &cmd);
void nxSetTxt(const String &obj, const String &txt);
String timeHHMMSS();
String stateName(State s);
String childStateName(uint8_t s);
String limitPairName(LimitPairState s);

void liftForward(bool on);
void liftReverse(bool on);
void setLiftEnable(bool on);

bool estopActive();
LimitPairState readLimitPair(uint8_t reachedPin, uint8_t healthyPin);
LimitPairState liftExtendState();
LimitPairState liftRetractState();
bool limitPairIsHealthy(LimitPairState state);
bool limitPairIsReached(LimitPairState state);

long getEncoderPulses();
void encISR();

void pollPhysicalButtons();
bool updateDebounced(DebouncedInput &db);
void onStartPressed();
void onStopPressed();
void onResetPressed();

bool sendChildCommand(ChildAction action);
bool pollChildStatus();
bool childReadyForNextAction();
bool childFaulted();
bool childStartReady();
bool childFreshnessRequired();
bool childRecoveryReady();
void beginChildResetPulse();

// -----------------------------
// Setup
// -----------------------------
void setup() {
  Serial.begin(115200);
  Serial.println("SETUP 1");

  // Inputs
#if USE_PHYSICAL_BUTTONS
  pinMode(PIN_START_BTN, INPUT_PULLUP);
  pinMode(PIN_STOP_BTN, INPUT_PULLUP);
  pinMode(PIN_RESET_BTN, INPUT_PULLUP);
#endif
  pinMode(PIN_ESTOP_NC, INPUT_PULLUP);
  pinMode(PIN_LS_EXT_REACHED, INPUT_PULLUP);
  pinMode(PIN_LS_RET_REACHED, INPUT_PULLUP);
  pinMode(PIN_LS_EXT_HEALTHY, INPUT_PULLUP);
  pinMode(PIN_LS_RET_HEALTHY, INPUT_PULLUP);

  // Outputs
  pinMode(PIN_CHILD_RESET_REQ, OUTPUT);
  digitalWrite(PIN_CHILD_RESET_REQ, HIGH);

  pinMode(PIN_LIFT_FWD, OUTPUT);
  pinMode(PIN_LIFT_REV, OUTPUT);
  pinMode(PIN_LIFT_EN, OUTPUT);
  pinMode(PIN_ALARM, OUTPUT);

  allOutputsOff();

Serial.println("SETUP 2");
#if USE_NEXTION
  nxt.begin(9600);
  Serial.println("SETUP 3");
  // nxCmd("bkcmd=0");
  // nxCmd("sendxy=0");
  // nxCmd("sendme");
  Serial.println("SETUP 4");
#endif

#if USE_RTC
  Serial.println("SETUP 5");
  if (rtc.begin()) {
    rtcOk = true;
    Serial.println("SETUP 6 RTC OK");
    if (rtc.lostPower()) {
      Serial.println(F("RTC lost power. Set time once using rtc.adjust(...), then comment it back out."));
      // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }
  } else {
    rtcOk = false;
    Serial.println(F("RTC not found."));
  }
#endif

#if USE_ENCODER
  Serial.println("SETUP 7");
  pinMode(PIN_ENC_A, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), encISR, RISING);
#endif

  Wire.begin();
  Serial.println("SETUP 8");
  logEvent("PARENT BOOT");

  if (!pollChildStatus()) {
    logEvent("SETUP 10 CHILD NO RESPONSE");
  } else if (!childStatus.limitsValid) {
    logEvent("SETUP 10 CHILD OK");
  } else {
    logEvent("CHILD ONLINE");
  }
  Serial.println("SETUP 11 DONE");
  enterState(ST_IDLE);
}

// -----------------------------
// Main loop
// -----------------------------
void loop() {
#if USE_PHYSICAL_BUTTONS
  pollPhysicalButtons();
#endif

  static unsigned long lastBeat = 0;
if (millis() - lastBeat > 1000) {
  lastBeat = millis();
  Serial.println("LOOP ALIVE");
}


  if (estopActive() && currentState != ST_FAULT) {
    faultNow("E-STOP ACTIVE");
  }

  if (millis() - lastHeartbeatMs >= HEARTBEAT_MS) {
    lastHeartbeatMs = millis();
    pollChildStatus();
  }

  if (childFaulted() &&
      currentState != ST_FAULT &&
      currentState != ST_RECOVERING_CHILD) {
    faultNow("CHILD FAULT");
  }

  if (childFreshnessRequired()) {
    if (!childStatusValid) {
      faultNow("CHILD STATUS INVALID");
    } else if ((millis() - lastChildGoodPollMs) > CHILD_STATUS_STALE_MS) {
      faultNow("CHILD STATUS STALE");
    }
  }

#if USE_NEXTION
  readNextion();
#endif

  handleState();

  if (millis() - lastClockMs >= CLOCK_UPDATE_MS) {
    lastClockMs = millis();
    updateAllClocks();
  }

  if (millis() - lastPageQueryMs >= PAGE_QUERY_MS) {
    lastPageQueryMs = millis();
#if USE_NEXTION
    nxCmd("sendme");
#endif
  }

  if (millis() - lastPageRefreshMs >= PAGE_REFRESH_MS) {
    lastPageRefreshMs = millis();
    updateHMI(false);
  }

  if (millis() - lastDebugMs >= DEBUG_PRINT_MS) {
    lastDebugMs = millis();
    Serial.print(F("Time: "));
    Serial.print(timeHHMMSS());
    Serial.print(F(" | State: "));
    Serial.print(stateName(currentState));
    Serial.print(F(" | Child: "));
    Serial.print(childStateName(childStatus.state));
    Serial.print(F(" | Comms: "));
    Serial.println(childStatusValid ? F("OK") : F("BAD"));
  }
}

// -----------------------------
// State machine handler
// -----------------------------
void handleState() {
  unsigned long elapsed = millis() - stateStartMs;

  switch (currentState) {
    case ST_IDLE:
      allOutputsOff();
      pendingChildAction = CA_NONE;
      break;

    case ST_EXTENDING_LIFT: {
      LimitPairState extend = liftExtendState();
      if (!limitPairIsHealthy(extend)) {
        faultNow("EXT LIMIT FAULT");
        break;
      }

      if (limitPairIsReached(extend)) {
        setLiftEnable(false);
        liftForward(false);
        logEvent("LIFT AT DEPTH");
        enterState(ST_OPENING_CUP_AT_DEPTH);
        break;
      }

      liftReverse(false);
      liftForward(true);
      setLiftEnable(true);

      if (elapsed > EXTEND_TIMEOUT_MS) {
        faultNow("EXTEND TIMEOUT");
      }
    } break;

    case ST_OPENING_CUP_AT_DEPTH:
      if (pendingChildAction == CA_NONE) {
        if (!sendChildCommand(CA_OPEN_CUP)) {
          faultNow("OPEN CMD FAILED");
          break;
        }
      }

      if (childReadyForNextAction()) {
        logEvent("CUP OPEN AT DEPTH");
        enterState(ST_FILL_DWELL);
      } else if (elapsed > CHILD_ACTION_TIMEOUT_MS) {
        faultNow("OPEN CUP TIMEOUT");
      }
      break;

    case ST_FILL_DWELL:
      if (elapsed >= FILL_DWELL_MS) {
        enterState(ST_CLOSING_CUP_AT_DEPTH);
      }
      break;

    case ST_CLOSING_CUP_AT_DEPTH:
      if (pendingChildAction == CA_NONE) {
        if (!sendChildCommand(CA_CLOSE_CUP)) {
          faultNow("CLOSE CMD FAILED");
          break;
        }
      }

      if (childReadyForNextAction()) {
        logEvent("CUP CLOSED AT DEPTH");
        enterState(ST_RETRACTING_LIFT);
      } else if (elapsed > CHILD_ACTION_TIMEOUT_MS) {
        faultNow("CLOSE CUP TIMEOUT");
      }
      break;

    case ST_RETRACTING_LIFT: {
      LimitPairState retract = liftRetractState();
      if (!limitPairIsHealthy(retract)) {
        faultNow("RET LIMIT FAULT");
        break;
      }

      if (limitPairIsReached(retract)) {
        setLiftEnable(false);
        liftReverse(false);
        logEvent("LIFT HOME");
        enterState(ST_TILTING_TO_DUMP);
        break;
      }

      liftForward(false);
      liftReverse(true);
      setLiftEnable(true);

      if (elapsed > RETRACT_TIMEOUT_MS) {
        faultNow("RETRACT TIMEOUT");
      }
    } break;

    case ST_TILTING_TO_DUMP:
      if (pendingChildAction == CA_NONE) {
        if (!sendChildCommand(CA_TILT_DUMP)) {
          faultNow("TILT CMD FAILED");
          break;
        }
      }

      if (childReadyForNextAction()) {
        logEvent("CUP TILTED TO DUMP");
        enterState(ST_OPENING_CUP_TO_DUMP);
      } else if (elapsed > CHILD_ACTION_TIMEOUT_MS) {
        faultNow("TILT TIMEOUT");
      }
      break;

    case ST_OPENING_CUP_TO_DUMP:
      if (pendingChildAction == CA_NONE) {
        if (!sendChildCommand(CA_OPEN_CUP)) {
          faultNow("OPEN DUMP CMD FAILED");
          break;
        }
      }

      if (childReadyForNextAction()) {
        logEvent("CUP OPEN FOR DUMP");
        enterState(ST_DUMP_DWELL);
      } else if (elapsed > CHILD_ACTION_TIMEOUT_MS) {
        faultNow("OPEN DUMP TIMEOUT");
      }
      break;

    case ST_DUMP_DWELL:
      if (elapsed >= DUMP_DWELL_MS) {
        enterState(ST_CLOSING_CUP_AFTER_DUMP);
      }
      break;

    case ST_CLOSING_CUP_AFTER_DUMP:
      if (pendingChildAction == CA_NONE) {
        if (!sendChildCommand(CA_CLOSE_CUP)) {
          faultNow("POST-DUMP CLOSE FAILED");
          break;
        }
      }

      if (childReadyForNextAction()) {
        logEvent("CUP CLOSED AFTER DUMP");
        enterState(ST_UNTILTING_HOME);
      } else if (elapsed > CHILD_ACTION_TIMEOUT_MS) {
        faultNow("POST-DUMP CLOSE TIMEOUT");
      }
      break;

    case ST_UNTILTING_HOME:
      if (pendingChildAction == CA_NONE) {
        if (!sendChildCommand(CA_UNTILT_HOME)) {
          faultNow("UNTILT CMD FAILED");
          break;
        }
      }

      if (childReadyForNextAction()) {
        logEvent("CUP TILT HOME");
        enterState(ST_COMPLETE);
      } else if (elapsed > CHILD_ACTION_TIMEOUT_MS) {
        faultNow("UNTILT TIMEOUT");
      }
      break;

    case ST_COMPLETE:
      cycleActive = false;
      logEvent("CYCLE COMPLETE");
      enterState(ST_IDLE);
      break;

    case ST_RECOVERING_CHILD:
      allOutputsOff();

      if (childResetPulseActive && (millis() - childResetPulseStartMs) >= CHILD_RESET_PULSE_MS) {
        digitalWrite(PIN_CHILD_RESET_REQ, HIGH);
        childResetPulseActive = false;
        logEvent("CHILD RESET PULSE COMPLETE");
      }

      if (!childResetPulseActive && childRecoveryReady()) {
        LimitPairState retract = liftRetractState();
        if (!limitPairIsHealthy(retract)) {
          faultNow("RECOVER RET LIMIT FAULT");
        } else if (limitPairIsReached(retract)) {
          completeRecovery();
        } else {
          enterState(ST_RECOVERING_LIFT);
        }
      } else if (elapsed > (CHILD_ACTION_TIMEOUT_MS + CHILD_RESET_PULSE_MS)) {
        faultNow("CHILD RESET TIMEOUT");
      }
      break;

    case ST_RECOVERING_LIFT: {
      LimitPairState retract = liftRetractState();
      if (!limitPairIsHealthy(retract)) {
        faultNow("RECOVER RET LIMIT FAULT");
        break;
      }

      if (limitPairIsReached(retract)) {
        setLiftEnable(false);
        liftReverse(false);
        completeRecovery();
        break;
      }

      liftForward(false);
      liftReverse(true);
      setLiftEnable(true);

      if (elapsed > RETRACT_TIMEOUT_MS) {
        faultNow("RECOVER RETRACT TIMEOUT");
      }
    } break;

    case ST_FAULT:
      allOutputsOff();
      break;
  }
}

// -----------------------------
// State / fault helpers
// -----------------------------
void enterState(State s) {
  currentState = s;
  stateStartMs = millis();
  pendingChildAction = CA_NONE;
  pendingChildFresh = false;

  if (s == ST_IDLE) {
    cycleActive = false;
    cycleStartMs = 0;
    childResetPulseActive = false;
    digitalWrite(PIN_CHILD_RESET_REQ, HIGH);
  }

  if (s == ST_EXTENDING_LIFT) {
    cycleActive = true;
    cycleStartMs = millis();
  }

  if (s == ST_RECOVERING_CHILD) {
    cycleActive = false;
    beginChildResetPulse();
  }

  logEvent("STATE -> " + stateName(s));
  updateHMI(true);
}

void faultNow(const String &msg) {
  faultLatched = true;
  msg.toCharArray(faultMsg, sizeof(faultMsg));
  cycleActive = false;
  childResetPulseActive = false;
  digitalWrite(PIN_CHILD_RESET_REQ, HIGH);

  allOutputsOff();
  sendChildCommand(CA_STOP_ALL);

  logEvent("FAULT: " + msg);

  currentState = ST_FAULT;
  stateStartMs = millis();
  updateHMI(true);
}

void completeRecovery() {
  allOutputsOff();
  faultLatched = false;
  faultMsg[0] = '\0';
  digitalWrite(PIN_ALARM, LOW);
  logEvent("RECOVERY COMPLETE");
  enterState(ST_IDLE);
}

void allOutputsOff() {
  digitalWrite(PIN_LIFT_FWD, LOW);
  digitalWrite(PIN_LIFT_REV, LOW);
  digitalWrite(PIN_LIFT_EN, LOW);
  digitalWrite(PIN_ALARM, faultLatched ? HIGH : LOW);
}

// -----------------------------
// Lift motor helpers
// -----------------------------
void liftForward(bool on) {
  if (on) digitalWrite(PIN_LIFT_REV, LOW);
  digitalWrite(PIN_LIFT_FWD, on ? HIGH : LOW);
}

void liftReverse(bool on) {
  if (on) digitalWrite(PIN_LIFT_FWD, LOW);
  digitalWrite(PIN_LIFT_REV, on ? HIGH : LOW);
}

void setLiftEnable(bool on) {
  digitalWrite(PIN_LIFT_EN, on ? HIGH : LOW);
}

// -----------------------------
// Input helpers
// -----------------------------
bool estopActive() {
  return digitalRead(PIN_ESTOP_NC) == HIGH;
}

LimitPairState readLimitPair(uint8_t reachedPin, uint8_t healthyPin) {
  bool noHigh = (digitalRead(reachedPin) == HIGH);
  bool ncHigh = (digitalRead(healthyPin) == HIGH);

  if (noHigh && !ncHigh) return LP_NOT_REACHED;
  if (!noHigh && ncHigh) return LP_REACHED;
  if (noHigh && ncHigh) return LP_FAULT_OPEN;
  return LP_FAULT_SHORT;
}

LimitPairState liftExtendState() {
  return readLimitPair(PIN_LS_EXT_REACHED, PIN_LS_EXT_HEALTHY);
}

LimitPairState liftRetractState() {
  return readLimitPair(PIN_LS_RET_REACHED, PIN_LS_RET_HEALTHY);
}

bool limitPairIsHealthy(LimitPairState state) {
  return state == LP_NOT_REACHED || state == LP_REACHED;
}

bool limitPairIsReached(LimitPairState state) {
  return state == LP_REACHED;
}

// -----------------------------
// Encoder helpers
// -----------------------------
void encISR() {
  encPulses++;
}

long getEncoderPulses() {
  noInterrupts();
  long p = encPulses;
  interrupts();
  return p;
}

// -----------------------------
// Button handling
// -----------------------------
void pollPhysicalButtons() {
#if !USE_PHYSICAL_BUTTONS
  return;
#else
  if (updateDebounced(dbStart) && dbStart.stableState == LOW) onStartPressed();
  if (updateDebounced(dbStop)  && dbStop.stableState  == LOW) onStopPressed();
  if (updateDebounced(dbReset) && dbReset.stableState == LOW) onResetPressed();
#endif
}

bool updateDebounced(DebouncedInput &db) {
  bool raw = digitalRead(db.pin);
  bool changed = false;

  if (raw != db.lastRead) {
    db.lastRead = raw;
    db.lastChangeMs = millis();
  }

  if ((millis() - db.lastChangeMs) > DEBOUNCE_MS && raw != db.stableState) {
    db.stableState = raw;
    changed = true;
  }

  return changed;
}

void onStartPressed() {
  if (currentState != ST_IDLE || faultLatched) return;

  if (estopActive()) {
    faultNow("E-STOP ACTIVE");
    return;
  }

  if (!childStartReady()) {
    faultNow("CHILD NOT READY");
    return;
  }

  logEvent("START BUTTON");
  enterState(ST_EXTENDING_LIFT);
}

void onStopPressed() {
  if (currentState != ST_IDLE && currentState != ST_FAULT) {
    faultNow("STOP/ABORT PRESSED");
  }
}

void onResetPressed() {
  if (!(currentState == ST_FAULT || faultLatched)) return;

  if (estopActive()) {
    logEvent("RESET BLOCKED: ESTOP");
    return;
  }

  logEvent("RESET / RECOVERY REQUEST");
  enterState(ST_RECOVERING_CHILD);
}

// -----------------------------
// I2C communication with Child
// -----------------------------
bool sendChildCommand(ChildAction action) {
  Wire.beginTransmission(CHILD_ADDR);
  Wire.write((uint8_t)action);
  uint8_t rc = Wire.endTransmission();

  if (rc == 0) {
    pendingChildAction = action;
    pendingChildFresh = false;
    return true;
  }

  childStatusValid = false;
  return false;
}

bool pollChildStatus() {
  const size_t expected = sizeof(ChildStatus);
  ChildStatus incoming;

  int received = Wire.requestFrom((int)CHILD_ADDR, (int)expected);
  if (received != (int)expected || Wire.available() < (int)expected) {
    while (Wire.available()) {
      Wire.read();
    }
    childStatusValid = false;
    return false;
  }

  uint8_t *raw = (uint8_t*)&incoming;
  for (size_t i = 0; i < expected; i++) {
    raw[i] = Wire.read();
  }

  childStatus = incoming;

  bool seqAdvanced = (!childSeqSeen || childStatus.seq != lastChildSeq);
  if (seqAdvanced) {
    lastChildSeq = childStatus.seq;
    childSeqSeen = true;
  }

  childStatusValid = (childStatus.limitsValid != 0);
  if (childStatusValid && seqAdvanced) {
    lastChildGoodPollMs = millis();
    if (pendingChildAction != CA_NONE) {
      pendingChildFresh = true;
    }
  }

  return true;
}

bool childReadyForNextAction() {
  if (pendingChildAction == CA_NONE) return false;
  if (!childStatusValid) return false;
  if (!pendingChildFresh) return false;
  if (childStatus.fault || childStatus.busy) return false;

  switch (pendingChildAction) {
    case CA_OPEN_CUP:
      if (childStatus.cupOpen) {
        pendingChildAction = CA_NONE;
        pendingChildFresh = false;
        return true;
      }
      break;

    case CA_CLOSE_CUP:
      if (childStatus.cupClosed) {
        pendingChildAction = CA_NONE;
        pendingChildFresh = false;
        return true;
      }
      break;

    case CA_TILT_DUMP:
      if (childStatus.tiltDump) {
        pendingChildAction = CA_NONE;
        pendingChildFresh = false;
        return true;
      }
      break;

    case CA_UNTILT_HOME:
      if (childStatus.tiltHome) {
        pendingChildAction = CA_NONE;
        pendingChildFresh = false;
        return true;
      }
      break;

    case CA_STOP_ALL:
      pendingChildAction = CA_NONE;
      pendingChildFresh = false;
      return true;

    default:
      break;
  }

  return false;
}

bool childFaulted() {
  return childStatusValid && (childStatus.fault != 0);
}

bool childStartReady() {
  return childStatusValid &&
         !childStatus.fault &&
         !childStatus.busy &&
         childStatus.cupClosed &&
         childStatus.tiltHome &&
         childStatus.limitsValid;
}

bool childFreshnessRequired() {
  return currentState != ST_IDLE && currentState != ST_FAULT;
}

bool childRecoveryReady() {
  return childStatusValid &&
         !childStatus.fault &&
         !childStatus.busy &&
         childStatus.limitsValid &&
         childStatus.cupClosed &&
         childStatus.tiltHome;
}

void beginChildResetPulse() {
  digitalWrite(PIN_CHILD_RESET_REQ, LOW);
  childResetPulseActive = true;
  childResetPulseStartMs = millis();
  logEvent("CHILD RESET PULSE START");
}

// -----------------------------
// HMI update helpers
// -----------------------------
void updateHMI(bool force) {
  if (visiblePage == VP_MAIN) {
    updateMainPage(force);
  } else if (visiblePage == VP_DIAG) {
    updateDiagPage(force);
  } else if (visiblePage == VP_LOGS) {
    updateLogPage(force);
  }
}

void updateMainPage(bool force) {
#if USE_NEXTION
  String t = timeHHMMSS();
  nxSetTxt("tClock", t);

  String s = stateName(currentState);
  nxSetTxt("tState", s);

  String f = faultLatched ? String(faultMsg) : "OK";
  nxSetTxt("tFault", f);
#endif
}

void updateDiagPage(bool force) {
#if USE_NEXTION
  String t = timeHHMMSS();
  nxSetTxt("tClock2", t);

  String lift = String("EXT:") + limitPairName(liftExtendState()) +
                " RET:" + limitPairName(liftRetractState()) +
                " ESTOP:" + (estopActive() ? "ON" : "OFF");
  nxSetTxt("tLiftDiag", lift);

  String child = String("COMM:") + (childStatusValid ? "OK" : "BAD") +
                 " LIM:" + (childStatus.limitsValid ? "OK" : "BAD") +
                 " OP:" + (childStatus.cupOpen ? "1" : "0") +
                 " CL:" + (childStatus.cupClosed ? "1" : "0") +
                 " TD:" + (childStatus.tiltDump ? "1" : "0") +
                 " TH:" + (childStatus.tiltHome ? "1" : "0") +
                 " S:" + childStateName(childStatus.state);
  nxSetTxt("tChildDiag", child);

  String enc = String(getEncoderPulses()) + " pulses";
  nxSetTxt("tEnc", enc);
#endif
}

void updateLogPage(bool force) {
#if USE_NEXTION
  String t = timeHHMMSS();
  nxSetTxt("tClock1", t);

  if (!force && !logDirty) return;

  String block = "";
  for (size_t i = 0; i < logCount; i++) {
    block += String(logLines[i]);
    if (i + 1 < logCount) block += "\r\n";
  }

  nxSetTxt("tLog", block);
  logDirty = false;
#endif
}

void updateAllClocks() {
#if USE_NEXTION
  if (visiblePage == VP_MAIN) {
    nxSetTxt("tClock", timeHHMMSS());
  } else if (visiblePage == VP_LOGS) {
    nxSetTxt("tClock1", timeHHMMSS());
  } else if (visiblePage == VP_DIAG) {
    nxSetTxt("tClock2", timeHHMMSS());
  }
#endif
}

// -----------------------------
// Nextion serial handling
// -----------------------------
void readNextion() {
#if USE_NEXTION
  static String line = "";
  static bool awaitingPageId = false;
  static bool awaitingPageTerminator = false;
  static uint8_t pageId = 0;
  static uint8_t ffCount = 0;

  while (nxt.available()) {
    uint8_t b = (uint8_t)nxt.read();

    if (awaitingPageId) {
      pageId = b;
      awaitingPageId = false;
      awaitingPageTerminator = true;
      ffCount = 0;
      continue;
    }

    if (awaitingPageTerminator) {
      if (b == 0xFF) {
        ffCount++;
        if (ffCount >= 3) {
          setCurrentPageFromId(pageId);
          awaitingPageTerminator = false;
          ffCount = 0;
        }
      } else {
        awaitingPageTerminator = false;
        ffCount = 0;
      }
      continue;
    }

    if (b == 0x66) {
      line = "";
      awaitingPageId = true;
      ffCount = 0;
      continue;
    }

    char c = (char)b;

    if (c == '\n') {
      line.trim();
      if (line.length()) handleNxLine(line);
      line = "";
    } else if (c != '\r' && c >= 32 && c <= 126) {
      line += c;
      if (line.length() > 80) line.remove(0, 40);
    }
  }
#endif
}

void setCurrentPageFromId(uint8_t pageId) {
  if (pageId == 0) {
    visiblePage = VP_MAIN;
  } else if (pageId == 1) {
    visiblePage = VP_LOGS;
  } else if (pageId == 2) {
    visiblePage = VP_DIAG;
  } else {
    return;
  }

  updateHMI(true);
}

void handleNxLine(String line) {
  line.trim();
  line.toUpperCase();

  if (line == "START") {
    onStartPressed();
  } else if (line == "STOP") {
    onStopPressed();
  } else if (line == "RESET") {
    onResetPressed();
  } else if (line == "PAGE:MAIN") {
    visiblePage = VP_MAIN;
    updateHMI(true);
  } else if (line == "PAGE:LOGS") {
    visiblePage = VP_LOGS;
    updateHMI(true);
  } else if (line == "PAGE:DIAG") {
    visiblePage = VP_DIAG;
    updateHMI(true);
  }
}

void nxCmd(const String &cmd) {
#if USE_NEXTION
  nxt.print(cmd);
  nxt.write(0xFF);
  nxt.write(0xFF);
  nxt.write(0xFF);
#endif
}

void nxSetTxt(const String &obj, const String &txt) {
  String safe = txt;
  safe.replace("\"", "'");
  nxCmd(obj + ".txt=\"" + safe + "\"");
}

// -----------------------------
// Utility / display helpers
// -----------------------------
String timeHHMMSS() {
#if USE_RTC
  if (rtcOk) {
    DateTime now = rtc.now();
    char buf[9];
    snprintf(buf, sizeof(buf), "%02d:%02d:%02d", now.hour(), now.minute(), now.second());
    return String(buf);
  }
#endif

  unsigned long s = millis() / 1000UL;
  unsigned long h = (s / 3600UL) % 24UL;
  unsigned long m = (s / 60UL) % 60UL;
  unsigned long sec = s % 60UL;

  char buf[9];
  snprintf(buf, sizeof(buf), "%02lu:%02lu:%02lu", h, m, sec);
  return String(buf);
}

String stateName(State s) {
  switch (s) {
    case ST_IDLE:                   return "IDLE";
    case ST_EXTENDING_LIFT:         return "EXTENDING";
    case ST_OPENING_CUP_AT_DEPTH:   return "OPEN CUP @DEPTH";
    case ST_FILL_DWELL:             return "FILL DWELL";
    case ST_CLOSING_CUP_AT_DEPTH:   return "CLOSE CUP @DEPTH";
    case ST_RETRACTING_LIFT:        return "RETRACTING";
    case ST_TILTING_TO_DUMP:        return "TILT TO DUMP";
    case ST_OPENING_CUP_TO_DUMP:    return "OPEN FOR DUMP";
    case ST_DUMP_DWELL:             return "DUMP DWELL";
    case ST_CLOSING_CUP_AFTER_DUMP: return "POST-DUMP CLOSE";
    case ST_UNTILTING_HOME:         return "UNTILT HOME";
    case ST_COMPLETE:               return "COMPLETE";
    case ST_RECOVERING_CHILD:       return "RECOVER CHILD";
    case ST_RECOVERING_LIFT:        return "RECOVER LIFT";
    case ST_FAULT:                  return "FAULT";
    default:                        return "UNKNOWN";
  }
}

String childStateName(uint8_t s) {
  switch (s) {
    case 0: return "IDLE";
    case 1: return "OPENING";
    case 2: return "CLOSING";
    case 3: return "TILTING";
    case 4: return "UNTILTING";
    case 5: return "RCLOSE";
    case 6: return "RUNTILT";
    case 7: return "FAULT";
    default:return "?";
  }
}

String limitPairName(LimitPairState s) {
  switch (s) {
    case LP_NOT_REACHED: return "NR";
    case LP_REACHED:     return "OK";
    case LP_FAULT_OPEN:  return "OPEN";
    case LP_FAULT_SHORT: return "SHORT";
    default:             return "?";
  }
}

void logEvent(const String &msg) {
  String line = timeHHMMSS() + " " + msg;
  char lineBuf[MAX_LOG_LINE_CHARS];
  line.toCharArray(lineBuf, sizeof(lineBuf));

  if (logCount < MAX_LOG_LINES) {
    strncpy(logLines[logCount], lineBuf, MAX_LOG_LINE_CHARS - 1);
    logLines[logCount][MAX_LOG_LINE_CHARS - 1] = '\0';
    logCount++;
  } else {
    for (size_t i = 1; i < MAX_LOG_LINES; i++) {
      strncpy(logLines[i - 1], logLines[i], MAX_LOG_LINE_CHARS - 1);
      logLines[i - 1][MAX_LOG_LINE_CHARS - 1] = '\0';
    }
    strncpy(logLines[MAX_LOG_LINES - 1], lineBuf, MAX_LOG_LINE_CHARS - 1);
    logLines[MAX_LOG_LINES - 1][MAX_LOG_LINE_CHARS - 1] = '\0';
  }

  Serial.println(line);
  logDirty = true;
}
