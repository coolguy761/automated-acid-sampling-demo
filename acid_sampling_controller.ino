/*
  Automated Acid Sampling Demo (Pneumatic) — Arduino Uno + Nextion + DS3231 + Encoder

  What it does:
  - IDLE: waits for START
  - EXTENDING: energize EXT solenoid until LS_EXT triggers (or timeout)
  - DWELL: wait fixed time (simulate sampling)
  - RETRACTING: energize RET solenoid until LS_RET triggers (or timeout)
  - COMPLETE: log + return to IDLE
  - FAULT: de-energize outputs, show fault, require RESET

  Safety/Interlocks:
  - E-STOP: stops actuators immediately (should also physically cut actuator power via E-STOP contact)
  - PressureOK: prevents motion; fault if lost during motion

  Nextion:
  - Updates text values + background colors (bco) for OK/FAULT/ON/OFF.
  - Page1 uses a Scroll Text component named "tLog"; we overwrite its .txt with the whole log buffer.

  Libraries:
  - Uses RTClib (Adafruit) for DS3231. If not used timestamps will not display.

*/

#include <SoftwareSerial.h>
#include <Wire.h>

// ---- Optional RTC ----
#define USE_RTC 1
#if USE_RTC
  #include <RTClib.h>
  RTC_DS3231 rtc;
#endif

// =====================
// Pin Mapping (Uno Rev3)
// =====================

// Inputs (wired NC -> GND, use INPUT_PULLUP for fail-safe)
// With NC + pullup: normal/ok = LOW, triggered/fault/broken wire = HIGH
const uint8_t PIN_ENC_A   = 2;   // Encoder A (interrupt) - optional
const uint8_t PIN_START   = 3;   // Start button (optional physical)
const uint8_t PIN_STOP    = 4;   // Stop/Abort button (optional physical)
const uint8_t PIN_RESET   = 5;   // Reset button (optional physical)
const uint8_t PIN_ESTOP   = 6;   // E-STOP status (NC contact to GND)
const uint8_t PIN_PRESSOK = 7;   // Pressure OK switch (NC contact to GND)
const uint8_t PIN_LS_EXT  = 8;   // Limit switch extend (NC contact to GND)
const uint8_t PIN_LS_RET  = 9;   // Limit switch retract (NC contact to GND)

// Outputs (drive relay/MOSFET modules; do NOT power loads directly from pins)
const uint8_t PIN_SOL_EXT = 12;  // Solenoid EXT driver
const uint8_t PIN_SOL_RET = 13;  // Solenoid RET driver
const uint8_t PIN_MOTOR_EN = A0; // optional motor enable (A0 as digital)
const uint8_t PIN_ALARM    = A1; // optional buzzer/alarm (A1 as digital)

// Nextion on SoftwareSerial (recommended pins)
const uint8_t NX_RX = 10; // Arduino RX  <- Nextion TX
const uint8_t NX_TX = 11; // Arduino TX  -> Nextion RX
SoftwareSerial nxt(NX_RX, NX_TX);

// =====================
// Nextion Color Constants (RGB565)
// =====================
const uint16_t NX_GRAY   = 50712;
const uint16_t NX_WHITE  = 65535;
const uint16_t NX_BLACK  = 0;
const uint16_t NX_GREEN  = 2016;
const uint16_t NX_RED    = 63488;
const uint16_t NX_YELLOW = 65504;

// =====================
// Encoder / Revs
// =====================
volatile long encPulses = 0;
const float PULSES_PER_REV = 20.0f; // set to your encoder resolution (current demo value)

// =====================
// State Machine
// =====================
enum State {
  ST_IDLE,
  ST_EXTENDING,
  ST_DWELL,
  ST_RETRACTING,
  ST_COMPLETE,
  ST_FAULT
};

State state = ST_IDLE;
unsigned long stateStartMs = 0;
unsigned long cycleStartMs = 0;

// Timing (fine tune for later)
const unsigned long EXTEND_TIMEOUT_MS = 6000;
const unsigned long RETRACT_TIMEOUT_MS = 6000;
const unsigned long DWELL_MS = 2000;

// Fault text
String faultMsg = "None";

// =====================
// Log buffer for Nextion Scroll Text (tLog)
// =====================
String bigLog = "";                 // entire log shown in tLog
const int MAX_LOG_CHARS = 900;      // keep under Nextion text limit
bool logDirty = true;

// Track current page (if needed)
enum Page { PAGE_UNKNOWN, PAGE_MAIN, PAGE_LOG, PAGE_DIAG };
Page currentPage = PAGE_UNKNOWN;

// =====================
// Helpers: input read (NC + pullup)
// =====================
bool readNC_ActiveHigh(uint8_t pin) {
  // With INPUT_PULLUP + NC to GND:
  // - Normal/closed = LOW
  // - Active/open (pressed/triggered or wire break) = HIGH
  return (digitalRead(pin) == HIGH);
}

bool estopActive()   { return readNC_ActiveHigh(PIN_ESTOP); }
bool pressureNotOK() { return readNC_ActiveHigh(PIN_PRESSOK); }
bool lsExtActive()   { return readNC_ActiveHigh(PIN_LS_EXT); }
bool lsRetActive()   { return readNC_ActiveHigh(PIN_LS_RET); }

// =====================
// Nextion: low-level send
// =====================
void nxtCmd(const String &cmd) {
  nxt.print(cmd);
  nxt.write(0xFF); nxt.write(0xFF); nxt.write(0xFF);
}

// Escape quotes
String nxEscape(String s) {
  s.replace("\"", "\\\"");
  return s;
}

void nxSetTxt(const char* obj, const String &val) {
  nxtCmd(String(obj) + ".txt=\"" + nxEscape(val) + "\"");
}

void nxSetBco(const char* obj, uint16_t color) {
  nxtCmd(String(obj) + ".bco=" + String(color));
  // Some Nextion models need a refresh:
  nxtCmd(String(obj) + ".ref");
}

// =====================
// Time string (RTC if available)
// =====================
String timeHHMMSS() {
#if USE_RTC
  DateTime now = rtc.now();
  char buf[9];
  snprintf(buf, sizeof(buf), "%02d:%02d:%02d", now.hour(), now.minute(), now.second());
  return String(buf);
#else
  return "00:00:00";
#endif
}

// =====================
// Logging
// =====================
void addLog(const String &line) {
  String entry = timeHHMMSS() + " | " + line;

  // Append to big log
  if (bigLog.length() > 0) bigLog += "\r\n";
  bigLog += entry;

  // Trim from the front if too long
  if ((int)bigLog.length() > MAX_LOG_CHARS) {
    int cut = bigLog.length() - MAX_LOG_CHARS;
    // cut to next newline so we don't break mid-line
    int nl = bigLog.indexOf('\n', cut);
    if (nl > 0) bigLog = bigLog.substring(nl + 1);
    else bigLog = bigLog.substring(cut);
  }

  logDirty = true;
}

void pushLogToNextion() {
  // Page1 has tLog (Scroll Text)
  nxSetTxt("tLog", bigLog);
  // Update clock on log page if needs be
  nxSetTxt("tClock2", "TIME " + timeHHMMSS());
  logDirty = false;
}

// =====================
// Output control
// =====================
void allOutputsOff() {
  digitalWrite(PIN_SOL_EXT, LOW);
  digitalWrite(PIN_SOL_RET, LOW);
  digitalWrite(PIN_MOTOR_EN, LOW);
  digitalWrite(PIN_ALARM, LOW);
}

void setExtend(bool on) { digitalWrite(PIN_SOL_EXT, on ? HIGH : LOW); }
void setRetract(bool on){ digitalWrite(PIN_SOL_RET, on ? HIGH : LOW); }
void setMotorEn(bool on){ digitalWrite(PIN_MOTOR_EN, on ? HIGH : LOW); }
void setAlarm(bool on)  { digitalWrite(PIN_ALARM, on ? HIGH : LOW); }

// =====================
// State transitions
// =====================
void enterState(State s) {
  state = s;
  stateStartMs = millis();

  // Log state transitions (optional)
  switch (state) {
    case ST_IDLE:       addLog("STATE | IDLE"); break;
    case ST_EXTENDING:  addLog("STATE | EXTENDING"); break;
    case ST_DWELL:      addLog("STATE | DWELL"); break;
    case ST_RETRACTING: addLog("STATE | RETRACTING"); break;
    case ST_COMPLETE:   addLog("STATE | COMPLETE"); break;
    case ST_FAULT:      addLog("FAULT | " + faultMsg); break;
  }
}

// =====================
// Fault handling
// =====================
void triggerFault(const String &msg) {
  faultMsg = msg;
  allOutputsOff();
  setAlarm(true);
  enterState(ST_FAULT);
}

// =====================
// Encoder ISR
// =====================
void isrEncA() {
  encPulses++;
}

// =====================
// Nextion command parsing
// =====================
// Expecting simple newline-terminated strings:
// START, STOP, RESET, CLEARLOG, PAGE:MAIN, PAGE:LOG, PAGE:DIAG
String rxLine = "";

void handleNxLine(const String &lineRaw) {
  String line = lineRaw;
  line.trim();

  if (line == "START") {
    if (state == ST_IDLE) {
      // Check interlocks before starting
      if (estopActive()) {
        triggerFault("E-STOP ACTIVE");
      } else if (pressureNotOK()) {
        triggerFault("PRESSURE LOW");
      } else {
        // Start cycle
        cycleStartMs = millis();
        encPulses = 0;
        faultMsg = "None";
        setAlarm(false);
        enterState(ST_EXTENDING);
      }
    }
  }
  else if (line == "STOP") {
    // Immediate stop/abort
    allOutputsOff();
    triggerFault("ABORTED");
  }
  else if (line == "RESET") {
    // Clear fault and return to idle
    faultMsg = "None";
    setAlarm(false);
    allOutputsOff();
    enterState(ST_IDLE);
  }
  else if (line == "CLEARLOG") {
    bigLog = "";
    addLog("INFO  | Log cleared");
    logDirty = true;
  }
  else if (line == "PAGE:MAIN") currentPage = PAGE_MAIN;
  else if (line == "PAGE:LOG")  { currentPage = PAGE_LOG;  logDirty = true; }
  else if (line == "PAGE:DIAG") currentPage = PAGE_DIAG;
}

// Read Nextion lines
void pollNextion() {
  while (nxt.available()) {
    char c = (char)nxt.read();
    if (c == '\n') {
      handleNxLine(rxLine);
      rxLine = "";
    } else if (c != '\r') {
      rxLine += c;
      if (rxLine.length() > 80) rxLine.remove(0, 40); // prevent runaway
    }
  }
}

// =====================
// HMI update helpers
// =====================
String stateName(State s) {
  switch (s) {
    case ST_IDLE: return "IDLE";
    case ST_EXTENDING: return "EXTENDING";
    case ST_DWELL: return "DWELL";
    case ST_RETRACTING: return "RETRACTING";
    case ST_COMPLETE: return "COMPLETE";
    case ST_FAULT: return "FAULT";
  }
  return "UNKNOWN";
}

String mmss(unsigned long ms) {
  unsigned long sec = ms / 1000;
  unsigned long m = sec / 60;
  unsigned long s = sec % 60;
  char buf[6];
  snprintf(buf, sizeof(buf), "%02lu:%02lu", m, s);
  return String(buf);
}

// Update Page 0 (main)
void updateMainPage() {
  // Time
  nxSetTxt("tClock", "TIME " + timeHHMMSS());

  // State
  nxSetTxt("tState", "STATE: " + stateName(state));

  // Cycle elapsed
  unsigned long elapsed = (cycleStartMs == 0) ? 0 : (millis() - cycleStartMs);
  if (state == ST_IDLE) elapsed = 0;
  nxSetTxt("tCycle", "Cycle Elapsed: " + mmss(elapsed));

  // Inputs display
  bool pBad = pressureNotOK();
  bool eAct = estopActive();
  bool lExt = lsExtActive();
  bool lRet = lsRetActive();

  nxSetTxt("tPress", String("Pressure: ") + (pBad ? "LOW" : "OK"));
  nxSetBco("tPress", pBad ? NX_RED : NX_GREEN);

  nxSetTxt("tEstop", String("E-STOP: ") + (eAct ? "YES" : "NO"));
  nxSetBco("tEstop", eAct ? NX_RED : NX_GRAY);

  nxSetTxt("tLsExt", String("LS_EXT: ") + (lExt ? "ON" : "OFF"));
  nxSetBco("tLsExt", lExt ? NX_GREEN : NX_GRAY);

  nxSetTxt("tLsRet", String("LS_RET: ") + (lRet ? "ON" : "OFF"));
  nxSetBco("tLsRet", lRet ? NX_GREEN : NX_GRAY);

  // Encoder/Revs
  long p = encPulses;
  float revs = p / PULSES_PER_REV;

  nxSetTxt("tEnc", "Encoder: " + String(p));
  nxSetTxt("tRevs", "Motor Revs: " + String(revs, 1));

  // Outputs display
  bool outExt = (digitalRead(PIN_SOL_EXT) == HIGH);
  bool outRet = (digitalRead(PIN_SOL_RET) == HIGH);
  bool outMot = (digitalRead(PIN_MOTOR_EN) == HIGH);

  nxSetTxt("tVext", String("Valve EXT: ") + (outExt ? "ON" : "OFF"));
  nxSetBco("tVext", outExt ? NX_GREEN : NX_GRAY);

  nxSetTxt("tVret", String("Valve RET: ") + (outRet ? "ON" : "OFF"));
  nxSetBco("tVret", outRet ? NX_GREEN : NX_GRAY);

  nxSetTxt("tMotor", String("Motor: ") + (outMot ? "ON" : "OFF"));
  nxSetBco("tMotor", outMot ? NX_GREEN : NX_GRAY);

  // Fault banner
  if (state == ST_FAULT) {
    nxSetTxt("tFault", "FAULT / MESSAGE: " + faultMsg);
    nxSetBco("tFault", NX_RED);
  } else {
    nxSetTxt("tFault", "FAULT / MESSAGE: None");
    nxSetBco("tFault", NX_GRAY);
  }
}

// Update Page 2 (diagnostics)
void updateDiagPage() {
  nxSetTxt("tClock2", "TIME " + timeHHMMSS());

  bool pBad = pressureNotOK();
  bool eAct = estopActive();
  bool lExt = lsExtActive();
  bool lRet = lsRetActive();

  // Inputs
  nxSetTxt("tInPress", String("PRESS: ") + (pBad ? "LOW" : "OK"));
  nxSetBco("tInPress", pBad ? NX_RED : NX_GREEN);

  nxSetTxt("tInLsExt", String("LS_EXT: ") + (lExt ? "ON" : "OFF"));
  nxSetBco("tInLsExt", lExt ? NX_GREEN : NX_GRAY);

  nxSetTxt("tInLsRet", String("LS_RET: ") + (lRet ? "ON" : "OFF"));
  nxSetBco("tInLsRet", lRet ? NX_GREEN : NX_GRAY);

  nxSetTxt("tInEstop", String("E-STOP: ") + (eAct ? "ON" : "OFF"));
  nxSetBco("tInEstop", eAct ? NX_RED : NX_GRAY);

  // Outputs
  bool outExt = (digitalRead(PIN_SOL_EXT) == HIGH);
  bool outRet = (digitalRead(PIN_SOL_RET) == HIGH);
  bool outMot = (digitalRead(PIN_MOTOR_EN) == HIGH);
  bool outAlm = (digitalRead(PIN_ALARM) == HIGH);

  nxSetTxt("tOutExt", String("VALVE_EXT: ") + (outExt ? "ON" : "OFF"));
  nxSetBco("tOutExt", outExt ? NX_GREEN : NX_GRAY);

  nxSetTxt("tOutRet", String("VALVE_RET: ") + (outRet ? "ON" : "OFF"));
  nxSetBco("tOutRet", outRet ? NX_GREEN : NX_GRAY);

  nxSetTxt("tOutMotor", String("MOTOR_EN: ") + (outMot ? "ON" : "OFF"));
  nxSetBco("tOutMotor", outMot ? NX_GREEN : NX_GRAY);

  nxSetTxt("tOutAlarm", String("ALARM: ") + (outAlm ? "ON" : "OFF"));
  nxSetBco("tOutAlarm", outAlm ? NX_RED : NX_GRAY);

  // Fault line
  if (state == ST_FAULT) {
    nxSetTxt("tFault2", "FAULT: " + faultMsg);
    nxSetBco("tFault2", NX_RED);
  } else {
    nxSetTxt("tFault2", "FAULT: None");
    nxSetBco("tFault2", NX_GRAY);
  }
}

// =====================
// Main control logic
// =====================
void loopStateMachine() {
  // Global interlocks: E-STOP always wins
  if (estopActive() && state != ST_IDLE && state != ST_FAULT) {
    triggerFault("E-STOP ACTIVE");
    return;
  }

  // Pressure lost during motion => fault
  if (pressureNotOK() && (state == ST_EXTENDING || state == ST_RETRACTING || state == ST_DWELL)) {
    triggerFault("PRESSURE LOST");
    return;
  }

  switch (state) {
    case ST_IDLE:
      allOutputsOff();
      // Optionally read physical buttons too:
      if (readNC_ActiveHigh(PIN_START)) handleNxLine("START");
      break;

    case ST_EXTENDING: {
      setRetract(false);
      setExtend(true);

      // Reached end?
      if (lsExtActive()) {
        setExtend(false);
        enterState(ST_DWELL);
      } else if (millis() - stateStartMs > EXTEND_TIMEOUT_MS) {
        triggerFault("EXT TIMEOUT");
      }
    } break;

    case ST_DWELL:
      allOutputsOff();
      if (millis() - stateStartMs > DWELL_MS) {
        enterState(ST_RETRACTING);
      }
      break;

    case ST_RETRACTING: {
      setExtend(false);
      setRetract(true);

      if (lsRetActive()) {
        setRetract(false);
        enterState(ST_COMPLETE);
      } else if (millis() - stateStartMs > RETRACT_TIMEOUT_MS) {
        triggerFault("RET TIMEOUT");
      }
    } break;

    case ST_COMPLETE:
      allOutputsOff();
      addLog("INFO  | Cycle complete");
      enterState(ST_IDLE);
      break;

    case ST_FAULT:
      allOutputsOff();
      setAlarm(true);
      // Optionally allow physical reset:
      if (readNC_ActiveHigh(PIN_RESET)) handleNxLine("RESET");
      break;
  }
}

// =====================
// Setup
// =====================
void setup() {
  // IO
  pinMode(PIN_ENC_A, INPUT_PULLUP);
  pinMode(PIN_START, INPUT_PULLUP);
  pinMode(PIN_STOP, INPUT_PULLUP);
  pinMode(PIN_RESET, INPUT_PULLUP);
  pinMode(PIN_ESTOP, INPUT_PULLUP);
  pinMode(PIN_PRESSOK, INPUT_PULLUP);
  pinMode(PIN_LS_EXT, INPUT_PULLUP);
  pinMode(PIN_LS_RET, INPUT_PULLUP);

  pinMode(PIN_SOL_EXT, OUTPUT);
  pinMode(PIN_SOL_RET, OUTPUT);
  pinMode(PIN_MOTOR_EN, OUTPUT);
  pinMode(PIN_ALARM, OUTPUT);

  allOutputsOff();

  // Encoder interrupt (optional)
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), isrEncA, RISING);

  // Serial
  Serial.begin(115200);   // USB debug
  nxt.begin(9600);        // common Nextion baud

  Wire.begin();

#if USE_RTC
  if (!rtc.begin()) {
    // If RTC missing, still run
  }
#endif

  // Boot logs
  addLog("INFO  | Boot");
  enterState(ST_IDLE);
}

// =====================
// Main loop
// =====================
void loop() {
  // Read Nextion commands
  pollNextion();

  // Physical STOP button as abort (optional for now)
  if (readNC_ActiveHigh(PIN_STOP) && state != ST_IDLE) {
    handleNxLine("STOP");
  }

  // Run control logic
  loopStateMachine();

  // Update HMI (currently: update main/diag every cycle; OK for demo)
  // If needed to get less serial spam, update every 200ms or only on changes.
  updateMainPage();

  if (currentPage == PAGE_DIAG) {
    updateDiagPage();
  }
  if (currentPage == PAGE_LOG && logDirty) {
    pushLogToNextion();
  }

  delay(80); // small pacing to reduce serial flood
}
