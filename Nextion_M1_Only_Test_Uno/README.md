# Nextion M1 Only Test

## File
[Nextion_M1_Only_Test_Uno.ino](C:/Users/coolg/Documents/Codex/2026-04-19-files-mentioned-by-the-user-acid/Nextion_M1_Only_Test_Uno/Nextion_M1_Only_Test_Uno.ino)

## Purpose
This sketch is for testing only Motor 1 with the existing Nextion screen and RTC clock.

It is meant to isolate lift motion so `M1` can be tuned without `M3`, `M2`, or the full production sequence interfering.

## What It Does
- Keeps Nextion communication on pins `10/11`
- Keeps DS3231 RTC clock updates on all pages
- Supports manual `M1` motion commands from the screen
- Supports a simple automatic cycle:
  - `M1 DOWN`
  - `M1 UP`

## Supported Commands
- `START`
  Runs one `M1 DOWN -> M1 UP` cycle
- `STOP`
  Stops motion immediately
- `RESET`
  Stops motion and clears the test state back to idle
- `STATUS`
  Logs the current sequence state
- `M1F`
  Manual Motor 1 forward
- `M1R`
  Manual Motor 1 reverse

## Pin Use
- `M1 PUL -> pin 2`
- `M1 DIR -> pin 3`
- `M1 ENA -> pin 8`
- `Nextion TX/RX -> pins 10/11`
- `RTC SDA/SCL -> A4/A5`

## Key Tuning Values
These are the main values to change during bench tuning:

```cpp
const unsigned long STEP_INTERVAL_M1_US = 800UL;
const unsigned long STEP_01_M1_DOWN_MS  = 8550UL;
const unsigned long STEP_02_M1_UP_MS    = 7850UL;
```

Meaning:
- `STEP_INTERVAL_M1_US`
  Lower value = faster stepping
  Higher value = slower stepping
- `STEP_01_M1_DOWN_MS`
  Total Motor 1 down travel time
- `STEP_02_M1_UP_MS`
  Total Motor 1 up travel time

## Nextion Fields Updated
Main page objects used by this sketch:
- `tClock`
- `tState`
- `tFault`
- `tPress`
- `tRevs`
- `tEstop`
- `tDriver`
- `tCycle`
- `tLsRet`
- `tLsExt`
- `tEnc`
- `tMotor`
- `tMotorfwd`
- `tMotorrev`

Log page objects:
- `tClock1`
- `tFrame`
- `tLog`

Extra page clock:
- `tClock2`

## Display Meaning
- `tState`
  Current motion state like `M1 DOWN`, `M1 UP`, or `IDLE`
- `tFault`
  Current sequence step label
- `tLsRet`
  `M1 DONE`
- `tLsExt`
  `M1 POS: DOWN/UP`
- `tMotorfwd`
  `M1 ACTIVE`
- `tMotorrev`
  Current step interval value

## Notes
- `RTC_SET_TIME_ON_BOOT` is currently `false`
- This sketch is time-based, not sensor-based
- Travel accuracy depends on timing, motor performance, and whether any steps are missed

## Best Use
Use this sketch when the goal is:
- tune `M1` direction
- tune `M1` speed
- tune `M1` down/up travel
- verify the Nextion and RTC still work while only `M1` is active
