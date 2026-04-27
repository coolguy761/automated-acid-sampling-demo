# Nextion M3 Only Test

## File
[Nextion_M3_Only_Test_Uno.ino](C:/Users/coolg/Documents/Codex/2026-04-19-files-mentioned-by-the-user-acid/Nextion_M3_Only_Test_Uno/Nextion_M3_Only_Test_Uno.ino)

## Purpose
This sketch is for testing only Motor 3 with the existing Nextion screen and RTC clock.

It is meant to isolate tilt motion so `M3` speed and travel can be tuned without `M1`, `M2`, or the full production sequence interfering.

## What It Does
- Keeps Nextion communication on pins `10/11`
- Keeps DS3231 RTC clock updates on all pages
- Supports manual `M3` motion commands from the screen
- Supports a simple automatic cycle:
  - `M3 TILT OUT`
  - `M3 TILT HOME`

## Supported Commands
- `START`
  Runs one `M3 OUT -> M3 HOME` cycle
- `STOP`
  Stops motion immediately
- `RESET`
  Stops motion and clears the test state back to idle
- `STATUS`
  Logs the current sequence state
- `M3F`
  Manual Motor 3 forward
- `M3R`
  Manual Motor 3 reverse

## Pin Use
- `M3 STEP -> pin 6`
- `M3 DIR -> pin 7`
- `Shared ENA -> pin 8`
- `Nextion TX/RX -> pins 10/11`
- `RTC SDA/SCL -> A4/A5`

## Key Tuning Values
These are the main values to change during bench tuning:

```cpp
const unsigned long STEP_INTERVAL_M3_US      = 1800UL;
const unsigned long STEP_01_M3_TILT_OUT_MS   = 3000UL;
const unsigned long STEP_02_M3_TILT_HOME_MS  = 3000UL;
```

Meaning:
- `STEP_INTERVAL_M3_US`
  Lower value = faster stepping
  Higher value = slower stepping
- `STEP_01_M3_TILT_OUT_MS`
  Total tilt-out travel time
- `STEP_02_M3_TILT_HOME_MS`
  Total tilt-home travel time

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
  Current motion state like `TILT OUT`, `TILT HOME`, or `IDLE`
- `tFault`
  Current sequence step label
- `tLsRet`
  `M3 POS: OUT/HOME`
- `tEnc`
  `M3 DONE`
- `tMotor`
  `M3 ACTIVE`
- `tMotorfwd`
  Current step interval value
- `tMotorrev`
  Current tilt travel time

## Notes
- `RTC_SET_TIME_ON_BOOT` is currently `false`
- This sketch is time-based, not sensor-based
- If you slow `M3` by increasing `STEP_INTERVAL_M3_US`, and still want the same travel, increase the tilt time constants too

## Best Use
Use this sketch when the goal is:
- tune `M3` direction
- tune `M3` speed
- tune `M3` tilt-out and tilt-home travel
- verify the Nextion and RTC still work while only `M3` is active
