# Feature: straightaway-progressive-acceleration

## Objective
Implement progressive straightaway speed ramping in `LineFollowerController`: maintain safe cruising at `BASE_SPEED = 135` during initial straight detection, wait for straight confirmation (`STRAIGHT_ACCEL_DELAY_MS = 150`), and ramp motor speed smoothly up to `MAX_STRAIGHT_SPEED = 180` (+5 PWM every 50 ms). Instantly cancel speed boost and reset to `BASE_SPEED` upon any sensor line detection (entering a curve or transverse mark) to ensure safe corner entry.

## Problem
Currently, the robot cruises on straightaways at a static `BASE_SPEED = 135`. While this speed is well-calibrated for entering sharp turns, it leaves significant track lap-time on straightaways where the robot could safely travel faster.

## Why
Accelerating gradually on straightaways reduces lap times without sacrificing turn stability, provided the acceleration is delayed enough to avoid false straightaways (such as quick chicane transitions) and is immediately clamped back to `BASE_SPEED` the moment a turn is sensed.

## Scope & Constraints
- Seam discipline: all domain logic resides in `include/LineFollowerController.h`.
- Time tracking leverages `currentTimeMs` passed into `LineFollowerController::update()`.
- Baseline constants:
  - `BASE_SPEED = 135`
  - `MAX_STRAIGHT_SPEED = 180`
  - `STRAIGHT_ACCEL_DELAY_MS = 150`
  - `ACCEL_STEP_INTERVAL_MS = 50`
  - `ACCEL_STEP_PWM = 5`
  - `TRIM_RIGHT = 20`
- Immediate reset: touching any line (curve left, curve right, transverse mark) or leaving RACING state resets the straight cruise timer immediately.
- 100% test pass on native host (`pio test -e native`) and clean Uno builds (`pio run -e uno`, `pio run -e uno_debug`).

## Tasks
- [x] TASK-01: Update `include/LineFollowerController.h` with straightaway acceleration constants, tracking members (`inStraightCruise_`, `straightStartTimeMs_`), and ramping logic.
- [x] TASK-02: Expand test suite in `test/test_controller_seam/test_controller.cpp` to verify straight acceleration ramping, maximum speed ceiling (180), and immediate reset upon curve entry.
- [x] TASK-03: Run host test suite (`pio test -e native`) and build Uno firmware (`pio run -e uno`, `pio run -e uno_debug`).
- [x] TASK-04: Commit work-unit changes with Conventional Commit message and record verification evidence (commit `028cd3a`).

## Implementation Route & Triggers
- Route: Delegated direct (Implementation touches 2 non-trivial source and test files: `LineFollowerController.h` and `test_controller.cpp`).
- Writer trigger fired: Delegated to bounded writer agent (`cd42907d-c896-47bc-a32f-b71e7d2fa504`).

## Verification Evidence
- Work-unit commit: `028cd3a` ("feat(navigation): implement progressive straightaway speed ramping with instant curve reset")
- Native host unit tests (`pio test -e native`):
  - 16/16 test cases passed (1.37 s), including:
    - `test_straightaway_progressive_acceleration`: PASSED
    - `test_curve_entry_instantly_cancels_straightaway_speed_boost`: PASSED
- Embedded firmware build (`pio run -e uno`):
  - SUCCESS: RAM 31 bytes (1.5%), Flash 2406 bytes (7.5%).
- Debug firmware build (`pio run -e uno_debug`):
  - SUCCESS: RAM 215 bytes (10.5%), Flash 4434 bytes (13.7%).

## Next Step
Flash firmware to Arduino Uno (`pio run -t upload`) and perform physical track verification.
