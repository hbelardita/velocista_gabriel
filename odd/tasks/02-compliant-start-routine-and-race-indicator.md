# Feature: 02-compliant-start-routine-and-race-indicator

## Objective
Implement and verify the official competition starting sequence and visual indicator in compliance with Rules 2.2.3, 2.2.7, and 4.1. The robot must stay stationary in `STANDBY` (motors stopped, LED indicator OFF) on the grid while the start button (Pin 7) is held, immediately transition to `RACING` with the indicator LED (Pin 13) illuminated upon button release, and completely eliminate hardcoded startup delays (`delay(2000)`).

## Problem
Previous code used an uncoordinated hardcoded delay (`delay(2000)`) without a start button sequence or race indicator LED, violating technical scrutineering rules and risking false starts or disqualification. The robot must maintain zero motor output on the grid regardless of sensor readings until the release of the start button.

## Why
Technical scrutineering (Rules 2.2.3, 2.2.7, and 4.1) requires a manual start button held on the grid and released on the referee's command, as well as an active optical indicator whenever the robot is in its autonomous race routine.

## Scope & Constraints
- Pure domain controller seam (`include/LineFollowerController.h`) enforces state machine: `STANDBY` -> `RACING`.
- Motors strictly stopped (`PWM = 0`, `isStopped = true`) while in `STANDBY`, even if sensors detect lines or noise both unpressed and held.
- Start button on Pin 7 configured with internal pull-up (`INPUT_PULLUP`), active LOW when pressed.
- Race indicator on Pin 13 (`LED_BUILTIN`), energized (`HIGH`) when `indicatorActive` is true.
- Zero `delay()` calls in startup or loop sequence.
- Comprehensive host unit tests under `test/test_controller_seam/test_controller.cpp`.
- UNO firmware build (`[env:uno]`) and native tests (`[env:native]`) must pass.

## Tasks
- [x] TASK-01: Verify hardware pin mapping, pull-up configuration (`INPUT_PULLUP`), and absence of hardcoded delays in `src/main.cpp`.
- [x] TASK-02: Write comprehensive TDD unit tests in `test/test_controller_seam/test_controller.cpp` for start sequence, sensor isolation during standby, indicator state, and release transition.
- [x] TASK-03: Verify domain controller implementation in `include/LineFollowerController.h` against all scrutineering requirements.
- [x] TASK-04: Execute host-native test suite (`pio test -e native`) and build target firmware (`pio run -e uno`).
- [x] TASK-05: Update issue specification checklist in `.scratch/velocista-amateur/issues/02-compliant-start-routine-and-race-indicator.md` and complete code review.

## Verification Evidence
- Native Unity tests (`pio test -e native`):
  - `test_initial_state_is_standby`: PASSED
  - `test_unpressed_button_on_boot_keeps_standby`: PASSED
  - `test_standby_ignores_sensor_readings_both_unpressed_and_held`: PASSED
  - `test_button_pressed_and_held_maintains_standby`: PASSED
  - `test_button_released_transitions_immediately_to_racing_with_indicator`: PASSED
  - `test_racing_state_persists_after_button_release`: PASSED
  - Total: 6/6 passed (1.72 s).
- Embedded firmware build (`pio run -e uno`):
  - SUCCESS: RAM 18 bytes (0.9%), Flash 1716 bytes (5.3%).
- Code review:
  - Standards: 0 hard violations. Adheres to canonical domain language (`pulsador de largada`, `Rutina de Carrera`). Refactored test assertions to helper function `assert_standby_outputs`.
  - Spec: 0 hard defects. Confirmed hardware pin mapping (Pin 7 pullup, Pin 13 LED output) and verified grid staging with button held under sensor noise.

## Next Step
Issue 02 complete. Ready for Issue 03: `03-forward-differential-steering-and-track-normalization.md`.
