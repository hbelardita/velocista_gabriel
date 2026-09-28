# Feature: curve-navigation-straddle-tuning

## Objective
Adapt `LineFollowerController` logic to the physical straddle sensor topology (line in the middle, dark background under both sensors during straight cruising), tune kinematic cruising and turning parameters (`BASE_SPEED = 135`, `CURVE_SPEED = 0`, `TRIM_RIGHT = 20`), and prevent curve collisions by executing crisp zero-speed inner-wheel pivots.

## Problem
In curve navigation on a white line with dark background, the robot was running straight off the track and crashing into the perimeter. The controller logic assumed both sensors ride on top of the white line (`left && right`), whereas on the physical chassis the sensors straddle the line (`!left && !right`). When a sensor hit the line in a curve, `CURVE_SPEED = 100` resulted in an excessively wide turning radius at `BASE_SPEED = 160`, throwing the robot off track and leaving both sensors in black, causing it to resume high-speed straight travel into the wall.

## Why
Adapting the software truth table to the actual chassis sensor placement and stopping the inner wheel (`CURVE_SPEED = 0`) allows the differential drive to execute sharp pivots that comply with official track turn radii (30 cm) without exceeding linear momentum.

## Scope & Constraints
- Seam discipline: all navigation logic resides in `include/LineFollowerController.h`.
- Straddle truth table:
  - `!leftDetected && !rightDetected` -> Straight cruise: left motor `BASE_SPEED` (135), right motor `BASE_SPEED - TRIM_RIGHT` (115), both forward.
  - `!leftDetected && rightDetected` -> Steer right: left motor `BASE_SPEED` (135), right motor inner `CURVE_SPEED` (0), both forward.
  - `leftDetected && !rightDetected` -> Steer left: left motor inner `CURVE_SPEED` (0), right motor `BASE_SPEED - TRIM_RIGHT` (115), both forward.
  - `leftDetected && rightDetected` -> Transverse mark: continue straight at `BASE_SPEED`.
- Counter-rotation strictly excluded (`leftForward = true`, `rightForward = true`).
- 100% test pass on native host (`pio test -e native`) and clean Uno build (`pio run -e uno`).

## Tasks
- [x] TASK-01: Update `LineFollowerController.h` constants (`BASE_SPEED = 135`, `CURVE_SPEED = 0`) and straddle truth table in `update()`.
- [x] TASK-02: Update host unit tests in `test/test_controller_seam/test_controller.cpp` for the straddle truth table and verify TDD test suite.
- [x] TASK-03: Run native test suite (`pio test -e native`) and build Uno firmware (`pio run -e uno`).
- [x] TASK-04: Commit work-unit changes with Conventional Commit message and record verification evidence (commit `3b29de9`).

## Verification Evidence
- Work-unit commit: `3b29de9` ("feat(navigation): adapt controller to straddle sensor topology and zero inner-wheel curve speed")
- Native Unity tests (`pio test -e native`):
  - `test_initial_state_is_standby`: PASSED
  - `test_unpressed_button_on_boot_keeps_standby`: PASSED
  - `test_standby_ignores_sensor_readings_both_unpressed_and_held`: PASSED
  - `test_button_pressed_and_held_maintains_standby`: PASSED
  - `test_button_released_transitions_immediately_to_racing_with_indicator`: PASSED
  - `test_racing_state_persists_after_button_release`: PASSED
  - `test_straight_cruising_applies_base_speed_with_trim`: PASSED
  - `test_left_drift_steers_right_smoothly_forward`: PASSED
  - `test_right_drift_steers_left_smoothly_forward`: PASSED
  - `test_transverse_mark_continues_straight_ahead`: PASSED
  - `test_counter_rotation_strictly_excluded_in_active_cruising`: PASSED
  - `test_unified_track_polarity_normalization`: PASSED
  - `test_navigation_commands_across_both_track_polarities`: PASSED
  - Total: 13/13 passed (2.04 s).
- Embedded firmware build (`pio run -e uno`):
  - SUCCESS: RAM 20 bytes (1.0%), Flash 1800 bytes (5.6%).
- Debug build (`pio run -e uno_debug`):
  - SUCCESS: RAM 204 bytes (10.0%), Flash 4412 bytes (13.7%).

## Next Step
Flash firmware to Arduino Uno and perform real track run test.
