# Feature: two-stage-curve-steering

## Objective
Implement two-stage steering damping in `LineFollowerController`: upon line detection by an outer sensor, apply stage-1 gentle steering (stopping the inner wheel forward at `CURVE_SPEED = 0`) for the first `REVERSE_ENGAGEMENT_DELAY_MS = 70` ms to eliminate zig-zag oscillation on straights and gentle bends, and transition to stage-2 active counter-rotation (`TURN_REVERSE_PWM = 90` in reverse) only if the curve persists beyond 70 ms to successfully navigate tight hairpin curves.

## Problem
In commit `3409b6c`, counter-rotation (`TURN_REVERSE_PWM = 90`) was enabled unconditionally on line touch. While this allowed the robot to successfully complete tight curves on the track without crashing, it introduced severe limit-cycle oscillation ("hunting" / zig-zag) during straight cruising and mild turns, because every transient line contact immediately applied aggressive reverse torque, causing yaw overshoot into the opposite sensor.

## Why
A time-persistence threshold separates transient deviations (which need only gentle damping to recenter) from true sharp curves (which require high yaw torque from inner-wheel reverse). This preserves tight curve completion while keeping straight cruising smooth and stable.

## Scope & Constraints
- Seam discipline: all domain logic remains in `include/LineFollowerController.h`.
- Time-based state tracking uses the existing `currentTimeMs` parameter in `LineFollowerController::update()`.
- Stage 1 (< 70 ms): inner wheel `CURVE_SPEED = 0`, forward direction (`forward = true`).
- Stage 2 (>= 70 ms): inner wheel `TURN_REVERSE_PWM = 90`, reverse direction (`forward = false`).
- Centered cruising (`!left && !right`) and transverse lines (`left && right`) immediately reset the turn persistence timer.
- 100% test pass on native host (`pio test -e native`) and Uno firmware build (`pio run -e uno`).

## Tasks
- [x] TASK-01: Update `include/LineFollowerController.h` with `REVERSE_ENGAGEMENT_DELAY_MS = 70`, turn timing tracking, and two-stage steering logic.
- [x] TASK-02: Update and expand unit tests in `test/test_controller_seam/test_controller.cpp` to verify Stage 1 (t < 70 ms) and Stage 2 (t >= 70 ms) for both left and right turns.
- [x] TASK-03: Run host test suite (`pio test -e native`) and compile firmware (`pio run -e uno`).
- [x] TASK-04: Commit work-unit changes with Conventional Commit message and record verification evidence (commit `d85033b`).

## Implementation Route & Triggers
- Route: Delegated direct (Implementation touches 2 non-trivial source and test files: `LineFollowerController.h` and `test_controller.cpp`).
- Writer trigger fired: Delegated to bounded writer agent (`be8d5801-8c21-477b-90cb-999a4a6b4227`).

## Verification Evidence
- Work-unit commit: `d85033b` ("feat(navigation): implement two-stage steering damping to eliminate track oscillations")
- Native host unit tests (`pio test -e native`):
  - `test_initial_state_is_standby`: PASSED
  - `test_unpressed_button_on_boot_keeps_standby`: PASSED
  - `test_standby_ignores_sensor_readings_both_unpressed_and_held`: PASSED
  - `test_button_pressed_and_held_maintains_standby`: PASSED
  - `test_button_released_transitions_immediately_to_racing_with_indicator`: PASSED
  - `test_racing_state_persists_after_button_release`: PASSED
  - `test_straight_cruising_applies_base_speed_with_trim`: PASSED
  - `test_left_drift_steers_right_smoothly_forward`: PASSED (Stage 1 soft brake @ t=320ms, Stage 2 reverse @ t=380ms)
  - `test_right_drift_steers_left_smoothly_forward`: PASSED (Stage 1 soft brake @ t=320ms, Stage 2 reverse @ t=380ms)
  - `test_transverse_mark_continues_straight_ahead`: PASSED
  - `test_counter_rotation_strictly_excluded_in_active_cruising`: PASSED
  - `test_unified_track_polarity_normalization`: PASSED
  - `test_navigation_commands_across_both_track_polarities`: PASSED (Stage 1 & Stage 2 across WHITE_LINE and BLACK_LINE)
  - `test_turn_timer_resets_when_re_centering`: PASSED
  - Total: 14/14 test cases passed (1.67 s).
- Embedded firmware build (`pio run -e uno`):
  - SUCCESS: RAM 26 bytes (1.3%), Flash 2040 bytes (6.3%).
- Debug firmware build (`pio run -e uno_debug`):
  - SUCCESS: RAM 210 bytes (10.3%), Flash 4424 bytes (13.7%).

## Next Step
Flash firmware to Arduino Uno (`pio run -t upload`) and perform physical track verification.
