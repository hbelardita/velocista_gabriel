# Feature: 03-forward-differential-steering-and-track-navigation

## Objective
Implement line tracking navigation with forward differential steering and configurable track polarity (`WHITE_LINE` or `BLACK_LINE`). When centered on the trajectory line, the robot cruises straight at `BASE_SPEED` with right motor trim compensation. When a sensor drifts off the line, the inner wheel smoothly slows down in the forward direction (`CURVE_SPEED` ~39% PWM) while the outer wheel sustains forward drive, strictly excluding motor reversal or counter-rotation.

## Problem
Previous code in `velocista_gabriel.ino` used hardcoded pin inversions, abrupt motor counter-rotation / reverse pivots that risk stripping the yellow DC motor plastic gearboxes, and duplicated track polarity handling in hardware-specific code without automated test coverage.

## Why
Differential forward steering preserves the robot's linear momentum and mechanical gearbox integrity (ADR 0002). A unified track polarity configuration allows rapid adaptation between white-line and black-line racing venues without code duplication.

## Scope & Constraints
- Pure domain controller seam (`include/LineFollowerController.h`) enforces navigation logic in `RACING` state.
- `TrackPolarity` enum (`WHITE_LINE`, `BLACK_LINE`) and unified normalization function (`normalizeSensors`).
- Straight cruising: Left motor `BASE_SPEED` (160), Right motor `BASE_SPEED - TRIM_RIGHT` (140), both forward.
- Differential steering on left drift (`leftDetected == false`, `rightDetected == true`): Left motor `BASE_SPEED` (160), Right motor inner `CURVE_SPEED` (100), both forward.
- Differential steering on right drift (`leftDetected == true`, `rightDetected == false`): Left motor inner `CURVE_SPEED` (100), Right motor outer `BASE_SPEED - TRIM_RIGHT` (140), both forward.
- Counter-rotation strictly forbidden in active cruising (`leftForward == true && rightForward == true`).
- Hardware adapter in `src/main.cpp` delegates sensor normalization and navigation to `LineFollowerController`.
- All native unit tests (`pio test -e native`) and target build (`pio run -e uno`) must pass.

## Tasks
- [x] TASK-01: Define `TrackPolarity` enum, normalization helper, and kinematic constants (`CURVE_SPEED = 100`) in `include/LineFollowerController.h`.
- [x] TASK-02: Write comprehensive TDD unit tests in `test/test_controller_seam/test_controller.cpp` for centered cruise, differential turns (left/right drift), forward-only directions, and track polarities.
- [x] TASK-03: Implement differential steering and sensor navigation in `LineFollowerController::update`.
- [x] TASK-04: Integrate unified track polarity in `src/main.cpp` using `LineFollowerController`.
- [x] TASK-05: Run host native unit test suite (`pio test -e native`) and AVR firmware build (`pio run -e uno`).
- [x] TASK-06: Update issue specification checklist in `.scratch/velocista-amateur/issues/03-forward-differential-steering-and-track-navigation.md` and complete code review.

## Verification Evidence
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
  - `test_counter_rotation_strictly_excluded_in_active_cruising`: PASSED
  - `test_unified_track_polarity_normalization`: PASSED
  - `test_navigation_commands_across_both_track_polarities`: PASSED
  - Total: 12/12 passed (1.35 s).
- Embedded firmware build (`pio run -e uno`):
  - SUCCESS: RAM 20 bytes (1.0%), Flash 1804 bytes (5.6%).
- Code review:
  - Standards: 0 hard violations. Clean domain seam in `LineFollowerController.h`, zero logic duplication for polarity, zero reverse-gear active steering.
  - Spec: 0 hard defects. Verified right motor trim compensation on straight, 39.2% PWM inner curve speed on drift, no motor reversal, unified track polarity.

## Next Step
Issue 03 complete. Ready for Issue 04: `04-despiste-detection-and-rescue-window-failsafe.md`.
