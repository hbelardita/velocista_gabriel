# 03: Forward Differential Steering and Track Navigation

**What to build:** Implement line tracking navigation with forward differential steering and configurable track polarity (black line or white line). The robot drives straight at cruise speed when centered on the line, and smoothly slows down the inner wheel in the forward direction without counter-rotation when a sensor drifts, protecting the plastic gearbox and preserving linear momentum.

**Blocked by:** 02: Compliant Start Routine and Race Indicator

**Status:** closed

- [x] Unified track polarity parameter configured to handle white-line or black-line surfaces without logic duplication.
- [x] Straight cruising logic applies `BASE_SPEED` with right motor trim compensation when both sensors track the line.
- [x] Differential turn reduces inner-wheel speed forward (to ~35-40% PWM) while sustaining outer-wheel speed forward.
- [x] Counter-rotation / motor reversal is strictly excluded from active cruising states.
- [x] Unit tests verify motor output commands for centered, left drift, and right drift across both track polarities.
