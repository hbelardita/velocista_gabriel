# 02: Compliant Start Routine and Race Indicator

**What to build:** Implement the official starting sequence and visual indicator so the robot remains stationary while the driver holds the start button on Pin 7 in the grid, immediately enters the active race routine with the onboard LED (Pin 13) illuminated upon button release, and eliminates the hardcoded delay to pass technical scrutineering (Rules 2.2.3, 2.2.7, and 4.1).

**Blocked by:** 01: Test Harness and Controller Seam

**Status:** completed

- [x] Controller maintains `STANDBY` state with motors stopped (`PWM = 0`) while start button is held.
- [x] Controller transitions immediately to `RACING` state and energizes race indicator LED when start button is released.
- [x] Hardcoded `delay(2000)` completely removed from startup sequence.
- [x] Unit tests verify state transitions from `STANDBY` to `RACING` based on button events.
- [x] Hardware wiring and pullup configuration verified on Arduino Pin 7 and Pin 13 (`LED_BUILTIN`).
