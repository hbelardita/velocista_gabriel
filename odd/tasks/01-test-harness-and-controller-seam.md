# Feature: 01-test-harness-and-controller-seam

## Objective
Set up a host-native testing environment and establish a pure domain controller interface (`LineFollowerController`) alongside an initial hardware adapter in `src/main.cpp`, proving end-to-end decoupling with a passing native test suite running without physical hardware.

## Problem
Tight coupling between Arduino hardware peripherals and navigation logic in `src/main.cpp` prevents automated testing on host environments and risks race homologation or mechanical failures. All motor commands and sensor reads are currently mixed in loop functions.

## Why
Decoupling the domain logic behind a clean seam enables fast, automated TDD testing on the host machine without connecting Arduino hardware, guaranteeing deterministic safety and state transitions.

## Scope & Constraints
- Pure C++ domain interface (`include/LineFollowerController.h`) with zero Arduino hardware headers (`<Arduino.h>`).
- Configured PlatformIO native testing environment (`[env:native]`) targeting host compiler (`toolchain-gccmingw32` / GCC).
- Host test suite under `test/` running with Unity on `native`.
- `src/main.cpp` acting solely as a hardware adapter feeding `SensorInputs` and applying `MotorOutputs`.
- Arduino UNO compilation (`[env:uno]`) must remain unbroken.

## Tasks
- [x] TASK-01: Configure `platformio.ini` with `[env:native]` supporting host unit testing without target microcontroller.
- [x] TASK-02: Implement pure domain seam `include/LineFollowerController.h` defining `SensorInputs`, `MotorOutputs`, `ControllerOutputs`, `RobotState`, and `LineFollowerController` class.
- [x] TASK-03: Create automated unit test suite under `test/test_controller_seam/test_controller.cpp` verifying the controller seam and initial state behaviors.
- [x] TASK-04: Refactor `src/main.cpp` hardware adapter to delegate navigation decisions to `LineFollowerController`.
- [x] TASK-05: Verify full test suite passing on host (`pio test -e native`) and build on UNO target (`pio run -e uno`).

## Verification Evidence
- Native Unity tests (`pio test -e native`):
  - `test_initial_state_is_standby`: PASSED
  - `test_button_pressed_keeps_standby`: PASSED
  - `test_button_released_transitions_to_racing`: PASSED
  - Total: 3/3 passed (1.33 s).
- Embedded firmware build (`pio run -e uno`):
  - SUCCESS: RAM 18 bytes (0.9%), Flash 1716 bytes (5.3%).
- Code review:
  - Standards: Cleaned up domain vocabulary to `PIN_PULSADOR_LARGADA` and extracted `setMotor` to eliminate H-bridge code duplication.
  - Spec: Verified seam decoupling and test harness operational.

## Next Step
Proceed to Issue 02: `02-compliant-start-routine-and-race-indicator.md`.
