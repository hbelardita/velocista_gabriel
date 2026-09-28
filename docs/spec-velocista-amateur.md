# Specification: Amateur Line Follower Robot (Velocista Amateur)

## Problem Statement

The existing robot codebase features an open-loop start sequence (`delay(2000)`), lacks the mandatory race status indicator, and couples sensor reading directly to aggressive counter-rotation motor commands without detecting track loss. Under the official racing regulation (`reglamento-carreras.pdf`), this configuration fails technical homologation (Rules 2.2.3, 2.2.7, and 4.1), risks disqualification for false starts or lane invasion (Rules 6.1.2, 6.3.4), and endangers mechanical components by reversing DC plastic-geared motors at speed. Furthermore, tight coupling between Arduino hardware peripherals and navigation logic prevents automated testing on host environments.

## Solution

Implement an autonomous navigation system structured as a deep controller module with a single high-level seam. The system introduces:
1. A compliant starting sequence that holds in standby while a physical push button is depressed and enters the race routine immediately upon button release.
2. An active indicator light signaling the race routine.
3. Configurable track polarity handling (white line on black background or black line on white background).
4. Smooth forward differential drive steering that reduces inner-wheel speed without mechanical counter-rotation.
5. A deterministic track-loss (*Despiste*) fail-safe that executes a bounded 200 ms rescue window toward the last known track direction before triggering a complete emergency stop to avoid opponent lane invasion.

## User Stories

1. As a competition driver, I want to keep the start button pressed on the starting grid, so that the robot remains stationary in standby without moving prematurely.
2. As a competition driver, I want the robot to initiate the race routine immediately when I release the start button upon the referee's command, so that I achieve a fast and compliant launch without false starts.
3. As a technical scrutineer (homologation judge), I want to see the robot remain stopped while the button is pressed and launch only upon release, so that the robot complies with Article 2.2.3 and Article 4.1.1.
4. As a technical scrutineer, I want a visible LED indicator to illuminate whenever the robot is in its active race routine, so that the robot complies with Article 2.2.7 and Article 4.1.2.
5. As a competition driver, I want the robot to navigate straight ahead when both line sensors detect the trajectory line, so that maximum forward velocity is sustained on straight track segments.
6. As a competition driver, I want the robot to execute forward differential steering when the left sensor loses the line, so that the robot steers right smoothly without stripping plastic gearbox teeth.
7. As a competition driver, I want the robot to execute forward differential steering when the right sensor loses the line, so that the robot steers left smoothly without stripping plastic gearbox teeth.
8. As a competition driver, I want the robot to remember the last detected line direction when entering a sharp curve or experiencing brief wheel slip, so that recovery steering is directed toward the correct turn.
9. As a race referee, I want the robot to attempt trajectory reacquisition for at most 200 ms upon complete line loss (*Despiste*), so that momentary deviations on surface irregularities do not abort the run unnecessarily.
10. As a race referee, I want the robot to execute an immediate full stop if line reacquisition fails after 200 ms, so that the robot never crosses into the opposing robot's lane (Article 6.3.4) or runs off the circuit.
11. As a race engineer, I want to toggle track polarity between black-line and white-line modes via a single configuration parameter, so that the robot adapts to competition venue changes without algorithmic rewrites.
12. As a firmware developer, I want to execute unit tests on my development host without hardware connected, so that navigation logic, safety timeouts, and state transitions are verified in milliseconds before deployment.

## Implementation Decisions

### 1. Navigation Controller Seam
All navigation logic, safety timeouts, steering kinematics, and race state transitions are isolated behind a single deep interface (`LineFollowerController`).

```cpp
// Pure domain types representing the seam
struct SensorInputs {
  bool leftDetected;
  bool rightDetected;
};

struct MotorOutputs {
  uint8_t leftPwm;
  uint8_t rightPwm;
  bool leftForward;
  bool rightForward;
};

struct ControllerOutputs {
  MotorOutputs motors;
  bool indicatorActive;
  bool isStopped;
};

enum class RobotState {
  STANDBY,
  RACING,
  RESCUING,
  EMERGENCY_STOP
};
```

### 2. State Machine and Start Routine
- `STANDBY`: Initial state upon boot. The indicator LED remains OFF and motors remain stopped (`PWM = 0`). The controller waits for the start button to transition from pressed (`LOW` on `INPUT_PULLUP`) to released (`HIGH`).
- `RACING`: Active navigation state. The indicator LED is energized (`HIGH`). Forward differential steering guides the robot along the line.
- `RESCUING`: Entered when both sensors simultaneously lose the line while in `RACING`. A timer records elapsed loss time while maintaining differential steering toward the last active sensor.
- `EMERGENCY_STOP`: Entered if track loss duration exceeds 200 ms. Both motors are cut (`PWM = 0`), and the indicator LED is deactivated.

### 3. Kinematic Drive Logic
- Straight cruise: Left motor at `BASE_SPEED`, right motor at `BASE_SPEED - TRIM_RIGHT`.
- Steer left: Left wheel inner motor speed drops to `CURVE_SPEED` (`~35-40%` PWM) forward; right wheel outer motor maintains `BASE_SPEED - TRIM_RIGHT` forward.
- Steer right: Left wheel outer motor maintains `BASE_SPEED` forward; right wheel inner motor drops to `CURVE_SPEED` forward.
- Reverse gear / counter-rotation is strictly forbidden during active navigation to protect the plastic gearbox assembly of the yellow DC motors.

### 4. Hardware Adapter Layer
The entry point (`main.cpp`) serves solely as an adapter at the outer perimeter:
- Reads digital sensor pins and the start button pin.
- Inverts sensor polarity according to the configured track setting.
- Queries `millis()` and passes elapsed time and inputs to `LineFollowerController`.
- Writes calculated PWM and direction signals to the motor driver pins and toggles the onboard indicator pin (`LED_BUILTIN`).

## Testing Decisions

### Seam Discipline
- External behavior is tested solely across the `LineFollowerController` seam. Tests feed simulated `SensorInputs`, button events, and clock timestamps (`currentTimeMs`), asserting expected `ControllerOutputs` and state transitions.
- Internal private state, specific timer variables, and pin mappings are never asserted.
- Tests run natively on host C++ using PlatformIO Native environment or Unity test framework.

### Test Scenarios
- **Start Sequence Test**: Verify motors stay stopped while button is pressed; verify immediate transition to `RACING` with LED enabled on button release.
- **Differential Steering Test**: Verify inner-wheel speed reduction occurs in the forward direction when one sensor leaves the line, maintaining outer-wheel speed.
- **Rescue Window Timing Test**: Verify controller remains in `RESCUING` state applying last-direction turn for `< 200 ms`.
- **Emergency Stop Expiry Test**: Verify controller cuts motors to zero and enters `EMERGENCY_STOP` when track loss exceeds `200 ms`.
- **Track Recovery Test**: Verify controller seamlessly transitions from `RESCUING` back to `RACING` when any sensor reacquires the line before timeout.

## Out of Scope

- Remote start control via IR or Bluetooth (Amateur standard relies strictly on manual start button).
- PID control using analog reflectance values (two digital sensors dictate discrete differential states).
- Multi-sensor array expansion beyond the initial 2-sensor layout (upgrade path documented in ADR 0001).
- Automatic lap counting or finish line detection sensors.

## Further Notes

- All implementation decisions align with `docs/adr/0001-sensores-de-linea.md`, `docs/adr/0002-traccion-diferencial.md`, and `docs/adr/0003-deteccion-despiste.md`.
- Domain terms strictly adhere to the glossary in `CONTEXT.md`.
