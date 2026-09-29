# Feature: four-sensor-array-navigation

## Objective
Upgrade the line follower robot navigation from a 2-sensor straddle layout to a 4-sensor discrete array (S1..S4 TCRT5000), maximizing the competition limit (Art 1.2.1), eliminating reliance on arbitrary temporal delays to detect curves, enabling spatial two-stage steering, and securing high-speed navigation without track loss in 30 cm radius curves on elevated tracks.

## Problem
With the previous 2-sensor straddle arrangement, the robot could not distinguish between a slight drift and an abrupt 30 cm curve by sensor position alone, relying instead on a time delay (`REVERSE_ENGAGEMENT_DELAY_MS`) to escalate from soft steering to counter-rotation. This limited straightaway speed to ~130 PWM and caused either zigzag oscillations or overshoot on sharp turns.

## Why
A 4-sensor layout provides physical/spatial discrimination:
- Inner sensors (S2, S3) ride inside the 20 mm line (12 mm center-to-center pitch), providing continuous confirmation of centered cruise and fine proportional damping (Etapa 1).
- Outer sensors (S1, S4) sit outside (19 mm pitch from inner sensors, 50 mm center span), triggering immediate sharp turning / counter-rotation (Etapa 2) the moment the line moves outward.
- Unambiguous line loss (`[0, 0, 0, 0]`) triggers a deterministic 200 ms rescue routine spinning toward the last active outer sensor before emergency shutdown (Art 6.3.4).

## Geometry & Pinout
- **Chassis**: 115 mm front width.
- **Sensor Footprint**: 10 mm width each.
- **Placement**:
  - `S1` (Ext Izq): x = -25 mm, Pin A0
  - `S2` (Int Izq): x = -6 mm, Pin 2
  - `S3` (Int Der): x = +6 mm, Pin 3
  - `S4` (Ext Der): x = +25 mm, Pin A1
- **Center span**: 50 mm between S1 and S4 centers (60 mm total footprint, 27.5 mm margins to chassis edges).
- **Line**: 20 mm width (spans -10 mm to +10 mm, covering S2 and S3 when centered).

## Scope & Constraints
- Seam discipline: Domain logic in `include/LineFollowerController.h`.
- Hardware layer in `src/main.cpp`.
- Host test suite in `test/test_controller_seam/test_controller.cpp`.
- 100% test pass on native host (`pio test -e native`).
- Clean builds for Uno production and debug (`pio run -e uno`, `pio run -e uno_debug`).

## Tasks
- [x] TASK-01: Update documentation (`docs/adr/0001-sensores-de-linea.md` and `docs/spec-velocista-amateur.md`) with the 4-sensor layout, geometry, pinout, and spatial 2-stage state model.
- [x] TASK-02: Refactor `LineFollowerController.h` to accept 4 sensors (`s1_outerLeft`, `s2_innerLeft`, `s3_innerRight`, `s4_outerRight`), implement spatial steering logic, and bounded 200 ms rescue window.
- [x] TASK-03: Update `src/main.cpp` pin definitions (S1=A0, S2=2, S3=3, S4=A1), normalization, and interactive debug telemetry menu for all 4 sensors.
- [x] TASK-04: Expand unit tests in `test/test_controller_seam/test_controller.cpp` covering all 4-sensor states, cruise acceleration, outer curve engagement, and rescue window expiry.
- [x] TASK-05: Run host test suite (`pio test -e native`), compile firmware targets, and commit work unit.

## Verification Evidence
- `pio test -e native`: 19/19 tests passed (100% success).
- `pio run -e uno`: Build succeeded (Flash: 2704 bytes [8.4%], RAM: 31 bytes [1.5%]).
- `pio run -e uno_debug`: Build succeeded (Flash: 5694 bytes [17.7%], RAM: 215 bytes [10.5%]).

## Implementation Route & Triggers
- Route: Delegated direct / Work-unit commit.
- Mandatory triggers: Writer trigger (touches `LineFollowerController.h`, `main.cpp`, `test_controller.cpp`, docs).
