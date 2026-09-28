# Feature: serial-debug-mode

## Objective
Implement an interactive Serial Debug Mode for hardware verification (motor test and real-time monitoring of sensors and pulsador de largada). The mode is conditionally compiled under `DEBUG_MODE` and configured in `platformio.ini` via a dedicated `[env:uno_debug]` environment to guarantee zero overhead in competitive racing mode (`[env:uno]`).

## Problem
Currently, validating sensor readings, line detection polarities, motor wiring directions, and the start button requires either blind runs or ad-hoc test code that risks contaminating the racing firmware.

## Why
A dedicated hardware testing environment allows rapid bench testing without uploading temporary sketches, verifying wiring, optical sensor detection, and motor forward directions safely.

## Scope & Constraints
- `platformio.ini`: add `[env:uno_debug]` with `build_flags = -DDEBUG_MODE`, `monitor_speed = 115200`, `monitor_filters = send_on_enter`.
- Production build `[env:uno]` must remain clean with zero debug strings or Serial polling overhead.
- Serial console interactive menu:
  - '1': Motor Izquierdo forward (PWM 150)
  - '2': Motor Derecho forward (PWM 150)
  - '3': Ambos Motores forward (PWM 150)
  - 's': Parar motores (PWM 0)
  - '4': Telemetría continua de sensores y pulsador cada 200 ms (detenible con cualquier tecla o 'q')
  - 'm' o 'h': Mostrar menú de comandos
- Continuous motor movement until explicit stop ('s') as agreed in the design interview.
- Pure domain unit tests (`pio test -e native`) must pass unaffected.

## Tasks
- [x] TASK-01: Configure `[env:uno_debug]` in `platformio.ini` with `-DDEBUG_MODE` and `monitor_speed = 115200`.
- [x] TASK-02: Implement `DebugConsole` / serial debug routine in `src/main.cpp` protected by `#ifdef DEBUG_MODE`.
- [x] TASK-03: Verify builds with `pio run -e uno`, `pio run -e uno_debug`, and test suite with `pio test -e native`.

## Verification Evidence
- Embedded production build (`pio run -e uno`):
  - RAM: 20 bytes (1.0%), Flash: 1804 bytes (5.6%). Zero overhead introduced.
- Embedded debug build (`pio run -e uno_debug`):
  - RAM: 204 bytes (10.0%), Flash: 4412 bytes (13.7%). Includes interactive Serial console at 115200 baud.
- Pure domain unit tests (`pio test -e native`):
  - 12/12 passed (2.52 s). All regression tests green.

## Next Step
Ready for bench testing with hardware using `pio run -t upload -e uno_debug` and `pio device monitor -e uno_debug`.
