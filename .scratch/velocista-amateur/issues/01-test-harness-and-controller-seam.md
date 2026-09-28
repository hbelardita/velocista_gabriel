# 01: Test Harness and Controller Seam

**What to build:** Set up a host-native testing environment and establish a pure domain controller interface (`LineFollowerController`) alongside an initial hardware adapter in `main.cpp`, proving end-to-end decoupling with a passing native test suite running without physical hardware.

**Blocked by:** None (can start immediately)

**Status:** ready-for-agent

- [ ] PlatformIO configured with a native environment to execute unit tests on the host machine.
- [ ] Pure C++ domain seam (`LineFollowerController`) created with no direct Arduino hardware dependencies.
- [ ] Hardware adapter in `main.cpp` refactored to pass sensor inputs to the controller and apply motor outputs.
- [ ] Initial automated test suite passes on host execution (`pio test -e native`).
