# 04: Despiste Detection and Rescue Window Fail-Safe

**What to build:** Implement track-loss detection (*Despiste*) and safety handling so that if both sensors lose the line, the controller tracks elapsed time, executes an active rescue turn toward the last detected sensor for up to 200 ms, and triggers an immediate complete motor shutdown if unrecovered, preventing runaway accidents and opponent lane invasion (Rule 6.3.4).

**Blocked by:** 03: Forward Differential Steering and Track Navigation

**Status:** ready-for-agent

- [ ] Controller records last known line direction during active navigation.
- [ ] Controller enters `RESCUING` state when both sensors lose the line, steering toward the last active sensor.
- [ ] Rescue window is bounded to 200 ms; if line is reacquired within 200 ms, robot returns to `RACING`.
- [ ] If track loss exceeds 200 ms, controller enters `EMERGENCY_STOP` and cuts all motor outputs to zero (`PWM = 0`).
- [ ] Unit tests verify rescue timing, reacquisition transitions, and emergency stop enforcement.
