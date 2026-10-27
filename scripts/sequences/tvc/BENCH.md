# Hornet TVC bench test

## Before you start

- Hornet flashed from `akshay/hornet-build-fix`. At boot both actuators start directly at neutral (1500 us, 25 mm). There is no full-retract pulse before centring.
- LAC signal wires: pitch on Teensy pin 28, yaw on pin 29. Pins 24 and 25 produce no PWM output. Confirmed on pitch only: the actuator centres at boot and tracks the bench trace. Yaw on pin 29 is untested.
- Board reachable at `169.254.99.99`. The client connects at startup, so power the board first.
- Keep clear of the gimbal. Each run moves one axis to +6 deg, back, then -6 deg, back.

## Run

```
cd scripts
uv run client-new.py
```

At `CMD>`:

1. `loadtvc`, answer `y` to "Load a saved sequence?", pick `tvc_bench_pitch`. State goes IDLE → TVC_PRIMED.
2. `starttvc`, confirm. State goes TVC → IDLE after 12 s.
3. Repeat with `tvc_bench_yaw`.

`s` shows live status. Watch "TVC pitch cmd" / "TVC yaw cmd" (deg), "Pitch servo" / "Yaw servo" (us, the command) and "Pitch servo out" / "Yaw servo out" (us, the pulse actually written after the actuator clamp).

## Expected, moving axis

| Time (s) | Command | Pulse (us) | Extension (mm) |
|---|---|---|---|
| 0 – 2 | 0 deg | 1500 | 25.00 |
| 2 – 5 | +6 deg | 1639 | 31.93 |
| 5 – 7 | 0 deg | 1500 | 25.00 |
| 7 – 10 | -6 deg | 1359 | 17.94 |
| 10 – 12 | 0 deg | 1500 | 25.00 |

The other axis stays at 1500 us throughout.

The command steps, but the actuator ramps at 18 mm/s: each 6 deg move takes about 0.39 s, so the pulse climbs through intermediate values before settling (a full 12 deg swing would take about 0.76 s). Pulses are rounded to the nearest us.

## Full range

`tvc_bench_pitch_full` and `tvc_bench_yaw_full` run the same shape at ±12 deg. 12 deg is the firmware clamp in HornetTvc, so a larger command moves the nozzle no further.

| Time (s) | Command | Pulse (us) | Extension (mm) |
|---|---|---|---|
| 0 – 2 | 0 deg | 1500 | 25.00 |
| 2 – 5 | +12 deg | 1773 | 38.67 |
| 5 – 7 | 0 deg | 1500 | 25.00 |
| 7 – 10 | -12 deg | 1216 | 10.81 |
| 10 – 12 | 0 deg | 1500 | 25.00 |

Each 12 deg move takes about 0.76 s toward +12 and 0.79 s toward -12, since the linkage needs more travel on the retract side.

## Stop

`abort` (or any error during the run) commands both actuators back to 1500 us, and telemetry shows 0 deg, 25 mm and 1500 us for both axes. They still move at 18 mm/s, so recentring takes about 0.4 s from 6 deg and 0.8 s from 12 deg. Quitting the client does not stop a running trace. For a hard stop, cut actuator power.

## Open hardware checks

The values below are set in `clover/src/hornet/HornetTvc.cpp:32-39`. They are defaults, not measurements. Through the angle layer the firmware only commands 10.81 to 38.67 mm (1216 to 1773 us), so check 2 needs a servo tester or signal generator on the LAC input.

1. At 0 deg command, measure rod extension and nozzle angle on each axis. Set `HORNET_TVC_PITCH_NEUTRAL_EXTENSION_MM` and `HORNET_TVC_YAW_NEUTRAL_EXTENSION_MM` from the measurement (default 25 mm).
2. With the linkage disconnected, confirm 1000 us is full retract and 2000 us is full extend on each LAC. If not, set `HORNET_TVC_*_RETRACT_PULSE_US` and `HORNET_TVC_*_EXTEND_PULSE_US`. If the LAC is reversed, swap the two values.
3. Confirm a positive angle extends each actuator and matches the vehicle's +pitch and +yaw. If an axis is mirrored, set `HORNET_TVC_PITCH_DIRECTION` or `HORNET_TVC_YAW_DIRECTION` to -1.
4. Yaw on pin 29 has never been run. Run `tvc_bench_yaw` first and confirm it moves.
