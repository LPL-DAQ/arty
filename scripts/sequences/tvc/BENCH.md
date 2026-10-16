# Hornet TVC bench test

## Before you start

- Hornet flashed from `akshay/hornet-build-fix`. At boot both actuators go to mid-stroke (1500 us).
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

`s` shows live status. Watch "TVC pitch cmd" / "TVC yaw cmd" (deg) and "Pitch servo" / "Yaw servo" (us).

## Expected, moving axis

| Time (s) | Command | Pulse (us) | Extension (mm) |
|---|---|---|---|
| 0 – 2 | 0 deg | 1500 | 25.00 |
| 2 – 5 | +6 deg | 1638 | 31.93 |
| 5 – 7 | 0 deg | 1500 | 25.00 |
| 7 – 10 | -6 deg | 1358 | 17.94 |
| 10 – 12 | 0 deg | 1500 | 25.00 |

The other axis stays at 1500 us throughout.

The command steps, but the actuator ramps at 18 mm/s: each 6 deg move takes about 0.39 s, so the pulse climbs through intermediate values before settling (a full 12 deg swing would take about 0.76 s). Pulses can read 1 us low.

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

`abort` (or any error during the run) commands both actuators back to 1500 us. They still move at 18 mm/s, so recentring takes about 0.4 s from 6 deg and 0.8 s from 12 deg. Quitting the client does not stop a running trace. For a hard stop, cut actuator power.
