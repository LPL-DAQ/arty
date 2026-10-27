# Hornet RCS bench test (open loop)

Spins one RCS propeller at a time from a throttle trace, with no roll control in the loop. Trace values are throttle percent (0-100). The firmware caps them at `HORNET_RCS_BENCH_MAX_THROTTLE_PERCENT` (15 percent, 1150 us) in `clover/src/config.h`.

Closed-loop RCS (the RCS and FLIGHT states) is a separate path. Its roll gains and thresholds have not been tested in degree units, so it should not run on the vehicle until they are reviewed. When it fires, it commands `HORNET_RCS_THROTTLE_PERCENT` from `config.h`, which is currently full throttle (2000 us), with no bench cap.

Closed-loop RCS output is disabled until the gains are reviewed: with `HORNET_RCS_CLOSED_LOOP_ENABLED = false` in `config.h`, the RCS and FLIGHT states hold both propellers at 1000 us while the controller and its metrics keep running. Only this RCS valve sequence drives the ESCs.

## Before you start

- This firmware drives the BetaCW and BetaCCW pins. Confirm the ESC signal wiring before connecting the battery.
- Props off.
- Motors clamped to the bench.
- Power the ESCs from the 4S LiPo only, not a bench supply.
- Power the Teensy first and confirm both RCS channels read 1000 us ("RCS CW PWM" / "RCS CCW PWM" under `s`) before connecting the battery.
- Board reachable at `169.254.99.99`. The client connects at startup, so power the board first.

## Run

```
cd scripts
uv run client-new.py
```

At `CMD>`:

1. `loadrcsvalve`, answer `y` to "Load a saved sequence?", pick `rcs_bench_cw`. State goes IDLE → RCS_VALVE_PRIMED.
2. `startrcsvalve`, confirm. State goes RCS_VALVE → IDLE after 15 s.
3. Repeat with `rcs_bench_ccw`.

`s` shows live status. Watch "RCS CW PWM" and "RCS CCW PWM" (us).

## Expected, moving channel

| Time (s) | Throttle | Pulse (us) |
|---|---|---|
| 0 – 3 | 0 % | 1000 |
| 3 – 6 | 0 → 10 % | 1000 → 1100 |
| 6 – 9 | 10 % | 1100 |
| 9 – 12 | 10 → 0 % | 1100 → 1000 |
| 12 – 15 | 0 % | 1000 |

The other channel stays at 1000 us throughout.

## Stop

`halt`, `abort` and the end of the trace all command both channels back to 1000 us. Quitting the client does not stop a running trace. For a hard stop, disconnect the battery.
