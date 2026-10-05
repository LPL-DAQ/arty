# Ranger TVC

Two linear actuators (pitch, yaw) on the engine gimbal, each driven by a moteus r4.1 over CAN-FD (FlexCAN3).

| File | What |
|---|---|
| `tvc_config.h` | Every geometry, limit, CAN, timing and fault constant. `TODO(adit)` marks unknowns. |
| `tvc_kinematics.*` | Closed-form inverse kinematics (law of cosines, from `inversekinematicsactuatorsnewgimbal.m`), its inverse, length <-> rev. |
| `moteus.*` | moteus multiplex frame encode/decode. Pure, allocation-free. |
| `tvc_control.*` | `tvc::Supervisor`: startup, homing, enable checks, clamps, slew, fault latch. Pure. |
| `moteus_can.*` | Zephyr CAN-FD transport: per-controller RX filter + msgq, non-blocking TX. |
| `../RangerTvc.*` | 50 Hz thread, controller hand-off, telemetry. |

Unit tests: `tests/clover/src/ranger_tvc/` (run on `native_sim`).

## Before the actuators will move

The TVC refuses to enable until these are filled in in `tvc_config.h`:

1. `turns_per_inch` per axis = (moteus output revs per lead screw rev) / (screw lead in inches per rev).
2. `direction_sign` per axis: +1 if positive moteus revolutions lengthen the actuator, else -1.
3. Confirm `moteus_id` (pitch 1, yaw 2) matches each controller's `id.id`.
4. Confirm `TVC_PSI0_DEG`: the supplied `TVCyawnew.csv` was generated with 110.155 deg, the MATLAB script says 113.696 deg.
   If 110.155 is right, also change `TVC_YAW.l_center_in` and `YAW_THETA0_DEG` in `gen_tvc_ik_vectors.py`, then re-run it.

Then tune `TVC_SLEW_LIMIT_DEG_S`, `TVC_ACCEL_LIMIT_DEG_S2`, `TVC_POSITION_ERROR_MAX_IN` and `TVC_MAX_TORQUE_NM`.

## Homing (every time the moteus controllers power up)

moteus only knows where gimbal center is after a "set output exact" since it last powered up (home state register
`0x00c` reads 2 = OUTPUT). The TVC checks this and will not enable otherwise.

1. Power up. The TVC goes STARTUP -> CLEARING -> READY and reports `enable_blockers` (4 = not homed).
2. Mechanically center the gimbal (jig) and hold it there.
3. Send `CalibrateTvcRequest` from IDLE. The TVC stops both controllers, sets the current position to 0 rev, verifies,
   and the controller returns to IDLE. The TVC then enables and holds center.

A Teensy reboot alone does not require re-homing as long as the gimbal is still within `TVC_HOMED_TOLERANCE_IN` of
center.

## Faults

Latched on: 3 missed replies, moteus fault (code kept in `latched_fault_code`), watchdog timeout, wrong mode, or
persistent position error. Policy `HOLD_CENTER`: healthy axes slew back to center, unresponsive/faulted axes are
stopped. With `TVC_FAULT_ABORTS_CONTROLLER`, an active TVC/static fire/flight sequence aborts. Re-home to clear.

## Bench test

Build with `-DCONFIG_RANGER_TVC_BENCH_SWEEP=y`: once enabled, each axis sweeps +/-2 deg at 0.5 deg/s (pitch, then yaw)
and commanded vs measured angle is logged at 5 Hz. Never fly or fire this build.
