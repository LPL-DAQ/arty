# Estimator bench checklist

Connect the USB serial console before powering or resetting the board, so the init logs are captured. Run the client for the live status panel.

## 1. VN-300 baud after power cycle and Teensy reset

1. Power-cycle the whole vehicle (VN-300 included). The console should show `[Vectornav] Switched from 115200 to 921600 baud`.
2. Reset only the Teensy, leaving the VN-300 powered. The console should show `[Vectornav] Already at 921600 baud`.
3. After each, check that the IMU rows in the client keep updating.
4. Record any `No response` error line. The VN-300 setting is not saved to its flash, so a power cycle must always bring it back at 115200.
5. If a scope or logic analyzer is available, check the VN-300 TX line (Teensy pin 0): one line about every 2.5 ms at 921600 baud.

## 2. VN-300 frame check

Use the axis arrows printed on the VN-300 case. Keep the vehicle's heading the same for all three poses. Hold each pose still for a few seconds and record the client's `IMU accel` and `IMU q xyz` rows.

First write down which way each printed VN-300 axis (+X, +Y, +Z) points on the vehicle: up or down, and toward which side or leg.

| Pose | accel if the VN-300 body is Z-up (what the estimator assumes) | accel if it is X forward, Y right, Z down (VectorNav convention) |
|---|---|---|
| Level | (0, 0, +9.8) | (0, 0, -9.8) |
| +X end raised about 30 deg | (+4.9, 0, +8.5) | (+4.9, 0, -8.5) |
| +Y side lowered about 30 deg | (0, -4.9, +8.5) | (0, -4.9, -8.5) |

The accelerometer reads the reaction to gravity, which points up. The sign of accel z is the deciding reading. X and Y read the same either way.

Also record the quaternion for each pose. When level, qx and qy should be near 0. Raising +X should mostly change qy, and lowering +Y should mostly change qx. Note the signs. VectorNav reports the quaternion relative to north-east-down, which the estimator does not yet account for.

## 3. Estimator execution time

Read the `State estimator` row in the client timing panel (microseconds) with all sensors running. Record the typical value and the highest value seen over about a minute, and the `Controller tick` row alongside it. During a sequence the CSV also logs it as `state_estimator_ns`.

## 4. Javad output rate

Each Javad epoch prints `[Gnss] sol_type=... sol_time=... ms` on the console. Record the difference between consecutive `sol_time` values over about 20 epochs. The estimator marks Javad stale after 300 ms, which assumes at least 10 Hz.
