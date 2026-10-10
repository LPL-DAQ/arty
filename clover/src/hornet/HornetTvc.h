#pragma once

#include "../Error.h"
#include "clover.pb.h"
#include <expected>
#include <tuple>

namespace HornetTvc {
void reset();
/// Takes pitch and yaw angular acceleration commands (rad/s²) and thrust (N), converts to linear
/// actuator extension internally using vehicle MOI, moment arm and gimbal linkage geometry, and
/// returns the pitch and yaw RC pulse widths (µs) for the LAC boards.
std::expected<std::tuple<float, float, HornetTvcMetrics>, Error> tick(float pitch_accel_rad_s2, float yaw_accel_rad_s2, float thrust_N);
}  // namespace HornetTvc
