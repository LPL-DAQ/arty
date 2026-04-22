#pragma once

#include "../Error.h"
#include "clover.pb.h"
#include <expected>
#include <tuple>

namespace HornetTvc {
void reset();
/// Takes pitch and yaw angular acceleration commands (rad/s²) and thrust (N),
/// converts to servo angles internally using vehicle MOI and moment arm.
std::expected<std::tuple<float, float, HornetTvcMetrics>, Error> tick(float pitch_accel_rad_s2, float yaw_accel_rad_s2, float thrustlbf);
}  // namespace HornetTvc
