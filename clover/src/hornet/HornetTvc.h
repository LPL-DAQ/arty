#pragma once

#include "../Error.h"
#include "clover.pb.h"
#include <cstdint>
#include <expected>
#include <tuple>

namespace HornetTvc {
void reset();
uint32_t neutral_pulse_us();
/// Moves the held extensions to neutral without rate limiting and returns the neutral pulse.
uint32_t command_neutral();
/// Takes pitch and yaw gimbal angles (deg) directly, bypassing thrust and MOI, for bench and trace driven TVC.
std::expected<std::tuple<float, float, HornetTvcMetrics>, Error> tick_gimbal_deg(float pitch_gimbal_deg, float yaw_gimbal_deg);
/// Takes pitch and yaw angular acceleration commands (rad/s²) and thrust (N), converts to linear
/// actuator extension internally using vehicle MOI, moment arm and gimbal linkage geometry, and
/// returns the pitch and yaw RC pulse widths (µs) for the LAC boards.
std::expected<std::tuple<float, float, HornetTvcMetrics>, Error> tick(float pitch_accel_rad_s2, float yaw_accel_rad_s2, float thrust_N);
}  // namespace HornetTvc
