#pragma once

#include "../Error.h"
#include "clover.pb.h"
#include <cstdint>
#include <expected>
#include <tuple>

namespace HornetTvc {
struct AxisCalibration {
    float neutral_extension_mm;
    uint32_t retract_pulse_us;
    uint32_t extend_pulse_us;
    float direction;
};

/// Actuator extension (mm from full retract) that holds a gimbal angle, from the linkage geometry.
float gimbal_deg_to_extension_mm(float gimbal_deg, const AxisCalibration& cal);
/// Pulse for an extension, clamped to the axis endpoints and to the PWM pulse range.
uint32_t extension_mm_to_pulse_us(float extension_mm, const AxisCalibration& cal);

void reset();
uint32_t neutral_pitch_pulse_us();
uint32_t neutral_yaw_pulse_us();
/// Moves the held extensions to neutral without rate limiting and returns the pitch and yaw neutral pulses.
std::tuple<uint32_t, uint32_t, HornetTvcMetrics> command_neutral();
/// Takes pitch and yaw gimbal angles (deg) directly, bypassing thrust and MOI, for bench and trace driven TVC.
std::expected<std::tuple<float, float, HornetTvcMetrics>, Error> tick_gimbal_deg(float pitch_gimbal_deg, float yaw_gimbal_deg);
/// Takes pitch and yaw angular acceleration commands (rad/s²) and thrust (N), converts to linear
/// actuator extension internally using vehicle MOI, moment arm and gimbal linkage geometry, and
/// returns the pitch and yaw RC pulse widths (µs) for the LAC boards.
std::expected<std::tuple<float, float, HornetTvcMetrics>, Error> tick(float pitch_accel_rad_s2, float yaw_accel_rad_s2, float thrust_N);
}  // namespace HornetTvc
