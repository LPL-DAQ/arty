#pragma once

// Ranger TVC inverse kinematics: gimbal angle <-> linear actuator length, and actuator length <-> moteus revolutions.
// Pure functions, no Zephyr dependencies, so they can be unit tested on the host.
//
// Each axis is a triangle formed by the gimbal center, the structure-side mount (fixed distance Lts) and the
// engine-side mount (fixed distance Lre). The actuator spans the third side, so by the law of cosines:
//     L(angle) = sqrt(Lts^2 + Lre^2 - 2 * Lts * Lre * cos(theta0 + angle))
// Pitch and yaw are treated independently (no coupling), matching the MATLAB model.

#include "tvc_config.h"

namespace tvc {

/// Actuator length for a gimbal angle. theta0_deg is the axis' center angle (TVC_PHI0_DEG / TVC_PSI0_DEG).
float length_in(float angle_deg, float theta0_deg);

/// Inverse of length_in(): gimbal angle for an actuator length. Used to back-compute the measured gimbal angle.
/// Lengths outside the physically reachable range saturate to theta0 + angle in [0, 180] deg.
float angle_deg(float length_in, float theta0_deg);

/// Actuator length -> moteus output revolutions, relative to the homed center position.
float length_to_rev(float length_in, float l_center_in, float turns_per_inch, float direction_sign);

/// moteus output revolutions -> actuator length. Inverse of length_to_rev(); requires a configured axis.
float rev_to_length(float rev, float l_center_in, float turns_per_inch, float direction_sign);

/// Convenience wrappers using an AxisConfig.
inline float length_in(const AxisConfig& axis, float angle_deg)
{
    return length_in(angle_deg, axis.theta0_deg);
}
inline float angle_deg(const AxisConfig& axis, float length_in)
{
    return angle_deg(length_in, axis.theta0_deg);
}
inline float length_to_rev(const AxisConfig& axis, float length_in)
{
    return length_to_rev(length_in, axis.l_center_in, axis.turns_per_inch, axis.direction_sign);
}
inline float rev_to_length(const AxisConfig& axis, float rev)
{
    return rev_to_length(rev, axis.l_center_in, axis.turns_per_inch, axis.direction_sign);
}

}  // namespace tvc
