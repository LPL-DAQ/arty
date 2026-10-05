#include "tvc_kinematics.h"

#include <algorithm>
#include <cmath>

namespace {

constexpr float DEG_TO_RAD = 0.017453292519943295f;
constexpr float RAD_TO_DEG = 57.29577951308232f;

/// Newton's-method square root usable in constant expressions. COS_B must be a compile-time constant: a dynamically
/// initialized global here could still be 0 when another translation unit's static constructor (e.g. the global
/// tvc::Supervisor, which precomputes the length clamp) calls length_in().
constexpr double constexpr_sqrt(double x)
{
    double r = x > 1.0 ? x : 1.0;
    for (int i = 0; i < 64; i++) {
        r = 0.5 * (r + x / r);
    }
    return r;
}

// Law of cosines terms. Lts and Lre are fixed by the structure, so precompute:
//     L^2 = A - B * cos(theta)   with   A = Lts^2 + Lre^2,   B = 2 * Lts * Lre
constexpr double LTS_SQ = double(tvc::TVC_X_IN) * tvc::TVC_X_IN + double(tvc::TVC_TE_IN) * tvc::TVC_TE_IN;
constexpr double LRE_SQ = double(tvc::TVC_E_IN) * tvc::TVC_E_IN + double(tvc::TVC_ER_IN) * tvc::TVC_ER_IN;
constexpr float COS_A = static_cast<float>(LTS_SQ + LRE_SQ);
constexpr float COS_B = static_cast<float>(2.0 * constexpr_sqrt(LTS_SQ) * constexpr_sqrt(LRE_SQ));
static_assert(COS_B > 223.6f && COS_B < 223.7f, "2 * Lts * Lre should be ~223.65 in^2 for the current geometry");

}  // namespace

float tvc::length_in(float angle_deg, float theta0_deg)
{
    const float theta_rad = (theta0_deg + angle_deg) * DEG_TO_RAD;
    return std::sqrt(COS_A - COS_B * std::cos(theta_rad));
}

float tvc::angle_deg(float length_in, float theta0_deg)
{
    const float cos_theta = std::clamp((COS_A - length_in * length_in) / COS_B, -1.0f, 1.0f);
    return std::acos(cos_theta) * RAD_TO_DEG - theta0_deg;
}

float tvc::length_to_rev(float length_in, float l_center_in, float turns_per_inch, float direction_sign)
{
    return (length_in - l_center_in) * turns_per_inch * direction_sign;
}

float tvc::rev_to_length(float rev, float l_center_in, float turns_per_inch, float direction_sign)
{
    return l_center_in + rev / (turns_per_inch * direction_sign);
}
