#include "tvc_kinematics.h"

#include <algorithm>
#include <cmath>

namespace {

constexpr float DEG_TO_RAD = 0.017453292519943295f;
constexpr float RAD_TO_DEG = 57.29577951308232f;

// Law of cosines terms. Lts and Lre are fixed by the structure, so precompute:
//     L^2 = A - B * cos(theta)   with   A = Lts^2 + Lre^2,   B = 2 * Lts * Lre
constexpr float LTS_SQ = tvc::TVC_X_IN * tvc::TVC_X_IN + tvc::TVC_TE_IN * tvc::TVC_TE_IN;
constexpr float LRE_SQ = tvc::TVC_E_IN * tvc::TVC_E_IN + tvc::TVC_ER_IN * tvc::TVC_ER_IN;
constexpr float COS_A = LTS_SQ + LRE_SQ;
const float COS_B = 2.0f * std::sqrt(LTS_SQ) * std::sqrt(LRE_SQ);

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
