#ifndef APP_LOOKUP_TABLE_1D_H
#define APP_LOOKUP_TABLE_1D_H

#include <algorithm>
#include <array>
#include <cmath>

template <int x_len, float x_min, float x_max, float x_gap, std::array<float, x_len> bps> class LookupTable1D {
private:
    static constexpr float EPSILON = 0.0001f;

public:
    static float sample(float x);
};

template <int x_len, float x_min, float x_max, float x_gap, std::array<float, x_len> bps> float LookupTable1D<x_len, x_min, x_max, x_gap, bps>::sample(float x)
{
    static_assert(x_len >= 2);
    static_assert(x_gap > 0);
    static_assert(x_max > x_min);
    static_assert(std::abs(x_min + x_gap * (x_len - 1) - x_max) < EPSILON, "x_max is incorrect given bp count and gap");

    // Determine bp indices bounding input point, clamping into bps array domain
    int x_low_idx = std::clamp(static_cast<int>(std::floor((x - x_min) / x_gap)), 0, x_len - 2);
    int x_high_idx = x_low_idx + 1;  // Bound to [1, x_len - 1]

    // Determine where input bp is within indexes bounding range, clamping such that it remains inside.
    float x_tween = std::clamp((std::clamp(x, x_min, x_max) - (x_low_idx * x_gap + x_min)) / x_gap, 0.0f, 1.0f);

    // Sample bounding range breakpoints
    float bp_x_low = bps[x_low_idx];
    float bp_x_high = bps[x_high_idx];

    // Tween along input axis
    return bp_x_low + (bp_x_high - bp_x_low) * x_tween;
}

#endif  // APP_LOOKUP_TABLE_1D_H
