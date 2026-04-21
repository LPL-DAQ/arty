#pragma once

#include <algorithm>
#include <array>
#include <cmath>

template <int x_len, float x_min, float x_max, float x_gap, int y_len, float y_min, float y_max, float y_gap, std::array<std::array<float, y_len>, x_len> bps>
class LookupTable2D {
private:
    static constexpr float EPSILON = 0.0001f;

public:
    static float sample(float x, float y);
};

template <int x_len, float x_min, float x_max, float x_gap, int y_len, float y_min, float y_max, float y_gap, std::array<std::array<float, y_len>, x_len> bps>
float LookupTable2D<x_len, x_min, x_max, x_gap, y_len, y_min, y_max, y_gap, bps>::sample(float x, float y)
{
    static_assert(x_len >= 2);
    static_assert(y_len >= 2);
    static_assert(x_gap > 0);
    static_assert(y_gap > 0);
    static_assert(x_max > x_min);
    static_assert(y_max > y_min);
    static_assert(std::abs(x_min + x_gap * (x_len - 1) - x_max) < EPSILON, "x_max is incorrect given bp count and gap");
    static_assert(std::abs(y_min + y_gap * (y_len - 1) - y_max) < EPSILON, "y_max is incorrect given bp count and gap");

    // Determine bp indices bounding input point, clamping into bps array domain
    int x_low_idx = std::clamp(static_cast<int>(std::floor((x - x_min) / x_gap)), 0, x_len - 2);
    int y_low_idx = std::clamp(static_cast<int>(std::floor((y - y_min) / y_gap)), 0, y_len - 2);
    int x_high_idx = x_low_idx + 1;  // Bound to [1, x_len - 1]
    int y_high_idx = y_low_idx + 1;  // Bound to [1, y_len - 1]

    // Determine where input bp is within indexes bounding box, clamping such that it remains inside.
    float x_tween = std::clamp((std::clamp(x, x_min, x_max) - x_min) / x_gap - x_low_idx, 0.0f, 1.0f);
    float y_tween = std::clamp((std::clamp(y, y_min, y_max) - y_min) / y_gap - y_low_idx, 0.0f, 1.0f);

    // Sample bounding box breakpoints
    float bp_x_low_y_low = bps[x_low_idx][y_low_idx];
    float bp_x_low_y_high = bps[x_low_idx][y_high_idx];
    float bp_x_high_y_low = bps[x_high_idx][y_low_idx];
    float bp_x_high_y_high = bps[x_high_idx][y_high_idx];

    // Tween along X axis
    float bp_x_tween_y_low = bp_x_low_y_low + (bp_x_high_y_low - bp_x_low_y_low) * x_tween;
    float bp_x_tween_y_high = bp_x_low_y_high + (bp_x_high_y_high - bp_x_low_y_high) * x_tween;

    // Tween along Y axis
    return bp_x_tween_y_low + (bp_x_tween_y_high - bp_x_tween_y_low) * y_tween;
}
