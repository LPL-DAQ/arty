#include "LookupTable.h"

// Placeholder curve data - MUST have equal increments in X for O(1) lookup
static const Point throttle_curve[] = {
    {0.0f, 0.0f},
    {100.0f, 25.0f},
    {200.0f, 50.0f},
    {300.0f, 75.0f},
    {400.0f, 100.0f}
};

static const int curve_size = sizeof(throttle_curve) / sizeof(throttle_curve[0]);

float interpolate_throttle(float current_pressure) {
    // Boundary checks
    if (current_pressure <= throttle_curve[0].x) return throttle_curve[0].y;
    if (current_pressure >= throttle_curve[curve_size - 1].x) return throttle_curve[curve_size - 1].y;

    // O(1) Constant-Time Lookup: Assumes breakpoints are provided in equal increments
    float increment = throttle_curve[1].x - throttle_curve[0].x;

    // Calculate the base index
    int i = (int)((current_pressure - throttle_curve[0].x) / increment);

    // Safety clamp to ensure we don't read out of bounds due to floating point rounding
    if (i < 0) i = 0;
    if (i >= curve_size - 1) i = curve_size - 2;

    float x0 = throttle_curve[i].x;
    float y0 = throttle_curve[i].y;
    float x1 = throttle_curve[i+1].x;
    float y1 = throttle_curve[i+1].y;

    // Linear interpolation formula
    return y0 + (y1 - y0) * ((current_pressure - x0) / (x1 - x0));
}
