#include "LookupTable.h"

// Placeholder curve data - team can generate this later via Python script!
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

    // Find the segment and interpolate
    for (int i = 0; i < curve_size - 1; i++) {
        if (current_pressure >= throttle_curve[i].x && current_pressure <= throttle_curve[i+1].x) {
            float x0 = throttle_curve[i].x;
            float y0 = throttle_curve[i].y;
            float x1 = throttle_curve[i+1].x;
            float y1 = throttle_curve[i+1].y;

            // Linear interpolation formula
            return y0 + (y1 - y0) * ((current_pressure - x0) / (x1 - x0));
        }
    }
    return 0.0f; // Fallback
}
