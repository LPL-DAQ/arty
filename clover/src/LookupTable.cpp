#include "LookupTable.h"

LookupTable::LookupTable(const Point* curve, int size)
    : curve_data(curve), curve_size(size) {

    // Calculate the increment for O(1) lookup based on the first two points.
    // MUST have equal increments in X for this to work correctly!
    if (curve_size > 1) {
        x_increment = curve_data[1].x - curve_data[0].x;
    } else {
        x_increment = 0.0f;
    }
}

float LookupTable::interpolate(float x_val) {
    // Boundary & safety checks
    if (curve_size == 0) return 0.0f;
    if (x_val <= curve_data[0].x) return curve_data[0].y;
    if (x_val >= curve_data[curve_size - 1].x) return curve_data[curve_size - 1].y;
    if (x_increment <= 0.0f) return curve_data[0].y;

    // O(1) Constant-Time Lookup: Calculate index directly
    int i = (int)((x_val - curve_data[0].x) / x_increment);

    // Safety clamp to ensure we don't read out of bounds due to floating point math
    if (i < 0) i = 0;
    if (i >= curve_size - 1) i = curve_size - 2;

    float x0 = curve_data[i].x;
    float y0 = curve_data[i].y;
    float x1 = curve_data[i+1].x;
    float y1 = curve_data[i+1].y;

    // Linear interpolation formula
    return y0 + (y1 - y0) * ((x_val - x0) / (x1 - x0));
}
