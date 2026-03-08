#ifndef APP_LOOKUP_TABLE_H
#define APP_LOOKUP_TABLE_H

struct Point {
    float x;
    float y;
};

// 1D Linear Interpolation Function Declaration
float interpolate_throttle(float current_pressure);

// 2D Bilinear Interpolation Function Declaration
float interp2D(const float* x_axis, int x_len, const float* y_axis, int y_len, const float* data_grid, float x_val, float y_val);

#endif  // APP_LOOKUP_TABLE_H
