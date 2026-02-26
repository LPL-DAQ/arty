#ifndef APP_LOOKUP_TABLE_H
#define APP_LOOKUP_TABLE_H

struct Point {
    float x;
    float y;
};

// 1D Linear Interpolation Function Declaration
float interpolate_throttle(float current_pressure);

#endif // APP_LOOKUP_TABLE_H
