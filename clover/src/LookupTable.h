#ifndef APP_LOOKUP_TABLE_H
#define APP_LOOKUP_TABLE_H

struct Point {
    float x;
    float y;
};

// 1D Linear Interpolation Function Declaration
float interpolate_throttle(float current_pressure);

// 2D Bilinear Interpolation Function Declaration
float interp2D(const float* x_axis, int x_len,
               const float* y_axis, int y_len,
               const float* data_grid,
               float x_val, float y_val);

// ISP lookup: chamber pressure (pc) vs O/F.
extern const float isp_pc_axis[];
extern const int isp_pc_len;
extern const float isp_of_axis[];
extern const int isp_of_len;
extern const float isp_data[];

// MPrime lookup: target thrust vs O/F.
extern const float thrust_axis[];
extern const int thrust_axis_len;
extern const float of_axis[];
extern const int of_axis_len;
extern const float fuel_valve_grid[];
extern const float lox_valve_grid[];

#endif // APP_LOOKUP_TABLE_H
