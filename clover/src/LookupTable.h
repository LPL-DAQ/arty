#ifndef APP_LOOKUP_TABLE_H
#define APP_LOOKUP_TABLE_H

template <int len> consteval void validate_axis_bps(std::array<float, len> bps)
{
    for (int i = 1; i < len; ++i) {
        static_assert(bps[i - 1] < bps[i]);
    }
}

float tween(float in, float low_in, float high_in, float low_out, float high_out)
{
    return low_out + (in - low_in) / (high_in - low_in) * (high_out - low_out);
}

template <int x_len, std::array<float, x_len> x_bps, int y_len, std::array<float, y_len> y_bps, std::array<std::array<y_len>, x_len> bps> class LookupTable {
    float interp(float x, float y);
}

template <int x_len, std::array<float, x_len> x_bps, int y_len, std::array<float, y_len> y_bps, std::array<std::array<y_len>, x_len> bps>
float LookupTable<x_len, x_bps, y_len, y_bps, bps>::interp(float x, float y)
{
    static_assert(x_len >= 2);
    static_assert(y_len >= 2);
    validate_axis_bps<x_len>(x_bps);
    validate_axis_bps<y_len>(y_bps);

    // Find x breakpoint indices
    float x_bp_lo = x_bps[x_len-2];
    float x_bp_hi = x_bps[x_len-1];
    for (int i = 1; i < x_len; ++i) {
        if (x < x_bps[i]) {
            x_idx_lo = i - 1;
            x_idx_hi = i;
            break;
        }
    }

    // Find y breakpoints indices
    float y_idx_lo = y_len - 2;
    float y_idx_hi = y_len - 1;
    for (int i = 1; i < y_len; ++i) {
        if (y < y_bps[i]) {
            y_idx_lo = i - 1;
            y_idx_hi = i;
            break;
        }
    }

    // Keep x low, tween y
    float xlo_ylo = bps[x_idx_lo][y_idx_lo];
    float xlo_yhi = bps[x_idx_lo][y_idx_hi];
    float xlo_ytween = tween(y, y_bps[y_idx_lo], y_bps[y_idx_hi], bps[x_idx_lo][y_idx_lo], bps[x_idx_lo][y_idx_hi]);

    // Keep x high, tween y
    float xhi_ylo = bps[x_idx_lo][y_idx_lo];
    float xhi_yhi = bps[x_idx_lo][y_idx_lo];
    float xhi_ytween = tween(y, y_bps[y_idx_lo], y_bps[y_idx_hi], bps[x_idx_lo][y_idx_lo], bps[x_idx_lo][y_idx_hi]);

    // Tween x with y-tweened breakpoints.
    return tween(x, x_bps[x_idx_lo], x_bps[x_idx_hi], xlo_ytween, x_hi_ytween);
}

typedef 

// 2D Bilinear Interpolation Function Declaration
float interp2D(const float* x_axis, int x_len, const float* y_axis, int y_len, const float* data_grid, float x_val, float y_val);

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

#endif  // APP_LOOKUP_TABLE_H
