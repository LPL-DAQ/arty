// ------------------------------
// Lookup tables (CSV-backed)
// ------------------------------
float interp2D(const float* x_vec, int x_len, const float* y_vec, int y_len, const float* grid, float x_val, float y_val) {
    // Find x index
    int xi = 0;
    for (int i = 0; i < x_len - 1; i++) {
        if (x_val <= x_vec[i + 1]) { xi = i; break; }
        xi = x_len - 2;
    }

    // Find y index
    int yi = 0;
    for (int i = 0; i < y_len - 1; i++) {
        if (y_val <= y_vec[i + 1]) { yi = i; break; }
        yi = y_len - 2;
    }

    float tx = (x_val - x_vec[xi]) / (x_vec[xi + 1] - x_vec[xi]);
    float ty = (y_val - y_vec[yi]) / (y_vec[yi + 1] - y_vec[yi]);

    float q00 = grid[xi       * y_len + yi    ];
    float q01 = grid[xi       * y_len + yi + 1];
    float q10 = grid[(xi + 1) * y_len + yi    ];
    float q11 = grid[(xi + 1) * y_len + yi + 1];

    return (1 - tx) * (1 - ty) * q00 +
           (1 - tx) *      ty  * q01 +
                tx  * (1 - ty) * q10 +
                tx  *      ty  * q11;
}
