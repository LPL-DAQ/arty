#pragma once

// Vertical Kalman filter over [z_m, vz_ms, accel_bias_mss]. The caller rotates acceleration into
// the world frame, so the model is linear despite the name. Measurement noise is passed per call as
// a variance, and updates outside the innovation gate are rejected and counted.
class ZAxisEkf {
public:
    // Starts at [z0, vz0, 0] with zero covariance.
    void init(float z0, float vz0);

    // As above, with a diagonal initial covariance.
    void init(float z0, float vz0, float z0_variance, float vz0_variance, float bias0_variance);

    // Invalid until init() is called again.
    void reset();

    // accel_world_z is world-frame vertical acceleration [m/s^2] including gravity. Ignored if the
    // filter is invalid, an input isn't finite, or dt_s is outside (0, MAX_PREDICT_DT_S].
    void predict(float accel_world_z, float dt_s);

    // Grows the covariance without moving the state, for steps with an untrusted accelerometer.
    void predict_covariance_only(float dt_s);

    void update_altitude(float z_meas, float r_variance);
    void update_velocity(float vz_meas, float r_variance);

    float z_m() const;
    float vz_ms() const;
    float accel_bias_mss() const;

    float z_variance() const;
    float vz_variance() const;
    float accel_bias_variance() const;

    // Updates rejected by the innovation gate since the last init() or reset().
    int rejected_update_count() const;

    bool is_valid() const;

#if CONFIG_TEST
    // init() always starts the bias at 0.
    void set_state_for_testing(float z, float vz, float bias);

    void get_covariance_for_testing(float out[3][3]) const;
#endif

private:
    // h_index selects the observed component: 0 for z, 1 for vz.
    void scalar_update(int h_index, float meas, float r_variance);

    float x_[3] = {0.0f, 0.0f, 0.0f};
    float P_[3][3] = {};

    bool valid_ = false;
    int rejected_update_count_ = 0;
};
