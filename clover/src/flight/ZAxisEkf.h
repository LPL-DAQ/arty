#pragma once

// Z-axis (altitude) Kalman filter. State: [z_m, vz_ms, accel_bias_mss], with a 3x3 covariance P.
// The bias state is what keeps double-integrated position from wandering during long predict-only
// stretches, e.g. a LiDAR/GNSS dropout.
//
// Attitude is an INPUT, not a state: callers pass accel_world_z already rotated body->world using
// StateEstimator's fused R_WB. F therefore depends only on dt_s, never on x_, so despite the "EKF"
// name this is a plain linear Kalman filter.
//
// Joseph-form covariance update (see ZAxisEkf.cpp): the simpler P = (I-KH)*P is equivalent only in
// exact arithmetic and can drift P asymmetric or non-PSD at float precision.
//
// r_variance is per-call so each measurement is weighted by its own reported uncertainty. Callers
// pass variance, not sigma.
//
// Updates are gated at INNOVATION_GATE_SIGMA and rejections counted, so one wild reading can't
// corrupt x_/P_ in a single call.
//
// Every constant in ZAxisEkf.cpp is an untuned placeholder, marked at its definition.
class ZAxisEkf {
public:
    // Initializes to [z0, vz0, 0] with zero covariance -- a known-exact starting condition.
    void init(float z0, float vz0);

    // As above, with an explicit diagonal initial covariance. Use when z0/vz0 come from a real
    // measurement (e.g. the first GNSS reading) rather than a known-exact state.
    void init(float z0, float vz0, float z0_variance, float vz0_variance, float bias0_variance);

    // Clears state and covariance and marks the filter invalid until init() is called again.
    void reset();

    // Propagates state by dt_s using world-frame vertical acceleration [m/s^2], already rotated
    // body->world by the caller. No-op if the filter is invalid, the inputs aren't finite, or dt_s
    // falls outside (0, MAX_PREDICT_DT_S].
    void predict(float accel_world_z, float dt_s);

    // Absolute altitude measurement (GNSS/LiDAR), H = [1,0,0]. r_variance is this reading's own
    // variance, so noisier readings are weighted less.
    void update_altitude(float z_meas, float r_variance);

    // Vertical velocity measurement, H = [0,1,0]. Same gating and covariance update as
    // update_altitude().
    void update_velocity(float vz_meas, float r_variance);

    float z_m() const;
    float vz_ms() const;
    float accel_bias_mss() const;

    // Diagonal of P -- current uncertainty in each state component. Lets a consumer decide how
    // much to trust z_m()/vz_ms() right now.
    float z_variance() const;
    float vz_variance() const;
    float accel_bias_variance() const;

    // Updates rejected by the innovation gate since the last init()/reset(), combined across both
    // channels. Diagnostics only.
    int rejected_update_count() const;

    // True once init() has been called and reset() hasn't been called since.
    bool is_valid() const;

#if CONFIG_TEST
    // Sets the state directly. Needed to test bias-driven drift: init() always starts bias at 0.
    void set_state_for_testing(float z, float vz, float bias);

    // Copies P into out[3][3]. Test-only -- off-diagonal terms aren't a consumer concern.
    void get_covariance_for_testing(float out[3][3]) const;
#endif

private:
    // Shared scalar-measurement update. h_index selects the observed component (0 -> z, 1 -> vz).
    void scalar_update(int h_index, float meas, float r_variance);

    // State: x_[0] = z_m, x_[1] = vz_ms, x_[2] = accel_bias_mss.
    float x_[3] = {0.0f, 0.0f, 0.0f};

    // Covariance matrix over x_, same index order.
    float P_[3][3] = {};

    bool valid_ = false;
    int rejected_update_count_ = 0;
};
