#pragma once

// Z-axis (altitude) Kalman filter. State vector: [z_m, vz_ms, accel_bias_mss] -- altitude,
// vertical velocity, and a scalar accelerometer bias term. A 3x3 covariance matrix P tracks the
// filter's uncertainty over that state.
//
// Deliberately its own class rather than living inline in StateEstimator.cpp: this is pure math
// with no sensor knowledge (no LidarReading/ImuReading/GnssReadings type appears anywhere in this
// file), so it's independently testable and reusable on its own. Rotating body-frame acceleration
// into world frame stays OUTSIDE this class, in StateEstimator -- the same division of
// responsibility as calculateVerticalAltitude(), which likewise takes an already-prepared value
// rather than doing its own frame math.
//
// Static allocation only: fixed-size arrays, no dynamic memory, no external matrix library --
// matches the rest of this codebase's style (see math_util.h, PID.h).
//
// predict() and update_altitude()/update_velocity() are all implemented -- see ZAxisEkf.cpp for
// the state-transition/Jacobian derivation and the Joseph-form measurement update.
//
// ── Design notes ────────────────────────────────────────────────────────────────────────────
//
// State vector -- why [z_m, vz_ms, accel_bias_mss] and nothing else: these are exactly the
// quantities StateEstimator needs out (z_m/vz_ms feed position.z/velocity.z directly) plus the
// one latent quantity needed to get there honestly: raw accelerometers drift, and folding that
// drift into a state the filter itself estimates and subtracts (rather than hard-coding a fixed
// bias constant, or ignoring it) is what keeps double-integrated position from wandering off
// during long unaided (predict-only) stretches, e.g. a lidar/GNSS dropout. Attitude is
// deliberately NOT part of this state -- see the next note.
//
// Why attitude is an INPUT, not a STATE (and why that makes this a linear KF, not truly
// "extended", despite the name): StateEstimator already maintains a fused, health-gated
// attitude (current_estimate.R_WB -- VN300 with backup-IMU fallback, frozen under a critical
// fault) for reasons that have nothing to do with the z-axis filter. Rather than duplicating
// that estimation here, this class takes accel_world_z as an already-rotated INPUT (like a
// control input u_k, not a state), computed by the caller via the same conjugateQuaternion
// pattern as calculateVerticalAltitude(). One consequence worth being explicit about: because
// accel_world_z is a given input rather than a function of this filter's own state, the
// z/vz/bias dynamics in predict() are linear in x_ for any fixed accel_world_z -- the
// coefficients in F depend only on dt_s, never on x_ itself. So despite the "EKF" name (kept
// for consistency with how this filter is described elsewhere/for future extensibility), this
// is currently a plain linear Kalman filter: F is computed exactly, not via a Taylor-series
// linearization re-evaluated each step. Had attitude instead been estimated jointly as part of
// this filter's own state, the coupling between attitude and the rotation of accel would make
// the dynamics genuinely state-dependent and nonlinear, requiring a real Jacobian evaluated at
// the current estimate every predict() call. The tradeoff: this filter can't compensate for or
// diagnose bad attitude input -- it trusts current_estimate.R_WB completely and has no way to
// know if that trust is misplaced.
//
// Why Joseph-form covariance update (P = (I-KH)*P*(I-KH)^T + K*r*K^T, not the simpler
// P = (I-KH)*P): numerical stability at float (not double) precision. The simple form is
// algebraically equivalent only in exact arithmetic; under float rounding it can drift P
// asymmetric or, worse, non-positive-semi-definite (an "impossible" negative variance), which
// silently corrupts every subsequent update. Joseph form guarantees P stays PSD by construction
// even with rounding error, at the cost of more arithmetic per update -- a cost this filter can
// easily afford (3x3 matrices, one update per sensor reading, not a hot inner loop). P is also
// explicitly symmetrized after every update as a second, cheap safety net on top of this.
//
// Why r_variance is per-call, not stored on the class: this is what lets the SAME update call
// dynamically weight a measurement by how much the sensor itself claims to trust that specific
// reading, instead of using one fixed uncertainty for a sensor regardless of conditions. Concretely,
// Javad/GNSS reports its own per-epoch vrms_m/vvel_rms_ms, and StateEstimator passes
// r_variance = max(vrms_m^2, floor) straight through -- a noisier epoch (e.g. fewer satellites,
// worse DOP) automatically gets weighted less, with no extra logic needed here. Note the
// SQUARING: vrms_m/vvel_rms_ms are standard deviations (RMS, i.e. sigma), but r_variance must be
// variance = sigma^2 -- this class trusts callers to have already done that squaring; it does not
// re-derive variance from a sigma-shaped input itself.
//
// Why the innovation sanity gate: a single wild measurement (GNSS multipath spike, a LiDAR
// return off debris/a person walking past, a dropped/corrupted packet) can otherwise be pulled
// in at full weight by the Kalman gain and corrupt both x_ and P_ in one call, potentially
// taking many subsequent good updates to recover from. Gating any update whose innovation
// exceeds INNOVATION_GATE_SIGMA standard deviations (see ZAxisEkf.cpp) rejects exactly that kind
// of single-sample outlier while still accepting real, if surprising, measurements that are
// merely unlikely rather than essentially impossible given the filter's current uncertainty.
// Rejected updates are counted (rejected_update_count()) rather than silently dropped, since a
// consistently high rejection rate is itself a diagnostic signal (gate too tight, sensor
// actually faulty, or P too confident) that a fixed gate constant can't distinguish on its own.
//
// Placeholders that still need real tuning: none of the constants below have been tuned against
// real flight or even representative bench/ground data -- they're initial guesses chosen only to
// make the filter behave sanely under test, not values anyone should trust for a real flight yet.
//   - Process noise Q (ZAxisEkf.cpp): PROCESS_NOISE_Z_M2, PROCESS_NOISE_VZ_M2_S2,
//     PROCESS_NOISE_BIAS_M2_S4 -- how much the filter trusts its own predict() step vs.
//     incoming measurements; too small and it over-trusts a stale prediction through a real
//     sensor dropout, too large and it discounts good sensor data unnecessarily.
//   - Innovation gate threshold (ZAxisEkf.cpp): INNOVATION_GATE_SIGMA -- currently a round-number
//     5-sigma placeholder, not derived from any real sensor noise characterization.
//   - Predict dt clamp (ZAxisEkf.cpp): MAX_PREDICT_DT_S -- currently 0.05s, a round-number
//     placeholder, not derived from VN300's actual worst-case jitter/dropout characteristics.
//   - Initial bootstrap covariance (StateEstimator.cpp): EKF_INITIAL_Z_VARIANCE_M2,
//     EKF_INITIAL_VZ_VARIANCE_M2_S2, EKF_INITIAL_BIAS_VARIANCE_M2_S4 -- how much the filter
//     initially distrusts the very first GNSS reading it bootstraps from.
//   - Per-sensor r_variance (StateEstimator.cpp): LIDAR_1_ALTITUDE_R_VARIANCE_M2,
//     LIDAR_2_ALTITUDE_R_VARIANCE_M2 -- fixed placeholders, since neither LiDAR reports its own
//     per-reading uncertainty the way Javad/GNSS does. JAVAD_ALTITUDE_R_VARIANCE_FLOOR_M2 and
//     JAVAD_VELOCITY_R_VARIANCE_FLOOR_M2_S2 are FLOORS applied to Javad's own reported
//     vrms_m^2/vvel_rms_ms^2, not fixed values themselves -- only the floor needs tuning, to
//     guard against a zero/implausibly-tiny reported value making the filter overconfident in a
//     single epoch. VN300_VEL_D_VELOCITY_R_VARIANCE_M2_S2 is a fixed placeholder for the
//     Javad-faulty vel_d fallback path, since VN300 doesn't expose its own INS velocity
//     uncertainty to this code.
//   - Adjacent, not EKF-internal but still an unmeasured placeholder: LIDAR_MOUNT_ANGLE_DEG
//     (config.h, currently 0.0) -- feeds the geometry that produces the z_meas value LiDAR
//     readings are fused from in the first place; wrong here means every LiDAR update is
//     fused against a systematically wrong altitude regardless of how well-tuned its
//     r_variance is.
class ZAxisEkf {
public:
    // Initializes state to [z0, vz0, 0] (accel bias starts at zero) with ZERO initial covariance
    // (a confident, known-exact starting condition) and marks the filter valid. Equivalent to
    // init(z0, vz0, 0, 0, 0) below.
    void init(float z0, float vz0);

    // Same as init(z0, vz0) above, but with an explicit initial covariance (diagonal only -- no
    // initial cross-correlation assumed between z/vz/bias). Use this overload when z0/vz0 come
    // from a real, uncertain measurement rather than a known-exact starting condition -- e.g.
    // bootstrapping from the first GNSS reading, which deserves a large initial covariance
    // reflecting how little the filter actually knows yet, not zero.
    void init(float z0, float vz0, float z0_variance, float vz0_variance, float bias0_variance);

    // Clears state and covariance and marks the filter invalid until init() is called again.
    void reset();

    // Propagates state forward by dt_s using accel_world_z (vertical acceleration, world frame,
    // m/s^2 -- already rotated body->world by the caller; see the class comment above). No-op
    // (silently rejects) if the filter isn't valid, accel_world_z/dt_s aren't finite, or dt_s is
    // outside (0, MAX_PREDICT_DT_S] -- see ZAxisEkf.cpp for why.
    void predict(float accel_world_z, float dt_s);

    // Measurement update from an absolute altitude reading (e.g. GNSS/LiDAR), H = [1,0,0].
    // r_variance is per-call, not stored: pass the current measurement's own variance each time
    // (e.g. from GnssReadings.pos_sigma_m^2), so noisier readings automatically get weighted less.
    // Joseph-form covariance update with an innovation sanity gate; see ZAxisEkf.cpp for both.
    void update_altitude(float z_meas, float r_variance);

    // Measurement update from a vertical velocity reading, H = [0,1,0]. Same per-call r_variance,
    // Joseph-form update, and innovation gate as update_altitude(); see ZAxisEkf.cpp.
    void update_velocity(float vz_meas, float r_variance);

    float z_m() const;
    float vz_ms() const;
    float accel_bias_mss() const;

    // Diagonal of P -- the filter's current uncertainty (variance) in each state component.
    // Genuinely useful output, not just a test hook: a consumer can use these to decide how much
    // to trust z_m()/vz_ms() right now (e.g. right after init(), before any predict/update cycles
    // have run).
    float z_variance() const;
    float vz_variance() const;
    float accel_bias_variance() const;

    // Count of measurement updates rejected by the innovation sanity gate (see ZAxisEkf.cpp) since
    // the last init()/reset(). A single, combined count across update_altitude()/update_velocity()
    // rather than split per-channel -- diagnostics-only, split further if that granularity ends up
    // useful later.
    int rejected_update_count() const;

    // True once init() has been called and reset() hasn't been called since.
    bool is_valid() const;

#if CONFIG_TEST
    // Directly sets the state vector, bypassing predict()/update_*(). Needed to test bias-driven
    // drift in isolation: there's no public way to give the filter a nonzero initial bias
    // otherwise (init() always starts bias at 0, by design -- see init()'s comment).
    void set_state_for_testing(float z, float vz, float bias);

    // Copies the full covariance matrix into out[3][3]. Deliberately test-only rather than a real
    // getter: off-diagonal covariance terms aren't something a normal consumer needs (unlike the
    // diagonal variances above), this exists purely to verify P stays symmetric during prediction.
    void get_covariance_for_testing(float out[3][3]) const;
#endif

private:
    // Shared scalar-measurement Kalman update for update_altitude()/update_velocity(). h_index
    // selects which state component this measurement directly observes (0 -> H=[1,0,0],
    // 1 -> H=[0,1,0]) -- private since it's purely an implementation detail shared between the two
    // public entry points, not something callers pick an arbitrary index for.
    void scalar_update(int h_index, float meas, float r_variance);

    // State: x_[0] = z_m, x_[1] = vz_ms, x_[2] = accel_bias_mss.
    float x_[3] = {0.0f, 0.0f, 0.0f};

    // Covariance matrix over x_, same index order.
    float P_[3][3] = {};

    bool valid_ = false;
    int rejected_update_count_ = 0;
};
