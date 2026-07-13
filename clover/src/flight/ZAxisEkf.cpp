#include "ZAxisEkf.h"
#include "../config.h"
#include <cmath>

// ── 3x3 matrix helpers ──────────────────────────────────────────────────────────────────────
// Minimal, filter-agnostic linear algebra -- only what ZAxisEkf's predict/update math needs
// (nothing speculative: no NxM support, no inverse, no determinant). Out-parameter style rather
// than return-by-value since raw C arrays can't be returned by value in C++, and introducing a
// wrapper type just to get value semantics isn't worth it for five functions used in one file.
//
// static (not declared in ZAxisEkf.h) so they're invisible outside this translation unit --
// matches the file-local `static` helper pattern already used in StateEstimator.cpp
// (quats_agree/vec3_close) rather than exposing them as private class members, which would force
// their signatures into the public header for no benefit to callers of ZAxisEkf.
//
static void mat3_mult(const float a[3][3], const float b[3][3], float out[3][3])
{
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            float sum = 0.0f;
            for (int k = 0; k < 3; k++) {
                sum += a[i][k] * b[k][j];
            }
            out[i][j] = sum;
        }
    }
}

static void mat3_transpose(const float a[3][3], float out[3][3])
{
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            out[j][i] = a[i][j];
        }
    }
}

static void mat3_add(const float a[3][3], const float b[3][3], float out[3][3])
{
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            out[i][j] = a[i][j] + b[i][j];
        }
    }
}

static void mat3_vec_mult(const float a[3][3], const float v[3], float out[3])
{
    for (int i = 0; i < 3; i++) {
        float sum = 0.0f;
        for (int j = 0; j < 3; j++) {
            sum += a[i][j] * v[j];
        }
        out[i] = sum;
    }
}

static float vec3_dot(const float a[3], const float b[3])
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

// Outer product out = a * b^T (a as a column vector, b^T as a row vector) -- a genuine 3x3 result,
// unlike vec3_dot's scalar. Needed for K*H (building I - K*H) and K*r*K^T in the Joseph-form
// update below.
static void mat3_outer(const float a[3], const float b[3], float out[3][3])
{
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            out[i][j] = a[i] * b[j];
        }
    }
}

// In-place P = 0.5*(P + P^T). Guards against covariance drifting asymmetric from floating-point
// rounding across repeated predict/update cycles -- standard EKF hygiene, not filter logic itself,
// which is why it's implemented now alongside the other pure-math helpers.
static void mat3_symmetrize(float a[3][3])
{
    float transposed[3][3];
    mat3_transpose(a, transposed);

    float summed[3][3];
    mat3_add(a, transposed, summed);

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            a[i][j] = 0.5f * summed[i][j];
        }
    }
}

// Max allowed dt for a single predict() step, in seconds. Placeholder tied to the same reasoning
// as VN300_STALE_THRESHOLD_NS in StateEstimator.cpp (50ms, ~20 missed samples at VN300's ~400Hz
// rate) -- NOT a literal shared constant (this class has no sensor knowledge and no dependency on
// StateEstimator, by design; see the class comment in ZAxisEkf.h), just the same underlying
// concern: if the upstream reading feed stalled for this long, a single predict() step covering
// that whole gap would propagate garbage (a huge, physically-meaningless position/velocity jump)
// rather than the short-dt integration step the linear kinematic model below assumes.
static constexpr float MAX_PREDICT_DT_S = 0.05f;

// Placeholder -- needs tuning against real sensor data. Process noise variance added to the
// diagonal of P every predict() step, one entry per state component [z, vz, bias]. Larger values
// mean the filter trusts its own kinematic model less and leans harder on measurement updates
// (once those are implemented); smaller values mean the opposite.
static constexpr float PROCESS_NOISE_Z_M2 = 1e-4f;
static constexpr float PROCESS_NOISE_VZ_M2_S2 = 1e-2f;
static constexpr float PROCESS_NOISE_BIAS_M2_S4 = 1e-6f;  // small: bias should drift slowly

// Placeholder -- needs tuning against real sensor data. Innovation gate threshold, in multiples of
// sqrt(S) (the innovation's own standard deviation, i.e. "sigma"). An innovation |y| beyond this
// many sigma is treated as implausible given the filter's current uncertainty and rejected rather
// than applied, protecting the filter from a single wild/corrupted measurement (e.g. a LiDAR
// multipath return or a GNSS multipath fix). 5 sigma is a common rule-of-thumb starting point --
// under a Gaussian noise assumption a >5-sigma innovation is astronomically unlikely from noise
// alone -- but hasn't been validated against this vehicle's real sensor data.
static constexpr float INNOVATION_GATE_SIGMA = 5.0f;

void ZAxisEkf::init(float z0, float vz0)
{
    init(z0, vz0, 0.0f, 0.0f, 0.0f);
}

void ZAxisEkf::init(float z0, float vz0, float z0_variance, float vz0_variance, float bias0_variance)
{
    x_[0] = z0;
    x_[1] = vz0;
    x_[2] = 0.0f;  // accel bias starts at zero

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            P_[i][j] = 0.0f;
        }
    }
    P_[0][0] = z0_variance;
    P_[1][1] = vz0_variance;
    P_[2][2] = bias0_variance;

    rejected_update_count_ = 0;
    valid_ = true;
}

void ZAxisEkf::reset()
{
    x_[0] = 0.0f;
    x_[1] = 0.0f;
    x_[2] = 0.0f;

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            P_[i][j] = 0.0f;
        }
    }

    rejected_update_count_ = 0;
    valid_ = false;
}

void ZAxisEkf::predict(float accel_world_z, float dt_s)
{
    if (!valid_) {
        // Predicting on an uninitialized filter (all-zero state, as if that were a real reading)
        // would be silently meaningless rather than an obvious error -- reject instead.
        return;
    }
    if (!std::isfinite(accel_world_z) || !std::isfinite(dt_s)) {
        return;
    }
    if (dt_s <= 0.0f || dt_s > MAX_PREDICT_DT_S) {
        // dt_s <= 0 isn't a physically meaningful elapsed time. dt_s > MAX_PREDICT_DT_S means the
        // reading feed likely stalled -- reject rather than clamp: propagating over a gap this
        // large with the constant-acceleration model below would inject a large, probably-wrong
        // state change, rather than holding the last known-good estimate (same "hold rather than
        // guess" preference used throughout StateEstimator's fault handling).
        return;
    }

    float bias = x_[2];

    // a = accel_world_z - GRAVITY_M_S2 - bias: the true vertical acceleration is the measured
    // acceleration, minus gravity (GRAVITY_M_S2 is the existing constant from config.h, not
    // redefined here), minus whatever constant bias the filter currently attributes to the
    // accelerometer.
    float a = accel_world_z - GRAVITY_M_S2 - bias;

    // Constant-acceleration kinematics over this step. bias is intentionally left unchanged here
    // (a random-walk model: nothing in predict() moves it: it can only be revised later by a
    // measurement update, which feeds back through the Kalman gain -- not implemented yet).
    x_[0] += x_[1] * dt_s + 0.5f * a * dt_s * dt_s;
    x_[1] += a * dt_s;

    // State transition Jacobian F = df/dx, x = [z, vz, bias], derived from:
    //
    //   f(x, dt) = [ z + vz*dt + 0.5*a*dt^2 ]     where a = accel_world_z - GRAVITY_M_S2 - bias
    //              [ vz + a*dt               ]     (accel_world_z is a known input, not part of
    //              [ bias                    ]      the state, so it contributes no Jacobian term)
    //
    //   df_z/dz    = 1        df_z/dvz    = dt       df_z/dbias    = 0.5*dt^2 * (da/dbias) = -0.5*dt^2
    //   df_vz/dz   = 0        df_vz/dvz   = 1         df_vz/dbias   = dt * (da/dbias)       = -dt
    //   df_bias/dz = 0        df_bias/dvz = 0         df_bias/dbias = 1
    //
    // da/dbias = -1 because a = accel_world_z - GRAVITY_M_S2 - bias: increasing the bias estimate
    // decreases the inferred true acceleration by the same amount, hence the negative sign on both
    // bias-column entries. Cross-checked against the requested F = [[1, dt, -0.5*dt^2],
    // [0, 1, -dt], [0, 0, 1]] -- matches.
    float dt2 = dt_s * dt_s;
    const float F[3][3] = {
        {1.0f, dt_s, -0.5f * dt2},
        {0.0f, 1.0f, -dt_s},
        {0.0f, 0.0f, 1.0f},
    };

    float Ft[3][3];
    mat3_transpose(F, Ft);

    float FP[3][3];
    mat3_mult(F, P_, FP);

    float FPFt[3][3];
    mat3_mult(FP, Ft, FPFt);

    const float Q[3][3] = {
        {PROCESS_NOISE_Z_M2, 0.0f, 0.0f},
        {0.0f, PROCESS_NOISE_VZ_M2_S2, 0.0f},
        {0.0f, 0.0f, PROCESS_NOISE_BIAS_M2_S4},
    };

    // P = F*P*F^T + Q
    mat3_add(FPFt, Q, P_);

    // Floating-point rounding across repeated F*P*F^T products can drift P off-symmetric over
    // many predict() calls; re-symmetrize every step rather than letting error accumulate.
    mat3_symmetrize(P_);
}

void ZAxisEkf::update_altitude(float z_meas, float r_variance)
{
    scalar_update(0, z_meas, r_variance);
}

void ZAxisEkf::update_velocity(float vz_meas, float r_variance)
{
    scalar_update(1, vz_meas, r_variance);
}

// Standard scalar-measurement Kalman update. h_index picks the one-hot observation row:
// h_index=0 -> H=[1,0,0] (update_altitude), h_index=1 -> H=[0,1,0] (update_velocity).
//
// r_variance is deliberately a parameter, not a stored member: callers pass the CURRENT
// measurement's own variance every call (e.g. Javad's GnssReadings.pos_sigma_m or vel_sigma_ms),
// so the filter automatically weighs a noisy reading less than a precise one without needing any
// separate "trust this sensor more/less" logic -- the Kalman gain K already does that weighting
// as a function of r_variance vs. the filter's own uncertainty S. Note the squaring: pos_sigma_m/
// vel_sigma_ms are standard deviations (sigma, matching their RMS-style naming), but r_variance
// here must be variance = sigma^2 -- callers need to square the reported sigma before passing it
// in, not pass the sigma directly.
void ZAxisEkf::scalar_update(int h_index, float meas, float r_variance)
{
    if (!valid_) {
        return;
    }
    if (!std::isfinite(meas) || !std::isfinite(r_variance) || r_variance <= 0.0f) {
        // r_variance <= 0 isn't a physically meaningful variance (and would risk divide-by-zero
        // or a nonsensical negative S below) -- reject rather than guess at a substitute value.
        return;
    }

    // H is the one-hot row vector selecting which state component this measurement observes.
    float H[3] = {0.0f, 0.0f, 0.0f};
    H[h_index] = 1.0f;

    // y = meas - H*x  (innovation: how far the measurement is from what the current state predicts)
    float y = meas - vec3_dot(H, x_);

    // S = H*P*H^T + r  (innovation covariance). P*H^T first (H^T is just H's values as a column
    // vector, same array), then H*(P*H^T) via dot product, since H*P*H^T is a scalar here.
    float PHt[3];
    mat3_vec_mult(P_, H, PHt);
    float S = vec3_dot(H, PHt) + r_variance;

    // Innovation sanity gate. S > 0 is guaranteed here: H*P*H^T >= 0 because P is positive
    // semi-definite (by construction -- see the Joseph-form comment below), and r_variance > 0 was
    // just checked above, so sqrt(S) below can't be sqrt of a negative number.
    float sigma = std::sqrt(S);
    if (std::fabs(y) > INNOVATION_GATE_SIGMA * sigma) {
        rejected_update_count_++;
        return;
    }

    // K = P*H^T / S  (Kalman gain)
    float K[3];
    for (int i = 0; i < 3; i++) {
        K[i] = PHt[i] / S;
    }

    // x += K*y
    for (int i = 0; i < 3; i++) {
        x_[i] += K[i] * y;
    }

    // Joseph form: P = (I-KH)*P*(I-KH)^T + K*r*K^T, used instead of the algebraically-equivalent
    // simple form P = (I-KH)*P.
    //
    // Why: the two forms are only equivalent under EXACT arithmetic. At float precision, repeated
    // applications of the simple form can let P drift asymmetric and even develop negative
    // diagonal entries from accumulated rounding error, since nothing about that formula
    // structurally guarantees the result stays positive semi-definite. The Joseph form is a sum of
    // two terms that are each PSD by construction -- (I-KH)*P*(I-KH)^T is a congruence transform of
    // the PSD matrix P, and K*r*K^T is r (positive) times an outer product with itself (always
    // PSD) -- so their sum is guaranteed PSD regardless of rounding error in K, H, or P. This is
    // the standard numerically-stable form for exactly this reason.
    float KH[3][3];
    mat3_outer(K, H, KH);

    float IminusKH[3][3];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            IminusKH[i][j] = (i == j ? 1.0f : 0.0f) - KH[i][j];
        }
    }

    float IminusKHt[3][3];
    mat3_transpose(IminusKH, IminusKHt);

    float temp[3][3];
    mat3_mult(IminusKH, P_, temp);

    float term1[3][3];
    mat3_mult(temp, IminusKHt, term1);

    float KKt[3][3];
    mat3_outer(K, K, KKt);

    float term2[3][3];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            term2[i][j] = KKt[i][j] * r_variance;
        }
    }

    mat3_add(term1, term2, P_);

    // Same floating-point-hygiene reasoning as predict(): re-symmetrize every step rather than
    // letting rounding error accumulate across many update() calls.
    mat3_symmetrize(P_);
}

float ZAxisEkf::z_m() const
{
    return x_[0];
}

float ZAxisEkf::vz_ms() const
{
    return x_[1];
}

float ZAxisEkf::accel_bias_mss() const
{
    return x_[2];
}

float ZAxisEkf::z_variance() const
{
    return P_[0][0];
}

float ZAxisEkf::vz_variance() const
{
    return P_[1][1];
}

float ZAxisEkf::accel_bias_variance() const
{
    return P_[2][2];
}

int ZAxisEkf::rejected_update_count() const
{
    return rejected_update_count_;
}

bool ZAxisEkf::is_valid() const
{
    return valid_;
}

#if CONFIG_TEST
void ZAxisEkf::set_state_for_testing(float z, float vz, float bias)
{
    x_[0] = z;
    x_[1] = vz;
    x_[2] = bias;
    valid_ = true;
}

void ZAxisEkf::get_covariance_for_testing(float out[3][3]) const
{
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            out[i][j] = P_[i][j];
        }
    }
}
#endif
