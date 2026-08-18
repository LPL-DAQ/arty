#include "ZAxisEkf.h"
#include "../config.h"
#include "../math_util.h"
#include <cmath>

// Placeholder -- needs tuning against real sensor data. Max dt for a single predict() step [s]:
// past this the reading feed has likely stalled, and one step spanning the whole gap would inject
// a large, probably-wrong state change.
static constexpr float MAX_PREDICT_DT_S = 0.05f;

// Placeholder -- needs tuning against real sensor data. Process noise variance added to P's
// diagonal every predict() step. Larger values trust the kinematic model less and lean harder on
// measurement updates.
static constexpr float PROCESS_NOISE_Z_M2 = 1e-4f;
static constexpr float PROCESS_NOISE_VZ_M2_S2 = 1e-2f;
static constexpr float PROCESS_NOISE_BIAS_M2_S4 = 1e-6f;  // small: bias should drift slowly

// Placeholder -- needs tuning against real sensor data. Innovation gate threshold in multiples of
// sqrt(S). Rejects single wild measurements (LiDAR/GNSS multipath, corrupted packets).
static constexpr float INNOVATION_GATE_SIGMA = 5.0f;

// Covariance half of a propagation step: P = F*P*F^T + Q. Shared by predict() and
// predict_covariance_only() so the two can't drift apart. F depends only on dt_s, never on x_,
// which is what makes propagating covariance without touching the state well-defined.
static void propagateCovariance(float P[3][3], float dt_s)
{
    // State transition Jacobian F = df/dx over x = [z, vz, bias]. The bias column is negative
    // because da/dbias = -1: raising the bias estimate lowers the inferred true acceleration.
    float dt2 = dt_s * dt_s;
    const float F[3][3] = {
        {1.0f, dt_s, -0.5f * dt2},
        {0.0f, 1.0f, -dt_s},
        {0.0f, 0.0f, 1.0f},
    };

    float Ft[3][3];
    math_util::transposeMatrix3(F, Ft);

    float FP[3][3];
    math_util::multiplyMatrix3(F, P, FP);

    float FPFt[3][3];
    math_util::multiplyMatrix3(FP, Ft, FPFt);

    const float Q[3][3] = {
        {PROCESS_NOISE_Z_M2, 0.0f, 0.0f},
        {0.0f, PROCESS_NOISE_VZ_M2_S2, 0.0f},
        {0.0f, 0.0f, PROCESS_NOISE_BIAS_M2_S4},
    };

    // P = F*P*F^T + Q
    math_util::addMatrix3(FPFt, Q, P);

    // Re-symmetrize every step rather than letting float rounding accumulate.
    math_util::symmetrizeMatrix3(P);
}

void ZAxisEkf::init(float z0, float vz0)
{
    init(z0, vz0, 0.0f, 0.0f, 0.0f);
}

void ZAxisEkf::init(float z0, float vz0, float z0_variance, float vz0_variance, float bias0_variance)
{
    // Back to the default member initializers (x_ and P_ zeroed, invalid, no rejections), then set
    // only what this overload specifies. Accel bias stays at 0.
    *this = ZAxisEkf{};

    x_[0] = z0;
    x_[1] = vz0;

    P_[0][0] = z0_variance;
    P_[1][1] = vz0_variance;
    P_[2][2] = bias0_variance;

    valid_ = true;
}

void ZAxisEkf::reset()
{
    *this = ZAxisEkf{};
}

void ZAxisEkf::predict(float accel_world_z, float dt_s)
{
    if (!valid_) {
        // Predicting on an uninitialized (all-zero) state would be silently meaningless.
        return;
    }
    if (!std::isfinite(accel_world_z) || !std::isfinite(dt_s)) {
        return;
    }
    if (dt_s <= 0.0f || dt_s > MAX_PREDICT_DT_S) {
        // Reject rather than clamp: hold the last known-good estimate instead of propagating over
        // a gap the constant-acceleration model doesn't apply to.
        return;
    }

    float bias = x_[2];

    // True vertical acceleration: measured, minus gravity, minus the bias the filter currently
    // attributes to the accelerometer.
    float a = accel_world_z - GRAVITY_M_S2 - bias;

    // Constant-acceleration kinematics. bias follows a random walk -- predict() leaves it alone;
    // only a measurement update revises it, via the Kalman gain.
    x_[0] += x_[1] * dt_s + 0.5f * a * dt_s * dt_s;
    x_[1] += a * dt_s;

    propagateCovariance(P_, dt_s);
}

void ZAxisEkf::predict_covariance_only(float dt_s)
{
    if (!valid_) {
        return;
    }
    if (!std::isfinite(dt_s)) {
        return;
    }
    if (dt_s <= 0.0f || dt_s > MAX_PREDICT_DT_S) {
        // Same rejection as predict(): a gap this long says the feed stalled, and inflating P by
        // one huge step would understate how little is actually known.
        return;
    }

    propagateCovariance(P_, dt_s);
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
// h_index=0 -> H=[1,0,0] (update_altitude), h_index=1 -> H=[0,1,0] (update_velocity). r_variance
// is this measurement's own variance, so the gain weights a noisy reading less on its own.
void ZAxisEkf::scalar_update(int h_index, float meas, float r_variance)
{
    if (!valid_) {
        return;
    }
    if (!std::isfinite(meas) || !std::isfinite(r_variance) || r_variance <= 0.0f) {
        // r_variance <= 0 would risk divide-by-zero or a negative S below.
        return;
    }

    // H is the one-hot row vector selecting which state component this measurement observes.
    float H[3] = {0.0f, 0.0f, 0.0f};
    H[h_index] = 1.0f;

    // y = meas - H*x  (innovation: how far the measurement is from what the current state predicts)
    float y = meas - math_util::dotProduct(H, x_);

    // S = H*P*H^T + r  (innovation covariance). P*H^T first (H^T is just H's values as a column
    // vector, same array), then H*(P*H^T) via dot product, since H*P*H^T is a scalar here.
    float PHt[3];
    math_util::multiplyMatrix3Vector(P_, H, PHt);
    float S = math_util::dotProduct(H, PHt) + r_variance;

    // Innovation gate. S > 0 holds because P stays PSD (Joseph form, below) and r_variance > 0.
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

    // Joseph form: P = (I-KH)*P*(I-KH)^T + K*r*K^T. Both terms are PSD by construction, so P stays
    // PSD under float rounding -- unlike the simpler P = (I-KH)*P, which only matches in exact
    // arithmetic and can develop negative diagonal entries.
    float KH[3][3];
    math_util::outerProduct(K, H, KH);

    float IminusKH[3][3];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            IminusKH[i][j] = (i == j ? 1.0f : 0.0f) - KH[i][j];
        }
    }

    float IminusKHt[3][3];
    math_util::transposeMatrix3(IminusKH, IminusKHt);

    float temp[3][3];
    math_util::multiplyMatrix3(IminusKH, P_, temp);

    float term1[3][3];
    math_util::multiplyMatrix3(temp, IminusKHt, term1);

    float KKt[3][3];
    math_util::outerProduct(K, K, KKt);

    float term2[3][3];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            term2[i][j] = KKt[i][j] * r_variance;
        }
    }

    math_util::addMatrix3(term1, term2, P_);

    // Same reasoning as predict(): re-symmetrize rather than letting rounding accumulate.
    math_util::symmetrizeMatrix3(P_);
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
