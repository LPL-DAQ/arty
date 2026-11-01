#include "ZAxisEkf.h"
#include "../config.h"
#include "../math_util.h"
#include <cmath>

// Placeholder. Longest predict step [s]; a longer gap means the feed stalled.
static constexpr float MAX_PREDICT_DT_S = 0.05f;

// Placeholder -- needs tuning from logged data. Continuous-time process noise, scaled by dt in
// each step so the filter behaves the same at any VN300 rate: white acceleration noise driving z
// and vz, and a random walk on the accelerometer bias. The acceleration term is deliberately large
// so the steady-state gain leans on LiDAR and Javad rather than the IMU-driven model.
static constexpr float ACCEL_NOISE_PSD_M2_S3 = 10.0f;
static constexpr float ACCEL_BIAS_RANDOM_WALK_PSD_M2_S5 = 4e-4f;

// Placeholder. Innovation gate in multiples of sqrt(S).
static constexpr float INNOVATION_GATE_SIGMA = 5.0f;

// P = F*P*F^T + Q, shared by predict() and predict_covariance_only().
static void propagateCovariance(float P[3][3], float dt_s)
{
    // The bias column is negative because a larger bias estimate lowers the inferred acceleration.
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

    const float qa = ACCEL_NOISE_PSD_M2_S3;
    const float Q[3][3] = {
        {qa * dt2 * dt_s / 3.0f, qa * dt2 / 2.0f, 0.0f},
        {qa * dt2 / 2.0f, qa * dt_s, 0.0f},
        {0.0f, 0.0f, ACCEL_BIAS_RANDOM_WALK_PSD_M2_S5 * dt_s},
    };

    math_util::addMatrix3(FPFt, Q, P);
    math_util::symmetrizeMatrix3(P);
}

void ZAxisEkf::init(float z0, float vz0)
{
    init(z0, vz0, 0.0f, 0.0f, 0.0f);
}

void ZAxisEkf::init(float z0, float vz0, float z0_variance, float vz0_variance, float bias0_variance)
{
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
        return;
    }
    if (!std::isfinite(accel_world_z) || !std::isfinite(dt_s)) {
        return;
    }
    if (dt_s <= 0.0f || dt_s > MAX_PREDICT_DT_S) {
        // Rejected rather than clamped: the model doesn't hold across a gap this long.
        return;
    }

    float bias = x_[2];

    float a = accel_world_z - GRAVITY_M_S2 - bias;

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

void ZAxisEkf::scalar_update(int h_index, float meas, float r_variance)
{
    if (!valid_) {
        return;
    }
    if (!std::isfinite(meas) || !std::isfinite(r_variance) || r_variance <= 0.0f) {
        return;
    }

    float H[3] = {0.0f, 0.0f, 0.0f};
    H[h_index] = 1.0f;

    float y = meas - math_util::dotProduct(H, x_);

    float PHt[3];
    math_util::multiplyMatrix3Vector(P_, H, PHt);
    float S = math_util::dotProduct(H, PHt) + r_variance;

    float sigma = std::sqrt(S);
    if (std::fabs(y) > INNOVATION_GATE_SIGMA * sigma) {
        rejected_update_count_++;
        return;
    }

    float K[3];
    for (int i = 0; i < 3; i++) {
        K[i] = PHt[i] / S;
    }

    for (int i = 0; i < 3; i++) {
        x_[i] += K[i] * y;
    }

    // Joseph form, P = (I-KH)*P*(I-KH)^T + K*r*K^T, keeps P positive semi-definite under rounding.
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
