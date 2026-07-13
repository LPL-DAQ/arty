#include "../../../../clover/src/flight/ZAxisEkf.h"
#include "../../../../clover/src/config.h"
#include <zephyr/ztest.h>
#include <cmath>

constexpr float EPSILON = 0.001f;

// ══ predict()-only tests (from the earlier predict() task) ═════════════════════════════════════

// Constant acceleration from rest matches closed-form kinematics

ZTEST(ZAxisEkf_tests, test_constant_acceleration_matches_closed_form_kinematics)
{
    ZAxisEkf ekf;
    ekf.init(0.0f, 0.0f);

    const float a_true = 2.0f;  // m/s^2, the true (bias-free) vertical acceleration for this test
    const float accel_world_z = GRAVITY_M_S2 + a_true;  // predict() subtracts gravity and bias
    const float dt = 0.01f;
    const int n = 50;  // 0.5s total

    for (int i = 0; i < n; i++) {
        ekf.predict(accel_world_z, dt);
    }

    // For piecewise-constant acceleration integrated with fixed dt, this discrete update is
    // exact (not just approximate) versus the continuous closed-form solution -- z(t) = 0.5*a*t^2,
    // vz(t) = a*t -- so a tight tolerance is appropriate here, not a loose one.
    const float t = n * dt;
    const float expected_z = 0.5f * a_true * t * t;
    const float expected_vz = a_true * t;

    zassert_within(ekf.z_m(), expected_z, EPSILON, "z should match closed-form kinematics for constant acceleration");
    zassert_within(ekf.vz_ms(), expected_vz, EPSILON, "vz should match closed-form kinematics for constant acceleration");
    zassert_within(ekf.accel_bias_mss(), 0.0f, 1e-6f, "bias should remain unchanged by predict() (random-walk model)");
}

// Nonzero initial bias with zero true accel causes drift exactly as the model predicts

ZTEST(ZAxisEkf_tests, test_nonzero_bias_with_zero_true_accel_causes_predicted_drift)
{
    ZAxisEkf ekf;
    ekf.init(0.0f, 0.0f);

    const float bias0 = 0.5f;  // m/s^2, injected directly -- init() always starts bias at 0
    ekf.set_state_for_testing(0.0f, 0.0f, bias0);

    const float dt = 0.01f;
    const int n = 50;

    // True vertical acceleration is exactly zero (vehicle at rest): accel_world_z reports only
    // gravity, nothing else. predict() computes a = accel_world_z - GRAVITY_M_S2 - bias, so the
    // nonzero bias alone should produce a spurious inferred acceleration of exactly -bias0.
    for (int i = 0; i < n; i++) {
        ekf.predict(GRAVITY_M_S2, dt);
    }

    const float t = n * dt;
    const float expected_a = -bias0;
    const float expected_z = 0.5f * expected_a * t * t;
    const float expected_vz = expected_a * t;

    zassert_within(ekf.z_m(), expected_z, EPSILON, "z should drift exactly per the bias-induced spurious acceleration");
    zassert_within(ekf.vz_ms(), expected_vz, EPSILON, "vz should drift exactly per the bias-induced spurious acceleration");
    zassert_within(ekf.accel_bias_mss(), bias0, 1e-6f, "bias must remain unchanged by predict() (random-walk model)");
}

// P grows during prediction, stays symmetric, diagonal stays positive

ZTEST(ZAxisEkf_tests, test_covariance_grows_stays_symmetric_and_positive_diagonal_during_prediction)
{
    ZAxisEkf ekf;
    ekf.init(0.0f, 0.0f);

    float p_before[3][3];
    ekf.get_covariance_for_testing(p_before);

    // P starts at all zeros right after init() -- confirmed explicitly so "grows" below is
    // measured against a known baseline, not just "whatever init() happens to leave behind".
    zassert_within(p_before[0][0], 0.0f, 1e-9f, "P should start at zero variance right after init()");
    zassert_within(p_before[1][1], 0.0f, 1e-9f, "P should start at zero variance right after init()");
    zassert_within(p_before[2][2], 0.0f, 1e-9f, "P should start at zero variance right after init()");

    const float dt = 0.01f;
    const int n = 20;
    for (int i = 0; i < n; i++) {
        ekf.predict(GRAVITY_M_S2 + 1.0f, dt);
    }

    float p_after[3][3];
    ekf.get_covariance_for_testing(p_after);

    // Diagonal (variance) should have grown from zero: Q adds uncertainty every predict() step,
    // and nothing removes it since no measurement update runs in this test.
    zassert_true(p_after[0][0] > p_before[0][0], "z variance should grow during prediction with no measurement updates");
    zassert_true(p_after[1][1] > p_before[1][1], "vz variance should grow during prediction with no measurement updates");
    zassert_true(p_after[2][2] > p_before[2][2], "bias variance should grow during prediction with no measurement updates");

    // Diagonal entries (variances) must stay strictly positive -- zero or negative variance would
    // be physically meaningless for a covariance matrix.
    zassert_true(p_after[0][0] > 0.0f, "z variance should be positive");
    zassert_true(p_after[1][1] > 0.0f, "vz variance should be positive");
    zassert_true(p_after[2][2] > 0.0f, "bias variance should be positive");

    // Symmetric: P[i][j] == P[j][i] for every off-diagonal pair.
    zassert_within(p_after[0][1], p_after[1][0], 1e-6f, "P should stay symmetric: P[0][1] == P[1][0]");
    zassert_within(p_after[0][2], p_after[2][0], 1e-6f, "P should stay symmetric: P[0][2] == P[2][0]");
    zassert_within(p_after[1][2], p_after[2][1], 1e-6f, "P should stay symmetric: P[1][2] == P[2][1]");

    // Same getters ZAxisEkf's real (non-test) interface exposes should agree with the raw matrix.
    zassert_within(ekf.z_variance(), p_after[0][0], 1e-9f, "z_variance() should match P[0][0]");
    zassert_within(ekf.vz_variance(), p_after[1][1], 1e-9f, "vz_variance() should match P[1][1]");
    zassert_within(ekf.accel_bias_variance(), p_after[2][2], 1e-9f, "accel_bias_variance() should match P[2][2]");
}

// ══ update_altitude()/update_velocity() tests (this task) ══════════════════════════════════════

// (1) Altitude update pulls z toward the measurement and shrinks P[0][0]

ZTEST(ZAxisEkf_tests, test_altitude_update_pulls_z_toward_measurement_and_shrinks_variance)
{
    ZAxisEkf ekf;
    ekf.init(0.0f, 0.0f);

    const float dt = 0.01f;
    for (int i = 0; i < 20; i++) {
        ekf.predict(GRAVITY_M_S2 + 1.0f, dt);
    }

    const float z_before = ekf.z_m();
    const float p_before = ekf.z_variance();
    zassert_true(p_before > 0.0f, "P[0][0] should be nonzero after predict-only steps");

    // Offset chosen to be a plausible measurement given the filter's current uncertainty (well
    // inside the innovation gate, not an outlier) -- a large arbitrary offset like +10m here would
    // exceed 5*sigma against the tiny P[0][0] built up from only 20 short predict steps and get
    // rejected by the innovation gate, defeating the point of this test (verified numerically
    // before picking this value, not guessed).
    const float z_meas = z_before + 0.5f;
    ekf.update_altitude(z_meas, 0.1f);

    zassert_true(ekf.z_m() > z_before, "z should move toward the measurement after the update");
    zassert_true(ekf.z_m() < z_meas, "z should not overshoot past the measurement (K should be < 1 for finite P, r)");
    zassert_true(ekf.z_variance() < p_before, "P[0][0] should shrink after a measurement update");
    zassert_equal(ekf.rejected_update_count(), 0, "a plausible measurement should not be rejected");
}

// (2) Smaller r_variance pulls harder than larger r_variance, same measurement

ZTEST(ZAxisEkf_tests, test_smaller_r_variance_pulls_harder_toward_measurement)
{
    ZAxisEkf ekf_small_r;
    ZAxisEkf ekf_large_r;
    ekf_small_r.init(0.0f, 0.0f);
    ekf_large_r.init(0.0f, 0.0f);

    const float dt = 0.01f;
    for (int i = 0; i < 20; i++) {
        ekf_small_r.predict(GRAVITY_M_S2 + 1.0f, dt);
        ekf_large_r.predict(GRAVITY_M_S2 + 1.0f, dt);
    }

    // Identical predict history on both filters -- z_before is the same for both. Offset kept
    // small enough to clear the innovation gate for the SMALL-r case (the tighter constraint,
    // since smaller r means smaller S means a smaller 5*sigma allowance) -- verified numerically.
    const float z_before = ekf_small_r.z_m();
    const float z_meas = z_before + 0.3f;

    ekf_small_r.update_altitude(z_meas, 0.01f);   // confident measurement
    ekf_large_r.update_altitude(z_meas, 100.0f);  // noisy measurement

    const float move_small_r = ekf_small_r.z_m() - z_before;
    const float move_large_r = ekf_large_r.z_m() - z_before;

    zassert_true(move_small_r > 0.0f && move_large_r > 0.0f, "both should move toward the measurement, just by different amounts");
    zassert_true(move_small_r > move_large_r, "smaller r_variance should pull z harder toward the measurement");
}

// (3) Repeated offset altitude updates converge the bias estimate toward the true bias

ZTEST(ZAxisEkf_tests, test_repeated_altitude_updates_with_constant_offset_converge_bias_estimate)
{
    ZAxisEkf ekf;
    ekf.init(0.0f, 0.0f);

    const float true_bias = 0.3f;    // m/s^2, the real (unknown to the filter) accelerometer bias
    const float a_true = 0.5f;       // m/s^2, the real (unbiased) vertical acceleration
    const float dt = 0.01f;
    const float r_variance = 0.01f;  // fairly confident altitude measurements

    // Accelerometer reports the true acceleration plus a constant, unmodeled bias every step --
    // predict() alone (bias estimate starts at 0) would drift z increasingly far from the true
    // trajectory. Correcting with the TRUE (unbiased) altitude every step is what lets the filter
    // attribute that persistent discrepancy to the bias state via the F matrix's bias-column
    // coupling, rather than to z/vz alone.
    //
    // 20,000 steps (200s simulated), not 2000: PROCESS_NOISE_BIAS_M2_S4 is deliberately tiny
    // (1e-6, "bias should drift slowly"), which is exactly what makes the filter cautious about
    // moving its bias estimate -- convergence is real but slow. Verified numerically that 2000
    // steps only reaches ~2% convergence, while 20,000 reaches ~73%, comfortably past this test's
    // 50% bar. native_sim runs this near-instantly either way (pure arithmetic, no simulated-time
    // delay), so the extra steps cost nothing in test runtime.
    float t = 0.0f;
    for (int i = 0; i < 20000; i++) {
        ekf.predict(GRAVITY_M_S2 + a_true + true_bias, dt);
        t += dt;

        const float z_true = 0.5f * a_true * t * t;
        ekf.update_altitude(z_true, r_variance);
    }

    // Check convergence as a relative reduction in error rather than a hand-picked absolute
    // threshold: the bias estimate should have moved substantially closer to the true value than
    // its 0 starting point, not just become nonzero.
    const float initial_error = std::fabs(0.0f - true_bias);
    const float final_error = std::fabs(ekf.accel_bias_mss() - true_bias);

    zassert_true(final_error < initial_error * 0.5f, "bias estimate should converge to within 50% of the true bias, starting from 0");
    zassert_equal(ekf.rejected_update_count(), 0, "none of these consistent, small-innovation updates should trip the outlier gate");
}

// (4) Innovation gate rejects a wild outlier, state unchanged

ZTEST(ZAxisEkf_tests, test_innovation_gate_rejects_wild_outlier_and_leaves_state_unchanged)
{
    ZAxisEkf ekf;
    ekf.init(0.0f, 0.0f);

    const float dt = 0.01f;
    for (int i = 0; i < 20; i++) {
        ekf.predict(GRAVITY_M_S2 + 1.0f, dt);
    }

    const float z_before = ekf.z_m();
    const float vz_before = ekf.vz_ms();
    const float bias_before = ekf.accel_bias_mss();
    float p_before[3][3];
    ekf.get_covariance_for_testing(p_before);

    zassert_equal(ekf.rejected_update_count(), 0, "no rejections should have happened yet");

    // Wildly implausible given the filter's tiny uncertainty after only 20 short predict steps --
    // 1000m makes this unambiguous regardless of P[0][0]'s exact value.
    ekf.update_altitude(z_before + 1000.0f, 0.01f);

    zassert_equal(ekf.rejected_update_count(), 1, "the wild outlier should have been counted as rejected");
    zassert_within(ekf.z_m(), z_before, 1e-6f, "z should be unchanged after a rejected update");
    zassert_within(ekf.vz_ms(), vz_before, 1e-6f, "vz should be unchanged after a rejected update");
    zassert_within(ekf.accel_bias_mss(), bias_before, 1e-6f, "bias should be unchanged after a rejected update");

    float p_after[3][3];
    ekf.get_covariance_for_testing(p_after);
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            zassert_within(p_after[i][j], p_before[i][j], 1e-9f, "P should be unchanged after a rejected update");
        }
    }
}

// (5) P remains symmetric, positive-diagonal through 1000 mixed predict/update cycles

ZTEST(ZAxisEkf_tests, test_covariance_stays_symmetric_and_positive_diagonal_through_1000_mixed_cycles)
{
    ZAxisEkf ekf;
    ekf.init(0.0f, 0.0f);

    const float dt = 0.01f;
    for (int i = 0; i < 1000; i++) {
        ekf.predict(GRAVITY_M_S2 + 1.0f, dt);

        // Alternate altitude and velocity updates to exercise both H vectors.
        if (i % 2 == 0) {
            ekf.update_altitude(static_cast<float>(i) * 0.001f, 0.05f);
        }
        else {
            ekf.update_velocity(static_cast<float>(i) * 0.0005f, 0.02f);
        }

        float p[3][3];
        ekf.get_covariance_for_testing(p);

        zassert_true(p[0][0] > 0.0f, "P[0][0] should stay positive through mixed predict/update cycles");
        zassert_true(p[1][1] > 0.0f, "P[1][1] should stay positive through mixed predict/update cycles");
        zassert_true(p[2][2] > 0.0f, "P[2][2] should stay positive through mixed predict/update cycles");
        zassert_within(p[0][1], p[1][0], 1e-4f, "P should stay symmetric through mixed predict/update cycles");
        zassert_within(p[0][2], p[2][0], 1e-4f, "P should stay symmetric through mixed predict/update cycles");
        zassert_within(p[1][2], p[2][1], 1e-4f, "P should stay symmetric through mixed predict/update cycles");
    }
}

ZTEST_SUITE(ZAxisEkf_tests, NULL, NULL, NULL, NULL, NULL);
