#include "../../../../clover/src/flight/StateEstimator.h"
#include "../../../../clover/src/config.h"
#include "../../../../clover/src/math_util.h"
#include <zephyr/ztest.h>
#include <cmath>
#include <optional>

namespace
{

constexpr uint64_t PERIOD_NS = 2'500'000;  // 2.5ms, matches VN300's ~400Hz update rate
constexpr float PI_F = 3.14159265f;

// Warms VN300 to HEALTHY: 41 fresh, mutually-agreeing readings from t=0, spaced PERIOD_NS apart
// (1 tick starts the re-convergence window, 40 more complete the 100ms requirement).
void warm_vn300_to_healthy(
    uint64_t& t_ns,
    LidarReading& lidar_1,
    LidarReading& lidar_2,
    ImuReading& imu,
    ImuReading& backup_1,
    ImuReading& backup_2,
    GnssReadings& gnss
)
{
    for (int i = 0; i < 41; i++) {
        t_ns += PERIOD_NS;
        StateEstimator::set_now_ns_for_testing(t_ns);
        imu.arrival_time_ns = t_ns;
        imu.has_arrival_time_ns = true;
        imu.sense_time_ns = static_cast<float>(i + 1);
        StateEstimator::estimate(lidar_1, lidar_2, imu, backup_1, backup_2, gnss);
    }
}

// Asserts R_WB component-wise, so a test can prove WHICH sensor the attitude actually came from
// rather than only that a health flag flipped.
void assert_attitude_is(const EstimatedState& state, float qw, float qx, float qy, float qz, const char* what)
{
    zassert_within(state.R_WB.qw, qw, 1e-6f, "%s (qw)", what);
    zassert_within(state.R_WB.qx, qx, 1e-6f, "%s (qx)", what);
    zassert_within(state.R_WB.qy, qy, 1e-6f, "%s (qy)", what);
    zassert_within(state.R_WB.qz, qz, 1e-6f, "%s (qz)", what);
}

}  // namespace

// ── (1) VN300 staleness: triggers at >50ms, clears only after a full 100ms re-convergence ──────

ZTEST(StateEstimator_tests, test_vn300_staleness_triggers_after_50ms_and_clears_after_100ms_retrust)
{
    StateEstimator::reset();
    StateEstimator::set_now_ns_for_testing(0);

    LidarReading lidar_1 = LidarReading_init_default;
    LidarReading lidar_2 = LidarReading_init_default;
    ImuReading backup_1 = ImuReading_init_default;
    ImuReading backup_2 = ImuReading_init_default;
    GnssReadings gnss = GnssReadings_init_default;

    zassert_true(StateEstimator::vn300_is_stale_for_testing(), "VN300 should start STALE before any reading arrives");

    ImuReading imu = ImuReading_init_default;
    imu.quat_w = 0.7071f;
    imu.quat_x = 0.7071f;  // non-identity, so "frozen" vs "updated" is unambiguous

    uint64_t t_ns = 0;
    warm_vn300_to_healthy(t_ns, lidar_1, lidar_2, imu, backup_1, backup_2, gnss);
    zassert_true(StateEstimator::vn300_is_healthy_for_testing(), "VN300 should be HEALTHY after 100ms of continuous good data");

    // Stop feeding new readings (has_arrival_time_ns = false, matching how Controller.cpp
    // zero-inits DataPacket.imu each tick when VectornavImu::read() returns nullopt) and advance
    // time past the 50ms staleness threshold.
    imu.has_arrival_time_ns = false;
    t_ns += 60'000'000;  // +60ms with no new reading
    StateEstimator::set_now_ns_for_testing(t_ns);
    StateEstimator::estimate(lidar_1, lidar_2, imu, backup_1, backup_2, gnss);
    zassert_true(StateEstimator::vn300_is_stale_for_testing(), "VN300 should go STALE after >50ms without a new reading");

    // A single fresh reading after the outage should NOT immediately restore trust -- the
    // re-convergence window must run again from scratch.
    t_ns += PERIOD_NS;
    StateEstimator::set_now_ns_for_testing(t_ns);
    imu.arrival_time_ns = t_ns;
    imu.has_arrival_time_ns = true;
    imu.sense_time_ns += 1.0f;
    StateEstimator::estimate(lidar_1, lidar_2, imu, backup_1, backup_2, gnss);
    zassert_false(StateEstimator::vn300_is_healthy_for_testing(), "A single reading after an outage should not instantly restore trust");
    zassert_true(StateEstimator::vn300_is_stale_for_testing(), "VN300 should remain STALE during the re-convergence window");

    // 100ms (40 more consecutive updates) of continuous good data should restore trust.
    for (int i = 0; i < 40; i++) {
        t_ns += PERIOD_NS;
        StateEstimator::set_now_ns_for_testing(t_ns);
        imu.arrival_time_ns = t_ns;
        imu.sense_time_ns += 1.0f;
        StateEstimator::estimate(lidar_1, lidar_2, imu, backup_1, backup_2, gnss);
    }
    zassert_true(StateEstimator::vn300_is_healthy_for_testing(), "VN300 should be HEALTHY again after a full 100ms re-convergence window");
}

// ── (2) VN300 divergence: flags at the Nth consecutive mismatch, not before ─────────────────────

ZTEST(StateEstimator_tests, test_vn300_divergence_flags_after_n_consecutive_mismatches)
{
    StateEstimator::reset();
    StateEstimator::set_now_ns_for_testing(0);

    LidarReading lidar_1 = LidarReading_init_default;
    LidarReading lidar_2 = LidarReading_init_default;
    GnssReadings gnss = GnssReadings_init_default;

    ImuReading imu = ImuReading_init_default;
    imu.quat_w = 1.0f;
    ImuReading backup_1 = ImuReading_init_default;
    backup_1.quat_w = 1.0f;
    ImuReading backup_2 = ImuReading_init_default;
    backup_2.quat_w = 1.0f;

    uint64_t t_ns = 0;
    warm_vn300_to_healthy(t_ns, lidar_1, lidar_2, imu, backup_1, backup_2, gnss);
    zassert_true(StateEstimator::vn300_is_healthy_for_testing(), "VN300 should be HEALTHY after warmup");

    // Backups continue to agree with each other; VN300 now diverges from both.
    imu.quat_w = 0.0f;
    imu.quat_x = 1.0f;  // 180 degrees off -- well outside the placeholder threshold

    int n = StateEstimator::vn300_divergence_streak_threshold_for_testing();
    for (int i = 0; i < n - 1; i++) {
        t_ns += PERIOD_NS;
        StateEstimator::set_now_ns_for_testing(t_ns);
        imu.arrival_time_ns = t_ns;
        imu.sense_time_ns += 1.0f;
        backup_1.has_arrival_time_ns = true;
        backup_1.arrival_time_ns = t_ns;
        backup_2.has_arrival_time_ns = true;
        backup_2.arrival_time_ns = t_ns;
        StateEstimator::estimate(lidar_1, lidar_2, imu, backup_1, backup_2, gnss);
        zassert_false(StateEstimator::vn300_is_faulty_for_testing(), "VN300 should not be flagged before N consecutive mismatches");
    }

    t_ns += PERIOD_NS;
    StateEstimator::set_now_ns_for_testing(t_ns);
    imu.arrival_time_ns = t_ns;
    imu.sense_time_ns += 1.0f;
    backup_1.arrival_time_ns = t_ns;
    backup_2.arrival_time_ns = t_ns;
    StateEstimator::estimate(lidar_1, lidar_2, imu, backup_1, backup_2, gnss);
    zassert_true(StateEstimator::vn300_is_faulty_for_testing(), "VN300 should be flagged faulty at the Nth consecutive mismatch");
}

// ── (3) VN300 divergence: un-flags only after N consecutive agreements ──────────────────────────

ZTEST(StateEstimator_tests, test_vn300_divergence_unflags_after_n_consecutive_agreements)
{
    StateEstimator::reset();
    StateEstimator::set_now_ns_for_testing(0);

    LidarReading lidar_1 = LidarReading_init_default;
    LidarReading lidar_2 = LidarReading_init_default;
    GnssReadings gnss = GnssReadings_init_default;

    ImuReading imu = ImuReading_init_default;
    imu.quat_w = 1.0f;
    ImuReading backup_1 = ImuReading_init_default;
    backup_1.quat_w = 1.0f;
    ImuReading backup_2 = ImuReading_init_default;
    backup_2.quat_w = 1.0f;

    uint64_t t_ns = 0;
    warm_vn300_to_healthy(t_ns, lidar_1, lidar_2, imu, backup_1, backup_2, gnss);
    zassert_true(StateEstimator::vn300_is_healthy_for_testing(), "VN300 should be HEALTHY after warmup");

    int n = StateEstimator::vn300_divergence_streak_threshold_for_testing();

    // Drive VN300 to FAULTY via N consecutive mismatches.
    imu.quat_w = 0.0f;
    imu.quat_x = 1.0f;
    for (int i = 0; i < n; i++) {
        t_ns += PERIOD_NS;
        StateEstimator::set_now_ns_for_testing(t_ns);
        imu.arrival_time_ns = t_ns;
        imu.sense_time_ns += 1.0f;
        backup_1.has_arrival_time_ns = true;
        backup_1.arrival_time_ns = t_ns;
        backup_2.has_arrival_time_ns = true;
        backup_2.arrival_time_ns = t_ns;
        StateEstimator::estimate(lidar_1, lidar_2, imu, backup_1, backup_2, gnss);
    }
    zassert_true(StateEstimator::vn300_is_faulty_for_testing(), "VN300 should be flagged faulty");

    // Bring VN300 back into agreement with the backups.
    imu.quat_w = 1.0f;
    imu.quat_x = 0.0f;
    for (int i = 0; i < n - 1; i++) {
        t_ns += PERIOD_NS;
        StateEstimator::set_now_ns_for_testing(t_ns);
        imu.arrival_time_ns = t_ns;
        imu.sense_time_ns += 1.0f;
        backup_1.arrival_time_ns = t_ns;
        backup_2.arrival_time_ns = t_ns;
        StateEstimator::estimate(lidar_1, lidar_2, imu, backup_1, backup_2, gnss);
        zassert_true(StateEstimator::vn300_is_faulty_for_testing(), "VN300 should remain flagged before N consecutive agreements");
    }

    t_ns += PERIOD_NS;
    StateEstimator::set_now_ns_for_testing(t_ns);
    imu.arrival_time_ns = t_ns;
    imu.sense_time_ns += 1.0f;
    backup_1.arrival_time_ns = t_ns;
    backup_2.arrival_time_ns = t_ns;
    StateEstimator::estimate(lidar_1, lidar_2, imu, backup_1, backup_2, gnss);
    zassert_false(StateEstimator::vn300_is_faulty_for_testing(), "VN300 should be un-flagged at the Nth consecutive agreement");
}

// ── (4) Javad check doesn't fire while VN300 is already flagged ─────────────────────────────────

ZTEST(StateEstimator_tests, test_javad_check_does_not_fire_when_vn300_already_flagged)
{
    StateEstimator::reset();
    StateEstimator::set_now_ns_for_testing(0);

    LidarReading lidar_1 = LidarReading_init_default;
    LidarReading lidar_2 = LidarReading_init_default;

    ImuReading imu = ImuReading_init_default;
    imu.quat_w = 1.0f;
    imu.has_vel_n = true;
    imu.has_vel_e = true;
    imu.has_vel_d = true;
    imu.vel_n = 0.0f;
    imu.vel_e = 0.0f;
    imu.vel_d = 0.0f;

    ImuReading backup_1 = ImuReading_init_default;
    backup_1.quat_w = 1.0f;
    ImuReading backup_2 = ImuReading_init_default;
    backup_2.quat_w = 1.0f;

    GnssReadings gnss = GnssReadings_init_default;
    gnss.vx_ms = 0.0f;
    gnss.vy_ms = 0.0f;
    gnss.vz_ms = 0.0f;

    uint64_t t_ns = 0;
    warm_vn300_to_healthy(t_ns, lidar_1, lidar_2, imu, backup_1, backup_2, gnss);
    zassert_true(StateEstimator::vn300_is_healthy_for_testing(), "VN300 should be HEALTHY after warmup");

    // Flag VN300 via quaternion divergence.
    imu.quat_w = 0.0f;
    imu.quat_x = 1.0f;
    int vn300_n = StateEstimator::vn300_divergence_streak_threshold_for_testing();
    for (int i = 0; i < vn300_n; i++) {
        t_ns += PERIOD_NS;
        StateEstimator::set_now_ns_for_testing(t_ns);
        imu.arrival_time_ns = t_ns;
        imu.sense_time_ns += 1.0f;
        backup_1.has_arrival_time_ns = true;
        backup_1.arrival_time_ns = t_ns;
        backup_2.has_arrival_time_ns = true;
        backup_2.arrival_time_ns = t_ns;
        StateEstimator::estimate(lidar_1, lidar_2, imu, backup_1, backup_2, gnss);
    }
    zassert_true(StateEstimator::vn300_is_faulty_for_testing(), "VN300 should now be flagged faulty");

    // Feed wildly mismatching Javad velocity for far more than the Javad flagging threshold would
    // normally require -- it must never flag, since VN300 (the corroborating reference for this
    // check) is itself untrustworthy.
    gnss.vx_ms = 999.0f;
    int javad_n = StateEstimator::javad_divergence_streak_threshold_for_testing();
    for (int i = 0; i < javad_n * 3; i++) {
        t_ns += PERIOD_NS;
        StateEstimator::set_now_ns_for_testing(t_ns);
        gnss.has_arrival_time_ns = true;
        gnss.arrival_time_ns = t_ns;
        gnss.sense_time_ns += 1.0f;
        imu.arrival_time_ns = t_ns;  // VN300 stays "fresh" (still faulty, still producing readings)
        imu.sense_time_ns += 1.0f;
        StateEstimator::estimate(lidar_1, lidar_2, imu, backup_1, backup_2, gnss);
    }
    zassert_false(StateEstimator::javad_is_faulty_for_testing(), "Javad should never be flagged while VN300 is already flagged faulty");
}

// ── (5) Simultaneous VN300+Javad fault: CRITICAL, attitude falls back to the backup IMU ────────

ZTEST(StateEstimator_tests, test_simultaneous_vn300_and_javad_fault_triggers_critical_and_falls_back_to_backup_imu)
{
    StateEstimator::reset();
    StateEstimator::set_now_ns_for_testing(0);

    LidarReading lidar_1 = LidarReading_init_default;
    LidarReading lidar_2 = LidarReading_init_default;

    ImuReading imu = ImuReading_init_default;
    imu.quat_w = 1.0f;
    imu.has_vel_n = true;
    imu.has_vel_e = true;
    imu.has_vel_d = true;
    imu.vel_n = 0.0f;
    imu.vel_e = 0.0f;
    imu.vel_d = 0.0f;

    ImuReading backup_1 = ImuReading_init_default;
    backup_1.quat_w = 1.0f;
    ImuReading backup_2 = ImuReading_init_default;
    backup_2.quat_w = 1.0f;

    GnssReadings gnss = GnssReadings_init_default;
    gnss.vx_ms = 0.0f;
    gnss.vy_ms = 0.0f;
    gnss.vz_ms = 0.0f;

    uint64_t t_ns = 0;
    warm_vn300_to_healthy(t_ns, lidar_1, lidar_2, imu, backup_1, backup_2, gnss);
    zassert_true(StateEstimator::vn300_is_healthy_for_testing(), "VN300 should be HEALTHY after warmup");

    // Flag Javad first, while VN300 is still healthy.
    gnss.vx_ms = 100.0f;  // wildly diverges from imu.vel_n/e/d = 0
    int javad_n = StateEstimator::javad_divergence_streak_threshold_for_testing();
    for (int i = 0; i < javad_n; i++) {
        t_ns += PERIOD_NS;
        StateEstimator::set_now_ns_for_testing(t_ns);
        imu.arrival_time_ns = t_ns;
        imu.sense_time_ns += 1.0f;
        gnss.has_arrival_time_ns = true;
        gnss.arrival_time_ns = t_ns;
        gnss.sense_time_ns += 1.0f;
        StateEstimator::estimate(lidar_1, lidar_2, imu, backup_1, backup_2, gnss);
    }
    zassert_true(StateEstimator::javad_is_faulty_for_testing(), "Javad should be flagged faulty");
    zassert_true(StateEstimator::vn300_is_healthy_for_testing(), "VN300 should still be healthy -- only Javad is faulty so far");

    // Now flag VN300 too (quaternion mismatch vs. the still-agreeing backups).
    imu.quat_w = 0.0f;
    imu.quat_x = 1.0f;
    int vn300_n = StateEstimator::vn300_divergence_streak_threshold_for_testing();
    std::optional<EstimatedState> last_result;
    for (int i = 0; i < vn300_n; i++) {
        t_ns += PERIOD_NS;
        StateEstimator::set_now_ns_for_testing(t_ns);
        imu.arrival_time_ns = t_ns;
        imu.sense_time_ns += 1.0f;
        backup_1.has_arrival_time_ns = true;
        backup_1.arrival_time_ns = t_ns;
        backup_2.has_arrival_time_ns = true;
        backup_2.arrival_time_ns = t_ns;
        last_result = StateEstimator::estimate(lidar_1, lidar_2, imu, backup_1, backup_2, gnss);
    }
    zassert_true(StateEstimator::vn300_is_faulty_for_testing(), "VN300 should now be flagged faulty too");
    zassert_true(last_result.has_value(), "estimate() should still return a value under critical fault");
    zassert_equal(last_result->fault_status, EstimatorFaultStatus_CRITICAL, "fault_status should be CRITICAL when both VN300 and Javad are flagged");

    EstimatedState frozen_snapshot = *last_result;

    // Attitude must now come from backup IMU 1 rather than freeze. A distinguishable quaternion,
    // applied only from here, makes "fell back to the backup" and "held its last VN300 value"
    // impossible to confuse. Only backup_1 feeds attitude; backup_2 is left as it was.
    backup_1.quat_w = 0.7071f;
    backup_1.quat_x = 0.0f;
    backup_1.quat_z = 0.7071f;

    // Feed yet more, clearly different data. Lateral must still hold -- Javad is the only lateral
    // source and it is faulty -- while attitude tracks the backup.
    for (int i = 0; i < 5; i++) {
        t_ns += PERIOD_NS;
        StateEstimator::set_now_ns_for_testing(t_ns);
        imu.arrival_time_ns = t_ns;
        imu.sense_time_ns += 1.0f;
        imu.quat_x = 0.0f;
        imu.quat_y = 1.0f;  // yet another different orientation
        backup_1.sense_time_ns += 1.0f;
        gnss.vx_ms = -55.0f;
        gnss.arrival_time_ns = t_ns;
        gnss.sense_time_ns += 1.0f;

        auto result = StateEstimator::estimate(lidar_1, lidar_2, imu, backup_1, backup_2, gnss);
        zassert_true(result.has_value(), "estimate() should keep returning a value under critical fault");
        zassert_equal(result->fault_status, EstimatorFaultStatus_CRITICAL, "Should remain CRITICAL");
        assert_attitude_is(
            *result, 0.7071f, 0.0f, 0.0f, 0.7071f,
            "attitude should fall back to backup IMU 1 under a critical fault rather than freeze"
        );
        zassert_within(result->position.x, frozen_snapshot.position.x, 0.0001f, "position.x should be frozen under critical fault");
        zassert_within(result->velocity.x, frozen_snapshot.velocity.x, 0.0001f, "velocity.x should be frozen under critical fault");
    }
}

// ── (6) calculateVerticalAltitude(): hand-computed expected outputs ─────────────────────────────
// Values verified against multiplyQuaternionVector's tested convention (see MathUtil_test.cpp:
// a +90-deg rotation about Y maps +Z to +X).

ZTEST(StateEstimator_tests, test_calculate_vertical_altitude_no_tilt_no_mount_angle)
{
    // Straight down, no mount offset, no vehicle tilt -- slant range IS the vertical altitude.
    Quaternion identity = math_util::createQuaternion(1.0f, 0.0f, 0.0f, 0.0f);
    float altitude = StateEstimator::calculate_vertical_altitude_for_testing(10.0f, identity, 0.0f);
    zassert_within(altitude, 10.0f, 0.01f, "Altitude should equal slant range with no tilt or mount angle");
}

ZTEST(StateEstimator_tests, test_calculate_vertical_altitude_with_vehicle_tilt)
{
    // R_WB representing a vehicle pitched 60 degrees about Y. calculateVerticalAltitude()
    // re-conjugates this internally to rotate body->world, recovering a clean +60-deg-about-Y
    // rotation to apply to the boresight. Expected: altitude = slant_range * cos(60deg) = 5.0.
    const float half = 30.0f * PI_F / 180.0f;  // half of 60 deg, in radians
    Quaternion attitude_wb = math_util::createQuaternion(std::cos(half), 0.0f, -std::sin(half), 0.0f);
    float altitude = StateEstimator::calculate_vertical_altitude_for_testing(10.0f, attitude_wb, 0.0f);
    zassert_within(altitude, 5.0f, 0.01f, "Altitude should be slant_range * cos(60deg) for a 60-deg vehicle tilt");
}

ZTEST(StateEstimator_tests, test_calculate_vertical_altitude_with_mount_angle)
{
    // No vehicle tilt, but a 30-degree LiDAR mount angle off straight-down.
    // Expected: altitude = slant_range * cos(30deg) = 8.660.
    Quaternion identity = math_util::createQuaternion(1.0f, 0.0f, 0.0f, 0.0f);
    float mount_angle_rad = 30.0f * PI_F / 180.0f;
    float altitude = StateEstimator::calculate_vertical_altitude_for_testing(10.0f, identity, mount_angle_rad);
    zassert_within(altitude, 8.660f, 0.01f, "Altitude should be slant_range * cos(mount_angle) with no vehicle tilt");
}

// ── (7) LiDAR-vs-Javad-vs-other-LiDAR cross-check ────────────────────────────────────────────────

ZTEST(StateEstimator_tests, test_lidar_cross_check_flags_diverging_lidar)
{
    StateEstimator::reset();
    StateEstimator::set_now_ns_for_testing(0);

    // imu.has_arrival_time_ns is left false throughout, so R_WB stays at its identity default,
    // keeping calculateVerticalAltitude()'s attitude input (and thus the math) trivial: with
    // LIDAR_MOUNT_ANGLE_DEG also 0 (current placeholder), altitude == distance_m exactly.
    ImuReading imu = ImuReading_init_default;
    ImuReading backup_1 = ImuReading_init_default;
    ImuReading backup_2 = ImuReading_init_default;

    GnssReadings gnss = GnssReadings_init_default;
    gnss.up_m = 10.0f;

    LidarReading lidar_1 = LidarReading_init_default;
    lidar_1.distance_m = 10.0f;  // agrees with Javad and (initially) with lidar_2
    LidarReading lidar_2 = LidarReading_init_default;
    lidar_2.distance_m = 3.0f;  // diverges from both Javad and lidar_1

    int n = StateEstimator::lidar_divergence_streak_threshold_for_testing();
    uint64_t t_ns = 0;

    for (int i = 0; i < n - 1; i++) {
        t_ns += PERIOD_NS;
        StateEstimator::set_now_ns_for_testing(t_ns);
        lidar_1.sense_time_ns = static_cast<float>(i + 1);
        lidar_2.sense_time_ns = static_cast<float>(i + 1);
        gnss.sense_time_ns = static_cast<float>(i + 1);
        gnss.has_arrival_time_ns = true;
        gnss.arrival_time_ns = t_ns;
        StateEstimator::estimate(lidar_1, lidar_2, imu, backup_1, backup_2, gnss);
        zassert_false(StateEstimator::lidar_2_is_faulty_for_testing(), "Lidar 2 should not be flagged before N consecutive mismatches");
        zassert_false(StateEstimator::lidar_1_is_faulty_for_testing(), "Lidar 1 should not be flagged -- it agrees with Javad throughout");
    }

    t_ns += PERIOD_NS;
    StateEstimator::set_now_ns_for_testing(t_ns);
    lidar_1.sense_time_ns = static_cast<float>(n);
    lidar_2.sense_time_ns = static_cast<float>(n);
    gnss.sense_time_ns = static_cast<float>(n);
    gnss.arrival_time_ns = t_ns;
    StateEstimator::estimate(lidar_1, lidar_2, imu, backup_1, backup_2, gnss);

    zassert_true(StateEstimator::lidar_2_is_faulty_for_testing(), "Lidar 2 should be flagged faulty after N consecutive mismatches with both Javad and Lidar 1");
    zassert_false(StateEstimator::lidar_1_is_faulty_for_testing(), "Lidar 1 should remain healthy -- it agrees with Javad throughout");
}

// ── (8) VN300 staleness escalates to FAULTY and attitude actually falls back to backup IMU 1 ────

ZTEST(StateEstimator_tests, test_vn300_long_outage_escalates_to_faulty_and_falls_back_to_backup_imu)
{
    StateEstimator::reset();
    StateEstimator::set_now_ns_for_testing(0);

    LidarReading lidar_1 = LidarReading_init_default;
    LidarReading lidar_2 = LidarReading_init_default;
    ImuReading backup_2 = ImuReading_init_default;
    GnssReadings gnss = GnssReadings_init_default;

    ImuReading imu = ImuReading_init_default;
    imu.quat_w = 0.7071f;
    imu.quat_x = 0.7071f;

    // Deliberately nothing like the VN300 quaternion, so "attitude came from the backup" and
    // "attitude is frozen at the last VN300 value" can never be confused for each other.
    ImuReading backup_1 = ImuReading_init_default;
    backup_1.quat_w = 0.0f;
    backup_1.quat_y = 1.0f;

    uint64_t t_ns = 0;
    warm_vn300_to_healthy(t_ns, lidar_1, lidar_2, imu, backup_1, backup_2, gnss);
    zassert_true(StateEstimator::vn300_is_healthy_for_testing(), "VN300 should be HEALTHY after warmup");

    // Stop feeding VN300 (has_arrival_time_ns = false, matching how Controller.cpp zero-inits
    // DataPacket.imu when VectornavImu::read() returns nullopt), but keep the backup fresh from
    // here on -- so any attitude change below can only have come from the backup.
    imu.has_arrival_time_ns = false;

    t_ns += 60'000'000;  // +60ms: past the 50ms stale threshold, well short of the 150ms fault one
    StateEstimator::set_now_ns_for_testing(t_ns);
    backup_1.sense_time_ns += 1.0f;
    auto stale_result = StateEstimator::estimate(lidar_1, lidar_2, imu, backup_1, backup_2, gnss);
    zassert_true(stale_result.has_value());
    zassert_true(StateEstimator::vn300_is_stale_for_testing(), "VN300 should be STALE after >50ms without a reading");
    assert_attitude_is(*stale_result, 0.7071f, 0.7071f, 0.0f, 0.0f, "a merely STALE VN300 should freeze the last attitude, not fall back to the backup");

    // Push total time without a reading past the fault threshold.
    t_ns += 100'000'000;  // 160ms total since the last VN300 reading
    StateEstimator::set_now_ns_for_testing(t_ns);
    backup_1.sense_time_ns += 1.0f;
    auto faulty_result = StateEstimator::estimate(lidar_1, lidar_2, imu, backup_1, backup_2, gnss);
    zassert_true(faulty_result.has_value());
    zassert_true(StateEstimator::vn300_is_faulty_for_testing(), "VN300 should escalate STALE -> FAULTY past the fault threshold, with no divergence check involved");
    assert_attitude_is(*faulty_result, 0.0f, 0.0f, 1.0f, 0.0f, "attitude should now come from backup IMU 1 -- the flag flipping is not enough, the fallback must actually happen");
}

// ── (9) A stale-induced FAULTY recovers on the re-trust timer, not the divergence agree-streak ──

ZTEST(StateEstimator_tests, test_vn300_stale_induced_fault_recovers_via_retrust_timer)
{
    StateEstimator::reset();
    StateEstimator::set_now_ns_for_testing(0);

    LidarReading lidar_1 = LidarReading_init_default;
    LidarReading lidar_2 = LidarReading_init_default;
    ImuReading backup_2 = ImuReading_init_default;
    GnssReadings gnss = GnssReadings_init_default;

    ImuReading imu = ImuReading_init_default;
    imu.quat_w = 0.7071f;
    imu.quat_x = 0.7071f;

    ImuReading backup_1 = ImuReading_init_default;
    backup_1.quat_w = 0.0f;
    backup_1.quat_y = 1.0f;

    uint64_t t_ns = 0;
    warm_vn300_to_healthy(t_ns, lidar_1, lidar_2, imu, backup_1, backup_2, gnss);

    // Drive it into stale-induced FAULTY.
    imu.has_arrival_time_ns = false;
    t_ns += 200'000'000;  // one jump straight past the fault threshold
    StateEstimator::set_now_ns_for_testing(t_ns);
    backup_1.sense_time_ns += 1.0f;
    StateEstimator::estimate(lidar_1, lidar_2, imu, backup_1, backup_2, gnss);
    zassert_true(StateEstimator::vn300_is_faulty_for_testing(), "VN300 should be FAULTY by staleness before recovery starts");

    // Data comes back. backup_1.sense_time_ns is deliberately frozen from here on, so the final
    // attitude assertion can only be satisfied by VN300 having genuinely reclaimed the channel.
    imu.has_arrival_time_ns = true;

    t_ns += PERIOD_NS;
    StateEstimator::set_now_ns_for_testing(t_ns);
    imu.arrival_time_ns = t_ns;
    imu.sense_time_ns += 1.0f;
    StateEstimator::estimate(lidar_1, lidar_2, imu, backup_1, backup_2, gnss);
    zassert_true(StateEstimator::vn300_is_faulty_for_testing(), "one reading back should not instantly restore trust -- the re-convergence window must run");

    // 40 more readings complete the 100ms re-trust window.
    std::optional<EstimatedState> result;
    for (int i = 0; i < 40; i++) {
        t_ns += PERIOD_NS;
        StateEstimator::set_now_ns_for_testing(t_ns);
        imu.arrival_time_ns = t_ns;
        imu.sense_time_ns += 1.0f;
        result = StateEstimator::estimate(lidar_1, lidar_2, imu, backup_1, backup_2, gnss);
    }

    zassert_true(result.has_value());
    zassert_true(StateEstimator::vn300_is_healthy_for_testing(), "a stale-induced FAULTY should clear via the re-trust timer once data returns");
    assert_attitude_is(*result, 0.7071f, 0.7071f, 0.0f, 0.0f, "attitude should be tracking VN300 again after recovery");
}

// ── (10) A brief stale window must NOT escalate -- guards the two-threshold split ────────────────

ZTEST(StateEstimator_tests, test_vn300_brief_stale_does_not_escalate_and_holds_last_attitude)
{
    StateEstimator::reset();
    StateEstimator::set_now_ns_for_testing(0);

    LidarReading lidar_1 = LidarReading_init_default;
    LidarReading lidar_2 = LidarReading_init_default;
    ImuReading backup_2 = ImuReading_init_default;
    GnssReadings gnss = GnssReadings_init_default;

    ImuReading imu = ImuReading_init_default;
    imu.quat_w = 0.7071f;
    imu.quat_x = 0.7071f;

    ImuReading backup_1 = ImuReading_init_default;
    backup_1.quat_w = 0.0f;
    backup_1.quat_y = 1.0f;

    uint64_t t_ns = 0;
    warm_vn300_to_healthy(t_ns, lidar_1, lidar_2, imu, backup_1, backup_2, gnss);
    zassert_true(StateEstimator::vn300_is_healthy_for_testing(), "VN300 should be HEALTHY after warmup");

    // Backup stays fresh throughout, so if STALE ever wrongly routed to the fallback this test
    // would catch it immediately.
    imu.has_arrival_time_ns = false;

    t_ns += 60'000'000;  // 60ms since the last reading
    StateEstimator::set_now_ns_for_testing(t_ns);
    backup_1.sense_time_ns += 1.0f;
    auto first = StateEstimator::estimate(lidar_1, lidar_2, imu, backup_1, backup_2, gnss);
    zassert_true(first.has_value());
    zassert_true(StateEstimator::vn300_is_stale_for_testing(), "60ms without a reading should be STALE, not FAULTY");
    zassert_false(StateEstimator::vn300_is_faulty_for_testing(), "60ms is short of the fault threshold");
    assert_attitude_is(*first, 0.7071f, 0.7071f, 0.0f, 0.0f, "attitude should stay frozen at the last VN300 value while merely STALE");

    t_ns += 40'000'000;  // 100ms total -- still inside the stale window, below the 150ms threshold
    StateEstimator::set_now_ns_for_testing(t_ns);
    backup_1.sense_time_ns += 1.0f;
    auto second = StateEstimator::estimate(lidar_1, lidar_2, imu, backup_1, backup_2, gnss);
    zassert_true(second.has_value());
    zassert_true(StateEstimator::vn300_is_stale_for_testing(), "100ms without a reading should still be STALE -- escalation must wait for the fault threshold");
    assert_attitude_is(*second, 0.7071f, 0.7071f, 0.0f, 0.0f, "attitude should still be frozen, not handed to the backup");
}

// ── (11) VN300 divergence counts fresh backup readings, not control-loop iterations ─────────────

ZTEST(StateEstimator_tests, test_vn300_divergence_streak_ignores_held_backup_readings)
{
    StateEstimator::reset();
    StateEstimator::set_now_ns_for_testing(0);

    LidarReading lidar_1 = LidarReading_init_default;
    LidarReading lidar_2 = LidarReading_init_default;
    GnssReadings gnss = GnssReadings_init_default;

    ImuReading imu = ImuReading_init_default;
    imu.quat_w = 1.0f;
    ImuReading backup_1 = ImuReading_init_default;
    backup_1.quat_w = 1.0f;
    ImuReading backup_2 = ImuReading_init_default;
    backup_2.quat_w = 1.0f;

    uint64_t t_ns = 0;
    warm_vn300_to_healthy(t_ns, lidar_1, lidar_2, imu, backup_1, backup_2, gnss);
    zassert_true(StateEstimator::vn300_is_healthy_for_testing(), "VN300 should be HEALTHY after warmup");

    // One tick with the backups present and still agreeing, to establish the last-counted stamp.
    t_ns += PERIOD_NS;
    StateEstimator::set_now_ns_for_testing(t_ns);
    imu.arrival_time_ns = t_ns;
    imu.sense_time_ns += 1.0f;
    backup_1.has_arrival_time_ns = true;
    backup_1.arrival_time_ns = t_ns;
    backup_2.has_arrival_time_ns = true;
    backup_2.arrival_time_ns = t_ns;
    StateEstimator::estimate(lidar_1, lidar_2, imu, backup_1, backup_2, gnss);

    const uint64_t held_backup_time_ns = t_ns;
    int n = StateEstimator::vn300_divergence_streak_threshold_for_testing();

    // Phase 1: VN300 diverges and the control loop keeps running, but the backups are HELD --
    // their arrival timestamp never moves, exactly as it would not between two 50-100Hz readings.
    // With the counter advancing per loop iteration these 3N ticks would have tripped N threefold.
    imu.quat_w = 0.0f;
    imu.quat_x = 1.0f;
    for (int i = 0; i < n * 3; i++) {
        t_ns += PERIOD_NS;
        StateEstimator::set_now_ns_for_testing(t_ns);
        imu.arrival_time_ns = t_ns;
        imu.sense_time_ns += 1.0f;
        backup_1.arrival_time_ns = held_backup_time_ns;
        backup_2.arrival_time_ns = held_backup_time_ns;
        StateEstimator::estimate(lidar_1, lidar_2, imu, backup_1, backup_2, gnss);
    }
    zassert_false(
        StateEstimator::vn300_is_faulty_for_testing(),
        "held backup readings must not advance the divergence streak, however many loop iterations run"
    );

    // Phase 2: the same sustained disagreement, now with a genuinely new backup pair each tick.
    // It must take exactly N distinct readings.
    for (int i = 0; i < n - 1; i++) {
        t_ns += PERIOD_NS;
        StateEstimator::set_now_ns_for_testing(t_ns);
        imu.arrival_time_ns = t_ns;
        imu.sense_time_ns += 1.0f;
        backup_1.arrival_time_ns = t_ns;
        backup_2.arrival_time_ns = t_ns;
        StateEstimator::estimate(lidar_1, lidar_2, imu, backup_1, backup_2, gnss);
        zassert_false(StateEstimator::vn300_is_faulty_for_testing(), "VN300 should not be flagged before N distinct backup readings");
    }

    t_ns += PERIOD_NS;
    StateEstimator::set_now_ns_for_testing(t_ns);
    imu.arrival_time_ns = t_ns;
    imu.sense_time_ns += 1.0f;
    backup_1.arrival_time_ns = t_ns;
    backup_2.arrival_time_ns = t_ns;
    StateEstimator::estimate(lidar_1, lidar_2, imu, backup_1, backup_2, gnss);
    zassert_true(StateEstimator::vn300_is_faulty_for_testing(), "VN300 should be flagged on exactly the Nth distinct backup reading");
}

// ── (12) Javad divergence counts fresh GNSS epochs, not control-loop iterations ──────────────────

ZTEST(StateEstimator_tests, test_javad_divergence_streak_ignores_held_gnss_readings)
{
    StateEstimator::reset();
    StateEstimator::set_now_ns_for_testing(0);

    LidarReading lidar_1 = LidarReading_init_default;
    LidarReading lidar_2 = LidarReading_init_default;

    // Backups stay absent (has_arrival_time_ns false), so the VN300 cross-check never runs and
    // VN300 health is driven purely by freshness here.
    ImuReading backup_1 = ImuReading_init_default;
    ImuReading backup_2 = ImuReading_init_default;

    ImuReading imu = ImuReading_init_default;
    imu.quat_w = 1.0f;
    imu.has_vel_n = true;
    imu.has_vel_e = true;
    imu.has_vel_d = true;
    imu.vel_n = 0.0f;
    imu.vel_e = 0.0f;
    imu.vel_d = 0.0f;

    GnssReadings gnss = GnssReadings_init_default;
    gnss.vx_ms = 0.0f;
    gnss.vy_ms = 0.0f;
    gnss.vz_ms = 0.0f;

    uint64_t t_ns = 0;
    warm_vn300_to_healthy(t_ns, lidar_1, lidar_2, imu, backup_1, backup_2, gnss);
    zassert_true(StateEstimator::vn300_is_healthy_for_testing(), "VN300 should be HEALTHY after warmup");

    // Baseline tick: GNSS present and still agreeing, establishing the last-counted stamp.
    t_ns += PERIOD_NS;
    StateEstimator::set_now_ns_for_testing(t_ns);
    imu.arrival_time_ns = t_ns;
    imu.sense_time_ns += 1.0f;
    gnss.has_arrival_time_ns = true;
    gnss.arrival_time_ns = t_ns;
    StateEstimator::estimate(lidar_1, lidar_2, imu, backup_1, backup_2, gnss);

    const uint64_t held_gnss_time_ns = t_ns;
    int n = StateEstimator::javad_divergence_streak_threshold_for_testing();

    // Phase 1: Javad's velocity disagrees wildly, but its epoch is HELD. At 10-20Hz against a
    // ~1000Hz loop this is the common case, and the un-gated counter reached N inside a single
    // epoch -- before Javad had sent a second reading to disagree independently.
    gnss.vx_ms = 100.0f;
    for (int i = 0; i < n * 3; i++) {
        t_ns += PERIOD_NS;
        StateEstimator::set_now_ns_for_testing(t_ns);
        imu.arrival_time_ns = t_ns;
        imu.sense_time_ns += 1.0f;
        gnss.arrival_time_ns = held_gnss_time_ns;
        StateEstimator::estimate(lidar_1, lidar_2, imu, backup_1, backup_2, gnss);
    }
    zassert_false(
        StateEstimator::javad_is_faulty_for_testing(),
        "a single held Javad epoch must not advance the divergence streak, however many loop iterations run"
    );

    // Phase 2: one genuinely new Javad epoch per tick -- exactly N to flag.
    for (int i = 0; i < n - 1; i++) {
        t_ns += PERIOD_NS;
        StateEstimator::set_now_ns_for_testing(t_ns);
        imu.arrival_time_ns = t_ns;
        imu.sense_time_ns += 1.0f;
        gnss.arrival_time_ns = t_ns;
        StateEstimator::estimate(lidar_1, lidar_2, imu, backup_1, backup_2, gnss);
        zassert_false(StateEstimator::javad_is_faulty_for_testing(), "Javad should not be flagged before N distinct epochs");
    }

    t_ns += PERIOD_NS;
    StateEstimator::set_now_ns_for_testing(t_ns);
    imu.arrival_time_ns = t_ns;
    imu.sense_time_ns += 1.0f;
    gnss.arrival_time_ns = t_ns;
    StateEstimator::estimate(lidar_1, lidar_2, imu, backup_1, backup_2, gnss);
    zassert_true(StateEstimator::javad_is_faulty_for_testing(), "Javad should be flagged on exactly the Nth distinct epoch");
}

// ── (13) LiDAR divergence counts fresh Javad epochs too, not just fresh LiDAR pairs ──────────────

ZTEST(StateEstimator_tests, test_lidar_divergence_streak_ignores_held_javad_readings)
{
    StateEstimator::reset();
    StateEstimator::set_now_ns_for_testing(0);

    // imu.has_arrival_time_ns stays false, so R_WB holds its identity default and
    // calculateVerticalAltitude() reduces to altitude == distance_m (LIDAR_MOUNT_ANGLE_DEG is 0).
    ImuReading imu = ImuReading_init_default;
    ImuReading backup_1 = ImuReading_init_default;
    ImuReading backup_2 = ImuReading_init_default;

    GnssReadings gnss = GnssReadings_init_default;
    gnss.up_m = 10.0f;

    LidarReading lidar_1 = LidarReading_init_default;
    lidar_1.distance_m = 10.0f;  // agrees with Javad throughout
    LidarReading lidar_2 = LidarReading_init_default;
    lidar_2.distance_m = 10.0f;  // starts in agreement, diverges below

    int n = StateEstimator::lidar_divergence_streak_threshold_for_testing();
    uint64_t t_ns = 0;
    float lidar_seq = 0.0f;

    // Baseline tick: both LiDARs fresh and agreeing, Javad fresh -- establishes the stamp.
    t_ns += PERIOD_NS;
    StateEstimator::set_now_ns_for_testing(t_ns);
    lidar_seq += 1.0f;
    lidar_1.sense_time_ns = lidar_seq;
    lidar_2.sense_time_ns = lidar_seq;
    gnss.has_arrival_time_ns = true;
    gnss.arrival_time_ns = t_ns;
    StateEstimator::estimate(lidar_1, lidar_2, imu, backup_1, backup_2, gnss);

    const uint64_t held_gnss_time_ns = t_ns;

    // Phase 1: LiDAR 2 diverges and the LiDAR pair refreshes every tick -- the half of the 2-of-3
    // comparison that was already fresh-gated. Javad is HELD, so one Javad epoch would otherwise
    // supply the Javad half of all N counts.
    lidar_2.distance_m = 3.0f;
    for (int i = 0; i < n * 3; i++) {
        t_ns += PERIOD_NS;
        StateEstimator::set_now_ns_for_testing(t_ns);
        lidar_seq += 1.0f;
        lidar_1.sense_time_ns = lidar_seq;
        lidar_2.sense_time_ns = lidar_seq;
        gnss.arrival_time_ns = held_gnss_time_ns;
        StateEstimator::estimate(lidar_1, lidar_2, imu, backup_1, backup_2, gnss);
    }
    zassert_false(
        StateEstimator::lidar_2_is_faulty_for_testing(),
        "a held Javad epoch must not advance the LiDAR divergence streak, however many fresh LiDAR pairs arrive"
    );

    // Phase 2: every participant fresh each tick -- exactly N to flag.
    for (int i = 0; i < n - 1; i++) {
        t_ns += PERIOD_NS;
        StateEstimator::set_now_ns_for_testing(t_ns);
        lidar_seq += 1.0f;
        lidar_1.sense_time_ns = lidar_seq;
        lidar_2.sense_time_ns = lidar_seq;
        gnss.arrival_time_ns = t_ns;
        StateEstimator::estimate(lidar_1, lidar_2, imu, backup_1, backup_2, gnss);
        zassert_false(StateEstimator::lidar_2_is_faulty_for_testing(), "Lidar 2 should not be flagged before N distinct Javad epochs");
    }

    t_ns += PERIOD_NS;
    StateEstimator::set_now_ns_for_testing(t_ns);
    lidar_seq += 1.0f;
    lidar_1.sense_time_ns = lidar_seq;
    lidar_2.sense_time_ns = lidar_seq;
    gnss.arrival_time_ns = t_ns;
    StateEstimator::estimate(lidar_1, lidar_2, imu, backup_1, backup_2, gnss);
    zassert_true(StateEstimator::lidar_2_is_faulty_for_testing(), "Lidar 2 should be flagged on exactly the Nth distinct Javad epoch");
    zassert_false(StateEstimator::lidar_1_is_faulty_for_testing(), "Lidar 1 should remain healthy -- it agrees with Javad throughout");
}

// ── (14) Vertical predict stops on a faulted VN300, but uncertainty keeps growing ───────────────

ZTEST(StateEstimator_tests, test_vertical_predict_stops_while_vn300_faulty)
{
    StateEstimator::reset();
    StateEstimator::set_now_ns_for_testing(0);

    LidarReading lidar_1 = LidarReading_init_default;
    LidarReading lidar_2 = LidarReading_init_default;

    ImuReading imu = ImuReading_init_default;
    imu.quat_w = 1.0f;                     // identity, so the body->world rotation is a no-op
    imu.accel_z = GRAVITY_M_S2 + 4.0f;     // net +4 m/s^2 up, so z climbs visibly while healthy

    // The backups agree with each other and not with VN300, which is what flags VN300 by
    // divergence -- readings keep arriving, so the accelerometer stays live-but-untrusted, the
    // case this gate exists for. Their sense_time_ns is deliberately never advanced, so
    // updateAttitude() never switches R_WB to them and the accel rotation stays identity.
    ImuReading backup_1 = ImuReading_init_default;
    backup_1.quat_w = 0.0f;
    backup_1.quat_x = 1.0f;
    ImuReading backup_2 = ImuReading_init_default;
    backup_2.quat_w = 0.0f;
    backup_2.quat_x = 1.0f;

    GnssReadings gnss = GnssReadings_init_default;
    gnss.up_m = 0.0f;
    gnss.vz_ms = 0.0f;

    uint64_t t_ns = 0;
    warm_vn300_to_healthy(t_ns, lidar_1, lidar_2, imu, backup_1, backup_2, gnss);
    zassert_true(StateEstimator::vn300_is_healthy_for_testing(), "VN300 should be HEALTHY after warmup");

    // Bootstrap the EKF from a single GNSS reading. GNSS is never refreshed again below, so its
    // dedup keeps it out and predict() is the only thing that can move z.
    t_ns += PERIOD_NS;
    StateEstimator::set_now_ns_for_testing(t_ns);
    imu.arrival_time_ns = t_ns;
    imu.sense_time_ns += 1.0f;
    gnss.has_arrival_time_ns = true;
    gnss.arrival_time_ns = t_ns;
    gnss.sense_time_ns += 1.0f;
    StateEstimator::estimate(lidar_1, lidar_2, imu, backup_1, backup_2, gnss);

    // Phase A: healthy VN300 -- predict() runs and z climbs off the accelerometer.
    std::optional<EstimatedState> result;
    for (int i = 0; i < 40; i++) {
        t_ns += PERIOD_NS;
        StateEstimator::set_now_ns_for_testing(t_ns);
        imu.arrival_time_ns = t_ns;
        imu.sense_time_ns += 1.0f;
        result = StateEstimator::estimate(lidar_1, lidar_2, imu, backup_1, backup_2, gnss);
    }
    zassert_true(result.has_value());
    zassert_true(
        result->position.z > 0.005f,
        "z should be climbing off the accelerometer while VN300 is HEALTHY -- otherwise the phase below proves nothing"
    );

    // Phase B: flag VN300 FAULTY by divergence, with its readings still arriving every tick.
    backup_1.has_arrival_time_ns = true;
    backup_2.has_arrival_time_ns = true;
    int n = StateEstimator::vn300_divergence_streak_threshold_for_testing();
    for (int i = 0; i < n; i++) {
        t_ns += PERIOD_NS;
        StateEstimator::set_now_ns_for_testing(t_ns);
        imu.arrival_time_ns = t_ns;
        imu.sense_time_ns += 1.0f;
        backup_1.arrival_time_ns = t_ns;
        backup_2.arrival_time_ns = t_ns;
        result = StateEstimator::estimate(lidar_1, lidar_2, imu, backup_1, backup_2, gnss);
    }
    zassert_true(StateEstimator::vn300_is_faulty_for_testing(), "VN300 should be FAULTY by divergence, with readings still arriving");
    zassert_true(result.has_value());

    const float z_at_fault = result->position.z;
    const float vz_at_fault = result->velocity.z;
    const float variance_at_fault = StateEstimator::ekf_z_variance_for_testing();

    // Phase C: same fresh accelerometer, now from a sensor we have declared faulty. The state must
    // stop moving, and the reported uncertainty must not stop growing.
    for (int i = 0; i < 40; i++) {
        t_ns += PERIOD_NS;
        StateEstimator::set_now_ns_for_testing(t_ns);
        imu.arrival_time_ns = t_ns;
        imu.sense_time_ns += 1.0f;
        result = StateEstimator::estimate(lidar_1, lidar_2, imu, backup_1, backup_2, gnss);
    }

    zassert_true(result.has_value());
    zassert_within(
        result->position.z, z_at_fault, 1e-6f,
        "z must stop advancing once VN300 is FAULTY -- predict() must not integrate a faulted accelerometer"
    );
    zassert_within(result->velocity.z, vz_at_fault, 1e-6f, "vz must stop advancing once VN300 is FAULTY");
    zassert_true(
        StateEstimator::ekf_z_variance_for_testing() > variance_at_fault,
        "altitude variance must keep growing while the estimate is held, so consumers see uncertainty rising rather than false confidence"
    );
}

// ══ Z-axis EKF integration tests (wiring into estimate()) ═══════════════════════════════════════

// (1) Constant-velocity ascent, all sensors healthy -- z/vz should track truth smoothly.

ZTEST(StateEstimator_tests, test_ekf_constant_velocity_ascent_all_sensors_healthy_tracks_truth)
{
    StateEstimator::reset();
    StateEstimator::set_now_ns_for_testing(0);

    ImuReading backup_1 = ImuReading_init_default;
    ImuReading backup_2 = ImuReading_init_default;

    ImuReading imu = ImuReading_init_default;
    imu.quat_w = 1.0f;              // identity attitude -- body frame == world frame
    imu.accel_z = GRAVITY_M_S2;     // net-zero vertical acceleration -> constant velocity

    GnssReadings gnss = GnssReadings_init_default;
    LidarReading lidar_1 = LidarReading_init_default;
    LidarReading lidar_2 = LidarReading_init_default;

    const float vz_true = 2.0f;  // m/s, constant ascent rate
    uint64_t t_ns = 0;
    std::optional<EstimatedState> result;

    for (int i = 0; i < 200; i++) {  // 0.5s simulated
        t_ns += PERIOD_NS;
        StateEstimator::set_now_ns_for_testing(t_ns);

        imu.arrival_time_ns = t_ns;
        imu.has_arrival_time_ns = true;
        imu.sense_time_ns = static_cast<float>(i + 1);

        float t_s = static_cast<float>(t_ns) / 1e9f;
        float z_true = vz_true * t_s;

        gnss.up_m = z_true;
        gnss.vz_ms = vz_true;
        gnss.vrms_m = 0.02f;
        gnss.vvel_rms_ms = 0.02f;
        gnss.has_arrival_time_ns = true;
        gnss.arrival_time_ns = t_ns;
        gnss.sense_time_ns = static_cast<float>(i + 1);

        lidar_1.distance_m = z_true;
        lidar_1.sense_time_ns = static_cast<float>(i + 1);
        lidar_2.distance_m = z_true;
        lidar_2.sense_time_ns = static_cast<float>(i + 1);

        result = StateEstimator::estimate(lidar_1, lidar_2, imu, backup_1, backup_2, gnss);
    }

    zassert_true(result.has_value(), "estimate() should return a value");
    zassert_equal(result->fault_status, EstimatorFaultStatus_NOMINAL, "should be NOMINAL with all sensors healthy and agreeing");

    float final_t_s = static_cast<float>(t_ns) / 1e9f;
    float expected_z = vz_true * final_t_s;

    // All sensors agree with the truth trajectory throughout, so tracking should be tight --
    // verified numerically (see this task's dev notes) that this converges to ~exact tracking,
    // not just "in the right ballpark". 0.05 leaves comfortable margin above float noise while
    // still being tight enough to catch a genuinely broken fusion.
    zassert_within(result->position.z, expected_z, 0.05f, "z should smoothly track the true constant-velocity ascent");
    zassert_within(result->velocity.z, vz_true, 0.05f, "vz should smoothly track the true constant ascent rate");
}

// (2) Flagging a LiDAR via divergence stops it from influencing the EKF -- a wild reading from
// the now-excluded LiDAR should barely move z.

ZTEST(StateEstimator_tests, test_flagged_lidar_wild_reading_does_not_influence_ekf)
{
    StateEstimator::reset();
    StateEstimator::set_now_ns_for_testing(0);

    // Same setup as test_lidar_cross_check_flags_diverging_lidar (reusing its exact pattern):
    // imu.has_arrival_time_ns left false throughout, so R_WB stays identity and
    // calculateVerticalAltitude()'s math is trivial (LIDAR_MOUNT_ANGLE_DEG is 0 -- altitude ==
    // distance_m exactly).
    ImuReading imu = ImuReading_init_default;
    ImuReading backup_1 = ImuReading_init_default;
    ImuReading backup_2 = ImuReading_init_default;

    GnssReadings gnss = GnssReadings_init_default;
    gnss.up_m = 10.0f;

    LidarReading lidar_1 = LidarReading_init_default;
    lidar_1.distance_m = 10.0f;  // agrees with Javad and (initially) with lidar_2
    LidarReading lidar_2 = LidarReading_init_default;
    lidar_2.distance_m = 3.0f;  // diverges from both -- will get flagged faulty

    int n = StateEstimator::lidar_divergence_streak_threshold_for_testing();
    uint64_t t_ns = 0;
    std::optional<EstimatedState> result;

    // Drive lidar_2 to FAULTY. gnss.up_m staying constant at 10.0 also bootstraps/anchors the EKF
    // there in the same loop -- gnss.vrms_m defaults to 0, floored to a tiny r_variance, so the
    // EKF locks onto z~=10 tightly (verified numerically).
    for (int i = 0; i < n; i++) {
        t_ns += PERIOD_NS;
        StateEstimator::set_now_ns_for_testing(t_ns);
        lidar_1.sense_time_ns = static_cast<float>(i + 1);
        lidar_2.sense_time_ns = static_cast<float>(i + 1);
        gnss.sense_time_ns = static_cast<float>(i + 1);
        gnss.has_arrival_time_ns = true;
        gnss.arrival_time_ns = t_ns;
        result = StateEstimator::estimate(lidar_1, lidar_2, imu, backup_1, backup_2, gnss);
    }

    zassert_true(StateEstimator::lidar_2_is_faulty_for_testing(), "Lidar 2 should be flagged faulty (setup precondition)");
    zassert_true(result.has_value());
    float z_before = result->position.z;
    zassert_within(z_before, 10.0f, 0.5f, "EKF should have converged near gnss.up_m=10 by now");

    // Inject a wildly wrong reading from the now-FLAGGED lidar_2. If the health gate added this
    // task is working, update_altitude() is never called for it, so this should have essentially
    // zero effect on the EKF's z.
    lidar_2.distance_m = 99999.0f;
    lidar_2.sense_time_ns = static_cast<float>(n + 1);
    gnss.sense_time_ns = static_cast<float>(n + 1);
    t_ns += PERIOD_NS;
    gnss.arrival_time_ns = t_ns;
    StateEstimator::set_now_ns_for_testing(t_ns);
    result = StateEstimator::estimate(lidar_1, lidar_2, imu, backup_1, backup_2, gnss);

    zassert_true(result.has_value());
    zassert_within(result->position.z, z_before, 0.5f, "z should barely move despite the wild reading from the flagged (excluded) lidar_2");
}

// (3) Javad FAULTY switches velocity updates to the VN300 vel_d fallback path -- verify the NED
// sign convention (vz_world = -vel_d) with an asymmetric, clearly-nonzero test value.

ZTEST(StateEstimator_tests, test_javad_faulty_switches_velocity_source_to_vn300_vel_d_with_correct_sign)
{
    StateEstimator::reset();
    StateEstimator::set_now_ns_for_testing(0);

    LidarReading lidar_1 = LidarReading_init_default;
    LidarReading lidar_2 = LidarReading_init_default;

    ImuReading imu = ImuReading_init_default;
    imu.quat_w = 1.0f;
    imu.has_vel_n = true;
    imu.has_vel_e = true;
    imu.has_vel_d = true;
    imu.vel_n = 0.0f;
    imu.vel_e = 0.0f;
    imu.vel_d = 0.0f;

    ImuReading backup_1 = ImuReading_init_default;
    backup_1.quat_w = 1.0f;
    ImuReading backup_2 = ImuReading_init_default;
    backup_2.quat_w = 1.0f;

    GnssReadings gnss = GnssReadings_init_default;
    gnss.vx_ms = 0.0f;
    gnss.vy_ms = 0.0f;
    gnss.vz_ms = 0.0f;
    gnss.up_m = 5.0f;
    // Realistic (non-floored) reported uncertainty during warmup, not the tiny default-0 floor --
    // verified numerically that flooring here would make the filter so confident that the later
    // vel_d fallback value gets rejected by the innovation gate instead of applied.
    gnss.vrms_m = 0.1f;
    gnss.vvel_rms_ms = 0.1f;

    uint64_t t_ns = 0;

    // Warm VN300 to HEALTHY -- also bootstraps/anchors the EKF via GNSS along the way, since
    // gnss.has_arrival_time_ns/sense_time_ns get set inside the flagging loop below (warm_vn300_
    // to_healthy() itself doesn't touch gnss).
    warm_vn300_to_healthy(t_ns, lidar_1, lidar_2, imu, backup_1, backup_2, gnss);
    zassert_true(StateEstimator::vn300_is_healthy_for_testing(), "VN300 should be healthy after warmup");

    // Flag Javad via velocity divergence (VN300 vel_* stay 0 throughout; gnss.vx_ms diverges).
    gnss.vx_ms = 100.0f;
    int javad_n = StateEstimator::javad_divergence_streak_threshold_for_testing();
    std::optional<EstimatedState> result;
    for (int i = 0; i < javad_n; i++) {
        t_ns += PERIOD_NS;
        StateEstimator::set_now_ns_for_testing(t_ns);
        imu.arrival_time_ns = t_ns;
        imu.sense_time_ns += 1.0f;
        gnss.has_arrival_time_ns = true;
        gnss.arrival_time_ns = t_ns;
        gnss.sense_time_ns += 1.0f;
        result = StateEstimator::estimate(lidar_1, lidar_2, imu, backup_1, backup_2, gnss);
    }
    zassert_true(StateEstimator::javad_is_faulty_for_testing(), "Javad should now be flagged faulty");
    zassert_true(result.has_value());
    float vz_before = result->velocity.z;

    // Asymmetric, clearly-nonzero vel_d: a wrong sign would move vz the WRONG way. 1.5 (not a
    // larger/rounder number) chosen because it's within the innovation gate given how confident
    // the filter already is after the warmup above -- verified numerically.
    imu.vel_d = 1.5f;  // NED down-positive: descending -> world vz should become MORE NEGATIVE
    imu.arrival_time_ns = t_ns + PERIOD_NS;
    imu.sense_time_ns += 1.0f;
    t_ns += PERIOD_NS;
    StateEstimator::set_now_ns_for_testing(t_ns);
    result = StateEstimator::estimate(lidar_1, lidar_2, imu, backup_1, backup_2, gnss);

    zassert_true(result.has_value());
    zassert_true(
        result->velocity.z < vz_before, "vz should move toward -vel_d (down/negative), not +vel_d, confirming the NED sign flip"
    );
}

// (4) EKF initializes from the first GNSS reading and (until then) leaves position.z/velocity.z
// at their prior/default value -- then distinguishes filtered EKF output from raw passthrough.

ZTEST(StateEstimator_tests, test_ekf_initializes_from_first_gnss_reading_and_then_filters_not_passes_through)
{
    StateEstimator::reset();
    StateEstimator::set_now_ns_for_testing(0);

    LidarReading lidar_1 = LidarReading_init_default;
    LidarReading lidar_2 = LidarReading_init_default;
    ImuReading imu = ImuReading_init_default;  // no VN300 predict cycles -- isolate GNSS/EKF init behavior
    ImuReading backup_1 = ImuReading_init_default;
    ImuReading backup_2 = ImuReading_init_default;
    GnssReadings gnss = GnssReadings_init_default;

    // Before any GNSS reading has ever arrived, position.z/velocity.z should stay at their
    // EstimatedState_init_default value (0) -- there's nothing yet to init the EKF from.
    auto result0 = StateEstimator::estimate(lidar_1, lidar_2, imu, backup_1, backup_2, gnss);
    zassert_true(result0.has_value());
    zassert_within(result0->position.z, 0.0f, 1e-6f, "position.z should stay at default before any GNSS reading arrives");
    zassert_within(result0->velocity.z, 0.0f, 1e-6f, "velocity.z should stay at default before any GNSS reading arrives");

    // First GNSS reading -- should initialize the EKF exactly from z0=up_m, vz0=vz_ms.
    gnss.up_m = 7.5f;
    gnss.vz_ms = 1.2f;
    gnss.has_arrival_time_ns = true;
    gnss.arrival_time_ns = 1'000'000;
    gnss.sense_time_ns = 1.0f;

    auto result1 = StateEstimator::estimate(lidar_1, lidar_2, imu, backup_1, backup_2, gnss);
    zassert_true(result1.has_value());
    zassert_within(result1->position.z, 7.5f, 1e-4f, "position.z should equal the first GNSS reading exactly (EKF init)");
    zassert_within(result1->velocity.z, 1.2f, 1e-4f, "velocity.z should equal the first GNSS reading exactly (EKF init)");

    // Second GNSS reading, a very different value. If this were still raw passthrough (the
    // pre-EKF behavior this task replaced), position.z would snap exactly to the new value.
    // Instead it should be a Kalman-filtered partial correction -- pulled toward, but not equal
    // to, the new reading -- proving the EKF is genuinely driving the output now, not a mirror of
    // gnss.up_m.
    gnss.up_m = 50.0f;
    gnss.vrms_m = 0.1f;  // nonzero, non-floor r_variance so the pull is a real partial correction
    gnss.arrival_time_ns = 2'000'000;
    gnss.sense_time_ns = 2.0f;

    auto result2 = StateEstimator::estimate(lidar_1, lidar_2, imu, backup_1, backup_2, gnss);
    zassert_true(result2.has_value());
    zassert_true(result2->position.z > 7.5f, "position.z should move toward the new reading...");
    zassert_true(result2->position.z < 50.0f, "...but not snap exactly to it -- this is filtering, not passthrough");
}

// (5) Accel dropout (no fresh VN300 readings) followed by resume must not produce a garbage
// spike. A dropout this long escalates VN300 to FAULTY, so the state is held deliberately until
// re-trust completes -- the dt clamp is not what protects us here, and is exercised on its own in
// the moderate-dropout test that follows.
//
// Updated for staleness escalation (67b92be): before it, a 500ms dropout left VN300 merely STALE
// and predict resumed on the very next reading. This test asserted that, and now asserts the
// escalate -> hold -> re-trust -> resume sequence that replaced it.

ZTEST(StateEstimator_tests, test_accel_dropout_then_resume_does_not_produce_garbage_spike)
{
    StateEstimator::reset();
    StateEstimator::set_now_ns_for_testing(0);

    LidarReading lidar_1 = LidarReading_init_default;
    LidarReading lidar_2 = LidarReading_init_default;
    ImuReading backup_1 = ImuReading_init_default;
    ImuReading backup_2 = ImuReading_init_default;

    ImuReading imu = ImuReading_init_default;
    imu.quat_w = 1.0f;
    // Nonzero net acceleration on purpose (not GRAVITY_M_S2 alone): if the dt clamp failed and a
    // huge dt were integrated, a=0 would barely move vz/z at all (v*huge_dt with v~0 is still
    // ~0), making a broken clamp look identical to a working one. a=1.0 makes a real spike
    // obvious if the clamp doesn't reject it -- verified numerically.
    imu.accel_z = GRAVITY_M_S2 + 1.0f;

    GnssReadings gnss = GnssReadings_init_default;
    gnss.up_m = 10.0f;
    gnss.vz_ms = 0.0f;

    uint64_t t_ns = 0;
    std::optional<EstimatedState> result;

    // Bootstrap the EKF via GNSS, then run a short, healthy predict/update sequence.
    for (int i = 0; i < 20; i++) {
        t_ns += PERIOD_NS;
        StateEstimator::set_now_ns_for_testing(t_ns);
        imu.arrival_time_ns = t_ns;
        imu.has_arrival_time_ns = true;
        imu.sense_time_ns = static_cast<float>(i + 1);
        gnss.has_arrival_time_ns = true;
        gnss.arrival_time_ns = t_ns;
        gnss.sense_time_ns = static_cast<float>(i + 1);
        result = StateEstimator::estimate(lidar_1, lidar_2, imu, backup_1, backup_2, gnss);
    }
    zassert_true(result.has_value());
    float z_before_dropout = result->position.z;
    float vz_before_dropout = result->velocity.z;

    // Simulate a long VN300 dropout: no fresh imu readings (has_arrival_time_ns left false,
    // matching how Controller.cpp zero-inits DataPacket.imu each tick when VectornavImu::read()
    // returns nullopt). GNSS/lidar also stay stale (their own sense_time_ns dedup already
    // matches gnss_update_timestamp_ns/lidar_*_update_timestamp_ns from the loop above), so
    // NOTHING should update the EKF's state during this tick at all.
    imu.has_arrival_time_ns = false;
    t_ns += 500'000'000;  // +500ms gap, 10x MAX_PREDICT_DT_S
    StateEstimator::set_now_ns_for_testing(t_ns);
    result = StateEstimator::estimate(lidar_1, lidar_2, imu, backup_1, backup_2, gnss);
    zassert_true(result.has_value());
    zassert_within(result->position.z, z_before_dropout, 1e-6f, "z should be untouched during the dropout tick itself");

    // Resume: a fresh VN300 reading arrives, but dt since the last one spans the whole 500ms gap.
    // GNSS/lidar are still stale this tick too, so the vertical predict step is the ONLY thing
    // that could change the EKF here -- and VN300 is FAULTY by now, so that step propagates
    // covariance only and leaves z/vz untouched. (The dt clamp would reject this dt as well; the
    // moderate-dropout test below is where the clamp is what does the rejecting.)
    imu.arrival_time_ns = t_ns;
    imu.has_arrival_time_ns = true;
    imu.sense_time_ns += 1.0f;
    result = StateEstimator::estimate(lidar_1, lidar_2, imu, backup_1, backup_2, gnss);

    zassert_true(result.has_value());
    zassert_within(result->position.z, z_before_dropout, 1e-6f, "z should not spike on resume -- a FAULTY VN300 must not integrate the gap");
    zassert_within(result->velocity.z, vz_before_dropout, 1e-6f, "vz should not spike on resume -- a FAULTY VN300 must not integrate the gap");

    // A gap this long escalated VN300 to staleness-induced FAULTY, so recovery is deliberately
    // not immediate: a FAULTY VN300 coasts on covariance rather than integrating an accelerometer
    // it does not trust yet.
    zassert_true(StateEstimator::vn300_is_faulty_for_testing(), "a 500ms dropout should escalate VN300 to FAULTY, not leave it merely STALE");

    // Part-way through the re-convergence window: data is flowing again, but VN300 is not trusted
    // yet, so the state must stay held.
    for (int i = 0; i < 10; i++) {
        t_ns += PERIOD_NS;
        StateEstimator::set_now_ns_for_testing(t_ns);
        imu.arrival_time_ns = t_ns;
        imu.sense_time_ns += 1.0f;
        result = StateEstimator::estimate(lidar_1, lidar_2, imu, backup_1, backup_2, gnss);
    }
    zassert_true(result.has_value());
    zassert_true(StateEstimator::vn300_is_faulty_for_testing(), "VN300 should still be FAULTY part-way through the re-trust window");
    zassert_within(result->velocity.z, vz_before_dropout, 1e-6f, "vz must stay held while VN300 is still FAULTY -- predict must not resume early");

    // Finish the 100ms re-trust window: the resume reading above started it, 40 more complete it.
    for (int i = 0; i < 30; i++) {
        t_ns += PERIOD_NS;
        StateEstimator::set_now_ns_for_testing(t_ns);
        imu.arrival_time_ns = t_ns;
        imu.sense_time_ns += 1.0f;
        result = StateEstimator::estimate(lidar_1, lidar_2, imu, backup_1, backup_2, gnss);
    }
    zassert_true(StateEstimator::vn300_is_healthy_for_testing(), "VN300 should be HEALTHY again after a full 100ms of continuous good data");

    // Only now should predict resume -- proving the dropout caused a bounded hold, not a
    // permanently stuck filter.
    t_ns += PERIOD_NS;
    StateEstimator::set_now_ns_for_testing(t_ns);
    imu.arrival_time_ns = t_ns;
    imu.sense_time_ns += 1.0f;
    result = StateEstimator::estimate(lidar_1, lidar_2, imu, backup_1, backup_2, gnss);

    zassert_true(result.has_value());
    zassert_true(result->velocity.z > vz_before_dropout, "predict should resume updating vz once VN300 is trusted again");
}

// (5b) The other dropout regime: long enough that the dt clamp must reject the resume step, short
// enough that VN300 stays STALE rather than escalating to FAULTY. This is where predict()'s clamp
// is actually load-bearing -- in test (5) above the FAULTY gate holds the state regardless of dt,
// so the clamp could be removed there without the assertions noticing.

ZTEST(StateEstimator_tests, test_moderate_dropout_rejects_oversized_dt_without_escalating_to_faulty)
{
    StateEstimator::reset();
    StateEstimator::set_now_ns_for_testing(0);

    LidarReading lidar_1 = LidarReading_init_default;
    LidarReading lidar_2 = LidarReading_init_default;
    ImuReading backup_1 = ImuReading_init_default;
    ImuReading backup_2 = ImuReading_init_default;

    ImuReading imu = ImuReading_init_default;
    imu.quat_w = 1.0f;
    // Same reasoning as test (5): a nonzero net acceleration is what makes a broken clamp visible.
    imu.accel_z = GRAVITY_M_S2 + 1.0f;

    GnssReadings gnss = GnssReadings_init_default;
    gnss.up_m = 10.0f;
    gnss.vz_ms = 0.0f;

    uint64_t t_ns = 0;
    std::optional<EstimatedState> result;

    for (int i = 0; i < 20; i++) {
        t_ns += PERIOD_NS;
        StateEstimator::set_now_ns_for_testing(t_ns);
        imu.arrival_time_ns = t_ns;
        imu.has_arrival_time_ns = true;
        imu.sense_time_ns = static_cast<float>(i + 1);
        gnss.has_arrival_time_ns = true;
        gnss.arrival_time_ns = t_ns;
        gnss.sense_time_ns = static_cast<float>(i + 1);
        result = StateEstimator::estimate(lidar_1, lidar_2, imu, backup_1, backup_2, gnss);
    }
    zassert_true(result.has_value());
    float z_before_dropout = result->position.z;
    float vz_before_dropout = result->velocity.z;

    // 100ms without a reading: past the 50ms stale threshold and past MAX_PREDICT_DT_S, but short
    // of the 150ms fault threshold.
    imu.has_arrival_time_ns = false;
    t_ns += 100'000'000;
    StateEstimator::set_now_ns_for_testing(t_ns);
    result = StateEstimator::estimate(lidar_1, lidar_2, imu, backup_1, backup_2, gnss);
    zassert_true(result.has_value());
    zassert_true(StateEstimator::vn300_is_stale_for_testing(), "a 100ms dropout should leave VN300 STALE");
    zassert_false(StateEstimator::vn300_is_faulty_for_testing(), "100ms is short of the fault threshold -- this must not escalate");

    // Resume. VN300 is STALE, not FAULTY, so predict() genuinely runs -- and the only thing
    // standing between a 100ms dt and a garbage spike is the clamp rejecting it.
    imu.arrival_time_ns = t_ns;
    imu.has_arrival_time_ns = true;
    imu.sense_time_ns += 1.0f;
    result = StateEstimator::estimate(lidar_1, lidar_2, imu, backup_1, backup_2, gnss);

    zassert_true(result.has_value());
    zassert_within(result->position.z, z_before_dropout, 1e-6f, "z should not spike on resume -- the dt clamp should reject the oversized predict step");
    zassert_within(result->velocity.z, vz_before_dropout, 1e-6f, "vz should not spike on resume -- the dt clamp should reject the oversized predict step");

    // A normal-dt step right after resumes updating immediately: a STALE VN300 still predicts by
    // design, so unlike the long-dropout case there is no re-trust window to wait out here.
    t_ns += PERIOD_NS;
    StateEstimator::set_now_ns_for_testing(t_ns);
    imu.arrival_time_ns = t_ns;
    imu.sense_time_ns += 1.0f;
    result = StateEstimator::estimate(lidar_1, lidar_2, imu, backup_1, backup_2, gnss);

    zassert_true(result.has_value());
    zassert_true(StateEstimator::vn300_is_stale_for_testing(), "VN300 should still be STALE -- the re-trust window has not elapsed");
    zassert_true(result->velocity.z > vz_before_dropout, "a normal-dt step should resume updating vz -- the rejection was a one-time skip, not a stuck filter");
}

// (6) predict()'s body->world accel rotation, exercised through the REAL StateEstimator path
// with a genuinely tilted (non-identity) attitude -- test 1 above used an identity quaternion, so
// it could not have caught a sign/axis bug in the rotation wiring; this one specifically can.

ZTEST(StateEstimator_tests, test_ekf_predict_rotation_tracks_truth_under_tilted_attitude)
{
    StateEstimator::reset();
    StateEstimator::set_now_ns_for_testing(0);

    LidarReading lidar_1 = LidarReading_init_default;
    LidarReading lidar_2 = LidarReading_init_default;
    ImuReading backup_1 = ImuReading_init_default;
    ImuReading backup_2 = ImuReading_init_default;

    // 45-degree pitch about Y -- same quaternion convention/axis as
    // test_calculate_vertical_altitude_with_vehicle_tilt (half-angle in qy). That test already
    // established, independently of this one, that under this exact attitude a body-frame
    // vector rotates through conjugateQuaternion()+multiplyQuaternionVector() (the SAME call
    // pair predict()'s rotation uses) such that a pure body+Z component maps to
    // world.z = body_z * cos(angle). The body accel below is picked from that established
    // relation, not reverse-engineered from this test's own expected output -- a rotation bug
    // would make this test fail, not pass vacuously.
    const float angle_deg = 45.0f;
    const float angle_rad = angle_deg * PI_F / 180.0f;
    const float half = angle_rad / 2.0f;

    ImuReading imu = ImuReading_init_default;
    imu.quat_w = std::cos(half);
    imu.quat_x = 0.0f;
    imu.quat_y = -std::sin(half);
    imu.quat_z = 0.0f;

    const float a_true = 2.0f;  // m/s^2, true WORLD-frame net vertical acceleration (the target)
    // K is deliberately LARGER than GRAVITY_M_S2 + a_true would be for an untilted vehicle --
    // exactly as a real tilted accelerometer would report (more thrust needed along the tilted
    // body axis to produce the same vertical component). K*cos(45deg) == GRAVITY_M_S2 + a_true.
    const float K = (GRAVITY_M_S2 + a_true) / std::cos(angle_rad);
    imu.accel_x = 0.0f;
    imu.accel_y = 0.0f;
    imu.accel_z = K;

    GnssReadings gnss = GnssReadings_init_default;
    uint64_t t_ns = 0;

    // Warm VN300 to HEALTHY with the tilted quaternion held throughout. This also establishes
    // current_estimate.R_WB = the tilted attitude by the end of warmup (R_WB is set from
    // imu.quat_* on the same tick health flips to HEALTHY). predict() calls during warmup are
    // no-ops regardless (z_axis_ekf isn't init()'d via GNSS yet), so nothing here contaminates
    // the clean measurement below.
    warm_vn300_to_healthy(t_ns, lidar_1, lidar_2, imu, backup_1, backup_2, gnss);
    zassert_true(StateEstimator::vn300_is_healthy_for_testing(), "VN300 should be healthy after warmup");

    // Bootstrap the EKF to a known-clean (0, 0) baseline via a single GNSS reading, with the
    // tilted attitude already active.
    gnss.up_m = 0.0f;
    gnss.vz_ms = 0.0f;
    gnss.has_arrival_time_ns = true;
    t_ns += PERIOD_NS;
    gnss.arrival_time_ns = t_ns;
    gnss.sense_time_ns += 1.0f;
    imu.arrival_time_ns = t_ns;
    imu.sense_time_ns += 1.0f;
    StateEstimator::set_now_ns_for_testing(t_ns);
    auto boot_result = StateEstimator::estimate(lidar_1, lidar_2, imu, backup_1, backup_2, gnss);
    zassert_true(boot_result.has_value());
    zassert_within(boot_result->position.z, 0.0f, 1e-4f, "EKF should bootstrap to exactly z=0");
    zassert_within(boot_result->velocity.z, 0.0f, 1e-4f, "EKF should bootstrap to exactly vz=0");

    // Predict-ONLY ticks from here on (gnss.sense_time_ns/lidar sense_time_ns are never advanced
    // again below, so the dedup gates keep both excluded) -- the tilted attitude and body accel
    // are held constant. If the body->world rotation is wired correctly, the EKF should track the
    // TRUE world-frame constant-acceleration trajectory (a_true=2.0 m/s^2) -- NOT the raw,
    // un-rotated body-frame reading (K, ~16.7 m/s^2 net after subtracting gravity), which is
    // nearly an order of magnitude different and would result from a broken/missing rotation.
    // predict()'s discrete update is exact for piecewise-constant acceleration (see
    // test_constant_acceleration_matches_closed_form_kinematics in ZAxisEkf_test.cpp), so a tight
    // tolerance is appropriate here despite going through quaternion trig -- verified numerically.
    const int n = 100;
    std::optional<EstimatedState> result;
    for (int i = 0; i < n; i++) {
        t_ns += PERIOD_NS;
        StateEstimator::set_now_ns_for_testing(t_ns);
        imu.arrival_time_ns = t_ns;
        imu.sense_time_ns += 1.0f;
        result = StateEstimator::estimate(lidar_1, lidar_2, imu, backup_1, backup_2, gnss);
    }

    zassert_true(result.has_value());
    float t_s = static_cast<float>(n) * static_cast<float>(PERIOD_NS) / 1e9f;
    float expected_z = 0.5f * a_true * t_s * t_s;
    float expected_vz = a_true * t_s;

    zassert_within(
        result->position.z, expected_z, 0.01f,
        "z should track the TRUE world-frame acceleration under a tilted attitude -- proves the body->world rotation is wired correctly, not just present in isolated math_util tests"
    );
    zassert_within(
        result->velocity.z, expected_vz, 0.01f,
        "vz should track the TRUE world-frame acceleration under a tilted attitude -- proves the body->world rotation is wired correctly, not just present in isolated math_util tests"
    );

    // Sanity check that this test is actually discriminating against the most likely real
    // regression (rotation silently skipped/no-op'd): the un-rotated prediction is far outside
    // the tolerance above, so this test can't be passing by accident.
    float wrong_a = K - GRAVITY_M_S2;
    float wrong_vz = wrong_a * t_s;
    zassert_true(
        std::fabs(result->velocity.z - wrong_vz) > 1.0f,
        "sanity check: correct and un-rotated predictions must be clearly distinguishable"
    );
}

ZTEST_SUITE(StateEstimator_tests, NULL, NULL, NULL, NULL, NULL);
