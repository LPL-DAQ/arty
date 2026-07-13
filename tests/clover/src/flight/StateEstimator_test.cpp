#include "../../../../clover/src/flight/StateEstimator.h"
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
        backup_2.has_arrival_time_ns = true;
        StateEstimator::estimate(lidar_1, lidar_2, imu, backup_1, backup_2, gnss);
        zassert_false(StateEstimator::vn300_is_faulty_for_testing(), "VN300 should not be flagged before N consecutive mismatches");
    }

    t_ns += PERIOD_NS;
    StateEstimator::set_now_ns_for_testing(t_ns);
    imu.arrival_time_ns = t_ns;
    imu.sense_time_ns += 1.0f;
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
        backup_2.has_arrival_time_ns = true;
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
        StateEstimator::estimate(lidar_1, lidar_2, imu, backup_1, backup_2, gnss);
        zassert_true(StateEstimator::vn300_is_faulty_for_testing(), "VN300 should remain flagged before N consecutive agreements");
    }

    t_ns += PERIOD_NS;
    StateEstimator::set_now_ns_for_testing(t_ns);
    imu.arrival_time_ns = t_ns;
    imu.sense_time_ns += 1.0f;
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
        backup_2.has_arrival_time_ns = true;
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

// ── (5) Simultaneous VN300+Javad fault: CRITICAL and holds last known-good ──────────────────────

ZTEST(StateEstimator_tests, test_simultaneous_vn300_and_javad_fault_triggers_critical_and_holds_last_known_good)
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
        backup_2.has_arrival_time_ns = true;
        last_result = StateEstimator::estimate(lidar_1, lidar_2, imu, backup_1, backup_2, gnss);
    }
    zassert_true(StateEstimator::vn300_is_faulty_for_testing(), "VN300 should now be flagged faulty too");
    zassert_true(last_result.has_value(), "estimate() should still return a value under critical fault");
    zassert_equal(last_result->fault_status, EstimatorFaultStatus_CRITICAL, "fault_status should be CRITICAL when both VN300 and Javad are flagged");

    EstimatedState frozen_snapshot = *last_result;

    // Feed yet more, clearly different data -- the estimate must not move.
    for (int i = 0; i < 5; i++) {
        t_ns += PERIOD_NS;
        StateEstimator::set_now_ns_for_testing(t_ns);
        imu.arrival_time_ns = t_ns;
        imu.sense_time_ns += 1.0f;
        imu.quat_x = 0.0f;
        imu.quat_y = 1.0f;  // yet another different orientation
        gnss.vx_ms = -55.0f;
        gnss.arrival_time_ns = t_ns;
        gnss.sense_time_ns += 1.0f;

        auto result = StateEstimator::estimate(lidar_1, lidar_2, imu, backup_1, backup_2, gnss);
        zassert_true(result.has_value(), "estimate() should keep returning a value under critical fault");
        zassert_equal(result->fault_status, EstimatorFaultStatus_CRITICAL, "Should remain CRITICAL");
        zassert_within(result->R_WB.qw, frozen_snapshot.R_WB.qw, 0.0001f, "R_WB.qw should be frozen under critical fault");
        zassert_within(result->R_WB.qx, frozen_snapshot.R_WB.qx, 0.0001f, "R_WB.qx should be frozen under critical fault");
        zassert_within(result->R_WB.qy, frozen_snapshot.R_WB.qy, 0.0001f, "R_WB.qy should be frozen under critical fault");
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

ZTEST_SUITE(StateEstimator_tests, NULL, NULL, NULL, NULL, NULL);
