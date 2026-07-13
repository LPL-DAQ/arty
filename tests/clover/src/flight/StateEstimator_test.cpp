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
// spike -- the dt clamp (verified directly in ZAxisEkf_test.cpp) must also work end to end here.

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
    // GNSS/lidar are still stale this tick too, so predict() is the ONLY thing that could change
    // the EKF here -- and it should reject this dt entirely (dt_s > MAX_PREDICT_DT_S), leaving
    // z/vz completely unchanged rather than integrating a huge, bogus step.
    imu.arrival_time_ns = t_ns;
    imu.has_arrival_time_ns = true;
    imu.sense_time_ns += 1.0f;
    result = StateEstimator::estimate(lidar_1, lidar_2, imu, backup_1, backup_2, gnss);

    zassert_true(result.has_value());
    zassert_within(result->position.z, z_before_dropout, 1e-6f, "z should not spike on resume -- the dt clamp should reject the huge-dt predict step");
    zassert_within(result->velocity.z, vz_before_dropout, 1e-6f, "vz should not spike on resume -- the dt clamp should reject the huge-dt predict step");

    // Recovery isn't permanently broken: a normal-dt predict step afterward should resume
    // updating the state as usual (proving the rejection was a one-time skip, not a stuck filter).
    t_ns += PERIOD_NS;
    imu.arrival_time_ns = t_ns;
    imu.sense_time_ns += 1.0f;
    StateEstimator::set_now_ns_for_testing(t_ns);
    result = StateEstimator::estimate(lidar_1, lidar_2, imu, backup_1, backup_2, gnss);

    zassert_true(result.has_value());
    zassert_true(result->velocity.z > vz_before_dropout, "a normal-dt predict step after the dropout should resume updating vz as usual");
}

ZTEST_SUITE(StateEstimator_tests, NULL, NULL, NULL, NULL, NULL);
