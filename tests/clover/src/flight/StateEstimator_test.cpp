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

// Sets Javad's ECEF velocity from a local NED velocity at the launch site, the inverse of the
// rotation the estimator applies to GREIS [VE].
void set_javad_velocity_ned(GnssReadings& gnss, float north, float east, float down)
{
    const float lat = LAUNCH_SITE_LAT_DEG * DEG2RAD_F;
    const float lon = LAUNCH_SITE_LON_DEG * DEG2RAD_F;
    const float up = -down;
    gnss.vx_ms = -std::sin(lon) * east - std::sin(lat) * std::cos(lon) * north + std::cos(lat) * std::cos(lon) * up;
    gnss.vy_ms = std::cos(lon) * east - std::sin(lat) * std::sin(lon) * north + std::cos(lat) * std::sin(lon) * up;
    gnss.vz_ms = std::cos(lat) * north + std::sin(lat) * up;
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
        lidar_1.has_arrival_time_ns = true;
        lidar_1.arrival_time_ns = t_ns;
        lidar_2.sense_time_ns = static_cast<float>(i + 1);
        lidar_2.has_arrival_time_ns = true;
        lidar_2.arrival_time_ns = t_ns;
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
    lidar_1.has_arrival_time_ns = true;
    lidar_1.arrival_time_ns = t_ns;
    lidar_2.sense_time_ns = static_cast<float>(n);
    lidar_2.has_arrival_time_ns = true;
    lidar_2.arrival_time_ns = t_ns;
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
    lidar_1.has_arrival_time_ns = true;
    lidar_1.arrival_time_ns = t_ns;
    lidar_2.sense_time_ns = lidar_seq;
    lidar_2.has_arrival_time_ns = true;
    lidar_2.arrival_time_ns = t_ns;
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
        lidar_1.has_arrival_time_ns = true;
        lidar_1.arrival_time_ns = t_ns;
        lidar_2.sense_time_ns = lidar_seq;
        lidar_2.has_arrival_time_ns = true;
        lidar_2.arrival_time_ns = t_ns;
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
        lidar_1.has_arrival_time_ns = true;
        lidar_1.arrival_time_ns = t_ns;
        lidar_2.sense_time_ns = lidar_seq;
        lidar_2.has_arrival_time_ns = true;
        lidar_2.arrival_time_ns = t_ns;
        gnss.arrival_time_ns = t_ns;
        StateEstimator::estimate(lidar_1, lidar_2, imu, backup_1, backup_2, gnss);
        zassert_false(StateEstimator::lidar_2_is_faulty_for_testing(), "Lidar 2 should not be flagged before N distinct Javad epochs");
    }

    t_ns += PERIOD_NS;
    StateEstimator::set_now_ns_for_testing(t_ns);
    lidar_seq += 1.0f;
    lidar_1.sense_time_ns = lidar_seq;
    lidar_1.has_arrival_time_ns = true;
    lidar_1.arrival_time_ns = t_ns;
    lidar_2.sense_time_ns = lidar_seq;
    lidar_2.has_arrival_time_ns = true;
    lidar_2.arrival_time_ns = t_ns;
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
        set_javad_velocity_ned(gnss, 0.0f, 0.0f, -vz_true);
        gnss.vrms_m = 0.02f;
        gnss.vvel_rms_ms = 0.02f;
        gnss.has_arrival_time_ns = true;
        gnss.arrival_time_ns = t_ns;
        gnss.sense_time_ns = static_cast<float>(i + 1);

        lidar_1.distance_m = z_true;
        lidar_1.sense_time_ns = static_cast<float>(i + 1);
        lidar_1.has_arrival_time_ns = true;
        lidar_1.arrival_time_ns = t_ns;
        lidar_2.distance_m = z_true;
        lidar_2.sense_time_ns = static_cast<float>(i + 1);
        lidar_2.has_arrival_time_ns = true;
        lidar_2.arrival_time_ns = t_ns;

        result = StateEstimator::estimate(lidar_1, lidar_2, imu, backup_1, backup_2, gnss);
    }

    zassert_true(result.has_value(), "estimate() should return a value");
    zassert_equal(result->fault_status, EstimatorFaultStatus_NOMINAL, "should be NOMINAL with all sensors healthy and agreeing");

    float final_t_s = static_cast<float>(t_ns) / 1e9f;
    float expected_z = vz_true * final_t_s;

    // Every sensor agrees with the truth, so tracking should be tight.
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

    // Drive lidar_2 to FAULTY. Javad at 10 m with its floored r_variance holds the EKF near 10 m.
    for (int i = 0; i < n; i++) {
        t_ns += PERIOD_NS;
        StateEstimator::set_now_ns_for_testing(t_ns);
        lidar_1.sense_time_ns = static_cast<float>(i + 1);
        lidar_1.has_arrival_time_ns = true;
        lidar_1.arrival_time_ns = t_ns;
        lidar_2.sense_time_ns = static_cast<float>(i + 1);
        lidar_2.has_arrival_time_ns = true;
        lidar_2.arrival_time_ns = t_ns;
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
    lidar_2.has_arrival_time_ns = true;
    lidar_2.arrival_time_ns = t_ns;
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
    // Non-floored uncertainty, so the filter isn't confident enough to gate out the later vel_d.
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

    // Nonzero vel_d, so a wrong sign moves vz the wrong way. 1.5 stays inside the innovation gate.
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

    // First GNSS reading -- should initialize the EKF exactly from z0=up_m and Javad's up velocity.
    gnss.up_m = 7.5f;
    set_javad_velocity_ned(gnss, 0.0f, 0.0f, -1.2f);
    gnss.has_arrival_time_ns = true;
    gnss.arrival_time_ns = 1'000'000;
    gnss.sense_time_ns = 1.0f;

    auto result1 = StateEstimator::estimate(lidar_1, lidar_2, imu, backup_1, backup_2, gnss);
    zassert_true(result1.has_value());
    zassert_within(result1->position.z, 7.5f, 1e-4f, "position.z should equal the first GNSS reading exactly (EKF init)");
    zassert_within(result1->velocity.z, 1.2f, 1e-4f, "velocity.z should equal the first GNSS reading exactly (EKF init)");

    // A second, different reading should give a partial correction, not a passthrough. With
    // r = 4 against P00 = 4, K = 0.5 and z lands near 9.75. The jump stays inside the gate.
    gnss.up_m = 12.0f;
    gnss.vrms_m = 2.0f;
    gnss.arrival_time_ns = 2'000'000;
    gnss.sense_time_ns = 2.0f;

    auto result2 = StateEstimator::estimate(lidar_1, lidar_2, imu, backup_1, backup_2, gnss);
    zassert_true(result2.has_value());
    zassert_true(result2->position.z > 7.5f, "position.z should move toward the new reading...");
    zassert_true(result2->position.z < 12.0f, "...but not snap exactly to it -- this is filtering, not passthrough");
    zassert_within(result2->position.z, 9.75f, 0.05f, "the correction should be roughly half the gap (K = P/(P+r) = 0.5), not a near-total snap");
}

// (5) An accel dropout followed by resume must not produce a spike. A dropout this long escalates
// VN300 to FAULTY, so the state is held until re-trust completes, then predict resumes. The dt
// clamp is tested separately in the moderate-dropout test below.

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
    // Nonzero net acceleration, so integrating an oversized dt would produce a visible spike.
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
    // predict() is exact for piecewise-constant acceleration, so the tolerance can be tight.
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

// ── Per-sensor staleness ────────────────────────────────────────────────────────────────────

struct StalenessCase {
    const char* name;
    uint64_t threshold_ns;
    bool (*is_stale)();
    void (*arrive)(LidarReading&, LidarReading&, ImuReading&, ImuReading&, GnssReadings&, uint64_t);
};

static void arrive_javad(LidarReading&, LidarReading&, ImuReading&, ImuReading&, GnssReadings& g, uint64_t t)
{
    g.has_arrival_time_ns = true;
    g.arrival_time_ns = t;
}
static void arrive_lidar_1(LidarReading& l1, LidarReading&, ImuReading&, ImuReading&, GnssReadings&, uint64_t t)
{
    l1.has_arrival_time_ns = true;
    l1.arrival_time_ns = t;
}
static void arrive_lidar_2(LidarReading&, LidarReading& l2, ImuReading&, ImuReading&, GnssReadings&, uint64_t t)
{
    l2.has_arrival_time_ns = true;
    l2.arrival_time_ns = t;
}
static void arrive_backup_1(LidarReading&, LidarReading&, ImuReading& b1, ImuReading&, GnssReadings&, uint64_t t)
{
    b1.has_arrival_time_ns = true;
    b1.arrival_time_ns = t;
}
static void arrive_backup_2(LidarReading&, LidarReading&, ImuReading&, ImuReading& b2, GnssReadings&, uint64_t t)
{
    b2.has_arrival_time_ns = true;
    b2.arrival_time_ns = t;
}

static void run_staleness_case(const StalenessCase& c)
{
    StateEstimator::reset();
    LidarReading lidar_1 = LidarReading_init_default;
    LidarReading lidar_2 = LidarReading_init_default;
    ImuReading imu = ImuReading_init_default;
    ImuReading backup_1 = ImuReading_init_default;
    ImuReading backup_2 = ImuReading_init_default;
    GnssReadings gnss = GnssReadings_init_default;

    uint64_t t_ns = 1'000'000'000;
    StateEstimator::set_now_ns_for_testing(t_ns);
    StateEstimator::estimate(lidar_1, lidar_2, imu, backup_1, backup_2, gnss);
    zassert_true(c.is_stale(), "%s: stale before any reading", c.name);

    c.arrive(lidar_1, lidar_2, backup_1, backup_2, gnss, t_ns);
    StateEstimator::estimate(lidar_1, lidar_2, imu, backup_1, backup_2, gnss);
    zassert_false(c.is_stale(), "%s: fresh on first reading", c.name);

    const uint64_t last_arrival_ns = t_ns;
    StateEstimator::set_now_ns_for_testing(last_arrival_ns + c.threshold_ns);
    StateEstimator::estimate(lidar_1, lidar_2, imu, backup_1, backup_2, gnss);
    zassert_false(c.is_stale(), "%s: still fresh at exactly the threshold", c.name);

    StateEstimator::set_now_ns_for_testing(last_arrival_ns + c.threshold_ns + 1'000'000);
    StateEstimator::estimate(lidar_1, lidar_2, imu, backup_1, backup_2, gnss);
    zassert_true(c.is_stale(), "%s: stale 1 ms past the threshold", c.name);

    t_ns = last_arrival_ns + c.threshold_ns + 2'000'000;
    StateEstimator::set_now_ns_for_testing(t_ns);
    c.arrive(lidar_1, lidar_2, backup_1, backup_2, gnss, t_ns);
    StateEstimator::estimate(lidar_1, lidar_2, imu, backup_1, backup_2, gnss);
    zassert_false(c.is_stale(), "%s: fresh again on the next reading", c.name);
}

ZTEST(StateEstimator_tests, test_javad_staleness_after_300ms)
{
    run_staleness_case({"javad", 300'000'000, StateEstimator::javad_is_stale_for_testing, arrive_javad});
}

ZTEST(StateEstimator_tests, test_lidar_staleness_after_100ms)
{
    run_staleness_case({"lidar 1", 100'000'000, StateEstimator::lidar_1_is_stale_for_testing, arrive_lidar_1});
    run_staleness_case({"lidar 2", 100'000'000, StateEstimator::lidar_2_is_stale_for_testing, arrive_lidar_2});
}

ZTEST(StateEstimator_tests, test_backup_imu_staleness_after_50ms)
{
    run_staleness_case({"backup imu 1", 50'000'000, StateEstimator::backup_imu_1_is_stale_for_testing, arrive_backup_1});
    run_staleness_case({"backup imu 2", 50'000'000, StateEstimator::backup_imu_2_is_stale_for_testing, arrive_backup_2});
}

ZTEST(StateEstimator_tests, test_sensors_go_stale_independently)
{
    StateEstimator::reset();
    LidarReading lidar_1 = LidarReading_init_default;
    LidarReading lidar_2 = LidarReading_init_default;
    ImuReading imu = ImuReading_init_default;
    ImuReading backup_1 = ImuReading_init_default;
    ImuReading backup_2 = ImuReading_init_default;
    GnssReadings gnss = GnssReadings_init_default;

    uint64_t t_ns = 1'000'000'000;
    StateEstimator::set_now_ns_for_testing(t_ns);
    arrive_lidar_1(lidar_1, lidar_2, backup_1, backup_2, gnss, t_ns);
    arrive_lidar_2(lidar_1, lidar_2, backup_1, backup_2, gnss, t_ns);
    StateEstimator::estimate(lidar_1, lidar_2, imu, backup_1, backup_2, gnss);

    for (int i = 1; i <= 20; i++) {
        t_ns += 10'000'000;
        StateEstimator::set_now_ns_for_testing(t_ns);
        arrive_lidar_1(lidar_1, lidar_2, backup_1, backup_2, gnss, t_ns);
        StateEstimator::estimate(lidar_1, lidar_2, imu, backup_1, backup_2, gnss);
    }
    zassert_false(StateEstimator::lidar_1_is_stale_for_testing(), "lidar 1 kept reporting");
    zassert_true(StateEstimator::lidar_2_is_stale_for_testing(), "lidar 2 stopped 200 ms ago");
}

ZTEST(StateEstimator_tests, test_stale_javad_degrades_fault_status)
{
    StateEstimator::reset();
    StateEstimator::set_now_ns_for_testing(0);
    LidarReading lidar_1 = LidarReading_init_default;
    LidarReading lidar_2 = LidarReading_init_default;
    ImuReading imu = ImuReading_init_default;
    imu.quat_w = 1.0f;
    ImuReading backup_1 = ImuReading_init_default;
    ImuReading backup_2 = ImuReading_init_default;
    GnssReadings gnss = GnssReadings_init_default;

    uint64_t t_ns = 0;
    warm_vn300_to_healthy(t_ns, lidar_1, lidar_2, imu, backup_1, backup_2, gnss);
    gnss.has_arrival_time_ns = true;
    gnss.arrival_time_ns = t_ns;
    gnss.sense_time_ns = 1.0f;
    auto result = StateEstimator::estimate(lidar_1, lidar_2, imu, backup_1, backup_2, gnss);
    zassert_equal(result->fault_status, EstimatorFaultStatus_NOMINAL, "NOMINAL with VN300 and Javad fresh");

    const uint64_t last_gnss_ns = t_ns;
    while (t_ns <= last_gnss_ns + 300'000'000) {
        t_ns += PERIOD_NS;
        StateEstimator::set_now_ns_for_testing(t_ns);
        imu.arrival_time_ns = t_ns;
        imu.sense_time_ns += 1.0f;
        result = StateEstimator::estimate(lidar_1, lidar_2, imu, backup_1, backup_2, gnss);
    }
    zassert_true(StateEstimator::javad_is_stale_for_testing(), "Javad stale after 300 ms without a reading");
    zassert_false(StateEstimator::javad_is_faulty_for_testing(), "stale is not faulty");
    zassert_equal(result->fault_status, EstimatorFaultStatus_DEGRADED, "DEGRADED while Javad is stale");
}

// ── LiDAR cross-check: per-LiDAR freshness and Javad gating ───────────────────────────────────

struct LidarRig {
    LidarReading lidar_1 = LidarReading_init_default;
    LidarReading lidar_2 = LidarReading_init_default;
    ImuReading imu = ImuReading_init_default;
    ImuReading backup_1 = ImuReading_init_default;
    ImuReading backup_2 = ImuReading_init_default;
    GnssReadings gnss = GnssReadings_init_default;
    uint64_t t_ns = 0;
    float seq = 0.0f;

    void step(bool lidar_1_fresh, bool lidar_2_fresh, bool gnss_fresh)
    {
        t_ns += PERIOD_NS;
        seq += 1.0f;
        StateEstimator::set_now_ns_for_testing(t_ns);
        if (lidar_1_fresh) {
            lidar_1.sense_time_ns = seq;
            lidar_1.has_arrival_time_ns = true;
            lidar_1.arrival_time_ns = t_ns;
        }
        if (lidar_2_fresh) {
            lidar_2.sense_time_ns = seq;
            lidar_2.has_arrival_time_ns = true;
            lidar_2.arrival_time_ns = t_ns;
        }
        if (gnss_fresh) {
            gnss.sense_time_ns = seq;
            gnss.has_arrival_time_ns = true;
            gnss.arrival_time_ns = t_ns;
        }
        if (imu.has_arrival_time_ns) {
            imu.arrival_time_ns = t_ns;
            imu.sense_time_ns = seq;
        }
        StateEstimator::estimate(lidar_1, lidar_2, imu, backup_1, backup_2, gnss);
    }
};

static LidarRig diverging_lidar_2_rig()
{
    StateEstimator::reset();
    StateEstimator::set_now_ns_for_testing(0);
    LidarRig rig;
    rig.gnss.up_m = 10.0f;
    rig.lidar_1.distance_m = 10.0f;
    rig.lidar_2.distance_m = 3.0f;
    return rig;
}

ZTEST(StateEstimator_tests, test_lidar_check_runs_when_lidars_update_on_different_ticks)
{
    LidarRig rig = diverging_lidar_2_rig();
    const int n = StateEstimator::lidar_divergence_streak_threshold_for_testing();

    // Each Javad epoch: LiDAR 1 on one tick, LiDAR 2 on the next, never together.
    for (int epoch = 0; epoch < n; epoch++) {
        zassert_false(StateEstimator::lidar_2_is_faulty_for_testing(), "not flagged before N epochs (epoch %d)", epoch);
        rig.step(true, false, true);
        rig.step(false, true, false);
    }
    zassert_true(StateEstimator::lidar_2_is_faulty_for_testing(), "LiDAR 2 flagged after N epochs without same-tick updates");
    zassert_false(StateEstimator::lidar_1_is_faulty_for_testing(), "LiDAR 1 agrees with Javad");
}

ZTEST(StateEstimator_tests, test_lidar_check_skipped_while_javad_stale)
{
    LidarRig rig = diverging_lidar_2_rig();
    const int n = StateEstimator::lidar_divergence_streak_threshold_for_testing();

    rig.step(true, true, true);
    // Javad stops; 300 ms later it is stale and the LiDARs keep diverging for many more readings.
    for (int i = 0; i < 120 + 4 * n; i++) {
        rig.step(true, true, false);
    }
    zassert_true(StateEstimator::javad_is_stale_for_testing(), "setup: Javad stale");
    zassert_false(StateEstimator::lidar_2_is_faulty_for_testing(), "no LiDAR verdict without a usable Javad");
    zassert_false(StateEstimator::lidar_1_is_faulty_for_testing(), "no LiDAR verdict without a usable Javad");
}

ZTEST(StateEstimator_tests, test_lidar_check_skipped_while_javad_faulty)
{
    LidarRig rig = diverging_lidar_2_rig();
    rig.imu.quat_w = 1.0f;
    rig.imu.has_vel_n = rig.imu.has_vel_e = rig.imu.has_vel_d = true;
    rig.backup_1.quat_w = 1.0f;
    rig.backup_2.quat_w = 1.0f;
    warm_vn300_to_healthy(rig.t_ns, rig.lidar_1, rig.lidar_2, rig.imu, rig.backup_1, rig.backup_2, rig.gnss);
    rig.seq = 100.0f;

    // Flag Javad faulty by velocity divergence, with both LiDARs agreeing with each other meanwhile.
    rig.gnss.vx_ms = 100.0f;
    rig.lidar_2.distance_m = 10.0f;
    for (int i = 0; i < StateEstimator::javad_divergence_streak_threshold_for_testing(); i++) {
        rig.step(true, true, true);
    }
    zassert_true(StateEstimator::javad_is_faulty_for_testing(), "setup: Javad faulty");

    rig.lidar_2.distance_m = 3.0f;
    for (int i = 0; i < 4 * StateEstimator::lidar_divergence_streak_threshold_for_testing(); i++) {
        rig.step(true, true, true);
    }
    zassert_false(StateEstimator::lidar_2_is_faulty_for_testing(), "no LiDAR verdict while Javad is faulty");
}

ZTEST(StateEstimator_tests, test_sensor_health_published_in_estimate)
{
    LidarRig rig = diverging_lidar_2_rig();
    auto result = StateEstimator::estimate(rig.lidar_1, rig.lidar_2, rig.imu, rig.backup_1, rig.backup_2, rig.gnss);
    zassert_equal(result->vn300_health, SensorHealth_STALE, "VN300 STALE before any reading");
    zassert_equal(result->javad_health, SensorHealth_STALE, "Javad STALE before any reading");

    for (int i = 0; i < StateEstimator::lidar_divergence_streak_threshold_for_testing(); i++) {
        rig.step(true, true, true);
    }
    result = StateEstimator::estimate(rig.lidar_1, rig.lidar_2, rig.imu, rig.backup_1, rig.backup_2, rig.gnss);
    zassert_equal(result->javad_health, SensorHealth_HEALTHY, "Javad HEALTHY while fresh");
    zassert_equal(result->lidar_1_health, SensorHealth_HEALTHY, "LiDAR 1 agrees with Javad");
    zassert_equal(result->lidar_2_health, SensorHealth_FAULTY, "LiDAR 2 flagged FAULTY");

    // LiDAR 1 stops reporting: STALE once past its threshold, while LiDAR 2 stays FAULTY.
    for (int i = 0; i < 45; i++) {
        rig.step(false, true, true);
    }
    result = StateEstimator::estimate(rig.lidar_1, rig.lidar_2, rig.imu, rig.backup_1, rig.backup_2, rig.gnss);
    zassert_equal(result->lidar_1_health, SensorHealth_STALE, "LiDAR 1 STALE after 100 ms without a reading");
    zassert_equal(result->lidar_2_health, SensorHealth_FAULTY, "FAULTY takes precedence over STALE");
}

// ── Hold limits: a held channel is marked invalid after 500 ms ───────────────────────────────

static std::optional<EstimatedState> step_all(LidarRig& rig, bool gnss_fresh, bool imu_fresh, bool backup_fresh)
{
    rig.t_ns += PERIOD_NS;
    rig.seq += 1.0f;
    StateEstimator::set_now_ns_for_testing(rig.t_ns);
    if (imu_fresh) {
        rig.imu.has_arrival_time_ns = true;
        rig.imu.arrival_time_ns = rig.t_ns;
        rig.imu.sense_time_ns = rig.seq;
    }
    if (gnss_fresh) {
        rig.gnss.has_arrival_time_ns = true;
        rig.gnss.arrival_time_ns = rig.t_ns;
        rig.gnss.sense_time_ns = rig.seq;
    }
    if (backup_fresh) {
        for (ImuReading* b : {&rig.backup_1, &rig.backup_2}) {
            b->has_arrival_time_ns = true;
            b->arrival_time_ns = rig.t_ns;
            b->sense_time_ns = rig.seq;
        }
    }
    return StateEstimator::estimate(rig.lidar_1, rig.lidar_2, rig.imu, rig.backup_1, rig.backup_2, rig.gnss);
}

static LidarRig healthy_vn300_rig()
{
    StateEstimator::reset();
    StateEstimator::set_now_ns_for_testing(0);
    LidarRig rig;
    rig.imu.quat_w = 1.0f;
    rig.imu.has_vel_n = rig.imu.has_vel_e = rig.imu.has_vel_d = true;
    rig.backup_1.quat_w = 1.0f;
    rig.backup_2.quat_w = 1.0f;
    warm_vn300_to_healthy(rig.t_ns, rig.lidar_1, rig.lidar_2, rig.imu, rig.backup_1, rig.backup_2, rig.gnss);
    rig.seq = 100.0f;
    return rig;
}

ZTEST(StateEstimator_tests, test_lateral_held_from_faulted_javad_is_invalid_after_500ms)
{
    LidarRig rig = healthy_vn300_rig();
    rig.gnss.north_m = 5.0f;
    rig.gnss.east_m = -3.0f;
    auto result = step_all(rig, true, true, false);
    zassert_true(result->lateral_valid, "lateral valid with Javad fresh and healthy");
    zassert_within(result->position.x, 5.0f, 1e-6f, "lateral position taken from Javad");

    rig.gnss.vx_ms = 100.0f;
    while (!StateEstimator::javad_is_faulty_for_testing()) {
        result = step_all(rig, true, true, false);
    }
    const uint64_t fault_ns = rig.t_ns;
    rig.gnss.north_m = 50.0f;

    while (rig.t_ns + PERIOD_NS <= fault_ns + 500'000'000) {
        result = step_all(rig, true, true, false);
        zassert_true(result->lateral_valid, "still valid within 500 ms of the fault (t=%llu)", rig.t_ns - fault_ns);
    }
    zassert_within(result->position.x, 5.0f, 1e-6f, "lateral position held while Javad is faulty");

    result = step_all(rig, true, true, false);
    zassert_false(result->lateral_valid, "invalid once held past 500 ms");
    zassert_equal(result->fault_status, EstimatorFaultStatus_CRITICAL, "CRITICAL once a channel is invalid");
    zassert_true(result->attitude_valid, "attitude unaffected");

    rig.gnss.vx_ms = 0.0f;
    while (StateEstimator::javad_is_faulty_for_testing()) {
        result = step_all(rig, true, true, false);
    }
    zassert_true(result->lateral_valid, "valid again once Javad recovers");
    zassert_not_equal(result->fault_status, EstimatorFaultStatus_CRITICAL, "no longer CRITICAL after recovery");
}

ZTEST(StateEstimator_tests, test_attitude_without_backup_is_invalid_100ms_after_last_vn300_reading)
{
    LidarRig rig = healthy_vn300_rig();
    rig.imu.quat_w = 0.9238795f;
    rig.imu.quat_x = 0.3826834f;
    auto result = step_all(rig, true, true, false);
    zassert_true(result->attitude_valid, "attitude valid with VN300 healthy");
    const uint64_t last_imu_ns = rig.t_ns;

    // VN300 stops and no backup IMU reports. The 100 ms limit includes the 50 ms stale window.
    while (rig.t_ns + PERIOD_NS <= last_imu_ns + 100'000'000) {
        result = step_all(rig, true, false, false);
        zassert_true(result->attitude_valid, "still valid within 100 ms of the last reading (t=%llu)", rig.t_ns - last_imu_ns);
    }
    assert_attitude_is(*result, 0.9238795f, 0.3826834f, 0.0f, 0.0f, "attitude held at last VN300 value");

    result = step_all(rig, true, false, false);
    zassert_false(result->attitude_valid, "invalid once 100 ms have passed with no live source");
    zassert_equal(result->fault_status, EstimatorFaultStatus_CRITICAL, "CRITICAL once a channel is invalid");

    for (int i = 0; i < 50; i++) {
        result = step_all(rig, true, true, false);
    }
    zassert_true(StateEstimator::vn300_is_healthy_for_testing(), "VN300 re-trusted");
    zassert_true(result->attitude_valid, "valid again once VN300 is healthy");
}

ZTEST(StateEstimator_tests, test_attitude_valid_again_once_backup_takes_over)
{
    LidarRig rig = healthy_vn300_rig();
    step_all(rig, true, true, true);
    const uint64_t last_imu_ns = rig.t_ns;
    std::optional<EstimatedState> result;

    // The backup only becomes the attitude source when VN300 escalates to FAULTY at 150 ms, so
    // attitude is invalid between the 100 ms limit and the handoff.
    for (result = step_all(rig, true, false, true); !StateEstimator::vn300_is_faulty_for_testing();
         result = step_all(rig, true, false, true)) {
        const bool within_limit = rig.t_ns - last_imu_ns <= 100'000'000;
        zassert_equal(result->attitude_valid, within_limit, "before handoff, valid only within 100 ms (t=%llu)", rig.t_ns - last_imu_ns);
    }
    for (int i = 0; i < 400; i++) {
        zassert_true(result->attitude_valid, "valid at tick %d: a fresh backup is a live source once VN300 is faulty", i);
        result = step_all(rig, true, false, true);
    }
}

// ── Backup IMU selection once VN300 is faulty ─────────────────────────────────────────────

static std::optional<EstimatedState> step_backups(LidarRig& rig, bool backup_1_fresh, bool backup_2_fresh)
{
    rig.t_ns += PERIOD_NS;
    rig.seq += 1.0f;
    StateEstimator::set_now_ns_for_testing(rig.t_ns);
    if (backup_1_fresh) {
        rig.backup_1.has_arrival_time_ns = true;
        rig.backup_1.arrival_time_ns = rig.t_ns;
        rig.backup_1.sense_time_ns = rig.seq;
    }
    if (backup_2_fresh) {
        rig.backup_2.has_arrival_time_ns = true;
        rig.backup_2.arrival_time_ns = rig.t_ns;
        rig.backup_2.sense_time_ns = rig.seq;
    }
    return StateEstimator::estimate(rig.lidar_1, rig.lidar_2, rig.imu, rig.backup_1, rig.backup_2, rig.gnss);
}

// VN300 stops; steps until it escalates to FAULTY with the given backups reporting.
static void fault_vn300_by_outage(LidarRig& rig, bool backup_1_fresh, bool backup_2_fresh)
{
    while (!StateEstimator::vn300_is_faulty_for_testing()) {
        step_backups(rig, backup_1_fresh, backup_2_fresh);
    }
}

ZTEST(StateEstimator_tests, test_backup_imu_2_takes_attitude_when_backup_1_stale)
{
    LidarRig rig = healthy_vn300_rig();
    rig.backup_2.quat_w = 0.7071f;
    rig.backup_2.quat_z = 0.7071f;
    fault_vn300_by_outage(rig, false, true);

    auto result = step_backups(rig, false, true);
    zassert_true(StateEstimator::backup_imu_1_is_stale_for_testing(), "setup: backup 1 stale");
    assert_attitude_is(*result, 0.7071f, 0.0f, 0.0f, 0.7071f, "attitude from backup IMU 2");
    zassert_true(result->attitude_valid, "backup IMU 2 is a live attitude source");

    for (int i = 0; i < 200; i++) {
        result = step_backups(rig, false, true);
        zassert_true(result->attitude_valid, "stays valid on backup IMU 2 (tick %d)", i);
    }
}

ZTEST(StateEstimator_tests, test_backup_imu_1_preferred_when_both_backups_fresh)
{
    LidarRig rig = healthy_vn300_rig();
    rig.backup_1.quat_w = 0.9238795f;
    rig.backup_1.quat_x = 0.3826834f;
    rig.backup_2.quat_w = 0.7071f;
    rig.backup_2.quat_z = 0.7071f;
    fault_vn300_by_outage(rig, true, true);

    auto result = step_backups(rig, true, true);
    assert_attitude_is(*result, 0.9238795f, 0.3826834f, 0.0f, 0.0f, "attitude from backup IMU 1");

    // Backup 1 stops: backup 2 takes over once backup 1 is stale.
    for (int i = 0; i < 25; i++) {
        result = step_backups(rig, false, true);
    }
    zassert_true(StateEstimator::backup_imu_1_is_stale_for_testing(), "setup: backup 1 stale");
    assert_attitude_is(*result, 0.7071f, 0.0f, 0.0f, 0.7071f, "attitude from backup IMU 2 after backup 1 goes stale");
    zassert_true(result->attitude_valid, "valid on backup IMU 2");
}

// ── Javad velocity frame: GREIS [VE] ECEF velocity is rotated into NED before use ────────────

ZTEST(StateEstimator_tests, test_agreeing_1ms_climb_does_not_flag_javad)
{
    LidarRig rig = healthy_vn300_rig();
    rig.imu.vel_n = 0.5f;
    rig.imu.vel_e = -0.25f;
    rig.imu.vel_d = -1.0f;
    set_javad_velocity_ned(rig.gnss, 0.5f, -0.25f, -1.0f);

    std::optional<EstimatedState> result;
    for (int i = 0; i < 3 * StateEstimator::javad_divergence_streak_threshold_for_testing(); i++) {
        result = step_all(rig, true, true, false);
    }
    zassert_false(StateEstimator::javad_is_faulty_for_testing(), "Javad agrees with VN300 once both are in NED");
    zassert_within(result->velocity.x, 0.5f, 1e-4f, "velocity.x is Javad north");
    zassert_within(result->velocity.y, -0.25f, 1e-4f, "velocity.y is Javad east");
}

ZTEST(StateEstimator_tests, test_opposite_vertical_velocity_flags_javad)
{
    LidarRig rig = healthy_vn300_rig();
    rig.imu.vel_d = 1.0f;
    set_javad_velocity_ned(rig.gnss, 0.0f, 0.0f, -1.0f);

    for (int i = 0; i < StateEstimator::javad_divergence_streak_threshold_for_testing(); i++) {
        zassert_false(StateEstimator::javad_is_faulty_for_testing(), "not flagged before N epochs (epoch %d)", i);
        step_all(rig, true, true, false);
    }
    zassert_true(StateEstimator::javad_is_faulty_for_testing(), "Javad climbing while VN300 descends is flagged");
}

// ── 400 Hz VN300 against the 1.5 ms control tick ──────────────────────────────────────────

struct TickRateCase {
    uint64_t imu_period_ns;
    int ticks_per_delivery;  // the controller sees a new reading at most once every this many ticks
};

// Runs 1 s of 1.5 ms ticks with a constant 1 m/s^2 climb from rest. Readings are stamped at the
// tick that reads them, as VectornavImu::read() does. Returns vz minus a * (last - first arrival).
static float run_tick_rate_case(const TickRateCase& c, int& deliveries)
{
    StateEstimator::reset();
    LidarReading lidar_1 = LidarReading_init_default;
    LidarReading lidar_2 = LidarReading_init_default;
    ImuReading imu = ImuReading_init_default;
    imu.quat_w = 1.0f;
    imu.accel_z = GRAVITY_M_S2 + 1.0f;
    ImuReading backup_1 = ImuReading_init_default;
    ImuReading backup_2 = ImuReading_init_default;
    GnssReadings gnss = GnssReadings_init_default;
    gnss.has_arrival_time_ns = true;
    gnss.arrival_time_ns = 1;
    gnss.sense_time_ns = 1.0f;

    constexpr uint64_t TICK_NS = 1'500'000;
    StateEstimator::set_now_ns_for_testing(1);
    StateEstimator::estimate(lidar_1, lidar_2, imu, backup_1, backup_2, gnss);

    uint64_t produced = 0;
    uint64_t consumed = 0;
    uint64_t first_arrival = 0;
    uint64_t last_arrival = 0;
    deliveries = 0;
    std::optional<EstimatedState> result;
    for (int tick = 1; tick <= 667; tick++) {
        const uint64_t now = tick * TICK_NS;
        StateEstimator::set_now_ns_for_testing(now);
        while ((produced + 1) * c.imu_period_ns <= now) {
            produced++;
        }
        if (produced > consumed && tick % c.ticks_per_delivery == 0) {
            consumed = produced;
            imu.has_arrival_time_ns = true;
            imu.arrival_time_ns = now;
            imu.sense_time_ns = static_cast<float>(consumed);
            if (deliveries++ == 0) {
                first_arrival = now;
            }
            last_arrival = now;
        }
        result = StateEstimator::estimate(lidar_1, lidar_2, imu, backup_1, backup_2, gnss);
    }
    const float expected_vz = static_cast<float>(last_arrival - first_arrival) / 1e9f;
    return result->velocity.z - expected_vz;
}

ZTEST(StateEstimator_tests, test_400hz_imu_on_1500us_ticks_integrates_each_interval_once)
{
    int deliveries = 0;
    const float error = run_tick_rate_case({2'500'000, 1}, deliveries);
    zassert_within(deliveries, 400, 1, "one prediction per 400 Hz reading, got %d", deliveries);
    zassert_within(error, 0.0f, 1e-3f, "vz matches a * elapsed: no double-counted or dropped time (error %f)", static_cast<double>(error));
}

ZTEST(StateEstimator_tests, test_readings_overwritten_between_ticks_keep_integrated_time)
{
    // The controller reads only every third tick, so about half the 400 Hz readings are overwritten
    // before they are seen. The surviving predictions must still span the whole interval.
    int deliveries = 0;
    const float error = run_tick_rate_case({2'500'000, 3}, deliveries);
    zassert_true(deliveries < 300, "setup: readings were dropped (%d delivered)", deliveries);
    zassert_within(error, 0.0f, 1e-3f, "vz matches a * elapsed despite dropped readings (error %f)", static_cast<double>(error));
}

ZTEST_SUITE(StateEstimator_tests, NULL, NULL, NULL, NULL, NULL);
