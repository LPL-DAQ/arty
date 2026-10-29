#include "StateEstimator.h"
#include "Error.h"
#include "ZAxisEkf.h"
#include "../config.h"
#include "../math_util.h"
#include <algorithm>
#include <cmath>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(StateEstimator, LOG_LEVEL_INF);


// Health flags change only after N consecutive readings agree, in either direction. STALE means no
// recent reading; FAULTY means readings arrive but disagree. VN300 is re-trusted on a timer after
// an outage so its filter can re-converge; other sensors are usable again on their next reading.

static EstimatedState current_estimate = EstimatedState_init_default;
static float update_timestamp_ns = 0;
static bool has_update_timestamp_ns = false;
static float lidar_1_update_timestamp_ns = 0;
static float lidar_2_update_timestamp_ns = 0;
static float imu_update_timestamp_ns = 0;
static float backup_imu_1_update_timestamp_ns = 0;
static float backup_imu_2_update_timestamp_ns = 0;
static float gnss_update_timestamp_ns = 0;
static float lidar_1_altitude_m = 0;
static float lidar_2_altitude_m = 0;

// Vertical state (position.z, velocity.z). Predict dt comes from consecutive VN300 arrival times.
static ZAxisEkf z_axis_ekf;
static uint64_t z_axis_ekf_last_predict_time_ns = 0;

// Initial covariance when the EKF starts from the first healthy GNSS reading: 2 m and 2 m/s sigma.
// The bias term is a placeholder.
static constexpr float EKF_INITIAL_Z_VARIANCE_M2 = 4.0f;
static constexpr float EKF_INITIAL_VZ_VARIANCE_M2_S2 = 4.0f;
static constexpr float EKF_INITIAL_BIAS_VARIANCE_M2_S4 = 1.0f;

// r_variance for each LiDAR's computed altitude: 5 cm sigma, the TF02-Pro datasheet accuracy below
// 5 m (1% of range beyond). Confirm against logged data.
static constexpr float LIDAR_1_ALTITUDE_R_VARIANCE_M2 = 0.0025f;
static constexpr float LIDAR_2_ALTITUDE_R_VARIANCE_M2 = 0.0025f;

// Placeholder. Floors on Javad's reported per-epoch uncertainty, so a zero value can't make the
// filter overconfident.
static constexpr float JAVAD_ALTITUDE_R_VARIANCE_FLOOR_M2 = 1e-4f;
static constexpr float JAVAD_VELOCITY_R_VARIANCE_FLOOR_M2_S2 = 1e-4f;

// Placeholder until VN300 INS velocity is configured. Used for the vel_d fallback while Javad is
// unusable.
static constexpr float VN300_VEL_D_VELOCITY_R_VARIANCE_M2_S2 = 0.25f;

// VN300 goes STALE after this long without a reading.
static constexpr uint64_t VN300_STALE_THRESHOLD_NS = 50'000'000;

// Continuous good data required before VN300 is trusted again after an outage, so its onboard
// filter can re-converge.
static constexpr uint64_t VN300_RETRUST_THRESHOLD_NS = 100'000'000;

// Placeholder. Time without a reading before VN300 escalates from STALE to FAULTY and attitude moves
// to the backup IMUs. While STALE, attitude is frozen and error grows at the body rate (about 4.5
// deg at 30 deg/s by this point). No backup IMU hardware exists yet, so a dead VN300 still freezes.
static constexpr uint64_t VN300_FAULT_THRESHOLD_NS = 150'000'000;
static_assert(
    VN300_FAULT_THRESHOLD_NS > VN300_STALE_THRESHOLD_NS, "fault threshold must sit beyond the stale window"
);

// Placeholders. Time without a reading before a sensor is stale, assuming Javad at 10 Hz or more
// (confirm against its output rate) and the LiDARs and backup IMUs at 50 Hz or more. A sensor that
// has never reported is stale.
static constexpr uint64_t JAVAD_STALE_THRESHOLD_NS = 300'000'000;
static constexpr uint64_t LIDAR_STALE_THRESHOLD_NS = 100'000'000;
static constexpr uint64_t BACKUP_IMU_STALE_THRESHOLD_NS = 50'000'000;

// Longest lateral may hold its last value without a usable Javad before it is marked invalid.
static constexpr uint64_t LATERAL_HOLD_LIMIT_NS = 500'000'000;

// Attitude is invalid this long after the last reading from a live source (a healthy VN300, or a
// backup IMU once VN300 is faulty). This includes the VN300 stale window.
static constexpr uint64_t ATTITUDE_HOLD_LIMIT_NS = 100'000'000;

// Placeholder. Max per-component quaternion difference for the two backups to agree.
static constexpr float BACKUP_IMU_AGREEMENT_THRESHOLD = 0.05f;

// Placeholder. Max per-component quaternion difference for VN300 to agree with a backup.
static constexpr float VN300_DIVERGENCE_THRESHOLD = 0.05f;

// Placeholder. Consecutive disagreeing readings to flag VN300 faulty, or agreeing ones to clear it.
static constexpr int VN300_DIVERGENCE_STREAK_THRESHOLD = 10;

// Placeholder. Max per-axis NED velocity difference [m/s] for Javad to agree with VN300.
static constexpr float JAVAD_VN300_VELOCITY_DIVERGENCE_THRESHOLD = 1.0f;

// Placeholder. Consecutive disagreeing Javad epochs to flag it faulty, or agreeing ones to clear it.
static constexpr int JAVAD_DIVERGENCE_STREAK_THRESHOLD = 10;

// Placeholder. Max altitude difference [m] for a LiDAR to agree with Javad or the other LiDAR.
static constexpr float LIDAR_ALTITUDE_DIVERGENCE_THRESHOLD = 1.0f;

// Placeholder. Consecutive disagreeing checks to flag a LiDAR faulty, or agreeing ones to clear it.
static constexpr int LIDAR_DIVERGENCE_STREAK_THRESHOLD = 10;

#if CONFIG_TEST
static bool now_ns_override_set = false;
static uint64_t now_ns_override = 0;
#endif

struct ArrivalTracker {
    bool seen = false;
    uint64_t last_arrival_ns = 0;
};

static ArrivalTracker javad_arrival;
static ArrivalTracker lidar_1_arrival;
static ArrivalTracker lidar_2_arrival;
static ArrivalTracker backup_imu_1_arrival;
static ArrivalTracker backup_imu_2_arrival;
static bool javad_stale = true;
static bool lateral_held = false;
static uint64_t lateral_held_since_ns = 0;
static bool attitude_clock_started = false;
static uint64_t last_live_attitude_ns = 0;
static bool lidar_1_stale = true;
static bool lidar_2_stale = true;
static bool backup_imu_1_stale = true;
static bool backup_imu_2_stale = true;

static SensorHealth vn300_health = SensorHealth_STALE;
static uint64_t vn300_last_arrival_time_ns = 0;
static uint64_t vn300_recovery_start_time_ns = 0;
static bool vn300_recovering = false;
static int vn300_disagree_streak = 0;
static int vn300_agree_streak = 0;

// Last backup readings VN300 was compared against, so a streak counts readings, not loop ticks.
static uint64_t vn300_divergence_last_backup_1_time_ns = 0;
static uint64_t vn300_divergence_last_backup_2_time_ns = 0;

// A staleness fault clears on the re-trust timer; a divergence fault clears only on an agree streak.
static bool vn300_fault_from_staleness = false;

// Divergence health only. Staleness is tracked in javad_stale.
static SensorHealth javad_health = SensorHealth_HEALTHY;
static int javad_disagree_streak = 0;
static int javad_agree_streak = 0;
static uint64_t javad_divergence_last_gnss_time_ns = 0;

// Divergence health only. Staleness is tracked in lidar_1_stale and lidar_2_stale.
static SensorHealth lidar_1_health = SensorHealth_HEALTHY;
static int lidar_1_disagree_streak = 0;
static int lidar_1_agree_streak = 0;
static SensorHealth lidar_2_health = SensorHealth_HEALTHY;
static int lidar_2_disagree_streak = 0;
static int lidar_2_agree_streak = 0;

// Last Javad epoch each LiDAR was compared against, so each LiDAR counts one disagreement per epoch.
static uint64_t lidar_1_divergence_last_gnss_time_ns = 0;
static uint64_t lidar_2_divergence_last_gnss_time_ns = 0;

// Height above whatever the beam hit, from slant range, attitude and the LiDAR mount angle. World is
// Z-up and attitude_wb is world-to-body, so its conjugate rotates body vectors into world.
//
// ASSUMPTION, to be verified against the mount: the boresight is body -Z at mount angle 0, tilted
// in the body X-Z plane.
static float calculateVerticalAltitude(float slant_range_m, const Quaternion& attitude_wb, float mount_angle_rad)
{
    Vector3D boresight_body = math_util::createVector3D(std::sin(mount_angle_rad), 0.0f, -std::cos(mount_angle_rad));

    Quaternion attitude_bw = math_util::conjugateQuaternion(attitude_wb);
    Vector3D boresight_world = math_util::multiplyQuaternionVector(attitude_bw, boresight_body);

    return -slant_range_m * boresight_world.z;
}


void StateEstimator::init()
{
    reset();
}

void StateEstimator::reset()
{
    current_estimate = EstimatedState_init_default;
    current_estimate.R_WB.qw = 1.0f; // to get identity q
    update_timestamp_ns = 0;
    has_update_timestamp_ns = false;
    lidar_1_update_timestamp_ns = 0;
    lidar_2_update_timestamp_ns = 0;
    imu_update_timestamp_ns = 0;
    backup_imu_1_update_timestamp_ns = 0;
    backup_imu_2_update_timestamp_ns = 0;
    gnss_update_timestamp_ns = 0;
    lidar_1_altitude_m = 0;
    lidar_2_altitude_m = 0;
    z_axis_ekf.reset();
    z_axis_ekf_last_predict_time_ns = 0;
    vn300_health = SensorHealth_STALE;
    vn300_last_arrival_time_ns = 0;
    vn300_recovery_start_time_ns = 0;
    vn300_recovering = false;
    vn300_fault_from_staleness = false;
    vn300_disagree_streak = 0;
    vn300_agree_streak = 0;
    vn300_divergence_last_backup_1_time_ns = 0;
    vn300_divergence_last_backup_2_time_ns = 0;
    javad_health = SensorHealth_HEALTHY;
    javad_disagree_streak = 0;
    javad_agree_streak = 0;
    javad_divergence_last_gnss_time_ns = 0;
    lidar_1_health = SensorHealth_HEALTHY;
    lidar_1_disagree_streak = 0;
    lidar_1_agree_streak = 0;
    lidar_2_health = SensorHealth_HEALTHY;
    lidar_2_disagree_streak = 0;
    lidar_2_agree_streak = 0;
    lidar_1_divergence_last_gnss_time_ns = 0;
    lidar_2_divergence_last_gnss_time_ns = 0;
    javad_arrival = {};
    lidar_1_arrival = {};
    lidar_2_arrival = {};
    backup_imu_1_arrival = {};
    backup_imu_2_arrival = {};
    javad_stale = true;
    lateral_held = false;
    lateral_held_since_ns = 0;
    attitude_clock_started = false;
    last_live_attitude_ns = 0;
    lidar_1_stale = true;
    lidar_2_stale = true;
    backup_imu_1_stale = true;
    backup_imu_2_stale = true;
#if CONFIG_TEST
    now_ns_override_set = false;
#endif
}

// Steps of estimate(), called in order below.

// Tracks VN300 arrivals and the re-trust window. Returns whether a new reading arrived this tick.
static bool trackVn300Freshness(const ImuReading& imu)
{
    if (!(imu.has_arrival_time_ns && imu.arrival_time_ns > vn300_last_arrival_time_ns)) {
        return false;
    }

    vn300_last_arrival_time_ns = imu.arrival_time_ns;

    // A divergence fault is not cleared here: data returning does not show the sensor agrees again.
    bool awaiting_reconvergence = vn300_health == SensorHealth_STALE ||
                                  (vn300_health == SensorHealth_FAULTY && vn300_fault_from_staleness);

    if (awaiting_reconvergence) {
        if (!vn300_recovering) {
            vn300_recovering = true;
            vn300_recovery_start_time_ns = imu.arrival_time_ns;
        }
        else if (imu.arrival_time_ns - vn300_recovery_start_time_ns >= VN300_RETRUST_THRESHOLD_NS) {
            // TODO: a VN300 that returns but disagrees with the backups is re-trusted here before
            // the disagree streak can flag it again. Unreachable until backup hardware exists.
            vn300_health = SensorHealth_HEALTHY;
            vn300_recovering = false;
            vn300_fault_from_staleness = false;
        }
    }

    return true;
}

static void predictVerticalState(const ImuReading& imu, bool vn300_fresh)
{
    if (!vn300_fresh) {
        return;
    }

    // Skipped on the first reading, which has no previous timestamp to form dt from.
    if (z_axis_ekf_last_predict_time_ns != 0) {
        float dt_s = static_cast<float>(imu.arrival_time_ns - z_axis_ekf_last_predict_time_ns) / 1e9f;

        if (vn300_health == SensorHealth_FAULTY) {
            // Don't integrate a faulty accelerometer, but keep growing the covariance.
            z_axis_ekf.predict_covariance_only(dt_s);
        }
        else {
            // R_WB is the health-gated attitude, so this follows the backup IMU fallback.
            Quaternion attitude_bw = math_util::conjugateQuaternion(current_estimate.R_WB);
            Vector3D accel_body = math_util::createVector3D(imu.accel_x, imu.accel_y, imu.accel_z);
            Vector3D accel_world = math_util::multiplyQuaternionVector(attitude_bw, accel_body);

            z_axis_ekf.predict(accel_world.z, dt_s);
        }
    }

    // Stamped even while faulty, so the first step after recovery doesn't span the fault window.
    z_axis_ekf_last_predict_time_ns = imu.arrival_time_ns;
}

static uint64_t currentTimeNs()
{
#if CONFIG_TEST
    return now_ns_override_set ? now_ns_override : k_cyc_to_ns_near64(k_cycle_get_64());
#else
    return k_cyc_to_ns_near64(k_cycle_get_64());
#endif
}

static void updateVn300Staleness()
{
    uint64_t time_since_last_reading_ns = currentTimeNs() - vn300_last_arrival_time_ns;

    if (time_since_last_reading_ns <= VN300_STALE_THRESHOLD_NS) {
        return;
    }

    vn300_recovering = false;

    if (time_since_last_reading_ns > VN300_FAULT_THRESHOLD_NS) {
        // FAULTY routes attitude to the backup IMUs; STALE alone would freeze it indefinitely.
        vn300_health = SensorHealth_FAULTY;
        vn300_fault_from_staleness = true;
    }
    else {
        // Overrides a divergence fault: with no data there is nothing left to disagree.
        vn300_health = SensorHealth_STALE;
        vn300_fault_from_staleness = false;
    }
}

static void trackArrival(ArrivalTracker& tracker, bool has_arrival_time, uint64_t arrival_time_ns)
{
    if (has_arrival_time && (!tracker.seen || arrival_time_ns > tracker.last_arrival_ns)) {
        tracker.seen = true;
        tracker.last_arrival_ns = arrival_time_ns;
    }
}

static bool isStale(const ArrivalTracker& tracker, uint64_t threshold_ns)
{
    return !tracker.seen || currentTimeNs() - tracker.last_arrival_ns > threshold_ns;
}

static void updateSensorStaleness(
    const LidarReading& lidar_1,
    const LidarReading& lidar_2,
    const ImuReading& backup_imu_1,
    const ImuReading& backup_imu_2,
    const GnssReadings& gnss
)
{
    trackArrival(javad_arrival, gnss.has_arrival_time_ns, gnss.arrival_time_ns);
    trackArrival(lidar_1_arrival, lidar_1.has_arrival_time_ns, lidar_1.arrival_time_ns);
    trackArrival(lidar_2_arrival, lidar_2.has_arrival_time_ns, lidar_2.arrival_time_ns);
    trackArrival(backup_imu_1_arrival, backup_imu_1.has_arrival_time_ns, backup_imu_1.arrival_time_ns);
    trackArrival(backup_imu_2_arrival, backup_imu_2.has_arrival_time_ns, backup_imu_2.arrival_time_ns);

    javad_stale = isStale(javad_arrival, JAVAD_STALE_THRESHOLD_NS);
    lidar_1_stale = isStale(lidar_1_arrival, LIDAR_STALE_THRESHOLD_NS);
    lidar_2_stale = isStale(lidar_2_arrival, LIDAR_STALE_THRESHOLD_NS);
    backup_imu_1_stale = isStale(backup_imu_1_arrival, BACKUP_IMU_STALE_THRESHOLD_NS);
    backup_imu_2_stale = isStale(backup_imu_2_arrival, BACKUP_IMU_STALE_THRESHOLD_NS);
}

static bool javadUsable()
{
    return !javad_stale && javad_health == SensorHealth_HEALTHY;
}

// All Javad velocity goes through here. GREIS [VE] is ECEF; this rotates it to NED at the
// placeholder launch site. Returns (north, east, down).
static Vector3D javadVelocityNed(const GnssReadings& gnss)
{
    return math_util::ecefToNedVelocity(
        gnss.vx_ms, gnss.vy_ms, gnss.vz_ms, LAUNCH_SITE_LAT_DEG * DEG2RAD_F, LAUNCH_SITE_LON_DEG * DEG2RAD_F
    );
}

static void checkVn300Divergence(
    const ImuReading& imu, const ImuReading& backup_imu_1, const ImuReading& backup_imu_2, bool vn300_fresh
)
{
    // VN300 is judged only when the two backups agree with each other. Never runs until backup
    // hardware exists.
    if (imu.has_arrival_time_ns && backup_imu_1.has_arrival_time_ns && backup_imu_2.has_arrival_time_ns) {
        if (!vn300_fresh || backup_imu_1.arrival_time_ns <= vn300_divergence_last_backup_1_time_ns ||
            backup_imu_2.arrival_time_ns <= vn300_divergence_last_backup_2_time_ns) {
            return;
        }
        vn300_divergence_last_backup_1_time_ns = backup_imu_1.arrival_time_ns;
        vn300_divergence_last_backup_2_time_ns = backup_imu_2.arrival_time_ns;

        bool backups_agree = math_util::quaternionsAgree(
            backup_imu_1.quat_w,
            backup_imu_1.quat_x,
            backup_imu_1.quat_y,
            backup_imu_1.quat_z,
            backup_imu_2.quat_w,
            backup_imu_2.quat_x,
            backup_imu_2.quat_y,
            backup_imu_2.quat_z,
            BACKUP_IMU_AGREEMENT_THRESHOLD
        );
        bool vn300_matches_backup_1 = math_util::quaternionsAgree(
            imu.quat_w, imu.quat_x, imu.quat_y, imu.quat_z,
            backup_imu_1.quat_w, backup_imu_1.quat_x, backup_imu_1.quat_y, backup_imu_1.quat_z,
            VN300_DIVERGENCE_THRESHOLD
        );
        bool vn300_matches_backup_2 = math_util::quaternionsAgree(
            imu.quat_w, imu.quat_x, imu.quat_y, imu.quat_z,
            backup_imu_2.quat_w, backup_imu_2.quat_x, backup_imu_2.quat_y, backup_imu_2.quat_z,
            VN300_DIVERGENCE_THRESHOLD
        );

        if (backups_agree && !vn300_matches_backup_1 && !vn300_matches_backup_2) {
            vn300_disagree_streak++;
            vn300_agree_streak = 0;

            if (vn300_health != SensorHealth_FAULTY && vn300_disagree_streak >= VN300_DIVERGENCE_STREAK_THRESHOLD) {
                vn300_health = SensorHealth_FAULTY;
                vn300_fault_from_staleness = false;
                vn300_disagree_streak = 0;
            }
        }
        else {
            vn300_agree_streak++;
            vn300_disagree_streak = 0;

            // A staleness fault recovers on the re-trust timer instead.
            if (vn300_health == SensorHealth_FAULTY && !vn300_fault_from_staleness &&
                vn300_agree_streak >= VN300_DIVERGENCE_STREAK_THRESHOLD) {
                vn300_health = SensorHealth_HEALTHY;
                vn300_agree_streak = 0;
            }
        }
    }
}

static void checkJavadDivergence(const ImuReading& imu, const GnssReadings& gnss, bool vn300_fresh)
{
    // Javad is judged only against a healthy VN300, once per Javad epoch.
    if (vn300_health == SensorHealth_HEALTHY && gnss.has_arrival_time_ns && imu.has_arrival_time_ns &&
        imu.has_vel_n && imu.has_vel_e && imu.has_vel_d) {
        if (!vn300_fresh || gnss.arrival_time_ns <= javad_divergence_last_gnss_time_ns) {
            return;
        }
        javad_divergence_last_gnss_time_ns = gnss.arrival_time_ns;

        const Vector3D javad_ned = javadVelocityNed(gnss);
        bool velocities_agree = math_util::vectorsClose(
            javad_ned.x, javad_ned.y, javad_ned.z,
            imu.vel_n, imu.vel_e, imu.vel_d,
            JAVAD_VN300_VELOCITY_DIVERGENCE_THRESHOLD
        );

        if (!velocities_agree) {
            javad_disagree_streak++;
            javad_agree_streak = 0;

            if (javad_health != SensorHealth_FAULTY && javad_disagree_streak >= JAVAD_DIVERGENCE_STREAK_THRESHOLD) {
                javad_health = SensorHealth_FAULTY;
                javad_disagree_streak = 0;
            }
        }
        else {
            javad_agree_streak++;
            javad_disagree_streak = 0;

            if (javad_health == SensorHealth_FAULTY && javad_agree_streak >= JAVAD_DIVERGENCE_STREAK_THRESHOLD) {
                javad_health = SensorHealth_HEALTHY;
                javad_agree_streak = 0;
            }
        }
    }
}

// False once the channel has gone longer than limit_ns without a live source.
static bool channelValid(bool has_live_source, bool& held, uint64_t& held_since_ns, uint64_t limit_ns)
{
    if (has_live_source) {
        held = false;
        return true;
    }
    if (!held) {
        held = true;
        held_since_ns = currentTimeNs();
    }
    return currentTimeNs() - held_since_ns <= limit_ns;
}

// Before any live reading, the limit runs from the first call, as the lateral hold does.
static bool attitudeValid()
{
    const uint64_t now_ns = currentTimeNs();
    if (!attitude_clock_started) {
        attitude_clock_started = true;
        last_live_attitude_ns = now_ns;
    }
    if (vn300_health == SensorHealth_HEALTHY) {
        last_live_attitude_ns = std::max(last_live_attitude_ns, vn300_last_arrival_time_ns);
    }
    else if (vn300_health == SensorHealth_FAULTY) {
        for (const ArrivalTracker* backup : {&backup_imu_1_arrival, &backup_imu_2_arrival}) {
            if (backup->seen) {
                last_live_attitude_ns = std::max(last_live_attitude_ns, backup->last_arrival_ns);
            }
        }
    }
    return now_ns - last_live_attitude_ns <= ATTITUDE_HOLD_LIMIT_NS;
}

static void updateFaultStatus()
{
    current_estimate.lateral_valid = channelValid(javadUsable(), lateral_held, lateral_held_since_ns, LATERAL_HOLD_LIMIT_NS);
    current_estimate.attitude_valid = attitudeValid();

    bool vn300_ok = vn300_health == SensorHealth_HEALTHY;
    bool javad_ok = javadUsable();
    bool critical_fault = vn300_health == SensorHealth_FAULTY && javad_health == SensorHealth_FAULTY;

    if (critical_fault || !current_estimate.lateral_valid || !current_estimate.attitude_valid) {
        current_estimate.fault_status = EstimatorFaultStatus_CRITICAL;
    }
    else if (!vn300_ok || !javad_ok) {
        current_estimate.fault_status = EstimatorFaultStatus_DEGRADED;
    }
    else {
        current_estimate.fault_status = EstimatorFaultStatus_NOMINAL;
    }
}

static void checkPrimarySensorFaults(
    const ImuReading& imu,
    const ImuReading& backup_imu_1,
    const ImuReading& backup_imu_2,
    const GnssReadings& gnss,
    bool vn300_fresh
)
{
    updateVn300Staleness();
    checkVn300Divergence(imu, backup_imu_1, backup_imu_2, vn300_fresh);
    checkJavadDivergence(imu, gnss, vn300_fresh);
    updateFaultStatus();
}

static void takeAttitudeFrom(const ImuReading& source, float& last_update_timestamp_ns)
{
    if (source.sense_time_ns > last_update_timestamp_ns) {
        last_update_timestamp_ns = source.sense_time_ns;
        current_estimate.R_WB.qw = source.quat_w;
        current_estimate.R_WB.qx = source.quat_x;
        current_estimate.R_WB.qy = source.quat_y;
        current_estimate.R_WB.qz = source.quat_z;
    }
}

static void updateAttitude(const ImuReading& imu, const ImuReading& backup_imu_1, const ImuReading& backup_imu_2)
{
    // A STALE VN300 freezes attitude. Once FAULTY, backup IMU 1 is used, or backup IMU 2 while
    // backup 1 is stale.
    if (vn300_health == SensorHealth_HEALTHY) {
        takeAttitudeFrom(imu, imu_update_timestamp_ns);
    }
    else if (vn300_health == SensorHealth_FAULTY) {
        if (!backup_imu_1_stale || backup_imu_2_stale) {
            takeAttitudeFrom(backup_imu_1, backup_imu_1_update_timestamp_ns);
        }
        else {
            takeAttitudeFrom(backup_imu_2, backup_imu_2_update_timestamp_ns);
        }
    }
}

// Returns whether a new Javad reading was used this tick.
static bool updateFromGnss(const ImuReading& imu, const GnssReadings& gnss)
{
    bool gnss_updated = false;

    // A faulty Javad leaves lateral state held at its last value.
    if (javad_health != SensorHealth_FAULTY && gnss.sense_time_ns > gnss_update_timestamp_ns) {
        gnss_update_timestamp_ns = gnss.sense_time_ns;
        gnss_updated = true;
        const Vector3D javad_ned = javadVelocityNed(gnss);
        const float javad_vz_up_ms = -javad_ned.z;
        current_estimate.position.x = gnss.north_m;
        current_estimate.position.y = gnss.east_m;
        current_estimate.velocity.x = javad_ned.x;
        current_estimate.velocity.y = javad_ned.y;

        if (!z_axis_ekf.is_valid()) {
            z_axis_ekf.init(
                gnss.up_m, javad_vz_up_ms, EKF_INITIAL_Z_VARIANCE_M2, EKF_INITIAL_VZ_VARIANCE_M2_S2, EKF_INITIAL_BIAS_VARIANCE_M2_S4
            );
        }
        else {
            // Weighted by Javad's own reported sigma for this epoch.
            float javad_z_r_variance = std::max(gnss.vrms_m * gnss.vrms_m, JAVAD_ALTITUDE_R_VARIANCE_FLOOR_M2);
            z_axis_ekf.update_altitude(gnss.up_m, javad_z_r_variance);

            float javad_vz_r_variance = std::max(gnss.vvel_rms_ms * gnss.vvel_rms_ms, JAVAD_VELOCITY_R_VARIANCE_FLOOR_M2_S2);
            z_axis_ekf.update_velocity(javad_vz_up_ms, javad_vz_r_variance);
        }
    }
    else if (!javadUsable() && vn300_health != SensorHealth_FAULTY &&
             imu.has_arrival_time_ns && imu.has_vel_d) {
        // VN300 vertical velocity stands in for Javad's. Its fused altitude is not used, since that
        // would feed an already filtered value back in. vel_d is positive down.
        float vz_world_from_vn300 = -imu.vel_d;
        z_axis_ekf.update_velocity(vz_world_from_vn300, VN300_VEL_D_VELOCITY_R_VARIANCE_M2_S2);
    }

    return gnss_updated;
}

static bool trackLidarFreshness(const LidarReading& lidar, float& last_update_timestamp_ns)
{
    if (lidar.sense_time_ns > last_update_timestamp_ns) {
        last_update_timestamp_ns = lidar.sense_time_ns;
        return true;
    }
    return false;
}

static void projectLidarAltitudes(
    const LidarReading& lidar_1, const LidarReading& lidar_2, bool lidar_1_updated, bool lidar_2_updated
)
{
    if (lidar_1_updated) {
        lidar_1_altitude_m = calculateVerticalAltitude(lidar_1.distance_m, current_estimate.R_WB, LIDAR_MOUNT_ANGLE_DEG * DEG2RAD_F);
    }
    if (lidar_2_updated) {
        lidar_2_altitude_m = calculateVerticalAltitude(lidar_2.distance_m, current_estimate.R_WB, LIDAR_MOUNT_ANGLE_DEG * DEG2RAD_F);
    }
}

static void updateLidarDivergence(bool disagrees, int& disagree_streak, int& agree_streak, SensorHealth& health)
{
    if (disagrees) {
        disagree_streak++;
        agree_streak = 0;
        if (health != SensorHealth_FAULTY && disagree_streak >= LIDAR_DIVERGENCE_STREAK_THRESHOLD) {
            health = SensorHealth_FAULTY;
            disagree_streak = 0;
        }
    }
    else {
        agree_streak++;
        disagree_streak = 0;
        if (health == SensorHealth_FAULTY && agree_streak >= LIDAR_DIVERGENCE_STREAK_THRESHOLD) {
            health = SensorHealth_HEALTHY;
            agree_streak = 0;
        }
    }
}

// 2-of-3 vote between the two LiDARs and Javad: a LiDAR disagrees only if it disagrees with both.
// Each LiDAR is checked on its own new readings, once per Javad epoch, and only while Javad is
// usable and the other LiDAR is not stale.
static void checkLidarDivergence(const GnssReadings& gnss, bool lidar_1_updated, bool lidar_2_updated)
{
    if (!javadUsable()) {
        return;
    }

    const bool lidars_match_each_other =
        std::fabs(lidar_1_altitude_m - lidar_2_altitude_m) < LIDAR_ALTITUDE_DIVERGENCE_THRESHOLD;

    if (lidar_1_updated && !lidar_2_stale && gnss.arrival_time_ns > lidar_1_divergence_last_gnss_time_ns) {
        lidar_1_divergence_last_gnss_time_ns = gnss.arrival_time_ns;
        const bool matches_javad = std::fabs(lidar_1_altitude_m - gnss.up_m) < LIDAR_ALTITUDE_DIVERGENCE_THRESHOLD;
        updateLidarDivergence(!matches_javad && !lidars_match_each_other, lidar_1_disagree_streak, lidar_1_agree_streak, lidar_1_health);
    }

    if (lidar_2_updated && !lidar_1_stale && gnss.arrival_time_ns > lidar_2_divergence_last_gnss_time_ns) {
        lidar_2_divergence_last_gnss_time_ns = gnss.arrival_time_ns;
        const bool matches_javad = std::fabs(lidar_2_altitude_m - gnss.up_m) < LIDAR_ALTITUDE_DIVERGENCE_THRESHOLD;
        updateLidarDivergence(!matches_javad && !lidars_match_each_other, lidar_2_disagree_streak, lidar_2_agree_streak, lidar_2_health);
    }
}

static void updateVerticalFromLidars(bool lidar_1_updated, bool lidar_2_updated)
{
    if (lidar_1_updated && lidar_1_health == SensorHealth_HEALTHY) {
        z_axis_ekf.update_altitude(lidar_1_altitude_m, LIDAR_1_ALTITUDE_R_VARIANCE_M2);
    }
    if (lidar_2_updated && lidar_2_health == SensorHealth_HEALTHY) {
        z_axis_ekf.update_altitude(lidar_2_altitude_m, LIDAR_2_ALTITUDE_R_VARIANCE_M2);
    }
}

static void publishVerticalState(const GnssReadings& gnss, bool gnss_updated)
{
    if constexpr (IS_ENABLED(CONFIG_USE_ZAXIS_EKF_FOR_ALTITUDE)) {
        if (z_axis_ekf.is_valid()) {
            current_estimate.position.z = z_axis_ekf.z_m();
            current_estimate.velocity.z = z_axis_ekf.vz_ms();
        }
        else if (gnss_updated) {
            current_estimate.position.z = gnss.up_m;
            current_estimate.velocity.z = -javadVelocityNed(gnss).z;
        }
    }
    else if (gnss_updated) {
        current_estimate.position.z = gnss.up_m;
        current_estimate.velocity.z = -javadVelocityNed(gnss).z;
    }
}

// Javad and the LiDARs track staleness apart from their divergence health; FAULTY takes precedence.
static SensorHealth publishedHealth(SensorHealth divergence_health, bool stale)
{
    if (divergence_health == SensorHealth_FAULTY) {
        return SensorHealth_FAULTY;
    }
    return stale ? SensorHealth_STALE : divergence_health;
}

static void publishSensorHealth()
{
    current_estimate.vn300_health = vn300_health;
    current_estimate.javad_health = publishedHealth(javad_health, javad_stale);
    current_estimate.lidar_1_health = publishedHealth(lidar_1_health, lidar_1_stale);
    current_estimate.lidar_2_health = publishedHealth(lidar_2_health, lidar_2_stale);
}

std::optional<EstimatedState> StateEstimator::estimate(
    LidarReading& lidar_1,
    LidarReading& lidar_2,
    ImuReading& imu,
    ImuReading& backup_imu_1,
    ImuReading& backup_imu_2,
    GnssReadings& gnss
)
{
    update_timestamp_ns = static_cast<float>(k_cycle_get_64()) / sys_clock_hw_cycles_per_sec() * 1e9f;
    has_update_timestamp_ns = true;

    bool vn300_fresh = trackVn300Freshness(imu);
    updateSensorStaleness(lidar_1, lidar_2, backup_imu_1, backup_imu_2, gnss);

    checkPrimarySensorFaults(imu, backup_imu_1, backup_imu_2, gnss, vn300_fresh);

    // Uses this tick's VN300 health and the previous tick's attitude.
    predictVerticalState(imu, vn300_fresh);

    updateAttitude(imu, backup_imu_1, backup_imu_2);
    bool gnss_updated = updateFromGnss(imu, gnss);

    bool lidar_1_updated = trackLidarFreshness(lidar_1, lidar_1_update_timestamp_ns);
    bool lidar_2_updated = trackLidarFreshness(lidar_2, lidar_2_update_timestamp_ns);
    projectLidarAltitudes(lidar_1, lidar_2, lidar_1_updated, lidar_2_updated);

    // After updateAttitude(), since the LiDAR altitudes depend on this tick's attitude.
    checkLidarDivergence(gnss, lidar_1_updated, lidar_2_updated);
    updateVerticalFromLidars(lidar_1_updated, lidar_2_updated);

    publishVerticalState(gnss, gnss_updated);
    publishSensorHealth();

    return current_estimate;
}

#if CONFIG_TEST
void StateEstimator::set_now_ns_for_testing(uint64_t ns)
{
    now_ns_override = ns;
    now_ns_override_set = true;
}

bool StateEstimator::vn300_is_healthy_for_testing()
{
    return vn300_health == SensorHealth_HEALTHY;
}

bool StateEstimator::vn300_is_stale_for_testing()
{
    return vn300_health == SensorHealth_STALE;
}

bool StateEstimator::vn300_is_faulty_for_testing()
{
    return vn300_health == SensorHealth_FAULTY;
}

bool StateEstimator::javad_is_faulty_for_testing()
{
    return javad_health == SensorHealth_FAULTY;
}

bool StateEstimator::javad_is_stale_for_testing()
{
    return javad_stale;
}

bool StateEstimator::lidar_1_is_stale_for_testing()
{
    return lidar_1_stale;
}

bool StateEstimator::lidar_2_is_stale_for_testing()
{
    return lidar_2_stale;
}

bool StateEstimator::backup_imu_1_is_stale_for_testing()
{
    return backup_imu_1_stale;
}

bool StateEstimator::backup_imu_2_is_stale_for_testing()
{
    return backup_imu_2_stale;
}

bool StateEstimator::lidar_1_is_faulty_for_testing()
{
    return lidar_1_health == SensorHealth_FAULTY;
}

bool StateEstimator::lidar_2_is_faulty_for_testing()
{
    return lidar_2_health == SensorHealth_FAULTY;
}

int StateEstimator::vn300_divergence_streak_threshold_for_testing()
{
    return VN300_DIVERGENCE_STREAK_THRESHOLD;
}

int StateEstimator::javad_divergence_streak_threshold_for_testing()
{
    return JAVAD_DIVERGENCE_STREAK_THRESHOLD;
}

int StateEstimator::lidar_divergence_streak_threshold_for_testing()
{
    return LIDAR_DIVERGENCE_STREAK_THRESHOLD;
}

float StateEstimator::ekf_z_variance_for_testing()
{
    return z_axis_ekf.z_variance();
}

float StateEstimator::calculate_vertical_altitude_for_testing(float slant_range_m, const Quaternion& attitude_wb, float mount_angle_rad)
{
    return calculateVerticalAltitude(slant_range_m, attitude_wb, mount_angle_rad);
}
#endif

