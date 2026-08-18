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


// ── Design notes ────────────────────────────────────────────────────────────────────────────
//
// Health flags flip only after N consecutive readings agree, in both directions, so sensor noise
// or a one-tick timing skew between independently-clocked sensors can't flap the state.
//
// STALE recovers on a timer, FAULTY on a streak: STALE means no data at all, and once it resumes
// the sensor's own filter needs wall-clock time to re-converge; FAULTY means data is arriving but
// disagrees, which only needs enough samples to confirm. VN300 is the only sensor here with a
// staleness concept, so it's the only one using both.

// Health state for a fused sensor. Tracked for VN300, Javad, and the two LiDARs.
enum class SensorHealth {
    HEALTHY,  // trusted, actively used in the estimate
    STALE,    // no reading recently enough to trust (see staleness check) -- timer-based recovery
    FAULTY,   // consistently disagrees with corroborating sensors (see divergence check) -- streak-based recovery
};

static EstimatedState current_estimate = EstimatedState_init_default;
static float update_timestamp_ns = 0;
static bool has_update_timestamp_ns = false;
static float lidar_1_update_timestamp_ns = 0;
static float lidar_2_update_timestamp_ns = 0;
static float imu_update_timestamp_ns = 0;
static float backup_imu_1_update_timestamp_ns = 0;
static float gnss_update_timestamp_ns = 0;
static float lidar_1_altitude_m = 0;
static float lidar_2_altitude_m = 0;

// Fusion policy for vertical state (position.z/velocity.z). Tracks its own dt from consecutive
// VN300 arrival times, separate from vn300_last_arrival_time_ns, which serves the staleness check.
static ZAxisEkf z_axis_ekf;
static uint64_t z_axis_ekf_last_predict_time_ns = 0;

// Ground-testing escape hatch: flip to false to bypass the EKF and pass raw GNSS altitude/vertical
// velocity straight through. Should be true for real flight.
static constexpr bool USE_ZAXIS_EKF_FOR_ALTITUDE = true;

// Initial covariance when bootstrapping the EKF from the first healthy GNSS reading, which is a
// single uncertain measurement rather than a known state. Bias starts less uncertain than z/vz,
// but not known-exact.
//
// z and vz are 2m and 2 m/s sigma (variance = sigma^2 = 4.0), set from review feedback on PR #234:
// a 10m / 5 m/s starting uncertainty is far larger than anything this vehicle actually starts with
// and makes the filter over-trust the first few measurements while it collapses. The bias term is
// still an untuned placeholder.
static constexpr float EKF_INITIAL_Z_VARIANCE_M2 = 4.0f;
static constexpr float EKF_INITIAL_VZ_VARIANCE_M2_S2 = 4.0f;
static constexpr float EKF_INITIAL_BIAS_VARIANCE_M2_S4 = 1.0f;

// Placeholder -- needs real LiDAR noise specs before flight. r_variance for each LiDAR's computed
// altitude.
static constexpr float LIDAR_1_ALTITUDE_R_VARIANCE_M2 = 0.01f;
static constexpr float LIDAR_2_ALTITUDE_R_VARIANCE_M2 = 0.01f;

// Placeholder -- needs real GNSS noise-floor analysis before flight. Floors on Javad's own
// reported per-epoch uncertainty, so a zero or implausibly tiny value can't make the filter
// overconfident in one epoch.
static constexpr float JAVAD_ALTITUDE_R_VARIANCE_FLOOR_M2 = 1e-4f;
static constexpr float JAVAD_VELOCITY_R_VARIANCE_FLOOR_M2_S2 = 1e-4f;

// Placeholder -- needs real VN300 INS velocity noise specs before flight. r_variance for the
// vel_d fallback, used only while Javad is faulty.
static constexpr float VN300_VEL_D_VELOCITY_R_VARIANCE_M2_S2 = 0.25f;

// VN300 updates at ~400Hz (2.5ms period). If this long passes without a new reading arriving,
// mark it stale and stop trusting its attitude output -- 50ms is ~20 missed readings.
static constexpr uint64_t VN300_STALE_THRESHOLD_NS = 50'000'000;

// After an outage, require this much continuous good data before trusting VN300 again -- 100ms
// (~40 updates at 400Hz) gives its onboard filter time to re-converge. Deliberately 2x the stale
// threshold: fail fast, recover cautiously.
static constexpr uint64_t VN300_RETRUST_THRESHOLD_NS = 100'000'000;

// Placeholder -- to be tuned against real VN300 dropout behavior. Total time without a reading
// before VN300 escalates STALE -> FAULTY, handing attitude over to backup IMU 1.
//
// STALE holds the last R_WB without integrating anything, so attitude error accrues at the full
// body rotation rate: at 30 deg/s every 100ms of freeze is ~3 deg. This sits only 100ms past the
// stale threshold (~4.5 deg total at 30 deg/s) -- long enough that a brief serial hiccup still
// just freezes, short enough that a dead VN300 is handed off quickly.
//
// On current hardware this fixes the attitude-source ROUTING, not the visible symptom: no backup
// IMU hardware exists, so backup_imu_1.sense_time_ns never advances and updateAttitude()'s backup
// branch no-ops -- a dead VN300 still practically freezes today. The fallback becomes real once
// backup hardware is fitted.
static constexpr uint64_t VN300_FAULT_THRESHOLD_NS = 150'000'000;
static_assert(
    VN300_FAULT_THRESHOLD_NS > VN300_STALE_THRESHOLD_NS, "fault threshold must sit beyond the stale window"
);

// Placeholder -- needs real backup IMU noise specs before flight. Max per-component quaternion
// difference for the two backups to count as agreeing with each other.
static constexpr float BACKUP_IMU_AGREEMENT_THRESHOLD = 0.05f;

// Placeholder -- needs real backup IMU noise specs before flight. Max per-component quaternion
// difference before VN300 and a backup count as disagreeing.
static constexpr float VN300_DIVERGENCE_THRESHOLD = 0.05f;

// Placeholder -- needs real backup IMU noise specs before flight. Consecutive disagreeing readings
// to flag VN300 faulty, and consecutive agreeing readings to un-flag it.
static constexpr int VN300_DIVERGENCE_STREAK_THRESHOLD = 10;

// Placeholder -- also needs a frame-alignment pass before flight: GNSS vx/vy/vz and VN300
// vel_n/vel_e/vel_d aren't guaranteed to be directly comparable without a proper transform. Max
// per-axis velocity difference [m/s] before Javad and VN300 count as disagreeing.
static constexpr float JAVAD_VN300_VELOCITY_DIVERGENCE_THRESHOLD = 1.0f;

// Placeholder -- needs real GNSS/VN300 velocity noise analysis before flight. Consecutive
// disagreeing GNSS refreshes to flag Javad faulty, and agreeing refreshes to un-flag it.
static constexpr int JAVAD_DIVERGENCE_STREAK_THRESHOLD = 10;

// Placeholder -- needs real LiDAR/GNSS altitude noise analysis before flight. Max altitude
// difference [m] before a LiDAR and Javad, or the two LiDARs, count as disagreeing.
static constexpr float LIDAR_ALTITUDE_DIVERGENCE_THRESHOLD = 1.0f;

// Placeholder -- needs real LiDAR/GNSS altitude noise analysis before flight. Consecutive
// disagreeing refreshes to flag a LiDAR faulty, and agreeing refreshes to un-flag it.
static constexpr int LIDAR_DIVERGENCE_STREAK_THRESHOLD = 10;

#if CONFIG_TEST
static bool now_ns_override_set = false;
static uint64_t now_ns_override = 0;
#endif

static SensorHealth vn300_health = SensorHealth::STALE;
static uint64_t vn300_last_arrival_time_ns = 0;
static uint64_t vn300_recovery_start_time_ns = 0;
static bool vn300_recovering = false;
static int vn300_disagree_streak = 0;
static int vn300_agree_streak = 0;

// Divergence streaks must count independent disagreements, not control-loop iterations. The loop
// runs ~1000Hz while the slowest participant in each comparison updates far slower, so without
// these the same held reading is re-counted on every iteration and N stops meaning "N readings".
// Each check advances its streak only on a tick where every sensor it compares is genuinely fresh.
static uint64_t vn300_divergence_last_backup_1_time_ns = 0;
static uint64_t vn300_divergence_last_backup_2_time_ns = 0;

// Why vn300_health is FAULTY, so recovery routes to the right mechanism: a fault raised by
// staleness clears on the VN300_RETRUST_THRESHOLD_NS timer once data returns, while one raised by
// checkVn300Divergence() still clears only via its agree-streak, which needs live readings to
// compare against.
static bool vn300_fault_from_staleness = false;

// Javad has no staleness tracking, so its health only toggles HEALTHY <-> FAULTY via the velocity
// divergence check. Starts HEALTHY rather than STALE because nothing would ever move it out of
// STALE.
static SensorHealth javad_health = SensorHealth::HEALTHY;
static int javad_disagree_streak = 0;
static int javad_agree_streak = 0;
static uint64_t javad_divergence_last_gnss_time_ns = 0;

// Same for the LiDARs: no staleness tracking, HEALTHY default, toggled by the altitude divergence
// check.
static SensorHealth lidar_1_health = SensorHealth::HEALTHY;
static int lidar_1_disagree_streak = 0;
static int lidar_1_agree_streak = 0;
static SensorHealth lidar_2_health = SensorHealth::HEALTHY;
static int lidar_2_disagree_streak = 0;
static int lidar_2_agree_streak = 0;

// The LiDAR pair's own freshness is already established by trackLidarFreshness(); this covers the
// Javad side of the same comparison.
static uint64_t lidar_divergence_last_gnss_time_ns = 0;

// Converts a LiDAR slant range into vertical altitude (height above whatever the beam hit), using
// the vehicle's current attitude and the LiDAR's fixed body-frame mount angle.
//
// Frame conventions: the world frame is Z-up, and attitude_wb (EstimatedState.R_WB /
// ImuReading.quat_*) is world-to-body, so rotating a body vector into world frame takes its
// conjugate. Getting that backwards would silently invert the tilt correction.
//
// ASSUMPTION pending confirmation: the boresight points straight down (body -Z) at
// mount_angle_rad 0, tilted in the body X-Z plane. A wrong axis assumption produces a
// plausible-looking but wrong altitude with no obvious failure signal, unlike a wrong angle.
static float calculateVerticalAltitude(float slant_range_m, const Quaternion& attitude_wb, float mount_angle_rad)
{
    Vector3D boresight_body = math_util::createVector3D(std::sin(mount_angle_rad), 0.0f, -std::cos(mount_angle_rad));

    Quaternion attitude_bw = math_util::conjugateQuaternion(attitude_wb);
    Vector3D boresight_world = math_util::multiplyQuaternionVector(attitude_bw, boresight_body);

    // boresight_world.z is negative (points down, toward -world-Z); negate so altitude comes out
    // positive, matching the rest of the codebase's Z-up-is-positive convention.
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
    gnss_update_timestamp_ns = 0;
    lidar_1_altitude_m = 0;
    lidar_2_altitude_m = 0;
    z_axis_ekf.reset();
    z_axis_ekf_last_predict_time_ns = 0;
    vn300_health = SensorHealth::STALE;
    vn300_last_arrival_time_ns = 0;
    vn300_recovery_start_time_ns = 0;
    vn300_recovering = false;
    vn300_fault_from_staleness = false;
    vn300_disagree_streak = 0;
    vn300_agree_streak = 0;
    vn300_divergence_last_backup_1_time_ns = 0;
    vn300_divergence_last_backup_2_time_ns = 0;
    javad_health = SensorHealth::HEALTHY;
    javad_disagree_streak = 0;
    javad_agree_streak = 0;
    javad_divergence_last_gnss_time_ns = 0;
    lidar_1_health = SensorHealth::HEALTHY;
    lidar_1_disagree_streak = 0;
    lidar_1_agree_streak = 0;
    lidar_2_health = SensorHealth::HEALTHY;
    lidar_2_disagree_streak = 0;
    lidar_2_agree_streak = 0;
    lidar_divergence_last_gnss_time_ns = 0;
#if CONFIG_TEST
    now_ns_override_set = false;
#endif
}

// ── estimate() sub-steps ────────────────────────────────────────────────────────────────────
// Called in order from estimate() below. Each operates on the same file-static state; the split
// is for readability only -- execution order and behavior are unchanged from the single function
// these were extracted from.

// Advances VN300 arrival-time tracking and the post-outage re-convergence window. Returns whether
// a genuinely new reading arrived this tick: this function updates vn300_last_arrival_time_ns, so
// predictVerticalState() can't re-test the same gate and has to be told.
static bool trackVn300Freshness(const ImuReading& imu)
{
    // VN300 staleness / re-trust tracking, based on wall-clock arrival time rather than
    // sense_time_ns (an inter-arrival delta, not usable for staleness checks).
    if (!(imu.has_arrival_time_ns && imu.arrival_time_ns > vn300_last_arrival_time_ns)) {
        return false;
    }

    vn300_last_arrival_time_ns = imu.arrival_time_ns;

    // A stale-induced FAULTY re-converges the same way STALE does -- on the re-trust timer, once
    // data comes back. A divergence-induced FAULTY deliberately does not: it clears only via
    // checkVn300Divergence()'s agree-streak, since "the data returned" says nothing about whether
    // the sensor is still lying.
    bool awaiting_reconvergence = vn300_health == SensorHealth::STALE ||
                                  (vn300_health == SensorHealth::FAULTY && vn300_fault_from_staleness);

    if (awaiting_reconvergence) {
        if (!vn300_recovering) {
            // First good reading since the outage began -- start the re-convergence window.
            vn300_recovering = true;
            vn300_recovery_start_time_ns = imu.arrival_time_ns;
        }
        else if (imu.arrival_time_ns - vn300_recovery_start_time_ns >= VN300_RETRUST_THRESHOLD_NS) {
            // TODO: a VN300 that returns but genuinely disagrees with the backups is re-trusted
            // here at the 100ms timer before the disagree-streak can re-raise FAULTY. Unreachable
            // today (no backup hardware); resolve alongside the comment-24 streak-gating work.
            vn300_health = SensorHealth::HEALTHY;
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

    // dt comes from consecutive arrival_time_ns deltas, so it stays correct if VN300's rate drifts
    // from ~400Hz. Skipped on the very first reading, where there's no prior timestamp to form one.
    if (z_axis_ekf_last_predict_time_ns != 0) {
        float dt_s = static_cast<float>(imu.arrival_time_ns - z_axis_ekf_last_predict_time_ns) / 1e9f;

        if (vn300_health == SensorHealth::FAULTY) {
            // Never integrate off an accelerometer already declared untrustworthy -- a confidently
            // wrong altitude is worse than a stale one. Covariance still propagates, so the held
            // estimate reports uncertainty rising instead of a confidence it no longer has.
            z_axis_ekf.predict_covariance_only(dt_s);
        }
        else {
            // Rotates body-frame acceleration into world frame using current_estimate.R_WB, which
            // is already health-gated, so this inherits the VN300/backup-IMU fallback rather than
            // trusting a possibly-faulty VN300 quaternion. Gravity and bias subtraction happen
            // inside predict().
            Quaternion attitude_bw = math_util::conjugateQuaternion(current_estimate.R_WB);
            Vector3D accel_body = math_util::createVector3D(imu.accel_x, imu.accel_y, imu.accel_z);
            Vector3D accel_world = math_util::multiplyQuaternionVector(attitude_bw, accel_body);

            z_axis_ekf.predict(accel_world.z, dt_s);
        }
    }

    // Stamped on every fresh reading, faulty or not. Letting it go stale across a fault would make
    // the first step after recovery integrate the whole fault window at once whenever that window
    // is shorter than MAX_PREDICT_DT_S.
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

    // Any in-progress re-convergence restarts from scratch.
    vn300_recovering = false;

    if (time_since_last_reading_ns > VN300_FAULT_THRESHOLD_NS) {
        // The freeze has run long enough that the held attitude is no longer worth trusting.
        // Escalating to FAULTY is what routes updateAttitude() to backup IMU 1 -- without this,
        // STALE is terminal (nothing else can flag a sensor that sends no readings to disagree
        // with) and a dead VN300 would hold last-good attitude forever.
        vn300_health = SensorHealth::FAULTY;
        vn300_fault_from_staleness = true;
    }
    else {
        // Brief dropout -- freeze and wait. Takes precedence over a divergence FAULTY flag: with
        // no data at all there's nothing left to diverge.
        vn300_health = SensorHealth::STALE;
        vn300_fault_from_staleness = false;
    }
}

static void checkVn300Divergence(
    const ImuReading& imu, const ImuReading& backup_imu_1, const ImuReading& backup_imu_2, bool vn300_fresh
)
{
    // The backups must corroborate each other before VN300 is compared against them: if they
    // disagreed, a VN300-vs-backup mismatch wouldn't say which sensor is at fault. Never fires
    // today -- no backup hardware exists, so has_arrival_time_ns is never set on them.
    if (imu.has_arrival_time_ns && backup_imu_1.has_arrival_time_ns && backup_imu_2.has_arrival_time_ns) {
        // One independent sample of this comparison needs new data on every side of it. The
        // backups are the slow participants (~50-100Hz against a ~1000Hz loop), so without this
        // the same held pair is re-counted on every iteration between their readings and N
        // measures loop ticks rather than disagreements.
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

            if (vn300_health != SensorHealth::FAULTY && vn300_disagree_streak >= VN300_DIVERGENCE_STREAK_THRESHOLD) {
                vn300_health = SensorHealth::FAULTY;
                vn300_fault_from_staleness = false;
                vn300_disagree_streak = 0;
            }
        }
        else {
            vn300_agree_streak++;
            vn300_disagree_streak = 0;

            // Only un-flags a divergence-induced fault. A stale-induced one recovers on the
            // re-trust timer in trackVn300Freshness() instead: agreeing with backups says the
            // sensor isn't lying, not that it has had time to re-converge after an outage.
            if (vn300_health == SensorHealth::FAULTY && !vn300_fault_from_staleness &&
                vn300_agree_streak >= VN300_DIVERGENCE_STREAK_THRESHOLD) {
                vn300_health = SensorHealth::HEALTHY;
                vn300_agree_streak = 0;
            }
        }
    }
}

static void checkJavadDivergence(const ImuReading& imu, const GnssReadings& gnss, bool vn300_fresh)
{
    // Only runs while VN300 is healthy: if VN300 itself is untrustworthy, a mismatch says nothing
    // about Javad. See the frame-alignment caveat on JAVAD_VN300_VELOCITY_DIVERGENCE_THRESHOLD.
    if (vn300_health == SensorHealth::HEALTHY && gnss.has_arrival_time_ns && imu.has_arrival_time_ns &&
        imu.has_vel_n && imu.has_vel_e && imu.has_vel_d) {
        // Javad is the slow participant (~10-20Hz against a ~1000Hz loop), so an un-gated counter
        // reaches N within a single Javad epoch -- before a second reading has even arrived to
        // disagree independently.
        if (!vn300_fresh || gnss.arrival_time_ns <= javad_divergence_last_gnss_time_ns) {
            return;
        }
        javad_divergence_last_gnss_time_ns = gnss.arrival_time_ns;

        bool velocities_agree = math_util::vectorsClose(
            gnss.vx_ms, gnss.vy_ms, gnss.vz_ms,
            imu.vel_n, imu.vel_e, imu.vel_d,
            JAVAD_VN300_VELOCITY_DIVERGENCE_THRESHOLD
        );

        if (!velocities_agree) {
            javad_disagree_streak++;
            javad_agree_streak = 0;

            if (javad_health != SensorHealth::FAULTY && javad_disagree_streak >= JAVAD_DIVERGENCE_STREAK_THRESHOLD) {
                javad_health = SensorHealth::FAULTY;
                javad_disagree_streak = 0;
            }
        }
        else {
            javad_agree_streak++;
            javad_disagree_streak = 0;

            if (javad_health == SensorHealth::FAULTY && javad_agree_streak >= JAVAD_DIVERGENCE_STREAK_THRESHOLD) {
                javad_health = SensorHealth::HEALTHY;
                javad_agree_streak = 0;
            }
        }
    }
}

// Publishes NOMINAL / DEGRADED / CRITICAL on the estimate from the primary sensors' health.
static void updateFaultStatus()
{
    // Carried on EstimatedState so consumers can tell "one primary sensor down" from "both" by
    // reading one field, rather than re-deriving it from the health states, which are file-local.
    //
    // CRITICAL needs both VN300 and Javad specifically FAULTY, not merely STALE -- no trustworthy
    // attitude or velocity source is left. It is a severity signal for consumers, not a mode: the
    // channels each fall back on their own (attitude to backup IMU 1, lateral held, vertical
    // coasting on covariance alone), so nothing keys off this flag to decide behavior.
    bool vn300_ok = vn300_health == SensorHealth::HEALTHY;
    bool javad_ok = javad_health == SensorHealth::HEALTHY;
    bool critical_fault = vn300_health == SensorHealth::FAULTY && javad_health == SensorHealth::FAULTY;

    if (critical_fault) {
        current_estimate.fault_status = EstimatorFaultStatus_CRITICAL;
    }
    else if (!vn300_ok || !javad_ok) {
        // One primary sensor down -- still updating via fallback or partial freeze.
        current_estimate.fault_status = EstimatorFaultStatus_DEGRADED;
    }
    else {
        current_estimate.fault_status = EstimatorFaultStatus_NOMINAL;
    }
}

// Single entry point for the primary-sensor (VN300 + Javad) health checks. The LiDAR cross-check
// is deliberately NOT here -- it depends on this tick's projected LiDAR altitudes, which in turn
// depend on the attitude chosen after this runs. See checkLidarDivergence().
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

static void updateAttitude(const ImuReading& imu, const ImuReading& backup_imu_1)
{
    // VN300 while healthy; backup IMU 1 once flagged faulty (backup_imu_2 is redundant with it
    // once corroborated). A merely STALE VN300 freezes the last estimate. A critical fault takes
    // this same backup path rather than a special freeze: holding last-good attitude with a usable
    // backup available is the more dangerous of the two options.
    if (vn300_health == SensorHealth::HEALTHY) {
        if (imu.sense_time_ns > imu_update_timestamp_ns) {
            imu_update_timestamp_ns = imu.sense_time_ns;
            current_estimate.R_WB.qw = imu.quat_w;
            current_estimate.R_WB.qx = imu.quat_x;
            current_estimate.R_WB.qy = imu.quat_y;
            current_estimate.R_WB.qz = imu.quat_z;
        }
    }
    else if (vn300_health == SensorHealth::FAULTY) {
        if (backup_imu_1.sense_time_ns > backup_imu_1_update_timestamp_ns) {
            backup_imu_1_update_timestamp_ns = backup_imu_1.sense_time_ns;
            current_estimate.R_WB.qw = backup_imu_1.quat_w;
            current_estimate.R_WB.qx = backup_imu_1.quat_x;
            current_estimate.R_WB.qy = backup_imu_1.quat_y;
            current_estimate.R_WB.qz = backup_imu_1.quat_z;
        }
    }
}

// Returns whether a fresh, trusted GNSS reading was consumed this tick.
static bool updateFromGnss(const ImuReading& imu, const GnssReadings& gnss)
{
    bool gnss_updated = false;

    // Lateral position/velocity from GNSS when the timestamp changed, unless Javad is flagged
    // faulty -- then the last known-good values are held instead. Vertical state is not set here;
    // it comes from the EKF, via publishVerticalState().
    if (javad_health != SensorHealth::FAULTY && gnss.sense_time_ns > gnss_update_timestamp_ns) {
        gnss_update_timestamp_ns = gnss.sense_time_ns;
        gnss_updated = true;
        current_estimate.position.x = gnss.north_m;
        current_estimate.position.y = gnss.east_m;
        current_estimate.velocity.x = gnss.vx_ms;
        current_estimate.velocity.y = gnss.vy_ms;

        // Bootstrap the EKF from the first healthy GNSS reading; feed later ones as updates.
        if (!z_axis_ekf.is_valid()) {
            z_axis_ekf.init(
                gnss.up_m, gnss.vz_ms, EKF_INITIAL_Z_VARIANCE_M2, EKF_INITIAL_VZ_VARIANCE_M2_S2, EKF_INITIAL_BIAS_VARIANCE_M2_S4
            );
        }
        else {
            // r_variance comes from Javad's own reported per-epoch uncertainty, so a noisy epoch
            // is weighted less. vrms_m/vvel_rms_ms are sigma; r_variance is sigma^2.
            float javad_z_r_variance = std::max(gnss.vrms_m * gnss.vrms_m, JAVAD_ALTITUDE_R_VARIANCE_FLOOR_M2);
            z_axis_ekf.update_altitude(gnss.up_m, javad_z_r_variance);

            float javad_vz_r_variance = std::max(gnss.vvel_rms_ms * gnss.vvel_rms_ms, JAVAD_VELOCITY_R_VARIANCE_FLOOR_M2_S2);
            z_axis_ekf.update_velocity(gnss.vz_ms, javad_vz_r_variance);
        }
    }
    else if (javad_health == SensorHealth::FAULTY && vn300_health != SensorHealth::FAULTY &&
             imu.has_arrival_time_ns && imu.has_vel_d) {
        // Javad is faulty -- use VN300's INS vertical velocity so the EKF still gets a velocity
        // correction. No altitude fallback: VN300's ins_alt is itself already fused, so feeding it
        // back in would be circular.
        //
        // Requires a non-faulty VN300: under a critical fault both are down, and taking this
        // sensor's velocity while predictVerticalState() refuses its acceleration would be
        // incoherent -- the vertical channel would still be riding a sensor we don't trust.
        //
        // vel_d follows the NED convention and is positive DOWN, opposite this codebase's Z-up,
        // hence the negation.
        // TODO: checkJavadDivergence() compares gnss.vz_ms against imu.vel_d with no such flip.
        // If vz_ms is genuinely up-positive, that comparison has a sign bug.
        float vz_world_from_vn300 = -imu.vel_d;
        z_axis_ekf.update_velocity(vz_world_from_vn300, VN300_VEL_D_VELOCITY_R_VARIANCE_M2_S2);
    }

    return gnss_updated;
}

// Returns whether this LiDAR produced a new reading, advancing its last-seen timestamp if so.
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
    // Uses current_estimate.R_WB rather than raw VN300 output, so the projection respects the
    // VN300/backup-IMU health tracking instead of trusting a flagged-faulty quaternion.
    if (lidar_1_updated) {
        lidar_1_altitude_m = calculateVerticalAltitude(lidar_1.distance_m, current_estimate.R_WB, LIDAR_MOUNT_ANGLE_DEG * DEG2RAD_F);
    }
    if (lidar_2_updated) {
        lidar_2_altitude_m = calculateVerticalAltitude(lidar_2.distance_m, current_estimate.R_WB, LIDAR_MOUNT_ANGLE_DEG * DEG2RAD_F);
    }
}

static void checkLidarDivergence(const GnssReadings& gnss, bool lidar_1_updated, bool lidar_2_updated)
{
    // 2-of-3 across LiDAR 1, LiDAR 2 and Javad: each LiDAR is flagged only if it disagrees with
    // both of the others. If all three disagree at once, both LiDARs get flagged, which is the
    // honest outcome when nothing can be corroborated.
    // TODO: unlike checkJavadDivergence(), this doesn't require Javad to be healthy first.
    if (lidar_1_updated && lidar_2_updated && gnss.has_arrival_time_ns) {
        // lidar_*_updated already establishes the LiDAR pair's freshness; Javad is the slow
        // participant here, so without this one Javad epoch supplies the Javad half of all N
        // counts. This costs LiDAR-fault detection latency (now bounded by Javad's rate) rather
        // than leaving the counter inflatable -- tune N if that latency matters.
        if (gnss.arrival_time_ns <= lidar_divergence_last_gnss_time_ns) {
            return;
        }
        lidar_divergence_last_gnss_time_ns = gnss.arrival_time_ns;

        bool lidar_1_matches_javad = std::fabs(lidar_1_altitude_m - gnss.up_m) < LIDAR_ALTITUDE_DIVERGENCE_THRESHOLD;
        bool lidar_2_matches_javad = std::fabs(lidar_2_altitude_m - gnss.up_m) < LIDAR_ALTITUDE_DIVERGENCE_THRESHOLD;
        bool lidars_match_each_other = std::fabs(lidar_1_altitude_m - lidar_2_altitude_m) < LIDAR_ALTITUDE_DIVERGENCE_THRESHOLD;

        // Lidar 1: disagrees with both Javad and Lidar 2.
        if (!lidar_1_matches_javad && !lidars_match_each_other) {
            lidar_1_disagree_streak++;
            lidar_1_agree_streak = 0;

            if (lidar_1_health != SensorHealth::FAULTY && lidar_1_disagree_streak >= LIDAR_DIVERGENCE_STREAK_THRESHOLD) {
                lidar_1_health = SensorHealth::FAULTY;
                lidar_1_disagree_streak = 0;
            }
        }
        else {
            lidar_1_agree_streak++;
            lidar_1_disagree_streak = 0;

            if (lidar_1_health == SensorHealth::FAULTY && lidar_1_agree_streak >= LIDAR_DIVERGENCE_STREAK_THRESHOLD) {
                lidar_1_health = SensorHealth::HEALTHY;
                lidar_1_agree_streak = 0;
            }
        }

        // Lidar 2: disagrees with both Javad and Lidar 1.
        if (!lidar_2_matches_javad && !lidars_match_each_other) {
            lidar_2_disagree_streak++;
            lidar_2_agree_streak = 0;

            if (lidar_2_health != SensorHealth::FAULTY && lidar_2_disagree_streak >= LIDAR_DIVERGENCE_STREAK_THRESHOLD) {
                lidar_2_health = SensorHealth::FAULTY;
                lidar_2_disagree_streak = 0;
            }
        }
        else {
            lidar_2_agree_streak++;
            lidar_2_disagree_streak = 0;

            if (lidar_2_health == SensorHealth::FAULTY && lidar_2_agree_streak >= LIDAR_DIVERGENCE_STREAK_THRESHOLD) {
                lidar_2_health = SensorHealth::HEALTHY;
                lidar_2_agree_streak = 0;
            }
        }
    }
}

static void updateVerticalFromLidars(bool lidar_1_updated, bool lidar_2_updated)
{
    // Only healthy LiDARs feed the EKF. Runs after checkLidarDivergence(), so a LiDAR flagged
    // faulty on this exact tick is excluded immediately rather than one tick late.
    if (lidar_1_updated && lidar_1_health == SensorHealth::HEALTHY) {
        z_axis_ekf.update_altitude(lidar_1_altitude_m, LIDAR_1_ALTITUDE_R_VARIANCE_M2);
    }
    if (lidar_2_updated && lidar_2_health == SensorHealth::HEALTHY) {
        z_axis_ekf.update_altitude(lidar_2_altitude_m, LIDAR_2_ALTITUDE_R_VARIANCE_M2);
    }
}

static void publishVerticalState(const GnssReadings& gnss, bool gnss_updated)
{
    // position.z/velocity.z come from the EKF -- VN300 acceleration via predict(), LiDAR and Javad
    // via update(), with the vel_d fallback while Javad is faulty. USE_ZAXIS_EKF_FOR_ALTITUDE
    // switches back to raw GNSS passthrough for ground testing.
    if constexpr (USE_ZAXIS_EKF_FOR_ALTITUDE) {
        if (z_axis_ekf.is_valid()) {
            current_estimate.position.z = z_axis_ekf.z_m();
            current_estimate.velocity.z = z_axis_ekf.vz_ms();
        }
        else if (gnss_updated) {
            // No healthy GNSS reading yet -- pass raw GNSS through so the estimate isn't frozen
            // at 0 in the meantime.
            current_estimate.position.z = gnss.up_m;
            current_estimate.velocity.z = gnss.vz_ms;
        }
    }
    else if (gnss_updated) {
        current_estimate.position.z = gnss.up_m;
        current_estimate.velocity.z = gnss.vz_ms;
    }
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

    checkPrimarySensorFaults(imu, backup_imu_1, backup_imu_2, gnss, vn300_fresh);

    // After the health checks so the faulted-VN300 gate inside sees this tick's health, and still
    // before updateAttitude() so it keeps rotating by the previous tick's R_WB, as it always has.
    predictVerticalState(imu, vn300_fresh);

    updateAttitude(imu, backup_imu_1);
    bool gnss_updated = updateFromGnss(imu, gnss);

    bool lidar_1_updated = trackLidarFreshness(lidar_1, lidar_1_update_timestamp_ns);
    bool lidar_2_updated = trackLidarFreshness(lidar_2, lidar_2_update_timestamp_ns);
    projectLidarAltitudes(lidar_1, lidar_2, lidar_1_updated, lidar_2_updated);

    // Runs here rather than alongside the primary-sensor checks above: it compares this tick's
    // projected LiDAR altitudes, which depend on the attitude selected by updateAttitude().
    checkLidarDivergence(gnss, lidar_1_updated, lidar_2_updated);
    updateVerticalFromLidars(lidar_1_updated, lidar_2_updated);

    publishVerticalState(gnss, gnss_updated);

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
    return vn300_health == SensorHealth::HEALTHY;
}

bool StateEstimator::vn300_is_stale_for_testing()
{
    return vn300_health == SensorHealth::STALE;
}

bool StateEstimator::vn300_is_faulty_for_testing()
{
    return vn300_health == SensorHealth::FAULTY;
}

bool StateEstimator::javad_is_faulty_for_testing()
{
    return javad_health == SensorHealth::FAULTY;
}

bool StateEstimator::lidar_1_is_faulty_for_testing()
{
    return lidar_1_health == SensorHealth::FAULTY;
}

bool StateEstimator::lidar_2_is_faulty_for_testing()
{
    return lidar_2_health == SensorHealth::FAULTY;
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

