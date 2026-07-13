#include "StateEstimator.h"
#include "Error.h"
#include "../config.h"
#include "../math_util.h"
#include <cmath>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(StateEstimator, LOG_LEVEL_INF);


// ── Design notes ────────────────────────────────────────────────────────────────────────────
//
// Why a streak (N consecutive readings) instead of acting on a single sample: a lone
// mismatch/match could just be sensor noise, a one-tick timing skew between independently-clocked
// sensors, or a transient glitch. Acting on it immediately would flap the health state on noise
// alone. Requiring N consecutive readings before flipping in EITHER direction means a fault only
// gets declared once it's a persistent pattern rather than a fluke, and a recovery is only trusted
// once it's persistent too -- see VN300_DIVERGENCE_STREAK_THRESHOLD / JAVAD_DIVERGENCE_STREAK_
// THRESHOLD / LIDAR_DIVERGENCE_STREAK_THRESHOLD below, and every *_disagree_streak/*_agree_streak
// pair.
//
// Why STALE recovers on a TIMER but FAULTY recovers on a STREAK: these are different physical
// situations, not just two ways of writing the same idea. STALE means "no data at all" -- once
// data resumes, the sensor's own onboard filter needs real wall-clock time to re-converge (see
// VN300_RETRUST_THRESHOLD_NS), so recovery is gated on elapsed time, not sample count, because
// trusting the very first post-outage sample would mean trusting a filter that hasn't caught up
// yet. FAULTY means "data is arriving but disagrees with corroborating sensors" -- there's no
// analogous convergence process to wait out, just a need for enough independent samples to be
// confident the disagreement (or its resolution) is real rather than noise, so recovery is gated
// on a sample count instead. VN300 is the only sensor here with a staleness concept at all
// (Javad/LiDAR don't have one in this file -- see their SensorHealth declarations below), so it's
// also the only one with both recovery mechanisms in play at once.

// Health state for a fused sensor. Generic so it can be reused for other redundant sensors as
// backup hardware comes online -- currently only tracked for VN300, Javad, and the two LiDARs.
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

// VN300 updates at ~400Hz (2.5ms period). If this long passes without a new reading arriving,
// mark it stale and stop trusting its attitude output -- 50ms is ~20 missed readings.
static constexpr uint64_t VN300_STALE_THRESHOLD_NS = 50'000'000;

// After an outage, require this much continuous good data before trusting VN300 again -- 100ms
// (~40 consecutive updates at 400Hz) gives its onboard filter time to re-converge, since a reading
// right after a dropout isn't as reliable as one from a filter that's been running steadily.
//
// Deliberately 2x the stale threshold, not the same value: declaring stale should be quick (any
// gap this size already means ~20 missed samples, a clear and cheap-to-detect problem, and staying
// STALE briefly costs little), but declaring "trustworthy again" should be conservative, since
// trusting attitude data too early risks feeding a not-yet-converged filter into flight control.
// The two thresholds encode "fail fast, recover cautiously" rather than a single symmetric window.
static constexpr uint64_t VN300_RETRUST_THRESHOLD_NS = 100'000'000;

// Placeholder -- needs real backup IMU noise specs before flight. Max per-component quaternion
// difference allowed between the two backup IMUs to consider them in agreement with each other.
static constexpr float BACKUP_IMU_AGREEMENT_THRESHOLD = 0.05f;

// Placeholder -- needs real backup IMU noise specs before flight. Max per-component quaternion
// difference allowed between VN300 and a backup IMU before considering them in disagreement.
static constexpr float VN300_DIVERGENCE_THRESHOLD = 0.05f;

// Placeholder -- needs real backup IMU noise specs before flight. Number of consecutive
// disagreeing readings (backups agree with each other but not with VN300) required to flag VN300
// as faulty, and consecutive agreeing readings required to un-flag it -- symmetric N in both
// directions so a single noisy sample can't flip the flag either way (avoids flapping).
static constexpr int VN300_DIVERGENCE_STREAK_THRESHOLD = 10;

// Placeholder -- needs real GNSS/VN300 velocity noise + frame-alignment analysis before flight
// (GNSS vx/vy/vz and VN300 vel_n/vel_e/vel_d aren't guaranteed to be directly comparable without
// a proper frame transform). Max per-axis velocity difference (m/s) before Javad and VN300 count
// as disagreeing.
static constexpr float JAVAD_VN300_VELOCITY_DIVERGENCE_THRESHOLD = 1.0f;

// Placeholder -- needs real GNSS/VN300 velocity noise analysis before flight. Consecutive
// disagreeing GNSS refreshes required to flag Javad as faulty, and consecutive agreeing
// refreshes required to un-flag it -- symmetric N in both directions to avoid flapping.
static constexpr int JAVAD_DIVERGENCE_STREAK_THRESHOLD = 10;

// Placeholder -- needs real LiDAR/GNSS altitude noise analysis before flight. Max altitude
// difference (m) before a LiDAR and Javad, or the two LiDARs, count as disagreeing.
static constexpr float LIDAR_ALTITUDE_DIVERGENCE_THRESHOLD = 1.0f;

// Placeholder -- needs real LiDAR/GNSS altitude noise analysis before flight. Consecutive
// disagreeing refreshes required to flag a LiDAR as faulty, and consecutive agreeing refreshes
// required to un-flag it -- symmetric N in both directions to avoid flapping.
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

// Javad has no dedicated staleness tracking here (out of scope for this check), so its health
// only ever toggles HEALTHY <-> FAULTY via the velocity divergence check below -- it starts
// HEALTHY (rather than STALE, like VN300 does) because there's no timer-based signal that would
// ever move it out of a STALE state in the first place; an "unknown until proven otherwise"
// default would just be dead state with no way to leave it.
static SensorHealth javad_health = SensorHealth::HEALTHY;
static int javad_disagree_streak = 0;
static int javad_agree_streak = 0;

// LiDARs have no dedicated staleness tracking here either, so their health only ever toggles
// HEALTHY <-> FAULTY via the altitude divergence check below (same HEALTHY-default reasoning as
// Javad above). Deliberately not consumed by current_estimate or fault_status yet -- see the TODO
// on the LiDAR altitude fusion below. This is a staged rollout: the health-tracking detector is
// independently useful and testable on its own, and doesn't need the (harder, more consequential)
// fusion-into-position.z policy decided first. Wiring a flagged-faulty LiDAR into the actual
// position estimate before that policy exists would mean guessing at a flight-safety-relevant
// behavior instead of having it decided explicitly.
static SensorHealth lidar_1_health = SensorHealth::HEALTHY;
static int lidar_1_disagree_streak = 0;
static int lidar_1_agree_streak = 0;
static SensorHealth lidar_2_health = SensorHealth::HEALTHY;
static int lidar_2_disagree_streak = 0;
static int lidar_2_agree_streak = 0;

// These two helpers answer "do these two readings agree right now" for a single tick -- they are
// NOT the fault-detection logic themselves. The disagree/agree streak counters below are what
// turn a sequence of these per-tick answers into a flag (see the "Design notes" comment above for
// why a streak is used instead of acting on one answer directly).
//
// Placeholder comparison -- max per-component absolute difference between two quaternions.
// Good enough until real backup IMU noise specs are available to design a proper metric.
static bool quats_agree(
    float aw, float ax, float ay, float az, float bw, float bx, float by, float bz, float threshold
)
{
    return std::fabs(aw - bw) < threshold && std::fabs(ax - bx) < threshold && std::fabs(ay - by) < threshold &&
           std::fabs(az - bz) < threshold;
}

// Placeholder comparison -- max per-axis absolute difference between two 3-vectors. Same caveats
// as quats_agree: good enough until real sensor/frame analysis is available.
static bool vec3_close(float ax, float ay, float az, float bx, float by, float bz, float threshold)
{
    return std::fabs(ax - bx) < threshold && std::fabs(ay - by) < threshold && std::fabs(az - bz) < threshold;
}

// Converts a LiDAR slant range into true vertical altitude (height above whatever the beam hit),
// accounting for both the vehicle's current attitude and the LiDAR's fixed body-frame mount
// angle.
//
// Frame conventions, verified against the rest of this codebase rather than assumed:
//  - World frame is Z-up: GnssReadings.up_m is assigned straight into EstimatedState.position.z
//    elsewhere in this file, and FlightController.cpp treats a rotated body vector's world-frame
//    Z component as "aligned with world up" when computing tilt (atan2(x, z) / atan2(y, z)).
//  - attitude_wb (matches EstimatedState.R_WB / ImuReading.quat_*) is WORLD-to-BODY: rotating a
//    body-frame vector INTO world frame requires its conjugate, not attitude_wb directly -- see
//    FlightController.cpp's `q_bw = conjugateQuaternion(q_wb)` before rotating unitZ() into world
//    frame. Getting this backwards would silently invert the tilt correction.
//
// ASSUMPTION pending confirmation, alongside the placeholder mount angle value itself: the
// boresight points straight down (body -Z) when mount_angle_rad is 0, tilted by mount_angle_rad
// in the body X-Z plane. Confirm this matches the real LiDAR mounting geometry once known.
//
// Both the VALUE (LIDAR_MOUNT_ANGLE_DEG, currently 0 as a placeholder in config.h) and this AXIS
// CONVENTION need flagging, not just the value: a wrong angle produces an altitude that's off by a
// bounded, checkable amount, but a wrong axis/plane assumption would silently produce a
// plausible-looking altitude that's wrong in a way nothing here would catch -- there's no
// "obviously broken" signal to notice, unlike e.g. a NaN or an out-of-range value.
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
    vn300_health = SensorHealth::STALE;
    vn300_last_arrival_time_ns = 0;
    vn300_recovery_start_time_ns = 0;
    vn300_recovering = false;
    vn300_disagree_streak = 0;
    vn300_agree_streak = 0;
    javad_health = SensorHealth::HEALTHY;
    javad_disagree_streak = 0;
    javad_agree_streak = 0;
    lidar_1_health = SensorHealth::HEALTHY;
    lidar_1_disagree_streak = 0;
    lidar_1_agree_streak = 0;
    lidar_2_health = SensorHealth::HEALTHY;
    lidar_2_disagree_streak = 0;
    lidar_2_agree_streak = 0;
#if CONFIG_TEST
    now_ns_override_set = false;
#endif
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

    bool lidar_1_updated = false;
    bool lidar_2_updated = false;
    bool gnss_updated = false;

    // VN300 staleness / re-trust tracking, based on wall-clock arrival time rather than
    // sense_time_ns (an inter-arrival delta, not usable for staleness checks).
    if (imu.has_arrival_time_ns && imu.arrival_time_ns > vn300_last_arrival_time_ns) {
        vn300_last_arrival_time_ns = imu.arrival_time_ns;

        if (vn300_health == SensorHealth::STALE) {
            if (!vn300_recovering) {
                // First good reading since the outage began -- start the re-convergence window.
                vn300_recovering = true;
                vn300_recovery_start_time_ns = imu.arrival_time_ns;
            }
            else if (imu.arrival_time_ns - vn300_recovery_start_time_ns >= VN300_RETRUST_THRESHOLD_NS) {
                vn300_health = SensorHealth::HEALTHY;
                vn300_recovering = false;
            }
        }
    }

#if CONFIG_TEST
    uint64_t now_ns = now_ns_override_set ? now_ns_override : k_cyc_to_ns_near64(k_cycle_get_64());
#else
    uint64_t now_ns = k_cyc_to_ns_near64(k_cycle_get_64());
#endif
    if (now_ns - vn300_last_arrival_time_ns > VN300_STALE_THRESHOLD_NS) {
        // No new reading in too long -- mark stale and reset any in-progress re-convergence, since
        // the "consecutive good data" requirement must restart from scratch after another dropout.
        // Takes precedence over a FAULTY flag: with no data at all there's nothing left to diverge.
        vn300_health = SensorHealth::STALE;
        vn300_recovering = false;
    }

    // VN300-vs-backup-IMU divergence check. Requires the two backups to agree with each other
    // FIRST, then checks whether VN300 disagrees with both -- this two-step structure (corroborate
    // the references, then compare against them) is what lets the check tell "VN300 is wrong" apart
    // from "one of the backups is wrong": if the backups themselves disagreed, a VN300-vs-backup
    // mismatch wouldn't say anything about which sensor is actually at fault. Only meaningful when
    // all three produced a genuinely new reading this tick -- with no real backup hardware yet,
    // has_arrival_time_ns on the backups is never set, so this never fires today, but is ready for
    // when it does.
    if (imu.has_arrival_time_ns && backup_imu_1.has_arrival_time_ns && backup_imu_2.has_arrival_time_ns) {
        bool backups_agree = quats_agree(
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
        bool vn300_matches_backup_1 = quats_agree(
            imu.quat_w, imu.quat_x, imu.quat_y, imu.quat_z,
            backup_imu_1.quat_w, backup_imu_1.quat_x, backup_imu_1.quat_y, backup_imu_1.quat_z,
            VN300_DIVERGENCE_THRESHOLD
        );
        bool vn300_matches_backup_2 = quats_agree(
            imu.quat_w, imu.quat_x, imu.quat_y, imu.quat_z,
            backup_imu_2.quat_w, backup_imu_2.quat_x, backup_imu_2.quat_y, backup_imu_2.quat_z,
            VN300_DIVERGENCE_THRESHOLD
        );

        if (backups_agree && !vn300_matches_backup_1 && !vn300_matches_backup_2) {
            vn300_disagree_streak++;
            vn300_agree_streak = 0;

            if (vn300_health != SensorHealth::FAULTY && vn300_disagree_streak >= VN300_DIVERGENCE_STREAK_THRESHOLD) {
                vn300_health = SensorHealth::FAULTY;
                vn300_disagree_streak = 0;
            }
        }
        else {
            vn300_agree_streak++;
            vn300_disagree_streak = 0;

            if (vn300_health == SensorHealth::FAULTY && vn300_agree_streak >= VN300_DIVERGENCE_STREAK_THRESHOLD) {
                vn300_health = SensorHealth::HEALTHY;
                vn300_agree_streak = 0;
            }
        }
    }

    // Javad-vs-VN300 velocity divergence check. Only runs while VN300 isn't already flagged --
    // if VN300 itself is untrustworthy, a mismatch tells us nothing about Javad. Compares GNSS
    // Cartesian velocity (vx/vy/vz) against VN300's INS-derived velocity (vel_n/vel_e/vel_d); see
    // the frame-alignment caveat on JAVAD_VN300_VELOCITY_DIVERGENCE_THRESHOLD above.
    if (vn300_health == SensorHealth::HEALTHY && gnss.has_arrival_time_ns && imu.has_arrival_time_ns &&
        imu.has_vel_n && imu.has_vel_e && imu.has_vel_d) {
        bool velocities_agree = vec3_close(
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

    // Combined fault status, carried directly on the returned EstimatedState so any consumer
    // (FlightController, telemetry, ...) can distinguish "one primary sensor down" from "both
    // primary sensors down" by reading this single field -- rather than re-deriving it themselves
    // from vn300_health/javad_health, which are internal to this file and not visible outside it.
    //
    // CRITICAL requires both VN300 AND Javad to be specifically FAULTY (not merely STALE) -- if
    // VN300 and Javad are BOTH flagged faulty at once, there's no trustworthy attitude or velocity
    // source left to fall back to. Documented default, not a confirmed requirement: hold the last
    // known-good estimate and raise a distinct critical fault rather than silently degrading to
    // whichever sensor happens to still be "less wrong." Open question: should CRITICAL instead
    // fall back to the less-wrong sensor rather than freezing entirely?
    bool vn300_ok = vn300_health == SensorHealth::HEALTHY;
    bool javad_ok = javad_health == SensorHealth::HEALTHY;
    bool critical_fault = vn300_health == SensorHealth::FAULTY && javad_health == SensorHealth::FAULTY;

    if (critical_fault) {
        current_estimate.fault_status = EstimatorFaultStatus_CRITICAL;
    }
    else if (!vn300_ok || !javad_ok) {
        // Exactly one primary sensor down (VN300 stale/faulty, or Javad faulty) -- the estimate is
        // still updating via fallback or partial freeze, not fully nominal but not critical either.
        current_estimate.fault_status = EstimatorFaultStatus_DEGRADED;
    }
    else {
        current_estimate.fault_status = EstimatorFaultStatus_NOMINAL;
    }

    // IMU: update quaternion from VN300 while healthy. Once flagged faulty by the divergence
    // check, switch to backup IMU 1 (backup_imu_2 is redundant with it once corroborated above).
    // A merely STALE VN300 still just freezes the last estimate, unchanged from before. Under a
    // critical fault, freeze entirely -- don't even fall back to backup_imu_1.
    if (!critical_fault) {
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

    // GNSS: update x/y position and x/y/z velocity if timestamp changed, unless Javad is flagged
    // faulty (freeze last known-good position/velocity instead). javad_health == FAULTY already
    // implies !critical_fault doesn't matter here: if Javad is faulty this block is skipped either
    // way, whether or not VN300 is also faulty.
    if (javad_health != SensorHealth::FAULTY && gnss.sense_time_ns > gnss_update_timestamp_ns) {
        gnss_update_timestamp_ns = gnss.sense_time_ns;
        gnss_updated = true;
        current_estimate.position.x = gnss.north_m;
        current_estimate.position.y = gnss.east_m;
        current_estimate.position.z = gnss.up_m; // will remove after adding filter
        current_estimate.velocity.x = gnss.vx_ms;
        current_estimate.velocity.y = gnss.vy_ms;
        current_estimate.velocity.z = gnss.vz_ms;
    }

    // Lidar 1: track timestamp.
    if (lidar_1.sense_time_ns > lidar_1_update_timestamp_ns) {
        lidar_1_update_timestamp_ns = lidar_1.sense_time_ns;
        lidar_1_updated = true;
    }

    // Lidar 2: track timestamp.
    if (lidar_2.sense_time_ns > lidar_2_update_timestamp_ns) {
        lidar_2_update_timestamp_ns = lidar_2.sense_time_ns;
        lidar_2_updated = true;
    }

    // Z position filter. calculateVerticalAltitude() converts each LiDAR's slant range into true
    // vertical altitude using the vehicle's current (fused/fallback-aware) attitude -- current_estimate.R_WB
    // rather than raw VN300 output, so this automatically respects the VN300/backup-IMU health
    // tracking above instead of trusting a flagged-faulty VN300 quaternion directly.
    //
    // TODO: fuse lidar_1_altitude_m / lidar_2_altitude_m with GNSS-derived altitude
    // (current_estimate.position.z, set from gnss.up_m above) -- not yet implemented, since a real
    // fusion strategy (LiDAR range limits, weighting, disagreement handling) hasn't been decided.
    if (lidar_1_updated) {
        lidar_1_altitude_m = calculateVerticalAltitude(lidar_1.distance_m, current_estimate.R_WB, LIDAR_MOUNT_ANGLE_DEG * DEG2RAD_F);
    }
    if (lidar_2_updated) {
        lidar_2_altitude_m = calculateVerticalAltitude(lidar_2.distance_m, current_estimate.R_WB, LIDAR_MOUNT_ANGLE_DEG * DEG2RAD_F);
    }

    // LiDAR-vs-Javad-vs-other-LiDAR altitude divergence check. Structurally different from the
    // VN300-vs-backups check above: instead of first requiring two references to agree and then
    // checking the third against them, this evaluates each LiDAR independently against the other
    // two ("does THIS LiDAR disagree with both of the others"). With three sensors this is
    // equivalent in the single-fault case (a healthy LiDAR will agree with Javad, so its own check
    // never trips) while also degrading sensibly if all three disagree with each other at once
    // (both LiDARs would get flagged, rather than neither, which is the more honest outcome when
    // nothing can be corroborated). Only meaningful when all three produced a fresh reading this
    // tick -- same synchronization simplification as the other divergence checks above. Unlike the
    // Javad-vs-VN300 check, this doesn't require Javad to be currently healthy first; not asked for
    // here, and with three independent altitude sources a 2-of-3 comparison is still informative
    // even if Javad itself is flagged.
    if (lidar_1_updated && lidar_2_updated && gnss.has_arrival_time_ns) {
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

float StateEstimator::calculate_vertical_altitude_for_testing(float slant_range_m, const Quaternion& attitude_wb, float mount_angle_rad)
{
    return calculateVerticalAltitude(slant_range_m, attitude_wb, mount_angle_rad);
}
#endif

