#ifndef APP_FLIGHT_STATE_ESTIMATOR_H
#define APP_FLIGHT_STATE_ESTIMATOR_H

#include "Error.h"
#include "clover.pb.h"
#include <cstdint>
#include <optional>

namespace StateEstimator {

    void init();
    void reset();
    std::optional<EstimatedState> estimate(
        LidarReading& lidar_1,
        LidarReading& lidar_2,
        ImuReading& imu,
        ImuReading& backup_imu_1,
        ImuReading& backup_imu_2,
        GnssReadings& gnss
    );

#if CONFIG_TEST
    // Overrides the wall-clock "now" used by the staleness check, instead of the real
    // k_cycle_get_64(). reset() clears the override (falls back to the real clock).
    void set_now_ns_for_testing(uint64_t ns);

    // Per-sensor health accessors. SensorHealth itself is internal to StateEstimator.cpp (not
    // part of the public interface), so these expose only the yes/no questions tests need.
    bool vn300_is_healthy_for_testing();
    bool vn300_is_stale_for_testing();
    bool vn300_is_faulty_for_testing();
    bool javad_is_faulty_for_testing();
    bool lidar_1_is_faulty_for_testing();
    bool lidar_2_is_faulty_for_testing();

    // Placeholder divergence-streak thresholds, so tests can assert "not flagged before N" /
    // "flagged at N" without duplicating the actual constants (which are expected to change once
    // real sensor noise specs are available).
    int vn300_divergence_streak_threshold_for_testing();
    int javad_divergence_streak_threshold_for_testing();
    int lidar_divergence_streak_threshold_for_testing();

    // Direct access to calculateVerticalAltitude(), which is otherwise static/internal to
    // StateEstimator.cpp.
    float calculate_vertical_altitude_for_testing(float slant_range_m, const Quaternion& attitude_wb, float mount_angle_rad);
#endif

}

#endif // APP_FLIGHT_STATE_ESTIMATOR_H
