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
    // Overrides the wall-clock "now" used by the staleness check. reset() clears the override.
    void set_now_ns_for_testing(uint64_t ns);

    // SensorHealth is internal to StateEstimator.cpp, so these expose only the yes/no answers.
    bool vn300_is_healthy_for_testing();
    bool vn300_is_stale_for_testing();
    bool vn300_is_faulty_for_testing();
    bool javad_is_faulty_for_testing();
    bool lidar_1_is_faulty_for_testing();
    bool lidar_2_is_faulty_for_testing();

    // Lets tests assert "not flagged before N" / "flagged at N" without duplicating the constants,
    // which are placeholders expected to change once real sensor noise specs are available.
    int vn300_divergence_streak_threshold_for_testing();
    int javad_divergence_streak_threshold_for_testing();
    int lidar_divergence_streak_threshold_for_testing();

    // z_axis_ekf is file-static to StateEstimator.cpp; exposes its altitude variance so a test can
    // assert uncertainty rises while the state itself is held.
    float ekf_z_variance_for_testing();

    // calculateVerticalAltitude() is otherwise static to StateEstimator.cpp.
    float calculate_vertical_altitude_for_testing(float slant_range_m, const Quaternion& attitude_wb, float mount_angle_rad);
#endif

}

#endif // APP_FLIGHT_STATE_ESTIMATOR_H
