#include "StateEstimator.h"
#include "Error.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(StateEstimator, LOG_LEVEL_INF);


static EstimatedState current_estimate = EstimatedState_init_default;
static float update_timestamp_ns = 0;
static bool has_update_timestamp_ns = false;
static float lidar_1_update_timestamp_ns = 0;
static float lidar_2_update_timestamp_ns = 0;
static float imu_update_timestamp_ns = 0;
static float gnss_update_timestamp_ns = 0;


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
    gnss_update_timestamp_ns = 0;
}

std::optional<EstimatedState> StateEstimator::estimate(
    LidarReading& lidar_1,
    LidarReading& lidar_2,
    ImuReading& imu,
    GnssReadings& gnss
)
{
    update_timestamp_ns = static_cast<float>(k_cycle_get_64()) / sys_clock_hw_cycles_per_sec() * 1e9f;
    has_update_timestamp_ns = true;

    bool lidar_1_updated = false;
    bool lidar_2_updated = false;
    bool gnss_updated = false;

    // IMU: update quaternion if timestamp changed.
    if (imu.sense_time_ns > imu_update_timestamp_ns) {
        imu_update_timestamp_ns = imu.sense_time_ns;
        current_estimate.R_WB.qw = imu.quat_w;
        current_estimate.R_WB.qx = imu.quat_x;
        current_estimate.R_WB.qy = imu.quat_y;
        current_estimate.R_WB.qz = imu.quat_z;
    }

    // GNSS: update x/y position and x/y/z velocity if timestamp changed.
    if (gnss.sense_time_ns > gnss_update_timestamp_ns) {
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

    // TODO: Z position filter
    {

    }

    return current_estimate;
}

