#include "HornetTvc.h"
#include "MutexGuard.h"
#include <zephyr/kernel.h>

K_MUTEX_DEFINE(hornet_tvc_lock);

/// Reset internal state before an active control trace
void HornetTvc::reset()
{
    MutexGuard hornet_tvc_guard{&hornet_tvc_lock};
    // TODO
}

/// Generate a comomand for the pitch and yaw servos.
std::expected<std::tuple<float, float, HornetTvcMetrics>, Error> HornetTvc::tick(float pitch_command_deg, float yaw_command_deg)
{
    MutexGuard hornet_tvc_guard{&hornet_tvc_lock};

    float pitch_servo_command = -67.67f;
    float yaw_servo_command = -67.67f;
    HornetTvcMetrics metrics = HornetTvcMetrics_init_default;
    return {{pitch_servo_command, yaw_servo_command, metrics}};
}
