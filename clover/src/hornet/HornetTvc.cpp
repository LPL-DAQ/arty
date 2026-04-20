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
    float r_gimble = 1.0f;
    float r_servo = 1.0f;

    float servo_pitch_angle = std::asin(r_gimble / r_servo) * sin(pitch_command_deg);
    float servo_yaw_angle = std::asin(r_gimble / r_servo) * sin(yaw_command_deg);


    servo_pitch_angle = std::clamp(servo_pitch_angle, -90.0f, 90.0f);
    servo_yaw_angle = std::clamp(servo_yaw_angle, -90.0f, 90.0f);
    // Convert angle to pulse width
    // Mapping: -90° -> 1000µs, 0° -> 1500µs, +90° -> 2000µs
    uint32_t pitch_pulse_us = static_cast<uint32_t>(1500.0f + (servo_pitch_angle * 5.55556f));
    uint32_t yaw_pulse_us = static_cast<uint32_t>(1500.0f + (servo_yaw_angle * 5.555556f));

    HornetTvcMetrics metrics = HornetTvcMetrics_init_default;
    return {{pitch_pulse_us, yaw_pulse_us, metrics}};
}
