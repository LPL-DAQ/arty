#include "HornetTvc.h"
#include "../MutexGuard.h"
#include "../config.h"
#include <zephyr/kernel.h>
#include <cmath>

K_MUTEX_DEFINE(hornet_tvc_lock);

// Vehicle-specific physics for Hornet TVC
static constexpr float HORNET_YAW_MOI          = -67.67f;  // TODO: fill in real value
static constexpr float HORNET_PITCH_MOI         = -67.67f;  // TODO: fill in real value
static constexpr float HORNET_YAW_MOMENT_ARM    = -67.67f;  // TODO: fill in real value
static constexpr float HORNET_PITCH_MOMENT_ARM  = -67.67f;  // TODO: fill in real value
static constexpr float HORNET_MAX_GIMBLE_DEG    = 12.0f;

static constexpr float HORNET_TVC_R_GIMBLE = 1.0f;  // TODO: fill in real value
static constexpr float HORNET_TVC_R_SERVO  = 1.0f;  // TODO: fill in real value

// Servo centre pulse (0 deg) in microseconds
static constexpr uint32_t SERVO_CENTER_PULSE_US = (MIN_PWM_PULSE_US + MAX_PWM_PULSE_US) / 2;  // 1500 µs
// Scale: degrees per microsecond offset from centre  (range ±90° over ±500 µs)
static constexpr float SERVO_US_PER_DEG = (MAX_PWM_PULSE_US - SERVO_CENTER_PULSE_US) / 90.0f;

/// Reset internal state before an active control trace
void HornetTvc::reset()
{
    MutexGuard hornet_tvc_guard{&hornet_tvc_lock};
}

/// Convert angular acceleration command to servo pulse width.
/// Physics: alpha = (T * arm * sin(gimbal)) / MOI  =>  gimbal = arcsin(alpha * MOI / (T * arm))
/// Then gimbal angle is converted to servo angle via linkage geometry.
std::expected<std::tuple<float, float, HornetTvcMetrics>, Error> HornetTvc::tick(float pitch_accel_rad_s2, float yaw_accel_rad_s2, float thrust_N)
{
    MutexGuard hornet_tvc_guard{&hornet_tvc_lock};


    // Compute gimbal angles from desired angular accelerations
    float pitch_sin = (pitch_accel_rad_s2 * HORNET_PITCH_MOI) / (thrust_N * HORNET_PITCH_MOMENT_ARM);
    float yaw_sin   = (yaw_accel_rad_s2   * HORNET_YAW_MOI)   / (thrust_N * HORNET_YAW_MOMENT_ARM);

    pitch_sin = std::clamp(pitch_sin, -1.0f, 1.0f);
    yaw_sin   = std::clamp(yaw_sin,   -1.0f, 1.0f);

    float pitch_gimbal_deg = std::asin(pitch_sin) * RAD2DEG_F;
    float yaw_gimbal_deg   = std::asin(yaw_sin)   * RAD2DEG_F;

    pitch_gimbal_deg = std::clamp(pitch_gimbal_deg, -HORNET_MAX_GIMBLE_DEG, HORNET_MAX_GIMBLE_DEG);
    yaw_gimbal_deg   = std::clamp(yaw_gimbal_deg,   -HORNET_MAX_GIMBLE_DEG, HORNET_MAX_GIMBLE_DEG);

    // TODO: plug in the real inverse kinematics
    // Convert gimbal angle to servo angle via linkage geometry
    float servo_pitch_angle = std::asin(HORNET_TVC_R_GIMBLE / HORNET_TVC_R_SERVO)
                              * std::sin(pitch_gimbal_deg * DEG2RAD_F) * RAD2DEG_F;
    float servo_yaw_angle   = std::asin(HORNET_TVC_R_GIMBLE / HORNET_TVC_R_SERVO)
                              * std::sin(yaw_gimbal_deg   * DEG2RAD_F) * RAD2DEG_F;

    servo_pitch_angle = std::clamp(servo_pitch_angle, -90.0f, 90.0f);
    servo_yaw_angle   = std::clamp(servo_yaw_angle,   -90.0f, 90.0f);

    // Convert angle to pulse width: -90° -> 1000 µs, 0° -> 1500 µs, +90° -> 2000 µs
    uint32_t pitch_pulse_us = static_cast<uint32_t>(SERVO_CENTER_PULSE_US + (servo_pitch_angle * SERVO_US_PER_DEG));
    uint32_t yaw_pulse_us   = static_cast<uint32_t>(SERVO_CENTER_PULSE_US + (servo_yaw_angle   * SERVO_US_PER_DEG));

    HornetTvcMetrics metrics = HornetTvcMetrics_init_default;
    return {{pitch_pulse_us, yaw_pulse_us, metrics}};
}
