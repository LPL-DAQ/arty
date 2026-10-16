#include "HornetTvc.h"
#include "../Controller.h"
#include "../MutexGuard.h"
#include "../config.h"
#include <zephyr/kernel.h>
#include <algorithm>
#include <cmath>

K_MUTEX_DEFINE(hornet_tvc_lock);

// Vehicle-specific physics for Hornet TVC
static constexpr float HORNET_YAW_MOI          = -67.67f;  // TODO: fill in real value
static constexpr float HORNET_PITCH_MOI         = -67.67f;  // TODO: fill in real value
static constexpr float HORNET_YAW_MOMENT_ARM    = -67.67f;  // TODO: fill in real value
static constexpr float HORNET_PITCH_MOMENT_ARM  = -67.67f;  // TODO: fill in real value
static constexpr float HORNET_MAX_GIMBLE_DEG    = 12.0f;

// Actuonix P16-50-64-12-P behind a LAC board. The LAC closes the position loop and maps its RC
// input pulse width linearly across the stroke, so MIN_PWM_PULSE_US is fully retracted and
// MAX_PWM_PULSE_US is fully extended.
static constexpr float HORNET_TVC_STROKE_MM        = 50.0f;
static constexpr float HORNET_TVC_MAX_SPEED_MM_S   = 18.0f;

// Measured at zero deflection in the Gimbal Assembly CAD: U-joint centre to P16 rod-end eyelet, and to the CLAM_P16 clamp hole.
static constexpr float HORNET_TVC_PIVOT_TO_LINK_MM  = 68.20f;
static constexpr float HORNET_TVC_PIVOT_TO_MOUNT_MM = 128.25f;
static constexpr float HORNET_TVC_NEUTRAL_ANGLE_DEG = 69.96f;

// Extension at zero gimbal. Mid-stroke leaves equal travel in both directions.
static constexpr float HORNET_TVC_NEUTRAL_EXTENSION_MM = HORNET_TVC_STROKE_MM / 2.0f;

static constexpr float HORNET_TVC_MIN_THRUST_N = 1e-3f;

static constexpr float PULSE_US_PER_MM = (MAX_PWM_PULSE_US - MIN_PWM_PULSE_US) / HORNET_TVC_STROKE_MM;

// Furthest the extension command may move in one tick without exceeding the actuator's no-load speed.
static constexpr float MAX_EXTENSION_STEP_MM = HORNET_TVC_MAX_SPEED_MM_S * Controller::SEC_PER_CONTROL_TICK;

static float pitch_extension_mm = HORNET_TVC_NEUTRAL_EXTENSION_MM;
static float yaw_extension_mm = HORNET_TVC_NEUTRAL_EXTENSION_MM;

/// Actuator extension required to hold a gimbal angle, from the linkage geometry above.
static float gimbal_deg_to_extension_mm(float gimbal_deg)
{
    const float link = HORNET_TVC_PIVOT_TO_LINK_MM;
    const float mount = HORNET_TVC_PIVOT_TO_MOUNT_MM;
    const float neutral_rad = HORNET_TVC_NEUTRAL_ANGLE_DEG * DEG2RAD_F;

    const float squares = link * link + mount * mount;
    const float cross = 2.0f * link * mount;

    const float length_mm = std::sqrt(squares - cross * std::cos(neutral_rad + gimbal_deg * DEG2RAD_F));
    const float neutral_length_mm = std::sqrt(squares - cross * std::cos(neutral_rad));

    return HORNET_TVC_NEUTRAL_EXTENSION_MM + (length_mm - neutral_length_mm);
}

/// Advance a held extension toward its target, capped by the actuator's maximum speed.
static float rate_limit_extension_mm(float target_mm, float& held_mm)
{
    const float step = std::clamp(target_mm - held_mm, -MAX_EXTENSION_STEP_MM, MAX_EXTENSION_STEP_MM);
    held_mm = std::clamp(held_mm + step, 0.0f, HORNET_TVC_STROKE_MM);
    return held_mm;
}

static uint32_t extension_mm_to_pulse_us(float extension_mm)
{
    const float clamped_mm = std::clamp(extension_mm, 0.0f, HORNET_TVC_STROKE_MM);
    return static_cast<uint32_t>(MIN_PWM_PULSE_US + (clamped_mm * PULSE_US_PER_MM));
}

/// Reset internal state before an active control trace
void HornetTvc::reset()
{
    MutexGuard hornet_tvc_guard{&hornet_tvc_lock};
    pitch_extension_mm = HORNET_TVC_NEUTRAL_EXTENSION_MM;
    yaw_extension_mm = HORNET_TVC_NEUTRAL_EXTENSION_MM;
}

uint32_t HornetTvc::neutral_pulse_us()
{
    return extension_mm_to_pulse_us(HORNET_TVC_NEUTRAL_EXTENSION_MM);
}

uint32_t HornetTvc::command_neutral()
{
    MutexGuard hornet_tvc_guard{&hornet_tvc_lock};
    pitch_extension_mm = HORNET_TVC_NEUTRAL_EXTENSION_MM;
    yaw_extension_mm = HORNET_TVC_NEUTRAL_EXTENSION_MM;
    return extension_mm_to_pulse_us(HORNET_TVC_NEUTRAL_EXTENSION_MM);
}

/// Clamp gimbal angles, convert to extension, rate limit and map to pulse width. Caller must hold hornet_tvc_lock.
static std::tuple<float, float, HornetTvcMetrics> command_gimbal_deg(float pitch_gimbal_deg, float yaw_gimbal_deg)
{
    pitch_gimbal_deg = std::clamp(pitch_gimbal_deg, -HORNET_MAX_GIMBLE_DEG, HORNET_MAX_GIMBLE_DEG);
    yaw_gimbal_deg   = std::clamp(yaw_gimbal_deg,   -HORNET_MAX_GIMBLE_DEG, HORNET_MAX_GIMBLE_DEG);

    const float pitch_target_mm = gimbal_deg_to_extension_mm(pitch_gimbal_deg);
    const float yaw_target_mm   = gimbal_deg_to_extension_mm(yaw_gimbal_deg);

    const float pitch_command_mm = rate_limit_extension_mm(pitch_target_mm, pitch_extension_mm);
    const float yaw_command_mm   = rate_limit_extension_mm(yaw_target_mm, yaw_extension_mm);

    const uint32_t pitch_pulse_us = extension_mm_to_pulse_us(pitch_command_mm);
    const uint32_t yaw_pulse_us   = extension_mm_to_pulse_us(yaw_command_mm);

    HornetTvcMetrics metrics = HornetTvcMetrics_init_default;
    metrics.commanded_pitch_gimbal_deg = pitch_gimbal_deg;
    metrics.commanded_yaw_gimbal_deg = yaw_gimbal_deg;
    metrics.target_pitch_extension_mm = pitch_target_mm;
    metrics.target_yaw_extension_mm = yaw_target_mm;
    metrics.commanded_pitch_extension_mm = pitch_command_mm;
    metrics.commanded_yaw_extension_mm = yaw_command_mm;
    metrics.pitch_pulse_us = pitch_pulse_us;
    metrics.yaw_pulse_us = yaw_pulse_us;

    return {pitch_pulse_us, yaw_pulse_us, metrics};
}

/// Convert angular acceleration command to actuator pulse width.
/// Physics: alpha = (T * arm * sin(gimbal)) / MOI  =>  gimbal = arcsin(alpha * MOI / (T * arm))
std::expected<std::tuple<float, float, HornetTvcMetrics>, Error> HornetTvc::tick(float pitch_accel_rad_s2, float yaw_accel_rad_s2, float thrust_N)
{
    // Written so that a NaN thrust also fails the check.
    if (!(std::fabs(thrust_N) >= HORNET_TVC_MIN_THRUST_N)) {
        return std::unexpected(Error::from_cause("thrust %f N is too small to compute a TVC gimbal angle", static_cast<double>(thrust_N)));
    }

    MutexGuard hornet_tvc_guard{&hornet_tvc_lock};

    float pitch_sin = (pitch_accel_rad_s2 * HORNET_PITCH_MOI) / (thrust_N * HORNET_PITCH_MOMENT_ARM);
    float yaw_sin   = (yaw_accel_rad_s2   * HORNET_YAW_MOI)   / (thrust_N * HORNET_YAW_MOMENT_ARM);

    pitch_sin = std::clamp(pitch_sin, -1.0f, 1.0f);
    yaw_sin   = std::clamp(yaw_sin,   -1.0f, 1.0f);

    return command_gimbal_deg(std::asin(pitch_sin) * RAD2DEG_F, std::asin(yaw_sin) * RAD2DEG_F);
}

std::expected<std::tuple<float, float, HornetTvcMetrics>, Error> HornetTvc::tick_gimbal_deg(float pitch_gimbal_deg, float yaw_gimbal_deg)
{
    MutexGuard hornet_tvc_guard{&hornet_tvc_lock};
    return command_gimbal_deg(pitch_gimbal_deg, yaw_gimbal_deg);
}
