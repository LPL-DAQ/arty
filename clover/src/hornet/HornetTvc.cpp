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
// input pulse width linearly across the stroke, between each axis's retract and extend pulses below.
static constexpr float HORNET_TVC_STROKE_MM        = 50.0f;
static constexpr float HORNET_TVC_MAX_SPEED_MM_S   = 18.0f;

// Measured at zero deflection in the Gimbal Assembly CAD: U-joint centre to P16 rod-end eyelet, and to the CLAM_P16 clamp hole.
static constexpr float HORNET_TVC_PIVOT_TO_LINK_MM  = 68.20f;
static constexpr float HORNET_TVC_PIVOT_TO_MOUNT_MM = 128.25f;
static constexpr float HORNET_TVC_NEUTRAL_ANGLE_DEG = 69.96f;

static constexpr float HORNET_TVC_MIN_THRUST_N = 1e-3f;

// Per-axis calibration. Set these from bench measurements; the logic below does not need to change.
// Direction +1 means a positive gimbal angle extends the actuator, -1 means it retracts.
static constexpr float HORNET_TVC_PITCH_NEUTRAL_EXTENSION_MM = 25.0f;
static constexpr float HORNET_TVC_YAW_NEUTRAL_EXTENSION_MM   = 25.0f;
static constexpr uint32_t HORNET_TVC_PITCH_RETRACT_PULSE_US  = 1000;
static constexpr uint32_t HORNET_TVC_PITCH_EXTEND_PULSE_US   = 2000;
static constexpr uint32_t HORNET_TVC_YAW_RETRACT_PULSE_US    = 1000;
static constexpr uint32_t HORNET_TVC_YAW_EXTEND_PULSE_US     = 2000;
static constexpr float HORNET_TVC_PITCH_DIRECTION            = 1.0f;
static constexpr float HORNET_TVC_YAW_DIRECTION              = 1.0f;

static constexpr HornetTvc::AxisCalibration PITCH_CALIBRATION{
    HORNET_TVC_PITCH_NEUTRAL_EXTENSION_MM, HORNET_TVC_PITCH_RETRACT_PULSE_US, HORNET_TVC_PITCH_EXTEND_PULSE_US, HORNET_TVC_PITCH_DIRECTION};
static constexpr HornetTvc::AxisCalibration YAW_CALIBRATION{
    HORNET_TVC_YAW_NEUTRAL_EXTENSION_MM, HORNET_TVC_YAW_RETRACT_PULSE_US, HORNET_TVC_YAW_EXTEND_PULSE_US, HORNET_TVC_YAW_DIRECTION};

static constexpr bool calibration_is_valid(const HornetTvc::AxisCalibration& cal)
{
    const auto pulse_ok = [](uint32_t pulse_us) { return pulse_us >= MIN_PWM_PULSE_US && pulse_us <= MAX_PWM_PULSE_US; };
    return cal.neutral_extension_mm >= 0.0f && cal.neutral_extension_mm <= HORNET_TVC_STROKE_MM && pulse_ok(cal.retract_pulse_us) &&
           pulse_ok(cal.extend_pulse_us) && cal.retract_pulse_us != cal.extend_pulse_us && (cal.direction == 1.0f || cal.direction == -1.0f);
}
static_assert(calibration_is_valid(PITCH_CALIBRATION));
static_assert(calibration_is_valid(YAW_CALIBRATION));

// Furthest the extension command may move in one tick without exceeding the actuator's no-load speed.
static constexpr float MAX_EXTENSION_STEP_MM = HORNET_TVC_MAX_SPEED_MM_S * Controller::SEC_PER_CONTROL_TICK;

static float pitch_extension_mm = PITCH_CALIBRATION.neutral_extension_mm;
static float yaw_extension_mm = YAW_CALIBRATION.neutral_extension_mm;

float HornetTvc::gimbal_deg_to_extension_mm(float gimbal_deg, const AxisCalibration& cal)
{
    const float link = HORNET_TVC_PIVOT_TO_LINK_MM;
    const float mount = HORNET_TVC_PIVOT_TO_MOUNT_MM;
    const float neutral_rad = HORNET_TVC_NEUTRAL_ANGLE_DEG * DEG2RAD_F;

    const float squares = link * link + mount * mount;
    const float cross = 2.0f * link * mount;

    const float linkage_deg = cal.direction * gimbal_deg;
    const float length_mm = std::sqrt(squares - cross * std::cos(neutral_rad + linkage_deg * DEG2RAD_F));
    const float neutral_length_mm = std::sqrt(squares - cross * std::cos(neutral_rad));

    return cal.neutral_extension_mm + (length_mm - neutral_length_mm);
}

/// Advance a held extension toward its target, capped by the actuator's maximum speed.
static float rate_limit_extension_mm(float target_mm, float& held_mm)
{
    const float step = std::clamp(target_mm - held_mm, -MAX_EXTENSION_STEP_MM, MAX_EXTENSION_STEP_MM);
    held_mm = std::clamp(held_mm + step, 0.0f, HORNET_TVC_STROKE_MM);
    return held_mm;
}

uint32_t HornetTvc::extension_mm_to_pulse_us(float extension_mm, const AxisCalibration& cal)
{
    const float clamped_mm = std::clamp(extension_mm, 0.0f, HORNET_TVC_STROKE_MM);
    const float retract_us = static_cast<float>(cal.retract_pulse_us);
    const float extend_us = static_cast<float>(cal.extend_pulse_us);
    const float pulse_us = retract_us + clamped_mm * ((extend_us - retract_us) / HORNET_TVC_STROKE_MM);

    const long low_us = std::max<long>(std::min(cal.retract_pulse_us, cal.extend_pulse_us), MIN_PWM_PULSE_US);
    const long high_us = std::min<long>(std::max(cal.retract_pulse_us, cal.extend_pulse_us), MAX_PWM_PULSE_US);
    return static_cast<uint32_t>(std::clamp(std::lround(pulse_us), low_us, high_us));
}

/// Reset internal state before an active control trace
void HornetTvc::reset()
{
    MutexGuard hornet_tvc_guard{&hornet_tvc_lock};
    pitch_extension_mm = PITCH_CALIBRATION.neutral_extension_mm;
    yaw_extension_mm = YAW_CALIBRATION.neutral_extension_mm;
}

uint32_t HornetTvc::neutral_pitch_pulse_us()
{
    return extension_mm_to_pulse_us(PITCH_CALIBRATION.neutral_extension_mm, PITCH_CALIBRATION);
}

uint32_t HornetTvc::neutral_yaw_pulse_us()
{
    return extension_mm_to_pulse_us(YAW_CALIBRATION.neutral_extension_mm, YAW_CALIBRATION);
}

std::tuple<uint32_t, uint32_t> HornetTvc::command_neutral()
{
    MutexGuard hornet_tvc_guard{&hornet_tvc_lock};
    pitch_extension_mm = PITCH_CALIBRATION.neutral_extension_mm;
    yaw_extension_mm = YAW_CALIBRATION.neutral_extension_mm;
    return {neutral_pitch_pulse_us(), neutral_yaw_pulse_us()};
}

/// Clamp gimbal angles, convert to extension, rate limit and map to pulse width. Caller must hold hornet_tvc_lock.
static std::tuple<float, float, HornetTvcMetrics> command_gimbal_deg(float pitch_gimbal_deg, float yaw_gimbal_deg)
{
    pitch_gimbal_deg = std::clamp(pitch_gimbal_deg, -HORNET_MAX_GIMBLE_DEG, HORNET_MAX_GIMBLE_DEG);
    yaw_gimbal_deg   = std::clamp(yaw_gimbal_deg,   -HORNET_MAX_GIMBLE_DEG, HORNET_MAX_GIMBLE_DEG);

    const float pitch_target_mm = HornetTvc::gimbal_deg_to_extension_mm(pitch_gimbal_deg, PITCH_CALIBRATION);
    const float yaw_target_mm   = HornetTvc::gimbal_deg_to_extension_mm(yaw_gimbal_deg, YAW_CALIBRATION);

    const float pitch_command_mm = rate_limit_extension_mm(pitch_target_mm, pitch_extension_mm);
    const float yaw_command_mm   = rate_limit_extension_mm(yaw_target_mm, yaw_extension_mm);

    const uint32_t pitch_pulse_us = HornetTvc::extension_mm_to_pulse_us(pitch_command_mm, PITCH_CALIBRATION);
    const uint32_t yaw_pulse_us   = HornetTvc::extension_mm_to_pulse_us(yaw_command_mm, YAW_CALIBRATION);

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
