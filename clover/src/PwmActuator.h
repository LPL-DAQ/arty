#ifndef ARTY_PWM_ACTUATOR_ZEPHYR_H
#define ARTY_PWM_ACTUATOR_ZEPHYR_H

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <expected>
#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "Error.h"

enum class ServoKind {
    SERVO_X,
    SERVO_Y
};

enum class MotorKind {
    BETA_TOP,
    BETA_BOTTOM,
    BETA_CW,
    BETA_CCW,
    MOTOR_TOP,
    MOTOR_BOTTOM
};

// -----------------------------------------------------------------------------
// PwmActuator
// -----------------------------------------------------------------------------
// Stateless PWM base with last-commanded pulse tracking.
// init() verifies hardware and writes the low/min pulse.
// tick() clamps and writes a pulse. Requires init() to have succeeded first.
// -----------------------------------------------------------------------------
template<
    const pwm_dt_spec PwmDt,
    int MinPulseUs,
    int MaxPulseUs>
class PwmActuator {
protected:
    static constexpr float k_min_pulse_us = static_cast<float>(MinPulseUs);
    static constexpr float k_max_pulse_us = static_cast<float>(MaxPulseUs);
    static constexpr const pwm_dt_spec& pwm_gpio = PwmDt;

    inline static uint32_t last_pulse_us = static_cast<uint32_t>(MinPulseUs);
    inline static bool initialized = false;

    static uint32_t clamp_pulse_us(uint32_t pulse_us) {
        const uint32_t min = static_cast<uint32_t>(k_min_pulse_us);
        const uint32_t max = static_cast<uint32_t>(k_max_pulse_us);
        return pulse_us < min ? min : (pulse_us > max ? max : pulse_us);
    }

    static std::expected<void, Error> write_pulse_us(uint32_t pulse_us) {
        constexpr uint32_t period_ns = 20000u * 1000u;
        const int err = pwm_set_dt(&pwm_gpio, period_ns, pulse_us * 1000u);
        if (err) {
            return std::unexpected(Error::from_code(err).context("PWM write failed"));
        }
        last_pulse_us = pulse_us;
        return {};
    }

public:
    static std::expected<void, Error> init(const char* prefix) {
        LOG_MODULE_DECLARE(pwm_actuator);
        LOG_INF("%s Initializing... pulse range [%d, %d] us",
                prefix, MinPulseUs, MaxPulseUs);

        if (!pwm_is_ready_dt(&pwm_gpio)) {
            LOG_ERR("%s PWM not ready", prefix);
            return std::unexpected(Error::from_device_not_ready(pwm_gpio.dev).context("PWM not ready"));
        }

        // Drive low/min immediately after init.
        auto result = write_pulse_us(static_cast<uint32_t>(k_min_pulse_us));
        if (!result) {
            return result;
        }

        initialized = true;
        LOG_INF("%s Ready.", prefix);
        return {};
    }

    static std::expected<void, Error> tick(const char* prefix, uint32_t pulse_us) {
        LOG_MODULE_DECLARE(pwm_actuator);

        if (!initialized) {
            LOG_WRN("%s tick() called before init()", prefix);
            return std::unexpected(Error::from_code(-ECANCELED).context("Not initialized"));
        }

        return write_pulse_us(clamp_pulse_us(pulse_us));
    }

    static uint32_t get_pulse_us() {
        return last_pulse_us;
    }
};

// =============================================================================
// Servo
// =============================================================================
template<
    ServoKind Kind,
    const pwm_dt_spec PwmDt,
    int MinPulseUs = 1000,
    int MaxPulseUs = 2000>
class Servo : public PwmActuator<PwmDt, MinPulseUs, MaxPulseUs> {
    using Base = PwmActuator<PwmDt, MinPulseUs, MaxPulseUs>;

    static consteval const char* kind_to_str(ServoKind k) {
        switch (k) {
        case ServoKind::SERVO_X: return "[servo_x]";
        case ServoKind::SERVO_Y: return "[servo_y]";
        }
        return "[servo]";
    }

public:
    Servo() = delete;

    static std::expected<void, Error> init() {
        return Base::init(kind_to_str(Kind));
    }

    static std::expected<void, Error> tick(uint32_t pulse_us) {
        return Base::tick(kind_to_str(Kind), pulse_us);
    }

    static uint32_t get_pulse_us() {
        return Base::get_pulse_us();
    }
};

// =============================================================================
// EscMotor
// =============================================================================
template<
    MotorKind Kind,
    const pwm_dt_spec PwmDt,
    int MinPulseUs = 1000,
    int MaxPulseUs = 2000>
class EscMotor : public PwmActuator<PwmDt, MinPulseUs, MaxPulseUs> {
    using Base = PwmActuator<PwmDt, MinPulseUs, MaxPulseUs>;

    static consteval const char* kind_to_str(MotorKind k) {
        switch (k) {
        case MotorKind::BETA_TOP:     return "[beta_top]";
        case MotorKind::BETA_BOTTOM:  return "[beta_bottom]";
        case MotorKind::BETA_CW:      return "[beta_cw]";
        case MotorKind::BETA_CCW:     return "[beta_ccw]";
        case MotorKind::MOTOR_TOP:    return "[motor_top]";
        case MotorKind::MOTOR_BOTTOM: return "[motor_bottom]";
        }
        return "[motor]";
    }

public:
    EscMotor() = delete;

    static std::expected<void, Error> init() {
        return Base::init(kind_to_str(Kind));
    }

    static std::expected<void, Error> tick(uint32_t pulse_us) {
        return Base::tick(kind_to_str(Kind), pulse_us);
    }

    static uint32_t get_pulse_us() {
        return Base::get_pulse_us();
    }
};

typedef Servo<
    ServoKind::SERVO_X,
    PWM_DT_SPEC_GET_BY_NAME(DT_PATH(zephyr_user), servo_x)>
    ServoX;

typedef Servo<
    ServoKind::SERVO_Y,
    PWM_DT_SPEC_GET_BY_NAME(DT_PATH(zephyr_user), servo_y)>
    ServoY;

typedef EscMotor<
    MotorKind::BETA_TOP,
    PWM_DT_SPEC_GET_BY_NAME(DT_PATH(zephyr_user), beta_top)>
    BetaTop;

typedef EscMotor<
    MotorKind::BETA_BOTTOM,
    PWM_DT_SPEC_GET_BY_NAME(DT_PATH(zephyr_user), beta_bottom)>
    BetaBottom;

typedef EscMotor<
    MotorKind::BETA_CW,
    PWM_DT_SPEC_GET_BY_NAME(DT_PATH(zephyr_user), beta_cw)>
    BetaCW;

typedef EscMotor<
    MotorKind::BETA_CCW,
    PWM_DT_SPEC_GET_BY_NAME(DT_PATH(zephyr_user), beta_ccw)>
    BetaCCW;

typedef EscMotor<
    MotorKind::MOTOR_TOP,
    PWM_DT_SPEC_GET_BY_NAME(DT_PATH(zephyr_user), motor_top)>
    MotorTop;

typedef EscMotor<
    MotorKind::MOTOR_BOTTOM,
    PWM_DT_SPEC_GET_BY_NAME(DT_PATH(zephyr_user), motor_bottom)>
    MotorBottom;

#endif