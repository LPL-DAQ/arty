#ifndef ARTY_PWM_ACTUATOR_ZEPHYR_H
#define ARTY_PWM_ACTUATOR_ZEPHYR_H

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <expected>
#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "MutexGuard.h"

enum class Error {
    // Hardware
    PWM_NOT_READY,
    PWM_WRITE_FAILED,

    // State
    NOT_INITIALIZED,
    NOT_ARMED,
    FAULT_ACTIVE,
    WRONG_STATE,
    ALREADY_ARMED,
    ALREADY_DISARMED,

    // Command
    OUT_OF_RANGE,
    ARM_FAILED,
};

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
//
// Base class for all PWM-controlled actuators.
// Owns the state machine, fault handling, arming logic, and raw PWM writes.
// Derived classes add actuator-specific command methods and unit conversion.
//
// -----------------------------------------------------------------------------
template<
    const pwm_dt_spec& PwmDt,
    int MinPulseUs,
    int MaxPulseUs,
    int ArmingMs   = 0>
class PwmActuator {
protected:

    static constexpr float    k_min_pulse_us      = static_cast<float>(MinPulseUs);
    static constexpr float    k_max_pulse_us      = static_cast<float>(MaxPulseUs);
    static constexpr const pwm_dt_spec& pwm_gpio  = PwmDt;

    enum class ActuatorState { OFF, DISARMED, ARMED, FAIL_SAFE };

    inline static ActuatorState state      = ActuatorState::OFF;
    inline static ActuatorState prevState  = ActuatorState::OFF;

    inline static k_mutex actuator_lock      = {};
    inline static bool    fault_active       = false;
    inline static float   commanded_pulse_us = k_min_pulse_us;
    inline static int64_t last_tick_ms       = 0;


    static std::expected<void, Error> write_pulse_us(uint32_t pulse_us) {
        constexpr uint32_t period_ns = 20000u * 1000u;
        const int err = pwm_set_dt(&pwm_gpio, period_ns, pulse_us * 1000u);
        if (err) return std::unexpected(Error::PWM_WRITE_FAILED);
        commanded_pulse_us = static_cast<float>(pulse_us);
        return {};
    }

    // Zero the PWM signal entirely.
    static std::expected<void, Error> write_zero() {
        constexpr uint32_t period_ns = 20000u * 1000u;
        const int err = pwm_set_dt(&pwm_gpio, period_ns, 0u);
        if (err) return std::unexpected(Error::PWM_WRITE_FAILED);
        commanded_pulse_us = 0.0f;
        return {};
    }

    static std::expected<void, Error> enter_fail_safe(const char* prefix) {
        fault_active = true;

        {
            MutexGuard lock(&actuator_lock);
            prevState = state;
            state     = ActuatorState::FAIL_SAFE;
        }

        auto result = write_pulse_us(static_cast<uint32_t>(k_min_pulse_us));

        LOG_MODULE_DECLARE(pwm_actuator);
        LOG_ERR("%s FAIL_SAFE — min pulse commanded immediately. Call clear_fault() + arm() to resume.", prefix);

        return result;
    }

public:

    static std::expected<void, Error> init_state(const char* prefix) {
        LOG_MODULE_DECLARE(pwm_actuator);
        LOG_INF("%s Initializing... pulse range [%d, %d] us  arming %d ms",
                prefix, MinPulseUs, MaxPulseUs, ArmingMs);

        // 1. Mutex first — valid before any early return.
        k_mutex_init(&actuator_lock);

        // 2. Full state reset.
        {
            MutexGuard lock(&actuator_lock);
            commanded_pulse_us = k_min_pulse_us;
            fault_active       = false;
            state              = ActuatorState::OFF;
            prevState          = ActuatorState::OFF;
        }

        // 3. Verify PWM hardware.
        if (!pwm_is_ready_dt(&pwm_gpio)) {
            LOG_ERR("%s PWM not ready", prefix);
            return std::unexpected(Error::PWM_NOT_READY);
        }

        // 4. Transition to DISARMED.
        last_tick_ms = k_uptime_get();
        {
            MutexGuard lock(&actuator_lock);
            state     = ActuatorState::DISARMED;
            prevState = ActuatorState::DISARMED;
        }

        LOG_INF("%s Ready. Call arm() to begin operation.", prefix);
        return {};
    }

    static std::expected<void, Error> init(const char* prefix) {
        LOG_MODULE_DECLARE(pwm_actuator);
        LOG_INF("%s Initializing... pulse range [%d, %d] us  arming %d ms",
                prefix, MinPulseUs, MaxPulseUs, ArmingMs);

        // 1. Mutex first — valid before any early return.
        k_mutex_init(&actuator_lock);

        // 2. Full state reset.
        {
            MutexGuard lock(&actuator_lock);
            commanded_pulse_us = k_min_pulse_us;
            fault_active       = false;
            state              = ActuatorState::OFF;
            prevState          = ActuatorState::OFF;
        }

        // 3. Verify PWM hardware.
        if (!pwm_is_ready_dt(&pwm_gpio)) {
            LOG_ERR("%s PWM not ready", prefix);
            return std::unexpected(Error::PWM_NOT_READY);
        }

        // 4. Transition to DISARMED.
        last_tick_ms = k_uptime_get();
        {
            MutexGuard lock(&actuator_lock);
            state     = ActuatorState::ARMED;
            prevState = ActuatorState::ARMED;
        }

        LOG_INF("%s Ready.", prefix);
        return {};
    }

    static std::expected<void, Error> arm(const char* prefix) {
        LOG_MODULE_DECLARE(pwm_actuator);

        if (state == ActuatorState::ARMED) {
            LOG_WRN("%s arm() called but already ARMED", prefix);
            return std::unexpected(Error::ALREADY_ARMED);
        }
        if (state == ActuatorState::FAIL_SAFE) {
            LOG_WRN("%s arm() rejected — FAIL_SAFE in progress", prefix);
            return std::unexpected(Error::WRONG_STATE);
        }
        if (state == ActuatorState::OFF) {
            LOG_WRN("%s arm() called before init_state()", prefix);
            return std::unexpected(Error::NOT_INITIALIZED);
        }
        if (fault_active) {
            LOG_WRN("%s arm() rejected — fault active. Call clear_fault() first.",
                    prefix);
            return std::unexpected(Error::FAULT_ACTIVE);
        }

        {
            MutexGuard lock(&actuator_lock);
            prevState = state;
            state     = ActuatorState::ARMED;
        }

        LOG_INF("%s ARMED", prefix);
        return {};
    }


    static std::expected<void, Error> disarm(const char* prefix) {
        LOG_MODULE_DECLARE(pwm_actuator);

        if (state == ActuatorState::OFF) {
            LOG_WRN("%s disarm() called before init_state()", prefix);
            return std::unexpected(Error::NOT_INITIALIZED);
        }
        if (state == ActuatorState::DISARMED) {
            return std::unexpected(Error::ALREADY_DISARMED);
        }

        LOG_INF("%s Disarming", prefix);

        {
            MutexGuard lock(&actuator_lock);
            prevState = state;
            state     = ActuatorState::DISARMED;
        }

        // tick() will command min pulse on next call.
        // No direct write here — tick() owns hardware interaction.
        return {};
    }

    static std::expected<void, Error> fail_safe(const char* prefix) {
        if (state == ActuatorState::FAIL_SAFE ||
            state == ActuatorState::DISARMED  ||
            state == ActuatorState::OFF) {
            return {};
        }
        return enter_fail_safe(prefix);
    }

    static std::expected<void, Error> emergency_stop(const char* prefix) {
        LOG_MODULE_DECLARE(pwm_actuator);
        LOG_ERR("%s EMERGENCY STOP — PWM zeroed. Call init_state() to recover.",
                prefix);

        auto result = write_zero();

        {
            MutexGuard lock(&actuator_lock);
            prevState = state;
            state     = ActuatorState::OFF;
        }

        return result;
    }

    static void clear_fault(const char* prefix) {
        LOG_MODULE_DECLARE(pwm_actuator);
        if (!fault_active) return;
        fault_active = false;
        {
            MutexGuard lock(&actuator_lock);
            prevState = state;
            state     = ActuatorState::DISARMED;
        }
        LOG_INF("%s Fault cleared. Call arm() to resume.", prefix);
    }

    static std::expected<void, Error> tick_base_state(const char* prefix) {
        LOG_MODULE_DECLARE(pwm_actuator);

        if (state == ActuatorState::OFF) [[unlikely]] {
            LOG_WRN("%s tick() called before init_state()", prefix);
            return std::unexpected(Error::NOT_INITIALIZED);
        }

        last_tick_ms = k_uptime_get();
        prevState    = state;

        switch (state) {

        case ActuatorState::DISARMED:
            write_pulse_us(static_cast<uint32_t>(k_min_pulse_us));
            break;

        case ActuatorState::FAIL_SAFE:
            write_pulse_us(static_cast<uint32_t>(k_min_pulse_us));
            break;

        case ActuatorState::ARMED:
            // Derived class handles ARMED tick — falls through to derived tick().
            break;

        default:
            break;
        }

        return {};
    }

static std::expected<void, Error> tick_base(const char* prefix, uint32_t pulse_us) {
    LOG_MODULE_DECLARE(pwm_actuator);

    if (state == ActuatorState::OFF) [[unlikely]] {
        LOG_WRN("%s tick() called before init()", prefix);
        return std::unexpected(Error::NOT_INITIALIZED);
    }

    last_tick_ms = k_uptime_get();
    write_pulse_us(pulse_us);
    return {};
}

    static ActuatorState get_state()          { return state; }
    static bool          is_armed()           { return state == ActuatorState::ARMED; }
    static bool          is_disarmed()        { return state == ActuatorState::DISARMED; }
    static bool          is_faulted()         { return fault_active; }
    static bool          is_fail_safe_active() { return state == ActuatorState::FAIL_SAFE; }
    static float         get_pulse_us()       { return commanded_pulse_us; }
    static int64_t       get_last_tick_ms()   { return last_tick_ms; }

};

// =============================================================================
// Servo
//
// Angle-based PWM actuator, wrapper over PwmActuator.
// Adds degree ↔ pulse conversion and set_target_angle().
// Angle convention:
//   -90° = full left  = 1000 µs
//     0° = center     = 1500 µs
//   +90° = full right = 2000 µs
// =============================================================================

template<
    ServoKind Kind,
    pwm_dt_spec PwmDt,
    int MinPulseUs     = 1000,
    int MaxPulseUs     = 2000,
    int MinDeg         = -90,
    int MaxDeg         =  90,
    int NeutralDeg     =   0,
    int DeadbandDegX10 =  10>
class Servo : public PwmActuator<PwmDt, MinPulseUs, MaxPulseUs, 0>
{
    using Base = PwmActuator<PwmDt, MinPulseUs, MaxPulseUs, 0>;

    static constexpr float k_min_deg     = static_cast<float>(MinDeg);
    static constexpr float k_max_deg     = static_cast<float>(MaxDeg);
    static constexpr float k_neutral_deg = static_cast<float>(NeutralDeg);
    static constexpr float k_deadband_deg = static_cast<float>(DeadbandDegX10) / 10.0f;

    // Target angle — tracked between tick() calls.
    inline static float target_deg = k_neutral_deg;

    static uint32_t deg_to_pulse_us(float deg) {
        const float clamped = std::clamp(deg, k_min_deg, k_max_deg);
        const float t = (clamped - k_min_deg) / (k_max_deg - k_min_deg);
        return static_cast<uint32_t>(
            Base::k_min_pulse_us + t * (Base::k_max_pulse_us - Base::k_min_pulse_us));
    }

    static float pulse_us_to_deg(float pulse_us) {
        const float t = (pulse_us - Base::k_min_pulse_us) /
                        (Base::k_max_pulse_us - Base::k_min_pulse_us);
        return k_min_deg + std::clamp(t, 0.0f, 1.0f) * (k_max_deg - k_min_deg);
    }

    static consteval const char* kind_to_str(ServoKind k) {
        switch (k) {
        case ServoKind::SERVO_X:       return "[servo_x]";
        case ServoKind::SERVO_Y:       return "[servo_y]";
        }
        return "[servo]";
    }

public:
    Servo() = delete;

    static std::expected<void, Error> init() {
        target_deg = k_neutral_deg;
        return Base::init(kind_to_str(Kind));
    }

    static std::expected<void, Error> arm() {
        return Base::arm(kind_to_str(Kind));
    }

    static std::expected<void, Error> disarm() {
        return Base::disarm(kind_to_str(Kind));
    }

    static std::expected<void, Error> fail_safe() {
        return Base::fail_safe(kind_to_str(Kind));
    }

    static std::expected<void, Error> emergency_stop() {
        return Base::emergency_stop(kind_to_str(Kind));
    }

    static void clear_fault() {
        Base::clear_fault(kind_to_str(Kind));
    }

    static std::expected<void, Error> tick_state() {
        // Run base state machine — handles DISARMED, FAIL_SAFE, OFF.
        auto result = Base::tick_base_state(kind_to_str(Kind));
        if (!result) return result;

        // ARMED — write PWM toward target_deg set by set_target_angle().
        if (Base::state == Base::ActuatorState::ARMED) {
            // Only write if outside deadband — avoids unnecessary PWM updates.
            if (std::abs(target_deg - get_angle_running()) >= k_deadband_deg) {
                return Base::write_pulse_us(deg_to_pulse_us(target_deg));
            }
        }

        return {};
    }

    static std::expected<void, Error> tick(uint32_t pulse_us) {
        auto result = Base::tick_base(kind_to_str(Kind),pulse_us);
        if (!result) return result;
        return {};
    }

    static std::expected<void, Error> set_target_angle(float deg) {
        if (deg < k_min_deg || deg > k_max_deg)
            return std::unexpected(Error::OUT_OF_RANGE);
        target_deg = deg;
        return {};
    }

    // -------------------------------------------------------------------------
    // Telemetry — servo specific
    // -------------------------------------------------------------------------

    static float get_angle_running() {
        return pulse_us_to_deg(Base::commanded_pulse_us);
    }
    static float get_angle_target()   { return target_deg; }
    static float get_angle_min()      { return k_min_deg; }
    static float get_angle_max()      { return k_max_deg; }
    static float get_angle_neutral()  { return k_neutral_deg; }
    static float get_deadband_deg()   { return k_deadband_deg; }
};

// =============================================================================
// EscMotor
//
// Throttle-based PWM actuator, wrapper over PwmActuator.
// Adds throttle [0.0, 1.0] ↔ pulse conversion and set_target_throttle().
// =============================================================================

template<
    MotorKind Kind,
    pwm_dt_spec PwmDt,
    int MinPulseUs = 1000,
    int MaxPulseUs = 2000,
    int ArmingMs   = 2000>
class EscMotor  : public PwmActuator<PwmDt, MinPulseUs, MaxPulseUs, ArmingMs> {
    using Base = PwmActuator<PwmDt, MinPulseUs, MaxPulseUs, ArmingMs>;

    inline static float target_throttle = 0.0f;

    static uint32_t throttle_to_pulse_us(float throttle) {
        const float clamped = std::clamp(throttle, 0.0f, 1.0f);
        return static_cast<uint32_t>(
            Base::k_min_pulse_us +
            clamped * (Base::k_max_pulse_us - Base::k_min_pulse_us));
    }

    static float pulse_us_to_throttle(float pulse_us) {
        return std::clamp(
            (pulse_us - Base::k_min_pulse_us) /
            (Base::k_max_pulse_us - Base::k_min_pulse_us),
            0.0f, 1.0f);
    }

static consteval const char* kind_to_str(MotorKind k) {
    switch (k) {
    case MotorKind::BETA_TOP:    return "[beta_top]";
    case MotorKind::BETA_BOTTOM: return "[beta_bottom]";
    case MotorKind::BETA_CW:     return "[beta_cw]";
    case MotorKind::BETA_CCW:    return "[beta_ccw]";
    case MotorKind::MOTOR_TOP:   return "[motor_top]";
    case MotorKind::MOTOR_BOTTOM:return "[motor_bottom]";
    }
    return "[motor]";
}

public:
    EscMotor() = delete;

    static std::expected<void, Error> init() {
        target_throttle = 0.0f;
        return Base::init(kind_to_str(Kind));
    }

    static std::expected<void, Error> arm() {
        return Base::arm(kind_to_str(Kind));
    }

    static std::expected<void, Error> disarm() {
        return Base::disarm(kind_to_str(Kind));
    }

    static std::expected<void, Error> fail_safe() {
        return Base::fail_safe(kind_to_str(Kind));
    }

    static std::expected<void, Error> emergency_stop() {
        return Base::emergency_stop(kind_to_str(Kind));
    }

    static void clear_fault() {
        Base::clear_fault(kind_to_str(Kind));
    }

    static std::expected<void, Error> tick_state() {
        auto result = Base::tick_base(kind_to_str(Kind));
        if (!result) return result;

        if (Base::state == Base::ActuatorState::ARMED) {
            return Base::write_pulse_us(throttle_to_pulse_us(target_throttle));
        }

        return {};
    }

    static std::expected<void, Error> tick(uint32_t pulse_us) {
        auto result = Base::tick_base(kind_to_str(Kind), pulse_us);
        if (!result) return result;
        return {};
    }

    static std::expected<void, Error> set_target_throttle(float throttle) {
        if (throttle < 0.0f || throttle > 1.0f)
            return std::unexpected(Error::OUT_OF_RANGE);
        target_throttle = throttle;
        return {};
    }

    // -------------------------------------------------------------------------
    // Telemetry — ESC specific
    // -------------------------------------------------------------------------

    static float get_throttle_running() {
        return pulse_us_to_throttle(Base::commanded_pulse_us);
    }

    static float get_throttle_target() { return target_throttle; }

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
    PWM_DT_SPEC_GET_BY_NAME(DT_PATH(zephyr_user), motor_top),
    1000,
    2000,
    3000>
    MotorTop;

typedef EscMotor<
    MotorKind::MOTOR_BOTTOM,
    PWM_DT_SPEC_GET_BY_NAME(DT_PATH(zephyr_user), motor_bottom),
    1000,
    2000,
    3000>
    MotorBottom;
#endif