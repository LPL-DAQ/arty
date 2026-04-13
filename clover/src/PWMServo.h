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

// -----------------------------------------------------------------------------
// Error
//
// All error codes returned via std::expected<void, Error>.
// Callers must handle — cannot be silently ignored like int return codes.
// -----------------------------------------------------------------------------
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

// -----------------------------------------------------------------------------
// ActuatorKind
//
// Identity tags for log prefixes.
// -----------------------------------------------------------------------------
enum class ServoKind  { SERVO_L, SERVO_R};
enum class MotorKind  { BETA1, BETA2, BETA3, BETA4, CR5025 };

// -----------------------------------------------------------------------------
// PwmActuator
//
// Base class for all PWM-controlled actuators.
// Owns the state machine, fault handling, arming logic, and raw PWM writes.
// Derived classes add actuator-specific command methods and unit conversion.
//
// Template parameters:
//   PwmDt      : PWM devicetree spec
//   MinPulseUs : pulse width (µs) at minimum position/throttle
//   MaxPulseUs : pulse width (µs) at maximum position/throttle
//   ArmingMs   : ms to hold min pulse during arm() sequence
//                0 = no arming sequence (servo)
//                >0 = ESC arming hold time
//   SpeedPct   : rated travel speed as % of full range per second
//                used for FAIL_SAFE timeout estimation
//                e.g. 50 means full range in 2 seconds
//
// State machine:
//   OFF       → pre-init. No PWM output. Only exits via init().
//   DISARMED  → safe default PWM (MinPulseUs). Commands rejected.
//               Entered from: init(), disarm(), FAIL_SAFE resolution.
//               Exits via: arm().
//   ARMED     → accepting commands. PWM tracks target.
//               Entered only via arm().
//               Exits via: disarm(), fail_safe().
//   FAIL_SAFE → fault. Commands MinPulseUs immediately and every tick().
//               Does not require init() to recover.
//               Self-transitions to DISARMED after estimated time × 1.5.
//               Requires clear_fault() before re-arming.
//               Callable before tick() — fires immediately.
//               No-op from DISARMED or OFF.
//
// Open loop — commanded position/throttle is assumed, not measured.
// -----------------------------------------------------------------------------
template<
    const pwm_dt_spec& PwmDt,
    int MinPulseUs,
    int MaxPulseUs,
    int ArmingMs   = 0,
    int SpeedPct   = 50>
class PwmActuator {
protected:

    // -------------------------------------------------------------------------
    // Compile-time constants
    // -------------------------------------------------------------------------

    static constexpr float    k_min_pulse_us     = static_cast<float>(MinPulseUs);
    static constexpr float    k_max_pulse_us     = static_cast<float>(MaxPulseUs);
    static constexpr float    k_failsafe_mult    = 1.5f;
    static constexpr int64_t  k_min_timeout_ms   = 50LL;

    // Estimated full range travel time in ms based on SpeedPct.
    // SpeedPct = % of full range per second.
    // e.g. SpeedPct=50 → full range in 2000ms
    static constexpr int64_t  k_full_range_ms    =
        static_cast<int64_t>(100.0f / static_cast<float>(SpeedPct) * 1000.0f);

    static constexpr const pwm_dt_spec& pwm_gpio = PwmDt;

    // -------------------------------------------------------------------------
    // State machine
    // -------------------------------------------------------------------------

    enum class ActuatorState { OFF, DISARMED, ARMED, FAIL_SAFE };

    inline static ActuatorState state     = ActuatorState::OFF;
    inline static ActuatorState prevState = ActuatorState::OFF;

    // -------------------------------------------------------------------------
    // Shared mutable state
    // -------------------------------------------------------------------------

    inline static k_mutex actuator_lock     = {};
    inline static bool    fault_active      = false;
    inline static float   commanded_pulse_us = k_min_pulse_us;
    inline static int64_t last_tick_ms      = 0;

    // -------------------------------------------------------------------------
    // FAIL_SAFE timing
    // -------------------------------------------------------------------------

    inline static int64_t fail_safe_start_ms   = 0;
    inline static int64_t fail_safe_timeout_ms = 0;

    // -------------------------------------------------------------------------
    // Protected helpers — available to derived classes
    // -------------------------------------------------------------------------

    // Write a raw PWM pulse in microseconds.
    // Used by derived class command methods to set actuator output.
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

    // Estimate FAIL_SAFE timeout from current pulse to minimum.
    // Based on distance from current commanded pulse to min as a
    // fraction of full range, scaled by full range travel time.
    static int64_t compute_failsafe_timeout_ms() {
        const float range     = k_max_pulse_us - k_min_pulse_us;
        const float dist      = std::abs(commanded_pulse_us - k_min_pulse_us);
        const float fraction  = (range > 0.0f) ? (dist / range) : 0.0f;
        const int64_t est_ms  = static_cast<int64_t>(
            fraction * static_cast<float>(k_full_range_ms));
        const int64_t timeout = static_cast<int64_t>(
            static_cast<float>(est_ms) * k_failsafe_mult);
        return timeout < k_min_timeout_ms ? k_min_timeout_ms : timeout;
    }

    // Internal FAIL_SAFE entry.
    // Writes minimum pulse immediately — safe to call from any context.
    static std::expected<void, Error> enter_fail_safe(const char* prefix) {
        fail_safe_timeout_ms = compute_failsafe_timeout_ms();
        fail_safe_start_ms   = k_uptime_get();
        fault_active         = true;

        {
            MutexGuard lock(&actuator_lock);
            prevState = state;
            state     = ActuatorState::FAIL_SAFE;
        }

        // Command minimum immediately — do not wait for tick().
        auto result = write_pulse_us(static_cast<uint32_t>(k_min_pulse_us));

        LOG_MODULE_DECLARE(pwm_actuator);
        LOG_ERR("%s FAIL_SAFE — min pulse commanded immediately. "
                "Timeout: %lld ms. Call clear_fault() + arm() to resume.",
                prefix, fail_safe_timeout_ms);

        return result;
    }

public:

    // -------------------------------------------------------------------------
    // init
    //
    // Must be called once before any other method.
    // Safe to call again after emergency_stop() — resets all state.
    //
    // Sequence:
    //   1. Mutex init
    //   2. Full state reset
    //   3. Verify PWM hardware
    //   4. Command minimum pulse
    //   5. Transition to DISARMED
    //
    // Returns std::expected<void, Error>.
    // -------------------------------------------------------------------------
    static std::expected<void, Error> init(const char* prefix) {
        LOG_MODULE_DECLARE(pwm_actuator);
        LOG_INF("%s Initializing... pulse range [%d, %d] us  arming %d ms",
                prefix, MinPulseUs, MaxPulseUs, ArmingMs);

        // 1. Mutex first — valid before any early return.
        k_mutex_init(&actuator_lock);

        // 2. Full state reset.
        {
            MutexGuard lock(&actuator_lock);
            commanded_pulse_us   = k_min_pulse_us;
            fault_active         = false;
            fail_safe_start_ms   = 0;
            fail_safe_timeout_ms = 0;
            state                = ActuatorState::OFF;
            prevState            = ActuatorState::OFF;
        }

        // 3. Verify PWM hardware.
        if (!pwm_is_ready_dt(&pwm_gpio)) {
            LOG_ERR("%s PWM not ready", prefix);
            return std::unexpected(Error::PWM_NOT_READY);
        }

        // prabhu: currently only state is updated and pwm is set at next tick,
        // not sure which is better structure
        // 4. Command minimum pulse — required for ESC to detect controller presence.
        //    For servo: harmless, tick() will correct position on first call.
        // auto result = write_pulse_us(static_cast<uint32_t>(k_min_pulse_us));
        // if (!result) return result;

        // 5. Transition to DISARMED.
        last_tick_ms = k_uptime_get();
        {
            MutexGuard lock(&actuator_lock);
            state     = ActuatorState::DISARMED;
            prevState = ActuatorState::DISARMED;
        }

        LOG_INF("%s Ready. Call arm() to begin operation.", prefix);
        return {};
    }

    // -------------------------------------------------------------------------
    // arm
    //
    // Transition DISARMED → ARMED.
    // Holds minimum pulse for ArmingMs (ESC arming sequence).
    // ArmingMs = 0 → instant transition (servo).
    //
    // Blocking if ArmingMs > 0 — do not call from time-critical thread.
    //
    // Returns:
    //   {}                        on success
    //   Error::WRONG_STATE        if not DISARMED
    //   Error::FAULT_ACTIVE       if fault not cleared
    //   Error::ALREADY_ARMED      if already ARMED
    // -------------------------------------------------------------------------
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
            LOG_WRN("%s arm() called before init()", prefix);
            return std::unexpected(Error::NOT_INITIALIZED);
        }
        if (fault_active) {
            LOG_WRN("%s arm() rejected — fault active. Call clear_fault() first.",
                    prefix);
            return std::unexpected(Error::FAULT_ACTIVE);
        }
        //prabhu should be handled by tick
        // // Hold minimum pulse for arming period — ESC listens for this.
        // // Servo: ArmingMs = 0, skipped entirely.
        // if (ArmingMs > 0) {
        //     LOG_INF("%s Arming sequence — holding min pulse for %d ms",
        //             prefix, ArmingMs);
        //     auto result = write_pulse_us(static_cast<uint32_t>(k_min_pulse_us));
        //     if (!result) return result;
        //     k_msleep(ArmingMs);
        // }

        {
            MutexGuard lock(&actuator_lock);
            prevState = state;
            state     = ActuatorState::ARMED;
        }

        LOG_INF("%s DISARMED → ARMED", prefix);
        return {};
    }

    // -------------------------------------------------------------------------
    // disarm
    //
    // Transition ARMED → DISARMED.
    // Commands minimum pulse. tick() holds it.
    // Call arm() to resume.
    //
    // Returns:
    //   {}                        on success
    //   Error::ALREADY_DISARMED   if already DISARMED
    //   Error::WRONG_STATE        if OFF
    // -------------------------------------------------------------------------
    static std::expected<void, Error> disarm(const char* prefix) {
        LOG_MODULE_DECLARE(pwm_actuator);

        if (state == ActuatorState::OFF) {
            LOG_WRN("%s disarm() called before init()", prefix);
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

    // -------------------------------------------------------------------------
    // fail_safe
    //
    // Trigger fault response. Commands minimum pulse immediately —
    // does not wait for tick(). Safe to call from any context.
    //
    // Self-transitions to DISARMED after estimated travel time × 1.5.
    // Requires clear_fault() + arm() to resume — no init() needed.
    //
    // No-op if already FAIL_SAFE, DISARMED, or OFF.
    // -------------------------------------------------------------------------
    static std::expected<void, Error> fail_safe(const char* prefix) {
        if (state == ActuatorState::FAIL_SAFE ||
            state == ActuatorState::DISARMED  ||
            state == ActuatorState::OFF) {
            return {};
        }
        return enter_fail_safe(prefix);
    }

    // -------------------------------------------------------------------------
    // emergency_stop
    //
    // Immediately zeros PWM signal. Use on ground only.
    // Transitions to OFF — requires init() to recover.
    //
    // In-air use risks uncontrolled actuator behavior.
    // -------------------------------------------------------------------------
    static std::expected<void, Error> emergency_stop(const char* prefix) {
        LOG_MODULE_DECLARE(pwm_actuator);
        LOG_ERR("%s EMERGENCY STOP — PWM zeroed. Call init() to recover.",
                prefix);

        auto result = write_zero();

        {
            MutexGuard lock(&actuator_lock);
            prevState = state;
            state     = ActuatorState::OFF;
        }

        return result;
    }

    // -------------------------------------------------------------------------
    // clear_fault
    //
    // Acknowledge and clear fault flag after FAIL_SAFE resolves to DISARMED.
    // Required before arm() will be accepted.
    // Flight controller is responsible for determining it is safe to clear.
    // -------------------------------------------------------------------------
    static void clear_fault(const char* prefix) {
        LOG_MODULE_DECLARE(pwm_actuator);
        if (!fault_active) return;
        fault_active = false;
        LOG_INF("%s Fault cleared. Call arm() to resume.", prefix);
    }

    // -------------------------------------------------------------------------
    // tick (base)
    //
    // Handles state machine updates common to all actuators.
    // Must be called by derived tick() — not directly by flight controller.
    //
    // Manages:
    //   DISARMED  → holds min pulse
    //   FAIL_SAFE → holds min pulse, checks timeout
    //
    // Returns:
    //   {}             on success
    //   Error::WRONG_STATE  if called in OFF state
    // -------------------------------------------------------------------------
    static std::expected<void, Error> tick_base(const char* prefix) {
        LOG_MODULE_DECLARE(pwm_actuator);

        if (state == ActuatorState::OFF) [[unlikely]] {
            LOG_WRN("%s tick() called before init()", prefix);
            return std::unexpected(Error::NOT_INITIALIZED);
        }

        last_tick_ms = k_uptime_get();
        prevState    = state;

        switch (state) {
            //prabhu explain
        case ActuatorState::DISARMED:
            // Hold minimum pulse. No commands accepted.
            // Re-command on transition from ARMED → DISARMED.
                write_pulse_us(static_cast<uint32_t>(k_min_pulse_us));
            break;

        case ActuatorState::FAIL_SAFE: {
            // Reinforce minimum pulse every tick.
            write_pulse_us(static_cast<uint32_t>(k_min_pulse_us));

            const int64_t elapsed = k_uptime_get() - fail_safe_start_ms;
            if (elapsed >= fail_safe_timeout_ms) {
                LOG_INF("%s FAIL_SAFE resolved — assuming min position reached. "
                        "Call clear_fault() + arm() to resume.", prefix);
                MutexGuard lock(&actuator_lock);
                prevState = state;
                state     = ActuatorState::DISARMED;
            }
            break;
        }

        case ActuatorState::ARMED:
            // Derived class handles ARMED tick — falls through to derived tick().
            break;

        default:
            break;
        }

        return {};
    }

    // -------------------------------------------------------------------------
    // Telemetry — base
    // -------------------------------------------------------------------------

    static ActuatorState get_state()         { return state; }
    static bool          is_armed()          { return state == ActuatorState::ARMED; }
    static bool          is_disarmed()       { return state == ActuatorState::DISARMED; }
    static bool          is_faulted()        { return fault_active; }
    static bool          is_fail_safe_active(){ return state == ActuatorState::FAIL_SAFE; }
    static float         get_pulse_us()      { return commanded_pulse_us; }
    static int64_t       get_last_tick_ms()  { return last_tick_ms; }

    static int64_t get_fail_safe_elapsed_ms() {
        if (state != ActuatorState::FAIL_SAFE) return 0;
        return k_uptime_get() - fail_safe_start_ms;
    }

    static int64_t get_fail_safe_remaining_ms() {
        if (state != ActuatorState::FAIL_SAFE) return 0;
        const int64_t remaining = fail_safe_timeout_ms -
                                  (k_uptime_get() - fail_safe_start_ms);
        return remaining > 0 ? remaining : 0;
    }
};

// =============================================================================
// Servo
//
// Angle-based PWM actuator. Thin wrapper over PwmActuator.
// Adds degree ↔ pulse conversion and set_target_angle().
//
// Template parameters:
//   Kind           : servo identity, for log prefixes
//   PwmDt          : PWM devicetree spec
//   MinPulseUs     : pulse at MinDeg                  default: 1000
//   MaxPulseUs     : pulse at MaxDeg                  default: 2000
//   MinDeg         : minimum angle                    default: -90
//   MaxDeg         : maximum angle                    default:  90
//   NeutralDeg     : disarm/fail-safe position        default:   0
//   DeadbandDegX10 : minimum correction × 10          default:  10 → 1.0°
//   SpeedDegPerSec : rated travel speed                default: 300
//
// Angle convention:
//   -90° = full left  = 1000 µs
//     0° = center     = 1500 µs
//   +90° = full right = 2000 µs
//
// Example:
//   static const pwm_dt_spec aileron_pwm =
//       PWM_DT_SPEC_GET(DT_NODELABEL(aileron_l_servo));
//
//   using AileronL = Servo<ServoKind::AILERON_L, aileron_pwm>;
//
//   AileronL::init();
//   AileronL::arm();
//   AileronL::tick(45.0f);
// =============================================================================

template<
    ServoKind Kind,
    const pwm_dt_spec& PwmDt,
    int MinPulseUs     = 1000,
    int MaxPulseUs     = 2000,
    int MinDeg         = -90,
    int MaxDeg         =  90,
    int NeutralDeg     =   0,
    int DeadbandDegX10 =  10,
    int SpeedDegPerSec = 300>
class Servo : public PwmActuator<
    PwmDt,
    MinPulseUs,
    MaxPulseUs,
    0,  // ArmingMs = 0 — no arming sequence for servo
    static_cast<int>(
        static_cast<float>(SpeedDegPerSec) /
        static_cast<float>(MaxDeg - MinDeg) * 100.0f)>
{
   using Base = PwmActuator<
    PwmDt,
    MinPulseUs,
    MaxPulseUs,
    0,
    static_cast<int>(
        static_cast<float>(SpeedDegPerSec) /
        static_cast<float>(MaxDeg - MinDeg) * 100.0f)
>;

    static constexpr float k_min_deg      = static_cast<float>(MinDeg);
    static constexpr float k_max_deg      = static_cast<float>(MaxDeg);
    static constexpr float k_neutral_deg  = static_cast<float>(NeutralDeg);
    static constexpr float k_deadband_deg = static_cast<float>(DeadbandDegX10) / 10.0f;

    // Target angle — tracked between tick() calls.
    inline static float target_deg = k_neutral_deg;

    // -------------------------------------------------------------------------
    // Degree ↔ pulse conversion
    // -------------------------------------------------------------------------

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
        case ServoKind::SERVO_L: return "[servo_l]";
        case ServoKind::SERVO_R: return "[servo_r]";
        }
        return "[servo]";
    }

public:
    Servo() = delete;

    // -------------------------------------------------------------------------
    // init
    // -------------------------------------------------------------------------
    static std::expected<void, Error> init() {
        target_deg = k_neutral_deg;
        return Base::init(kind_to_str(Kind));
    }

    // -------------------------------------------------------------------------
    // arm
    // -------------------------------------------------------------------------
    static std::expected<void, Error> arm() {
        return Base::arm(kind_to_str(Kind));
    }

    // -------------------------------------------------------------------------
    // disarm
    // -------------------------------------------------------------------------
    static std::expected<void, Error> disarm() {
        return Base::disarm(kind_to_str(Kind));
    }

    // -------------------------------------------------------------------------
    // fail_safe
    // -------------------------------------------------------------------------
    static std::expected<void, Error> fail_safe() {
        return Base::fail_safe(kind_to_str(Kind));
    }

    // -------------------------------------------------------------------------
    // emergency_stop
    // -------------------------------------------------------------------------
    static std::expected<void, Error> emergency_stop() {
        return Base::emergency_stop(kind_to_str(Kind));
    }

    // -------------------------------------------------------------------------
    // clear_fault
    // -------------------------------------------------------------------------
    static void clear_fault() {
        Base::clear_fault(kind_to_str(Kind));
    }

    // -------------------------------------------------------------------------
    // tick
    //
    // Call from a single thread at fixed rate — 10ms recommended.
    // Do NOT call from ISR.
    //
    // Parameters:
    //   target_angle_deg : desired angle. Only used when ARMED.
    //
    // Returns:
    //   {}                   on success
    //   Error::NOT_INITIALIZED if called before init()
    // -------------------------------------------------------------------------
    static std::expected<void, Error> tick(float target_angle_deg) {
        // Run base state machine — handles DISARMED, FAIL_SAFE, OFF.
        auto result = Base::tick_base(kind_to_str(Kind));
        if (!result) return result;

        // ARMED — track target angle.
        if (Base::state == Base::ActuatorState::ARMED) {
            const float clamped = std::clamp(target_angle_deg, k_min_deg, k_max_deg);

            // Only write if outside deadband — avoids unnecessary PWM updates.
            if (std::abs(clamped - get_angle_commanded()) >= k_deadband_deg) {
                target_deg = clamped;
                return Base::write_pulse_us(deg_to_pulse_us(clamped));
            }
        }

        return {};
    }

    // -------------------------------------------------------------------------
    // prabhu  test class to be removed
    //  set_target_angle 
    //
    // Command a new angle directly without waiting for tick().
    // Must be ARMED. Useful for immediate one-shot commands.
    //
    // Returns:
    //   {}                on success
    //   Error::NOT_ARMED  if not ARMED
    //   Error::OUT_OF_RANGE if deg outside [MinDeg, MaxDeg]
    // -------------------------------------------------------------------------
    static std::expected<void, Error> set_target_angle(float deg) {
        if (!Base::is_armed()) return std::unexpected(Error::NOT_ARMED);
        if (deg < k_min_deg || deg > k_max_deg) {
            return std::unexpected(Error::OUT_OF_RANGE);
        }
        target_deg = deg;
        return Base::write_pulse_us(deg_to_pulse_us(deg));
    }

    // -------------------------------------------------------------------------
    // Telemetry — servo specific
    // -------------------------------------------------------------------------

    // Commanded angle derived from current pulse width.
    // Open loop — assumed position, not measured.
    static float get_angle_commanded() {
        return pulse_us_to_deg(Base::commanded_pulse_us);
    }

    static float get_angle_target()   { return target_deg; }
    static float get_angle_min()      { return k_min_deg; }
    static float get_angle_max()      { return k_max_deg; }
    static float get_angle_neutral()  { return k_neutral_deg; }
    static float get_deadband_deg()   { return k_deadband_deg; }

    // Signed error: positive = servo needs to move toward higher degrees.
    // Open loop — based on commanded, not measured position.
    static float get_angle_error() {
        return target_deg - get_angle_commanded();
    }
};

// =============================================================================
// EscMotor
//
// Throttle-based PWM actuator. Thin wrapper over PwmActuator.
// Adds throttle [0.0, 1.0] ↔ pulse conversion and set_target_throttle().
//
// Template parameters:
//   Kind       : motor identity, for log prefixes
//   PwmDt      : PWM devicetree spec
//   MinPulseUs : idle/disarmed pulse          default: 1000
//   MaxPulseUs : full throttle pulse          default: 2000
//   ArmingMs   : ESC arming hold time in ms   default: 2000
//   SpeedPct   : throttle slew speed          default: 50
//
// Example:
//   static const pwm_dt_spec motor_fl_pwm =
//       PWM_DT_SPEC_GET(DT_NODELABEL(motor_fl));
//
//   using MotorFL = EscMotor<MotorKind::MOTOR_FL, motor_fl_pwm>;
//
//   MotorFL::init();
//   MotorFL::arm();    // blocking — holds min pulse for ArmingMs
//   MotorFL::tick(0.75f);
// =============================================================================

template<
    MotorKind Kind,
    const pwm_dt_spec& PwmDt,
    int MinPulseUs = 1000,
    int MaxPulseUs = 2000,
    int ArmingMs   = 2000,
    int SpeedPct   = 50>
class EscMotor : public PwmActuator<PwmDt, MinPulseUs, MaxPulseUs, ArmingMs, SpeedPct> {
    using Base = PwmActuator<PwmDt, MinPulseUs, MaxPulseUs, ArmingMs, SpeedPct>;

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
        case MotorKind::BETA1: return "[beta1]";
        case MotorKind::BETA2: return "[beta2]";
        case MotorKind::BETA3: return "[beta3]";
        case MotorKind::BETA4: return "[beta4]";
        case MotorKind::CR5025: return "[cr5025]";
        }
        return "[motor]";
    }

public:
    EscMotor() = delete;

    // -------------------------------------------------------------------------
    // init
    // -------------------------------------------------------------------------
    static std::expected<void, Error> init() {
        target_throttle = 0.0f;
        return Base::init(kind_to_str(Kind));
    }

    // -------------------------------------------------------------------------
    // arm
    //
    // Blocking — holds min pulse for ArmingMs while ESC arms.
    // Do not call from time-critical thread.
    // -------------------------------------------------------------------------
    static std::expected<void, Error> arm() {
        return Base::arm(kind_to_str(Kind));
    }

    // -------------------------------------------------------------------------
    // disarm
    // -------------------------------------------------------------------------
    static std::expected<void, Error> disarm() {
        return Base::disarm(kind_to_str(Kind));
    }

    // -------------------------------------------------------------------------
    // fail_safe
    // -------------------------------------------------------------------------
    static std::expected<void, Error> fail_safe() {
        return Base::fail_safe(kind_to_str(Kind));
    }

    // -------------------------------------------------------------------------
    // emergency_stop
    // -------------------------------------------------------------------------
    static std::expected<void, Error> emergency_stop() {
        return Base::emergency_stop(kind_to_str(Kind));
    }

    // -------------------------------------------------------------------------
    // clear_fault
    // -------------------------------------------------------------------------
    static void clear_fault() {
        Base::clear_fault(kind_to_str(Kind));
    }

    // -------------------------------------------------------------------------
    // tick
    //
    // Call from a single thread at fixed rate — 10ms recommended.
    // Do NOT call from ISR.
    //
    // Parameters:
    //   throttle : desired throttle [0.0, 1.0]. Only used when ARMED.
    //
    // Returns:
    //   {}                    on success
    //   Error::NOT_INITIALIZED if called before init()
    // -------------------------------------------------------------------------
    static std::expected<void, Error> tick(float throttle) {
        auto result = Base::tick_base(kind_to_str(Kind));
        if (!result) return result;

        if (Base::state == Base::ActuatorState::ARMED) {
            const float clamped = std::clamp(throttle, 0.0f, 1.0f);
            target_throttle = clamped;
            return Base::write_pulse_us(throttle_to_pulse_us(clamped));
        }

        return {};
    }

    // -------------------------------------------------------------------------
    // set_target_throttle
    //
    // Command throttle directly without waiting for tick().
    // Must be ARMED.
    //
    // Returns:
    //   {}                on success
    //   Error::NOT_ARMED  if not ARMED
    //   Error::OUT_OF_RANGE if throttle outside [0.0, 1.0]
    // -------------------------------------------------------------------------
    static std::expected<void, Error> set_target_throttle(float throttle) {
        if (!Base::is_armed()) return std::unexpected(Error::NOT_ARMED);
        if (throttle < 0.0f || throttle > 1.0f) {
            return std::unexpected(Error::OUT_OF_RANGE);
        }
        target_throttle = throttle;
        return Base::write_pulse_us(throttle_to_pulse_us(throttle));
    }

    // -------------------------------------------------------------------------
    // Telemetry — ESC specific
    // -------------------------------------------------------------------------

    static float get_throttle_commanded() {
        return pulse_us_to_throttle(Base::commanded_pulse_us);
    }

    static float get_throttle_target() { return target_throttle; }

    // Throttle error — positive means needs more throttle.
    static float get_throttle_error() {
        return target_throttle - get_throttle_commanded();
    }
};

extern const pwm_dt_spec servo_l_pwm;
extern const pwm_dt_spec servo_r_pwm;
extern const pwm_dt_spec motor_beta1_pwm;
extern const pwm_dt_spec motor_beta2_pwm;
extern const pwm_dt_spec motor_beta3_pwm;
extern const pwm_dt_spec motor_beta4_pwm;
extern const pwm_dt_spec motor_cr_pwm;

using ServoL    = Servo<ServoKind::SERVO_L, servo_l_pwm>;
using ServoR    = Servo<ServoKind::SERVO_R, servo_r_pwm>;

using MotorBeta1 = EscMotor<MotorKind::BETA1, motor_beta1_pwm>;
using MotorBeta2 = EscMotor<MotorKind::BETA2, motor_beta2_pwm>;
using MotorBeta3 = EscMotor<MotorKind::BETA3, motor_beta3_pwm>;
using MotorBeta4 = EscMotor<MotorKind::BETA4, motor_beta4_pwm>;

using MotorCR = EscMotor<
    MotorKind::CR5025,
    motor_cr_pwm,
    1000,
    2000,
    3000
>;

#endif // ARTY_PWM_ACTUATOR_ZEPHYR_H