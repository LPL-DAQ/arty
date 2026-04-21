template
    PwmKind Kind,
    const pwm_dt_spec PwmDt,
    int MinPulseUs = 1000,
    int MaxPulseUs = 2000>
class PwmActuator {
protected:
    static constexpr float k_min_pulse_us = static_cast<float>(MinPulseUs);
    static constexpr float k_max_pulse_us = static_cast<float>(MaxPulseUs);
    static constexpr const pwm_dt_spec& pwm_gpio = PwmDt;
    inline static uint32_t last_pulse_us = static_cast<uint32_t>(MinPulseUs);

    static consteval const char* kind_to_prefix(PwmKind k) {
        switch (k) {
            case PwmKind::SERVO_X:      return "[servo_x]";
            case PwmKind::SERVO_Y:      return "[servo_y]";
            case PwmKind::BETA_TOP:     return "[beta_top]";
            case PwmKind::BETA_BOTTOM:  return "[beta_bottom]";
            case PwmKind::BETA_CW:      return "[beta_cw]";
            case PwmKind::BETA_CCW:     return "[beta_ccw]";
            case PwmKind::MOTOR_TOP:    return "[motor_top]";
            case PwmKind::MOTOR_BOTTOM: return "[motor_bottom]";
        }
        return "[pwm_actuator]";
    }

    static uint32_t clamp_pulse_us(uint32_t pulse_us) {
        const uint32_t min = static_cast<uint32_t>(k_min_pulse_us);
        const uint32_t max = static_cast<uint32_t>(k_max_pulse_us);
        return pulse_us < min ? min : (pulse_us > max ? max : pulse_us);
    }

    static std::expected<void, Error> write_pulse_us(uint32_t pulse_us) {
        constexpr uint32_t period_ns = 20000u * 1000u;
        const int err = pwm_set_dt(&pwm_gpio, period_ns, pulse_us * 1000u);
        if (err) {
            return std::unexpected(Error::from_code(err).context("PWM write failed on %s", kind_to_prefix(Kind)));
        }
        last_pulse_us = pulse_us;
        return {};
    }

public:
    static std::expected<void, Error> init() {
        LOG_MODULE_DECLARE(pwm_actuator);
        LOG_INF("%s Initializing... pulse range [%d, %d] us",
                kind_to_prefix(Kind), MinPulseUs, MaxPulseUs);

        if (!pwm_is_ready_dt(&pwm_gpio)) {
            LOG_ERR("%s PWM not ready", kind_to_prefix(Kind));
            return std::unexpected(Error::from_device_not_ready(pwm_gpio.dev).context("PWM not ready on %s", kind_to_prefix(Kind)));
        }

        auto result = write_pulse_us(static_cast<uint32_t>(k_min_pulse_us));
        if (!result) {
            LOG_ERR("%s Failed to set initial pulse width: err %d", kind_to_prefix(Kind), result.error().code());
            return result;
        }

        LOG_INF("%s Ready.", kind_to_prefix(Kind));
        return {};
    }

    static std::expected<void, Error> tick(uint32_t pulse_us) {
        LOG_MODULE_DECLARE(pwm_actuator);

        auto result = write_pulse_us(clamp_pulse_us(pulse_us));
        if (!result) {
            LOG_ERR("%s Failed to write pulse: err %d", kind_to_prefix(Kind), result.error().code());
        }
        return result;
    }

    static uint32_t get_pulse_us() {
        return last_pulse_us;
    }
};