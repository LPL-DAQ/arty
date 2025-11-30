#ifndef CLOVER_THROTTLEVALVE_H
#define CLOVER_THROTTLEVALVE_H

#include <cstdint>
#include <zephyr/drivers/gpio.h>

constexpr float MICROSTEPS = 8.0f;
constexpr float GEARBOX_RATIO = 5.0f;
constexpr float STEPS_PER_REVOLUTION = 200.0f;
constexpr float DEG_PER_STEP = 360.0f / STEPS_PER_REVOLUTION / GEARBOX_RATIO / MICROSTEPS;

constexpr float MAX_VELOCITY = 225.0f * 4; // In deg/s
constexpr float MAX_ACCELERATION = 12000.0f * 40; // In deg/s^2

enum class ValveKind {
    FUEL,
    LOX,
};

template<ValveKind K>
class ThrottleValve {
private:
    ThrottleValve(const gpio_dt_spec pul, const gpio_dt_spec dir, const gpio_dt_spec ena, const gpio_dt_spec enc_a, const gpio_dt_spec enc_b, device *control_counter);

    /// Internal step count of what has been sent to the driver. We use a closed-loop driver so this does not
    /// necessarily match the true valve position.
    static volatile int steps = 0;
    /// CPU cycle count of last interrupt.
    static volatile uint64_t last_pulse_cycle = 0;
    /// CPU cycles since last time pulse interrupt was called.
    static volatile uint64_t pulse_interval_cycles = 0;

    /// Sends control signal to controller, each rising edge on PUL is one step.
    static void pulse(const struct device *, void *);

    //

    static struct motor_lock
    static float velocity = 0;
    static float acceleration = 0;




public:
    int init();

    void tick();

    int calibrate();

    int move(float degrees);

    int stop();


};

template<ValveKind K>
ThrottleValve(const gpio_dt_spec pul, const gpio_dt_spec dir, const gpio_dt_spec ena, const gpio_dt_spec enc_a, const gpio_dt_spec enc_b, device *control_counter)
{

}

template<gpio_dt_spec pul, gpio_dt_spec dir, gpio_dt_spec ena, gpio_dt_spec enc_a, gpio_dt_spec enc_b, device *control_counter>
int class ThrottleValve<pul, dir, ena, enc_a, enc_b, control_counter>::init()
{
    LOG_INF("Initializing throttle valve...");

    if (!device_is_ready(pul_gpio.port) || !device_is_ready(dir_gpio.port)) {
        LOG_ERR("GPIO device(s) not ready");
        return -ENODEV;
    }

    gpio_pin_configure_dt(&pul_gpio, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&dir_gpio, GPIO_OUTPUT_INACTIVE);

    if (!device_is_ready(stepper_pulse_counter_dev)) {
    LOG_ERR("Stepper timer device is not ready.");
    return 1;
    }

    LOG_INF("Throttle valve initialized.");

    return 0;
}

int throttle_valve_init();

int throttle_valve_start_calibrate();

float throttle_valve_get_pos();

float throttle_valve_get_velocity();

float throttle_valve_get_acceleration();

uint64_t throttle_valve_get_nsec_per_pulse();

void throttle_valve_tick();

void throttle_valve_move(float degrees);

void throttle_valve_stop();

int throttle_valve_set_open();

int throttle_valve_set_closed();

ThrottleValve fuel_valve{
        GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), fuel_valve_stepper_pul_gpios),
        GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), fuel_valve_stepper_dir_gpios),
        GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), fuel_valve_stepper_ena_gpios),
        GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), fuel_valve_encoder_a_gpios),
        GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), fuel_valve_encoder_a_gpios),
        DEVICE_DT_GET(DT_ALIAS(stepper_pulse_counter))
};
ThrottleValve lox_valve();

#endif //CLOVER_THROTTLEVALVE_H
