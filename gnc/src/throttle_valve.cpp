#include "throttle_valve.h"

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <cmath>
#include <algorithm>

static const struct gpio_dt_spec pul_gpio = GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), stepper_pul_gpios);
static const struct gpio_dt_spec dir_gpio = GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), stepper_dir_gpios);

/// Timer for delivering pulses to driver
static const struct device *stepper_pulse_counter_dev = DEVICE_DT_GET(DT_ALIAS(stepper_pulse_counter));
volatile uint32_t pulse_counter_ticks = 0;

LOG_MODULE_REGISTER(throttle_valve, CONFIG_LOG_DEFAULT_LEVEL);

constexpr float MICROSTEPS = 8.0f;
constexpr float GEARBOX_RATIO = 5.0f;
constexpr float STEPS_PER_REVOLUTION = 200.0f;
constexpr float DEG_PER_STEP = 360.0f / STEPS_PER_REVOLUTION / GEARBOX_RATIO / MICROSTEPS;

constexpr float MAX_VELOCITY = 225.0f * 4; // In deg/s
constexpr float MAX_ACCELERATION = 12000.0f * 40; // In deg/s^2


enum MotorState {
    STOPPED,
    RUNNING,
};
static MotorState state = STOPPED;

volatile static int steps = 0;
static float velocity = 0;
static float acceleration = 0;
static volatile uint64_t last_pulse_cycle = 0;
static volatile uint64_t pulse_interval_cycles = 0;
K_MUTEX_DEFINE(motor_lock);


/// Directly controls signal to controller, each rising edge on PUL is one step.
static void pulse(const struct device *, void *) {
    // Time cycles since last pulse
    uint64_t now_cycles = k_cycle_get_64();
    pulse_interval_cycles = now_cycles - last_pulse_cycle;
    last_pulse_cycle = now_cycles;

    // Switch direction, if we must.
    // gpio high -> flipped by converter to low -> more open.
    // pgio low -> flipped by converter to high -> more close.
    if ((gpio_pin_get_dt(&dir_gpio) == 1 && velocity > 0) || (!gpio_pin_get_dt(&dir_gpio) && velocity < 0)) {
        gpio_pin_toggle_dt(&dir_gpio);
        // We need to wait a while after changing dir before for next pulses.
        return;
    }
    // high to low -> flipped by converter to low to high -> rising edge, count a step.
    if (gpio_pin_get_dt(&pul_gpio)) {
        if (gpio_pin_get_dt(&dir_gpio)) {
            steps -= 1;
        } else {
            steps += 1;
        }
    }
    gpio_pin_toggle_dt(&pul_gpio);
}

/// Initializes throttle valve driver.
int throttle_valve_init() {
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

int throttle_valve_start_calibrate() {
    LOG_INF("Beginning calibration.");

//    int err = throttle_valve_move(105.0f, 30.0f);
//    if (err) {
//        LOG_ERR("Move failed during calibration: err %d", err);
//        return err;
//    }
//    steps = static_cast<int>(90.0f / DEG_PER_STEP);
//
//    LOG_INF("Done initial movement. Backing off.");
//    throttle_valve_move(-5.0f, 0.5f);
//
//    LOG_INF("Calibrated to fully open.");
    return 0;
}

/// Moves to a certain degree position within the specified time. Does not necessarily guarantee that this will happen
/// as speed and acceleration limits will be enforced, but it is what we will target.
void throttle_valve_move(float target_deg) {
    constexpr float CONTROL_TIME = 0.001;

    float target_velocity = (target_deg - throttle_valve_get_pos()) / CONTROL_TIME;

    // If target velocity would require excessive acceleration, clamp it.
    float required_acceleration = (target_velocity - velocity) / CONTROL_TIME;
    if (required_acceleration > MAX_ACCELERATION) {
        target_velocity = velocity + CONTROL_TIME * MAX_ACCELERATION;
    } else if (required_acceleration < -MAX_ACCELERATION) {
        target_velocity = velocity - CONTROL_TIME * MAX_ACCELERATION;
    }

    // Clamp velocity
    target_velocity = std::clamp(target_velocity, -MAX_VELOCITY, MAX_VELOCITY);

    // Based on velocity, calculate pulse interval. Must divide by two as each counter trigger
    // only toggles pulse, so two triggers are needed for full step on rising edge.
    auto usec_per_pulse = static_cast<uint64_t>(1e6 / static_cast<double>(std::abs(target_velocity)) *
                                                static_cast<double>(DEG_PER_STEP) / 2.0);

    // Set true values for acceleration and velocity.
    acceleration = (target_velocity - velocity) / CONTROL_TIME;
    velocity = target_velocity;

    uint32_t ticks = std::min(counter_us_to_ticks(stepper_pulse_counter_dev, usec_per_pulse),
                              counter_get_max_top_value(stepper_pulse_counter_dev));
    counter_top_cfg pulse_counter_config{
            .ticks = ticks,
            .callback = pulse,
            .user_data = nullptr,
            .flags = 0
    };
    int err = counter_set_top_value(stepper_pulse_counter_dev, &pulse_counter_config);
    if (err) {
        LOG_ERR("Failed to set pulse counter top value: errno %d", err);
    }

    // Ensure timer is running
    counter_start(stepper_pulse_counter_dev);

    state = RUNNING;
}

void throttle_valve_stop() {
    k_mutex_lock(&motor_lock, K_FOREVER);
    counter_stop(stepper_pulse_counter_dev);
    acceleration = 0;
    velocity = 0;
    state = STOPPED;
    k_mutex_unlock(&motor_lock);
}

/// Get the current acceleration in deg/s^2. It is only updated per call to
/// throttle_valve_move.
float throttle_valve_get_acceleration() {
    return acceleration;
}

/// Get the current velocity in deg/s. It is only updated per call to
/// throttle_valve_move.
float throttle_valve_get_velocity() {
    return velocity;
}

/// Get current degree position of motor in degrees.
float throttle_valve_get_pos() {
    return static_cast<float>(steps) * DEG_PER_STEP;
}

/// Get interval between each call to pulse counter.
uint64_t throttle_valve_get_nsec_per_pulse() {
    return k_cyc_to_ns_near64(pulse_interval_cycles);
}

int throttle_valve_set_open() {
    k_mutex_lock(&motor_lock, K_FOREVER);
    if (state != MotorState::STOPPED) {
        k_mutex_unlock(&motor_lock);
        LOG_ERR("Cannot reset to position open when motor is stopped.");
        return 1;
    }
    steps = static_cast<int>(90.0f / DEG_PER_STEP);
    k_mutex_unlock(&motor_lock);
    return 0;
}

int throttle_valve_set_closed() {
    k_mutex_lock(&motor_lock, K_FOREVER);
    if (state != MotorState::STOPPED) {
        k_mutex_unlock(&motor_lock);
        LOG_ERR("Cannot reset to position closed when motor is stopped.");
        return 1;
    }
    steps = 0;
    k_mutex_unlock(&motor_lock);
    return 0;
}
