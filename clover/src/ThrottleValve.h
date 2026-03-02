#ifndef ARTY_THROTTLEVALVE_H
#define ARTY_THROTTLEVALVE_H

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <zephyr/drivers/counter.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

constexpr float MICROSTEPS = 8.0f;
constexpr float GEARBOX_RATIO = 5.0f;
constexpr float STEPS_PER_REVOLUTION = 200.0f;
constexpr float DEG_PER_STEP = 360.0f / STEPS_PER_REVOLUTION / GEARBOX_RATIO / MICROSTEPS;

constexpr float MAX_VELOCITY = 225.0f * 4;         // In deg/s
constexpr float MAX_ACCELERATION = 12000.0f * 40;  // In deg/s^2

constexpr float ENCODER_CPR = 4000.0f;  // counts per motor revolution (quadrature)
constexpr float DEG_PER_ENCODER_COUNT = 360.0f / (ENCODER_CPR * GEARBOX_RATIO);

enum class ValveKind { FUEL, LOX };

template <
    ValveKind kind,
    gpio_dt_spec pul_dt_init,
    gpio_dt_spec dir_dt_init,
    gpio_dt_spec ena_dt_init,
    gpio_dt_spec enc_a_dt_init,
    gpio_dt_spec enc_b_dt_init,
    const device* control_counter_dt_init>
class ThrottleValve {
private:
    enum class ValveState {
        STOPPED,
        RUNNING,
        OFF,
    };

    constexpr static gpio_dt_spec pul_gpio = pul_dt_init;
    constexpr static gpio_dt_spec ena_gpio = ena_dt_init;
    constexpr static gpio_dt_spec dir_gpio = dir_dt_init;
    constexpr static gpio_dt_spec enc_a_gpio = enc_a_dt_init;
    constexpr static gpio_dt_spec enc_b_gpio = enc_b_dt_init;
    constexpr static const device* control_counter = control_counter_dt_init;

    static consteval const char* kind_to_prefix(ValveKind valve_kind);

    inline static k_mutex motor_lock = {};  // Set in init()
    inline static ValveState state = ValveState::STOPPED;
    inline static ValveState prevState = ValveState::STOPPED;

    inline static float velocity = 0;
    inline static float acceleration = 0;
    /// Internal step count of what has been sent to the driver. We use a closed-loop driver so this does not
    /// necessarily match the true valve position.
    inline static volatile int steps = 0;
    /// CPU cycle count of last interrupt.
    inline static volatile uint64_t last_pulse_cycle = 0;
    /// CPU cycles since last time pulse interrupt was called.
    inline static volatile uint64_t pulse_interval_cycles = 0;

    inline static float current_encoder_position = 0.0f;
    inline static float previous_encoder_position = 0.0f;

    static void control_pulse_isr(const device*, void*);

    inline static volatile int encoder_count = 0;
    inline static volatile uint8_t prev_encoder_state = 0;  // Set in init()
    inline static gpio_callback encoder_a_callback = {};    // Set in init()
    inline static gpio_callback encoder_b_callback = {};    // Set in init()
    static void encoder_update_isr(const device*, gpio_callback*, gpio_port_pins_t);

    static uint8_t read_encoder_state();

public:
    ThrottleValve() = delete;

    static int init();

    static int tick(bool on, bool set_pos, float target_deg);
    static void move(float target_deg);
    static void stop();
    static void reset_pos(float new_pos);
    static void power_on(bool on);

    static float get_pos_internal();
    static float get_pos_encoder();
    static float get_velocity();
    static float get_encoder_velocity();
    static float get_acceleration();
    static uint64_t get_nsec_per_pulse();
    static bool get_power_on();
};

/// Generates prefix used for log statements from valve kind.
template <
    ValveKind kind,
    gpio_dt_spec pul_dt_init,
    gpio_dt_spec dir_dt_init,
    gpio_dt_spec ena_dt_init,
    gpio_dt_spec enc_a_dt_init,
    gpio_dt_spec enc_b_dt_init,
    const device* control_counter_dt_init>
consteval const char*
ThrottleValve<kind, pul_dt_init, dir_dt_init, ena_dt_init, enc_a_dt_init, enc_b_dt_init, control_counter_dt_init>::kind_to_prefix(ValveKind valve_kind)
{
    switch (valve_kind) {
    case ValveKind::FUEL:
        return "[fuel]";
    case ValveKind::LOX:
        return "[lox]";
    }
}

/// Sends control signal to controller, each rising edge on PUL is one step. Called as counter ISR.
template <
    ValveKind kind,
    gpio_dt_spec pul_dt_init,
    gpio_dt_spec dir_dt_init,
    gpio_dt_spec ena_dt_init,
    gpio_dt_spec enc_a_dt_init,
    gpio_dt_spec enc_b_dt_init,
    const device* control_counter_dt_init>
void ThrottleValve<kind, pul_dt_init, dir_dt_init, ena_dt_init, enc_a_dt_init, enc_b_dt_init, control_counter_dt_init>::control_pulse_isr(
    const struct device*, void*)
{
    LOG_MODULE_DECLARE(throttle_valve);

    // Time cycles since last pulse
    uint64_t now_cycles = k_cycle_get_64();
    pulse_interval_cycles = now_cycles - last_pulse_cycle;
    last_pulse_cycle = now_cycles;

    int prev_dir_state = gpio_pin_get_dt(&dir_gpio);
    if (prev_dir_state < 0) [[unlikely]] {
        //        LOG_ERR("%s Failed to read from direction GPIO in pulse ISR: err %d", kind_to_prefix(kind), prev_dir_state);
        prev_dir_state = 0;
    }
    int prev_pul_state = gpio_pin_get_dt(&pul_gpio);
    if (prev_pul_state < 0) [[unlikely]] {
        //        LOG_ERR("%s Failed to read from pulse GPIO in pulse ISR: err %d", kind_to_prefix(kind), prev_pul_state);
        prev_pul_state = 0;
    }

    // Switch direction, if we must. Positive velocity -> dir gpio should be high. Negative velocity -> dir gpio should
    // be low.
    if ((!prev_dir_state && velocity > 0) || (prev_dir_state && velocity < 0)) {
        gpio_pin_toggle_dt(&dir_gpio);
        // We need to wait a while after changing dir before we continue sending pulses.
        return;
    }
    // About to switch low -> high. This is a rising edge, so count a step.
    if (!prev_pul_state) {
        if (!prev_dir_state) {
            steps = steps - 1;
        }
        else {
            steps = steps + 1;
        }
    }
    gpio_pin_toggle_dt(&pul_gpio);
}

/// ISR on any edge
template <
    ValveKind kind,
    gpio_dt_spec pul_dt_init,
    gpio_dt_spec dir_dt_init,
    gpio_dt_spec ena_dt_init,
    gpio_dt_spec enc_a_dt_init,
    gpio_dt_spec enc_b_dt_init,
    const device* control_counter_dt_init>
void ThrottleValve<kind, pul_dt_init, dir_dt_init, ena_dt_init, enc_a_dt_init, enc_b_dt_init, control_counter_dt_init>::encoder_update_isr(
    const device*, struct gpio_callback*, gpio_port_pins_t)
{
    // Quadrature encoder lookup table. Index as ENCODER_STEP_TABLE[old_state][new_state] to output either -1, 0, or +1
    // for the steps that the encoder has advanced.
    // forward: 00 → 01 → 11 → 10 → 00
    // reverse: 00 → 10 → 11 → 01 → 00
    constexpr int8_t ENCODER_STEP_TABLE[4][4] = {
        {0, +1, -1, 0},
        {-1, 0, 0, +1},
        {+1, 0, 0, -1},
        {0, -1, +1, 0},
    };

    uint8_t new_state = read_encoder_state();
    // Temp hack: Fuel encoder line is reversed.
    if constexpr (kind == ValveKind::FUEL) {
        encoder_count -= ENCODER_STEP_TABLE[prev_encoder_state][new_state];
    }
    else {
        encoder_count += ENCODER_STEP_TABLE[prev_encoder_state][new_state];
    }
    prev_encoder_state = new_state;
}

/// Read current 2-bit encoder state: bit0 = A, bit1 = B. Will always return [0b00 to 0b11]. This may be called from
/// interrupts.
template <
    ValveKind kind,
    gpio_dt_spec pul_dt_init,
    gpio_dt_spec dir_dt_init,
    gpio_dt_spec ena_dt_init,
    gpio_dt_spec enc_a_dt_init,
    gpio_dt_spec enc_b_dt_init,
    const device* control_counter_dt_init>
uint8_t ThrottleValve<kind, pul_dt_init, dir_dt_init, ena_dt_init, enc_a_dt_init, enc_b_dt_init, control_counter_dt_init>::read_encoder_state()
{
    LOG_MODULE_DECLARE(throttle_valve);

    int a = gpio_pin_get_dt(&enc_a_gpio);
    // Technically, gpio_pin_get_dt may return an error code. Thus, we clamp it to 0 or 1.
    if (a < 0) [[unlikely]] {
        //        LOG_ERR("%s Failed to read from encoder A in read_encoder_state: err %d", kind_to_prefix(kind), a);
        a = 0;
    }

    int b = gpio_pin_get_dt(&enc_b_gpio);
    // Ditto.
    if (b < 0) [[unlikely]] {
        //        LOG_ERR("%s Failed to read from encoder B in read_encoder_state: err %d", kind_to_prefix(kind), b);
        b = 0;
    }
    return (static_cast<uint8_t>(b) << 1) | static_cast<uint8_t>(a);
}

/// Initialize the throttle valve.
template <
    ValveKind kind,
    gpio_dt_spec pul_dt_init,
    gpio_dt_spec dir_dt_init,
    gpio_dt_spec ena_dt_init,
    gpio_dt_spec enc_a_dt_init,
    gpio_dt_spec enc_b_dt_init,
    const device* control_counter_dt_init>
int ThrottleValve<kind, pul_dt_init, dir_dt_init, ena_dt_init, enc_a_dt_init, enc_b_dt_init, control_counter_dt_init>::init()
{
    LOG_MODULE_DECLARE(throttle_valve);
    LOG_INF("%s Initializing throttle valve...", kind_to_prefix(kind));

    if (!device_is_ready(pul_gpio.port)) {
        LOG_ERR("%s Pulse GPIO not ready: err %d", kind_to_prefix(kind), pul_gpio.port->state->init_res);
        return -ENODEV;
    }
    if (!device_is_ready(dir_gpio.port)) {
        LOG_ERR("%s Direction GPIO not ready: err %d", kind_to_prefix(kind), dir_gpio.port->state->init_res);
        return -ENODEV;
    }
    if (!device_is_ready(ena_gpio.port)) {
        LOG_ERR("%s Enable GPIO not ready: err %d", kind_to_prefix(kind), ena_gpio.port->state->init_res);
        return -ENODEV;
    }
    if (!device_is_ready(enc_a_gpio.port)) {
        LOG_ERR("%s Encoder A GPIO not ready: err %d", kind_to_prefix(kind), enc_a_gpio.port->state->init_res);
        return -ENODEV;
    }
    if (!device_is_ready(enc_b_gpio.port)) {
        LOG_ERR("%s Encoder B GPIO not ready: err %d", kind_to_prefix(kind), enc_b_gpio.port->state->init_res);
        return -ENODEV;
    }
    if (!device_is_ready(control_counter)) {
        LOG_ERR("%s Stepper driver control signal counter not ready: err %d", kind_to_prefix(kind), control_counter->state->init_res);
        return -ENODEV;
    }

    int err;
    err = gpio_pin_configure_dt(&pul_gpio, GPIO_OUTPUT_INACTIVE);
    if (err) {
        LOG_ERR("%s Failed to put pulse GPIO in default low: err %d", kind_to_prefix(kind), err);
        return err;
    }
    err = gpio_pin_configure_dt(&dir_gpio, GPIO_OUTPUT_INACTIVE);
    if (err) {
        LOG_ERR("%s Failed to put direction GPIO in default low: err %d", kind_to_prefix(kind), err);
        return err;
    }
    // err = gpio_pin_configure_dt(&ena_gpio, GPIO_OUTPUT_ACTIVE);
    err = gpio_pin_configure_dt(&ena_gpio, GPIO_OUTPUT_ACTIVE);

    if (err) {
        LOG_ERR("%s Failed to put enable GPIO in default high: err %d", kind_to_prefix(kind), err);
        return err;
    }
    err = gpio_pin_configure_dt(&enc_a_gpio, GPIO_INPUT);
    if (err) {
        LOG_ERR("%s Failed to set encoder A gpio as input: err %d", kind_to_prefix(kind), err);
        return err;
    }
    err = gpio_pin_configure_dt(&enc_b_gpio, GPIO_INPUT);
    if (err) {
        LOG_ERR("%s Failed to set encoder B gpio as input: err %d", kind_to_prefix(kind), err);
        return err;
    }

    // Set initial encoder state.
    prev_encoder_state = read_encoder_state();

    // Set interrupts for encoder state.
    err = gpio_pin_interrupt_configure_dt(&enc_a_gpio, GPIO_INT_EDGE_BOTH);
    if (err) {
        LOG_ERR("%s Failed to enable encoder A pin interrupts: err %d", kind_to_prefix(kind), err);
        return err;
    }
    err = gpio_pin_interrupt_configure_dt(&enc_b_gpio, GPIO_INT_EDGE_BOTH);
    if (err) {
        LOG_ERR("%s Failed to enable encoder B pin interrupts: err %d", kind_to_prefix(kind), err);
        return err;
    }

    gpio_init_callback(&encoder_a_callback, encoder_update_isr, BIT(enc_a_gpio.pin));
    gpio_add_callback_dt(&enc_a_gpio, &encoder_a_callback);
    gpio_init_callback(&encoder_b_callback, encoder_update_isr, BIT(enc_b_gpio.pin));
    gpio_add_callback_dt(&enc_b_gpio, &encoder_b_callback);

    k_mutex_init(&motor_lock);

    LOG_INF("Throttle valve initialized.");

    return 0;
}

template <
    ValveKind kind,
    gpio_dt_spec pul_dt_init,
    gpio_dt_spec dir_dt_init,
    gpio_dt_spec ena_dt_init,
    gpio_dt_spec enc_a_dt_init,
    gpio_dt_spec enc_b_dt_init,
    const device* control_counter_dt_init>
int ThrottleValve<kind, pul_dt_init, dir_dt_init, ena_dt_init, enc_a_dt_init, enc_b_dt_init, control_counter_dt_init>::tick(bool on, bool set_pos, float target_deg)
{
    previous_encoder_position = current_encoder_position;
    current_encoder_position = get_pos_encoder();

    LOG_MODULE_DECLARE(throttle_valve);
    prevState = state;
    if (!on) {
        state = ValveState::OFF;
    } else if (set_pos) {
        state = ValveState::RUNNING;
    } else {
        state = ValveState::STOPPED;
    }

    switch (state) {
        case ValveState::OFF:
            power_on(false);
            break;
        case ValveState::STOPPED:
            stop();
            break;
        case ValveState::RUNNING:
            move(target_deg);
    }
    return 0;
}

template <
    ValveKind kind,
    gpio_dt_spec pul_dt_init,
    gpio_dt_spec dir_dt_init,
    gpio_dt_spec ena_dt_init,
    gpio_dt_spec enc_a_dt_init,
    gpio_dt_spec enc_b_dt_init,
    const device* control_counter_dt_init>
void ThrottleValve<kind, pul_dt_init, dir_dt_init, ena_dt_init, enc_a_dt_init, enc_b_dt_init, control_counter_dt_init>::move(float target_deg)
{
    LOG_MODULE_DECLARE(throttle_valve);

    power_on(true);
    constexpr float CONTROL_TIME = 0.001;

    float target_velocity = (target_deg - get_pos_internal()) / CONTROL_TIME;

    // If target velocity would require excessive acceleration, clamp it.
    float required_acceleration = (target_velocity - velocity) / CONTROL_TIME;
    if (required_acceleration > MAX_ACCELERATION) {
        target_velocity = velocity + CONTROL_TIME * MAX_ACCELERATION;
    }
    else if (required_acceleration < -MAX_ACCELERATION) {
        target_velocity = velocity - CONTROL_TIME * MAX_ACCELERATION;
    }

    // Clamp velocity
    target_velocity = std::clamp(target_velocity, -MAX_VELOCITY, MAX_VELOCITY);

    // Based on velocity, calculate pulse interval. Must divide by two as each counter trigger
    // only toggles pulse, so two triggers are needed for full step on rising edge.
    auto usec_per_pulse = static_cast<uint64_t>(1e6 / static_cast<double>(std::abs(target_velocity)) * static_cast<double>(DEG_PER_STEP) / 2.0);

    // Set true values for acceleration and velocity.
    acceleration = (target_velocity - velocity) / CONTROL_TIME;
    velocity = target_velocity;

    // Schedule pulses.
    uint32_t ticks = std::min(counter_us_to_ticks(control_counter, usec_per_pulse), counter_get_max_top_value(control_counter));
    counter_top_cfg pulse_counter_config{.ticks = ticks, .callback = control_pulse_isr, .user_data = nullptr, .flags = 0};

    int err = counter_set_top_value(control_counter, &pulse_counter_config);
    if (err) [[unlikely]] {
        LOG_ERR("%s Failed to set pulse counter top value: err %d", kind_to_prefix(kind), err);
    }

    // Ensure timer is running
    err = counter_start(control_counter);
    if (err) [[unlikely]] {
        LOG_ERR("%s Failed to start pulse counter: err %d", kind_to_prefix(kind), err);
    }

}

template <
    ValveKind kind,
    gpio_dt_spec pul_dt_init,
    gpio_dt_spec dir_dt_init,
    gpio_dt_spec ena_dt_init,
    gpio_dt_spec enc_a_dt_init,
    gpio_dt_spec enc_b_dt_init,
    const device* control_counter_dt_init>
void ThrottleValve<kind, pul_dt_init, dir_dt_init, ena_dt_init, enc_a_dt_init, enc_b_dt_init, control_counter_dt_init>::stop()
{
    power_on(true);
    k_mutex_lock(&motor_lock, K_FOREVER);
    counter_stop(control_counter);
    acceleration = 0;
    velocity = 0;
    state = ThrottleValve::ValveState::STOPPED;
    k_mutex_unlock(&motor_lock);
}

/// Reset internal and encoder positions to a new value without moving the motor. The current physical valve position
/// will be set as the input new position.
template <
    ValveKind kind,
    gpio_dt_spec pul_dt_init,
    gpio_dt_spec dir_dt_init,
    gpio_dt_spec ena_dt_init,
    gpio_dt_spec enc_a_dt_init,
    gpio_dt_spec enc_b_dt_init,
    const device* control_counter_dt_init>
void ThrottleValve<kind, pul_dt_init, dir_dt_init, ena_dt_init, enc_a_dt_init, enc_b_dt_init, control_counter_dt_init>::reset_pos(float new_pos)
{
    LOG_MODULE_DECLARE(throttle_valve);

    k_mutex_lock(&motor_lock, K_FOREVER);
    if (state != ValveState::STOPPED) {
        k_mutex_unlock(&motor_lock);
        LOG_ERR("Cannot reset to position open when motor is stopped.");
        return;
    }

    int new_steps = static_cast<int>(std::round(new_pos / DEG_PER_STEP));
    steps = new_steps;
    int new_encoder_count = static_cast<int>(std::round(new_pos / DEG_PER_ENCODER_COUNT));
    encoder_count = new_encoder_count;

    k_mutex_unlock(&motor_lock);
}

/// turns on or off the driver, using ena as logic to a MOSFET slide switch
template <
    ValveKind kind,
    gpio_dt_spec pul_dt_init,
    gpio_dt_spec dir_dt_init,
    gpio_dt_spec ena_dt_init,
    gpio_dt_spec enc_a_dt_init,
    gpio_dt_spec enc_b_dt_init,
    const device* control_counter_dt_init>
void ThrottleValve<kind, pul_dt_init, dir_dt_init, ena_dt_init, enc_a_dt_init, enc_b_dt_init, control_counter_dt_init>::power_on(bool on)
{
    LOG_MODULE_DECLARE(throttle_valve);
    k_mutex_lock(&motor_lock, K_FOREVER);

    // HIGH is ON
    if (on) {
        if (prevState == ValveState::OFF) {
            LOG_INF("%s Powering on valve driver", kind_to_prefix(kind));
            gpio_pin_set_dt(&ena_gpio, 1);
        }
    }
    else {
        if (prevState != ValveState::OFF) {
            LOG_INF("%s Powering off valve driver", kind_to_prefix(kind));
            gpio_pin_set_dt(&ena_gpio, 0);
        }
    }

    k_mutex_unlock(&motor_lock);
}



/// Get internal motor position via how many pulses we sent to the controller.
template <
    ValveKind kind,
    gpio_dt_spec pul_dt_init,
    gpio_dt_spec dir_dt_init,
    gpio_dt_spec ena_dt_init,
    gpio_dt_spec enc_a_dt_init,
    gpio_dt_spec enc_b_dt_init,
    const device* control_counter_dt_init>
float ThrottleValve<kind, pul_dt_init, dir_dt_init, ena_dt_init, enc_a_dt_init, enc_b_dt_init, control_counter_dt_init>::get_pos_internal()
{
    return static_cast<float>(steps) * DEG_PER_STEP;
}

/// Get motor position from encoder feedback.
template <
    ValveKind kind,
    gpio_dt_spec pul_dt_init,
    gpio_dt_spec dir_dt_init,
    gpio_dt_spec ena_dt_init,
    gpio_dt_spec enc_a_dt_init,
    gpio_dt_spec enc_b_dt_init,
    const device* control_counter_dt_init>
float ThrottleValve<kind, pul_dt_init, dir_dt_init, ena_dt_init, enc_a_dt_init, enc_b_dt_init, control_counter_dt_init>::get_pos_encoder()
{
    return static_cast<float>(encoder_count) * DEG_PER_ENCODER_COUNT;
}

/// Get the current velocity in deg/s. It is only updated per call to
/// throttle_valve_move. NOT A MEASUREMENT OF ENCODER
template <
    ValveKind kind,
    gpio_dt_spec pul_dt_init,
    gpio_dt_spec dir_dt_init,
    gpio_dt_spec ena_dt_init,
    gpio_dt_spec enc_a_dt_init,
    gpio_dt_spec enc_b_dt_init,
    const device* control_counter_dt_init>
float ThrottleValve<kind, pul_dt_init, dir_dt_init, ena_dt_init, enc_a_dt_init, enc_b_dt_init, control_counter_dt_init>::get_velocity()
{
    return velocity;
}


/// Get the current velocity in deg/s. It is only updated per call to
/// throttle_valve_move. NOT A MEASUREMENT OF ENCODER
template <
    ValveKind kind,
    gpio_dt_spec pul_dt_init,
    gpio_dt_spec dir_dt_init,
    gpio_dt_spec ena_dt_init,
    gpio_dt_spec enc_a_dt_init,
    gpio_dt_spec enc_b_dt_init,
    const device* control_counter_dt_init>
float ThrottleValve<kind, pul_dt_init, dir_dt_init, ena_dt_init, enc_a_dt_init, enc_b_dt_init, control_counter_dt_init>::get_encoder_velocity()
{
    return (previous_encoder_position - current_encoder_position) / 0.001f; // CONTROL_TIME
}


/// Get the current acceleration in deg/s^2. It is only updated per call to
/// throttle_valve_move.
template <
    ValveKind kind,
    gpio_dt_spec pul_dt_init,
    gpio_dt_spec dir_dt_init,
    gpio_dt_spec ena_dt_init,
    gpio_dt_spec enc_a_dt_init,
    gpio_dt_spec enc_b_dt_init,
    const device* control_counter_dt_init>
float ThrottleValve<kind, pul_dt_init, dir_dt_init, ena_dt_init, enc_a_dt_init, enc_b_dt_init, control_counter_dt_init>::get_acceleration()
{
    return acceleration;
}

/// Get interval between each call to pulse counter.
template <
    ValveKind kind,
    gpio_dt_spec pul_dt_init,
    gpio_dt_spec dir_dt_init,
    gpio_dt_spec ena_dt_init,
    gpio_dt_spec enc_a_dt_init,
    gpio_dt_spec enc_b_dt_init,
    const device* control_counter_dt_init>
uint64_t ThrottleValve<kind, pul_dt_init, dir_dt_init, ena_dt_init, enc_a_dt_init, enc_b_dt_init, control_counter_dt_init>::get_nsec_per_pulse()
{
    return k_cyc_to_ns_near64(pulse_interval_cycles);
}

/// Get power status
template <
    ValveKind kind,
    gpio_dt_spec pul_dt_init,
    gpio_dt_spec dir_dt_init,
    gpio_dt_spec ena_dt_init,
    gpio_dt_spec enc_a_dt_init,
    gpio_dt_spec enc_b_dt_init,
    const device* control_counter_dt_init>
bool ThrottleValve<kind, pul_dt_init, dir_dt_init, ena_dt_init, enc_a_dt_init, enc_b_dt_init, control_counter_dt_init>::get_power_on()
{
    if (state == ValveState::OFF) {
        return false;
    }
    return true;
}

typedef ThrottleValve<
    ValveKind::FUEL,
    GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), fuel_valve_stepper_pul_gpios),
    GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), fuel_valve_stepper_dir_gpios),
    GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), fuel_valve_stepper_ena_gpios),
    GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), fuel_valve_encoder_a_gpios),
    GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), fuel_valve_encoder_b_gpios),
    DEVICE_DT_GET(DT_ALIAS(fuel_valve_stepper_pulse_counter))>
    FuelValve;

typedef ThrottleValve<
    ValveKind::LOX,
    GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), lox_valve_stepper_pul_gpios),
    GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), lox_valve_stepper_dir_gpios),
    GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), lox_valve_stepper_ena_gpios),
    GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), lox_valve_encoder_a_gpios),
    GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), lox_valve_encoder_b_gpios),
    DEVICE_DT_GET(DT_ALIAS(lox_valve_stepper_pulse_counter))>
    LoxValve;

#endif  // ARTY_THROTTLEVALVE_H
