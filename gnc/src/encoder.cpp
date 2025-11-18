#include "encoder.h"
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(encoder, CONFIG_LOG_DEFAULT_LEVEL);

#define USER_NODE DT_PATH(zephyr_user)

// Encoder GPIOs from devicetree
static const struct gpio_dt_spec encA = GPIO_DT_SPEC_GET(USER_NODE, encoder_a_gpios);
static const struct gpio_dt_spec encB = GPIO_DT_SPEC_GET(USER_NODE, encoder_b_gpios);

// Mechanical parameters (motor side)
static constexpr float ENCODER_CPR  = 4000.0f; // counts per motor revolution (quadrature)
static constexpr float GEAR_RATIO   = 20.0f;   // gearbox ratio (motor revs per output rev)
static constexpr float DEG_PER_TICK = 360.0f / (ENCODER_CPR * GEAR_RATIO);

// Quadrature state + position
static volatile uint8_t s_prev_state = 0;  // last 2-bit state (00, 01, 11, 10)
static volatile int32_t s_position   = 0;  // signed tick count from boot

static struct gpio_callback encA_cb;
static struct gpio_callback encB_cb;

// Read current 2-bit state: bit0 = A, bit1 = B
static inline uint8_t read_state()
{
    uint8_t a = gpio_pin_get_dt(&encA) & 1;
    uint8_t b = gpio_pin_get_dt(&encB) & 1;
    return static_cast<uint8_t>((b << 1) | a);
}

// Quadrature lookup table
// rows = previous state, cols = new state, value = +1, -1, or 0
// forward: 00 -> 01 -> 11 -> 10 -> 00
// reverse: 00 -> 10 -> 11 -> 01 -> 00
static constexpr int8_t Q_TABLE[4][4] = {
    // new: 00,  01,  11,  10
    /*00*/ {  0, +1,  0, -1 },
    /*01*/ { -1,  0, +1,  0 },
    /*11*/ {  0, -1,  0, +1 },
    /*10*/ { +1,  0, -1,  0 },
};

// ISR for both A and B edges (rising + falling)
static void encoder_gpio_isr(const struct device *dev,
                             struct gpio_callback *cb,
                             gpio_port_pins_t pins)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(cb);
    ARG_UNUSED(pins);

    uint8_t new_state = read_state();
    int8_t delta = Q_TABLE[s_prev_state][new_state];

    if (delta != 0) {
        s_position += delta;
    }

    s_prev_state = new_state;
}

int encoder_init()
{
    // Check that GPIO controllers are ready
    if (!device_is_ready(encA.port) || !device_is_ready(encB.port)) {
        LOG_ERR("Encoder GPIO not ready");
        return -ENODEV;
    }

    int err = gpio_pin_configure_dt(&encA, GPIO_INPUT);
    if (err) {
        LOG_ERR("Failed to configure encoder A pin: err %d", err);
        return err;
    }

    err = gpio_pin_configure_dt(&encB, GPIO_INPUT);
    if (err) {
        LOG_ERR("Failed to configure encoder B pin: err %d", err);
        return err;
    }

    // Initialize previous state from current A/B levels
    s_prev_state = read_state();

    // Interrupt on both edges for A and B
    err = gpio_pin_interrupt_configure_dt(&encA, GPIO_INT_EDGE_BOTH);
    if (err) {
        LOG_ERR("Failed to configure encoder A interrupt: err %d", err);
        return err;
    }

    err = gpio_pin_interrupt_configure_dt(&encB, GPIO_INT_EDGE_BOTH);
    if (err) {
        LOG_ERR("Failed to configure encoder B interrupt: err %d", err);
        return err;
    }

    // Use *_dt helpers per review
    gpio_init_callback(&encA_cb, encoder_gpio_isr, BIT(encA.pin));
    gpio_add_callback_dt(&encA, &encA_cb);

    gpio_init_callback(&encB_cb, encoder_gpio_isr, BIT(encB.pin));
    gpio_add_callback_dt(&encB, &encB_cb);

    LOG_INF("Encoder initialized (4-state quadrature)");
    return 0;
}

int32_t encoder_get_position()
{
    return s_position;
}

float encoder_get_degrees()
{
    // Convert counts → output-shaft degrees using CPR + gearbox ratio
    return static_cast<float>(s_position) * DEG_PER_TICK;
}
