#include "encoder.h"
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(encoder, CONFIG_LOG_DEFAULT_LEVEL);

#define USER_NODE DT_PATH(zephyr_user)

static const struct gpio_dt_spec encA = GPIO_DT_SPEC_GET(USER_NODE, encoder_a_gpios);
static const struct gpio_dt_spec encB = GPIO_DT_SPEC_GET(USER_NODE, encoder_b_gpios);

/*
* Encoder state variables.
*
* s_position:
*   A global count of "ticks" since boot.
*   Positive → forward, negative → reverse.
*
* s_last_delta:
*   Most recent step direction (+1 or -1).
*   Useful for "instantaneous velocity" or stall detection.
*
*/
static volatile int32_t s_position = 0;
static volatile int32_t s_last_delta = 0;

static struct gpio_callback encA_cb;
static struct gpio_callback encB_cb;

/*
 * encoder_gpio_isr()
 *
 * This ISR runs on ANY edge of A or B (both rising + falling).
 *
 * It performs quadrature decoding using a simple rule:
 *   - If A == B -> fwd
 *   - If A != B -> rev
 *   - When A changes, comparing A and B reveals direction
 *   - Same logic applies when B changes (if both pins trigger interrupt)
 *
 * This is not the most advanced quadrature decoder (lookup table),
 * but it is stable enough for motor valve applications.
 */
static void encoder_gpio_isr(const struct device *dev,
                             struct gpio_callback *cb,
                             gpio_port_pins_t pins) {
    ARG_UNUSED(dev);
    ARG_UNUSED(cb);
    ARG_UNUSED(pins);

    int a = gpio_pin_get_dt(&encA);
    int b = gpio_pin_get_dt(&encB);

    int32_t delta = (a == b) ? 1 : -1;

    s_position += delta;
    s_last_delta = delta;
}

int encoder_init()
{
    if (!device_is_ready(encA.port) || !device_is_ready(encB.port)) {
        LOG_ERR("Encoder GPIO ports not ready");
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

    gpio_init_callback(&encA_cb, encoder_gpio_isr, BIT(encA.pin));
    gpio_add_callback(encA.port, &encA_cb);

    gpio_init_callback(&encB_cb, encoder_gpio_isr, BIT(encB.pin));
    gpio_add_callback(encB.port, &encB_cb);

    LOG_INF("Encoder initialized");
    return 0;
}

int32_t encoder_get_position() {
    return s_position;
}

int32_t encoder_get_velocity_raw() {
    return s_last_delta;
}
