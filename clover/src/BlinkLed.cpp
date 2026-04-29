#include "BlinkLed.h"
#include "config.h"
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(BlinkLed, CONFIG_LOG_DEFAULT_LEVEL);

static constexpr gpio_dt_spec blink_gpio = GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), blink_gpios);

static void blink()
{
    while (true) {
        k_sleep(K_MSEC(1000));
        gpio_pin_toggle_dt(&blink_gpio);
    }
}

K_THREAD_DEFINE(blink_thread, 128, blink, nullptr, nullptr, nullptr, BLINK_THREAD_PRIORITY, 0, 0);

std::expected<void, Error> BlinkLed::init()
{
    if (!device_is_ready(blink_gpio.port)) {
        return std::unexpected(Error::from_device_not_ready(blink_gpio.port).context("blink GPIO not ready"));
    }

    int err = gpio_pin_configure_dt(&blink_gpio, GPIO_OUTPUT_ACTIVE);
    if (err) {
        return std::unexpected(Error::from_code(err).context("failed to set up blink LED"));
    }

    err = gpio_pin_set_dt(&blink_gpio, 1);
    if (err) {
        return std::unexpected(Error::from_code(err).context("failed to set blink LED to on"));
    }

    return {};
}
