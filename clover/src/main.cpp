#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_pkt.h>
#include <zephyr/net/socket.h>
#include <zephyr/sys/util.h>
#include <zephyr/usb/usb_device.h>

#include "ThrottleValve.h"
#include "pts.h"
#include "server.h"

extern "C" {
#include <app/drivers/blink.h>
}

LOG_MODULE_REGISTER(main, CONFIG_LOG_DEFAULT_LEVEL);

BUILD_ASSERT(
    DT_NODE_HAS_COMPAT(DT_CHOSEN(zephyr_console), zephyr_cdc_acm_uart), "Console device is not ACM CDC UART device");

int main()
{
    // Status LED
    const device* blink = DEVICE_DT_GET(DT_NODELABEL(blink_led));
    if (!device_is_ready(blink)) {
        return 0;
    }
    blink_set_period_ms(blink, 1000u);

    // Serial over USB setup
    if (usb_enable(nullptr)) {
        LOG_ERR("USB is not enabled.");
        while (1) {
        }
    }

    // Try connecting to serial over usb for 3 seconds.
    const device* usb_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
    for (int i = 0; i < 30; ++i) {
        k_sleep(K_MSEC(100));

        uint32_t dtr = 0;
        uart_line_ctrl_get(usb_dev, UART_LINE_CTRL_DTR, &dtr);
        if (dtr) {
            break;
        }
    }

    LOG_INF("Initializing fuel throttle valve");
    auto result = FuelValve::init();
    if (!result) {
        LOG_ERR("%s", result.error().context("failed to initialize fuel throttle valve").build_message().c_str());
        return 1;
    }

    LOG_INF("Initializing lox throttle valve");
    result = LoxValve::init();
    if (!result) {
        LOG_ERR("%s", result.error().context("failed to initialize lox throttle valve").build_message().c_str());
        return 1;
    }

    LOG_INF("Initializing PTs");
    result = pts_init();
    if (!result) {
        LOG_ERR("%s", result.error().context("failed to initialize pts").build_message().c_str());
        return 1;
    }

    k_sleep(K_MSEC(500));
    LOG_INF("Starting server");
    serve_connections();

    k_sleep(K_FOREVER);
}
