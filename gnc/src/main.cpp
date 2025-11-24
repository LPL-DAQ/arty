#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/util.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/socket.h>
#include <zephyr/net/net_pkt.h>

#include "server.h"
#include "throttle_valve.h"
#include "pts.h"
#include "servotesting.h"

extern "C" {
#include <app/drivers/blink.h>
}

LOG_MODULE_REGISTER(main, CONFIG_LOG_DEFAULT_LEVEL);

BUILD_ASSERT(DT_NODE_HAS_COMPAT(DT_CHOSEN(zephyr_console), zephyr_cdc_acm_uart),
             "Console device is not ACM CDC UART device");

int main(void) {
    // Status LED
    const struct device *blink = DEVICE_DT_GET(DT_NODELABEL(blink_led));
    if (!device_is_ready(blink)) {
        return 0;
    }
    blink_set_period_ms(blink, 1000u);

    // Serial over USB setup
    if (usb_enable(nullptr)) {
        LOG_ERR("USB is not enabled.");
        while (1);
    }
    const struct device *usb_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
    uint32_t dtr = 0;
    while (!dtr) {
        uart_line_ctrl_get(usb_dev, UART_LINE_CTRL_DTR, &dtr);
        k_sleep(K_MSEC(100));
    }

    LOG_INF("Initializing throttle valve");
    int err = throttle_valve_init();
    if (err) {
        LOG_ERR("Failed to initialize throttle valve");
        return 0;
    }

    LOG_INF("Initializing PTs");
    err = pts_init();
    if (err) {
        LOG_ERR("Failed to initialize PTs");
        return 0;
    }

    LOG_INF("Initializing SERVOS");

    err = servos_init();
    if (err) {
        LOG_ERR("Failed to initialize servos");
        return 0;
    }
    servotesting_demo();

    LOG_INF("Starting server");
    serve_connections();
    while (1);
}

