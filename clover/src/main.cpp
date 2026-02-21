#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/usb/usb_device.h>

#include "Controller.h"
#include "server.h"
#include "pts.h"
#include "ThrottleValve.h"

extern "C" {
#include <app/drivers/blink.h>
}

LOG_MODULE_REGISTER(Main, CONFIG_LOG_DEFAULT_LEVEL);

// Keep the build safety check for USB Console
BUILD_ASSERT(
    DT_NODE_HAS_COMPAT(DT_CHOSEN(zephyr_console), zephyr_cdc_acm_uart),
    "Console device is not ACM CDC UART device");

int main()
{
    // 1. Status LED - Vital for visual "heartbeat" on the board
    const device* blink = DEVICE_DT_GET(DT_NODELABEL(blink_led));
    if (device_is_ready(blink)) {
        blink_set_period_ms(blink, 1000u);
    }

    // 2. Serial over USB setup - Vital for seeing LOG_INF messages
    if (usb_enable(nullptr) == 0) {
        const device* usb_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
        for (int i = 0; i < 30; ++i) {
            k_sleep(K_MSEC(100));
            uint32_t dtr = 0;
            uart_line_ctrl_get(usb_dev, UART_LINE_CTRL_DTR, &dtr);
            if (dtr) break;
        }
    }

    LOG_INF("Starting Clover Engine Controller...");

    // 3. Initialize Physical Valves
    auto fuel_res = FuelValve::init();
    if (!fuel_res) {
        LOG_ERR("Fuel Valve Init Failed: %s", fuel_res.error().build_message().c_str());
    }

    auto lox_res = LoxValve::init();
    if (!lox_res) {
        LOG_ERR("Lox Valve Init Failed: %s", lox_res.error().build_message().c_str());
    }

    // 4. Initialize Sensors
    auto pts_result = pts_init();
    if (!pts_result) {
        LOG_ERR("PTS Init Failed: %s", pts_result.error().build_message().c_str());
    }

    // 5. Initialize the State Machine Controller
    // This starts the 1ms internal control loop
    Controller::get().init();

    // 6. Start the Network Servers
    // This starts the Command (TCP) and Data (UDP) threads
    serve_connections();

    LOG_INF("System Ready. All services started.");

    // Keep main thread alive
    while (true) {
        k_sleep(K_FOREVER);
    }

    return 0;
}
