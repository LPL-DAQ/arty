#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_pkt.h>
#include <zephyr/net/socket.h>
#include <zephyr/sys/util.h>
#include <zephyr/usb/usb_device.h>

#include "Controller.h"
#include "sensors/AnalogSensors.h"
#include "server.h"

#include "sensors/adc_ads7953.h"

#ifdef CONFIG_HORNET

#elif CONFIG_RANGER
#include "ranger/ThrottleValve.h"
#include "ranger/Valves.h"

#else
#error Either CONFIG_HORNET or CONFIG_RANGER must be set.
#endif

LOG_MODULE_REGISTER(main, CONFIG_LOG_DEFAULT_LEVEL);

int main(void)
{
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

    // const struct device* uart = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
    // if (!device_is_ready(uart)) {
    //     return 0;
    // }

#ifdef CONFIG_RANGER

    LOG_INF("Initializing general valves");
    if (auto result = Valves::init(); !result) {
        LOG_ERR("Failed to initialize general valves: %s", result.error().build_message().c_str());
        return 0;
    }

    LOG_INF("Initializing fuel throttle valve");
    if (auto result = FuelValve::init(); !result) {
        LOG_ERR("Failed to initialize fuel throttle valve: %s", result.error().build_message().c_str());
        return 0;
    }

    LOG_INF("Initializing lox throttle valve");
    if (auto result = LoxValve::init(); !result) {
        LOG_ERR("Failed to initialize lox throttle valve: %s", result.error().build_message().c_str());
        return 0;
    }

#endif

    // LOG_INF("Initializing LiDAR");
    // int err = lidar_init();
    // if (err) {
    //     LOG_ERR("Failed to initialize LiDAR");
    //     return 0;
    // }

    k_sleep(K_MSEC(500));
    LOG_INF("Initializing analog sensors");
    if (auto result = AnalogSensors::init(); !result) {
        LOG_ERR("Failed to initialize analog sensors: %s", result.error().build_message().c_str());
        return 0;
    }

    //ADC quick test
    LOG_INF("Running ADC quick test");
    for (int i = 0; i < 5; i++) {
        int16_t samples[ADC_ADS7953_CHANNEL_COUNT] = {0};

        int ret = adc_ads7953_read_all(samples);
        if (ret < 0) {
            LOG_ERR("ADC read failed: %d", ret);
        } else {
            LOG_INF("ADC: b1a0=%d b1a1=%d b2a0=%d b2a1=%d",
                    samples[0], samples[1], samples[2], samples[3]);
        }

        k_sleep(K_MSEC(500));
    }

    LOG_INF("Initializing Controller");
    if (auto result = Controller::init(); !result) {
        LOG_ERR("Failed to initialize Controller: %s", result.error().build_message().c_str());
        return 0;
    }

    // LOG_INF("initializing SNTP");
    // err = sntp_init();
    // if (err) {
    //     LOG_ERR("Failed to initialize SNTP");
    //     return 0;
    // }

    k_sleep(K_MSEC(500));
    LOG_INF("Starting server");
    serve_connections();
}
