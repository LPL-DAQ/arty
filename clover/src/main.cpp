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
#include "sensors/Gnss.h"
#include "sensors/Lidar.h"
#include "sensors/VectornavIMU.h"
#include "server.h"

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
                LOG_INF("Starting server");
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

    LOG_INF("Initializing GNSS");
    if (auto result = Gnss::init(); !result) {
        LOG_ERR("Failed to initialize GNSS: %s", result.error().build_message().c_str());
        return 0;
    }
    Gnss::start_sense();

    // Sensors
    LOG_INF("Initializing analog sensors");
    if (auto result = AnalogSensors::init(); !result) {
        LOG_ERR("Failed to initialize analog sensors: %s", result.error().build_message().c_str());
        return 0;
    }


    LOG_INF("Initializing Lidar 1");
    if (auto result = Lidar1::init(); !result) {
        LOG_ERR("Failed to initialize Lidar 1: %s", result.error().build_message().c_str());
        return 0;
    }

    LOG_INF("Initializing Lidar 2");
    if (auto result = Lidar2::init(); !result) {
        LOG_ERR("Failed to initialize Lidar 2: %s", result.error().build_message().c_str());
        return 0;
    }

    LOG_INF("Initializing IMU");
    if (auto result = VectornavImu::init(); !result) {
        LOG_ERR("Failed to initialize IMU: %s", result.error().build_message().c_str());
        return 0;
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
