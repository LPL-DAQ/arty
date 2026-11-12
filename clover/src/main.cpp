
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_pkt.h>
#include <zephyr/net/socket.h>
#include <zephyr/sys/util.h>
#include <zephyr/usb/usb_device.h>

#include "BlinkLed.h"
#include "config.h"
#include "Controller.h"
#include "ThrottleValve.h"
#include "Valves.h"
#include "sensors/AnalogSensors.h"
#include "sensors/Gnss.h"
#include "sensors/Lidar.h"
#include "sensors/VectornavIMU.h"
#include "server.h"
#include "watchdog.h"

LOG_MODULE_REGISTER(main, CONFIG_LOG_DEFAULT_LEVEL);

int main(void)
{
    // Upon Startup - Evaluate the reboot cause before doing anything else.
    // This also calls the on_watchdog_reboot() function if the cause was determined to be a watchdog reset.
    const Watchdog::RebootCheckResult reboot = Watchdog::evaluate_reboot_cause();

#ifdef CONFIG_USB_LOGGING
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
#elifdef CONFIG_UART_LOGGING
    const device* uart = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
    if (!device_is_ready(uart)) {
        LOG_ERR("Failed to check uart log device readiness: %s", Error::from_device_not_ready(uart).build_message().c_str());
    }
#else
#warning "No logging backend set -- should use either CONFIG_USB_LOGGING or CONFIG_UART_LOGGING"
#endif

    // Now that logging has been established, print the reboot cause for the user to view.
    // Note: If we want more sophisticated handling for the two error cases, they can be added here. 
    // 1) reboot.cause_available. If this is false then we don't know why the teensy restarted.
    // 2) reboot.clear_error. If this is non-zero then we were unable to clear the reset reason. Just good to have documented.
    if (!reboot.cause_available) {
        LOG_WRN("Reset cause unavailable (%d)", reboot.read_error);
    } else {
        if (reboot.watchdog_reset) {
            LOG_WRN("Watchdog reset detected");
        }

        if (reboot.clear_error != 0) {
            LOG_WRN("Failed to clear reset flags (%d); "
                    "future checks may include an older reset",
                    reboot.clear_error);
        }
    }

    // Initialize the Watchdog with the appropriate timeout (established in config.h)
    if (!Watchdog::init(WATCHDOG_TIMEOUT)) {
        LOG_ERR("Failed to initialize Watchdog. Aborting boot.");
        return 0;
    }

#ifdef CONFIG_BLINK_LED
    if (auto result = BlinkLed::init(); !result) {
        LOG_ERR("Failed to initialize blink LED: %s", result.error().build_message().c_str());
        return 1;
    }
#endif

#ifdef CONFIG_LIDAR
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
#endif

#ifdef CONFIG_IMU
    LOG_INF("Initializing IMU");
    if (auto result = VectornavImu::init(); !result) {
        LOG_ERR("Failed to initialize IMU: %s", result.error().build_message().c_str());
        return 0;
    }
#endif

#ifdef CONFIG_GNSS
    LOG_INF("Initializing GNSS");
    if (auto result = Gnss::init(); !result) {
        LOG_ERR("Failed to initialize GNSS: %s", result.error().build_message().c_str());
        return 0;
    }
    Gnss::start_sense();
#endif

#ifdef CONFIG_ANALOG_SENSORS
    LOG_INF("Initializing analog sensors");
    if (auto result = AnalogSensors::init(); !result) {
        LOG_ERR("Failed to initialize analog sensors: %s", result.error().build_message().c_str());
        return 0;
    }
#endif

#ifdef CONFIG_VALVES
    LOG_INF("Initializing general valves");
    if (auto result = Valves::init(); !result) {
        LOG_ERR("Failed to initialize general valves: %s", result.error().build_message().c_str());
        return 0;
    }
#endif

#ifdef CONFIG_THROTTLE_VALVES
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

    LOG_INF("Initializing Controller");
    if (auto result = Controller::init(); !result) {
        LOG_ERR("Failed to initialize Controller: %s", result.error().build_message().c_str());
        return 0;
    }

    k_sleep(K_MSEC(500));
    LOG_INF("Starting server");
    serve_connections();
}
