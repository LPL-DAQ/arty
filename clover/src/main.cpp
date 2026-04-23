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
        while (1) {}
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

    LOG_INF("USB Connected. Bypassing flight hardware for TVC standalone test.");

    // DO NOT initialize Valves, LiDAR, or Analog Sensors right now.
    // Just keep the main thread alive so the background TVC thread can run!
    while (1) {
        k_sleep(K_MSEC(1000));
    }
    
    return 0;
}