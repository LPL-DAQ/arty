#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_pkt.h>
#include <zephyr/net/socket.h>
#include <zephyr/sys/util.h>
#include <zephyr/usb/usb_device.h>

#include "ranger/RangerTvc.h"
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
    // Motor init first — before USB wait — so the watchdog never fires on boot.
    RangerTvc::reset();

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

    float max_pos = 5.0f;
    float min_pos = 0.0f;

    float target_pos = max_pos;
    int ticks = 0;

    // 5 seconds at 50 Hz
    constexpr int hold_ticks = 50;

    // auto result = RangerTvc::tick(target_pos);

    // if (!result) {
    //     LOG_ERR("tick failed: %s", result.error().build_message().c_str());
    // }

    // k_sleep(K_MSEC(20));

    while (1) {
        auto result = RangerTvc::tick(target_pos);

        if (!result) {
            LOG_ERR("tick failed: %s", result.error().build_message().c_str());
        }

        k_sleep(K_MSEC(20));

        if (++ticks >= hold_ticks) {
            target_pos = (target_pos == max_pos) ? min_pos : max_pos;
            ticks = 0;
        }
    }

    return 0;
}