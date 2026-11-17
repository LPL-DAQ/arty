#include <zephyr/drivers/can.h>
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
    // Serial over USB setup
    if (usb_enable(nullptr)) {
        LOG_ERR("USB is not enabled.");
        while (1) {}
    }

    // Motor init first — before USB wait — so the watchdog never fires on boot.
    RangerTvc::reset();

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

    // 1 seconds at 8 Hz
    constexpr int hold_ticks = 8;

    // auto result = RangerTvc::tick(target_pos);

    // if (!result) {
    //     LOG_ERR("tick failed: %s", result.error().build_message().c_str());
    // }

    // k_sleep(K_MSEC(20));

    LOG_INF("Starting silly movement");

    while (1) {
        auto result = RangerTvc::tick(target_pos);

        if (!result) {
            LOG_ERR("tick failed: %s", result.error().build_message().c_str());
        }

        k_sleep(K_MSEC(20));

        if (++ticks >= hold_ticks) {

            const device* can_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus));
            can_state state;
            can_bus_err_cnt err_cnt{};
            int ret = can_get_state(can_dev, &state, &err_cnt);

            LOG_INF("can_get_state ret=%d state=%d tx_err=%u rx_err=%u",
                    ret, (int)state, err_cnt.tx_err_cnt, err_cnt.rx_err_cnt);

            target_pos = (target_pos == max_pos) ? min_pos : max_pos;
            ticks = 0;
        }
    }

    return 0;
}