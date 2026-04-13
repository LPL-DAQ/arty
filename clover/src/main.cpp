#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_pkt.h>
#include <zephyr/net/socket.h>
#include <zephyr/sys/util.h>
#include <zephyr/usb/usb_device.h>

#include "PWMServo.h"

LOG_MODULE_REGISTER(pwm_test, CONFIG_LOG_DEFAULT_LEVEL);


int main(void)
{
LOG_INF("hi");
    // USB serial setup — same pattern as your main.cpp
    if (usb_enable(nullptr)) {
        LOG_ERR("USB not enabled");
        return 0;
    }
    const device* usb_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
    for (int i = 0; i < 30; ++i) {
        k_sleep(K_MSEC(100));
        uint32_t dtr = 0;
        uart_line_ctrl_get(usb_dev, UART_LINE_CTRL_DTR, &dtr);
        if (dtr) break;
    }

    LOG_INF("PWM test starting");

    // -------------------------------------------------------------------------
    // Init servo — writes min pulse, transitions to DISARMED
    // -------------------------------------------------------------------------
    if (auto r = ServoR::init(); !r) {
        LOG_ERR("ServoR init failed");
        return 0;
    }
    LOG_INF("ServoR init OK — in DISARMED, pulse at min");
    k_sleep(K_MSEC(1000));

    // -------------------------------------------------------------------------
    // Arm — state only, min pulse already held by tick() during DISARMED
    // -------------------------------------------------------------------------
    if (auto r = ServoR::arm(); !r) {
        LOG_ERR("ServoR arm failed");
        return 0;
    }
    LOG_INF("ServoR armed");

    // -------------------------------------------------------------------------
    // Sweep: center → full left → full right → center
    // Each position held for 1 second, tick() called at 10ms
    // -------------------------------------------------------------------------
    struct {
        float deg;
        const char* label;
    } positions[] = {
        { -90.0f, "full left (-90 deg, ~1000us)" },
        {  90.0f, "full right (+90 deg, ~2000us)" },
        {   0.0f, "center (0 deg, ~1500us)" },
    };

    for (auto& pos : positions) {
        LOG_INF("Moving to %s", pos.label);

        // Command for 1 second at 10ms tick rate = 100 ticks
        for (int i = 0; i < 1000; ++i) {
            if (auto r = ServoR::tick(pos.deg); !r) {
                LOG_ERR("tick() failed at step %d", i);
                return 0;
            }
            k_sleep(K_MSEC(10));
        }

        LOG_INF("Held. Commanded: %.1f deg ",
                ServoR::get_angle_commanded(),
                ServoR::get_pulse_us());
        LOG_INF("done g"); 

    }

    // -------------------------------------------------------------------------
    // Disarm — tick() will write min pulse on next call
    // -------------------------------------------------------------------------
    if (auto r = ServoR::disarm(); !r) {
        LOG_ERR("ServoR disarm failed");
        return 0;
    }
    LOG_INF("ServoR disarmed");

    // One tick to confirm min pulse written
    ServoR::tick(0.0f);
    LOG_INF("Final pulse: %lf us  (expect ~%d us)",
            ServoR::get_pulse_us(), 1000);

    LOG_INF("PWM test complete");
    return 0;
}