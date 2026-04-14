#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_pkt.h>
#include <zephyr/net/socket.h>
#include <zephyr/sys/util.h>
#include <zephyr/usb/usb_device.h>

#include "PwmActuator.h"

LOG_MODULE_REGISTER(pwm_test, CONFIG_LOG_DEFAULT_LEVEL);


int main(void)
{
    LOG_INF("hi");
    // USB serial setup
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
    // Sweep: full left → full right → center
    // set_target_angle() sets the target, tick() drives the PWM at 10ms rate
    // -------------------------------------------------------------------------
    static const struct {
        float deg;
        const char* label;
    } positions[] = {
        { -90.0f, "full left (-90 deg, ~1000us)" },
        {  90.0f, "full right (+90 deg, ~2000us)" },
        {   0.0f, "center (0 deg, ~1500us)" },
    };

    for (auto& pos : positions) {
        LOG_INF("Moving to %s", pos.label);

        // Set target once — tick() will write PWM each call
        ServoR::set_target_angle(pos.deg);

        // Hold for 10 seconds at 10ms tick rate = 1000 ticks
        for (int i = 0; i < 1000; ++i) {
            if (auto r = ServoR::tick(); !r) {
                LOG_ERR("tick() failed at step %d", i);
                return 0;
            }
            k_sleep(K_MSEC(10));
        }

        LOG_INF("Held. Commanded: %.1f deg",
                (double)ServoR::get_angle_running());
        LOG_INF("done %s", pos.label);
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
    ServoR::tick();
    LOG_INF("Final pulse: %u us  (expect ~1000 us)",
            (uint32_t)ServoR::get_pulse_us());

    LOG_INF("PWM test complete");
    return 0;
}