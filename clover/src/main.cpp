#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_pkt.h>
#include <zephyr/net/socket.h>
#include <zephyr/sys/util.h>

#include "Controller.h"
#include "ThrottleValve.h"
#include "pts.h"
#include "server.h"
#include "sntp_imp.h"

extern "C" {
#include <app/drivers/blink.h>
}

LOG_MODULE_REGISTER(main, CONFIG_LOG_DEFAULT_LEVEL);


int main(void)
{

    // Status LED
    const struct device* blink = DEVICE_DT_GET(DT_NODELABEL(blink_led));
    if (!device_is_ready(blink)) {
        LOG_ERR("Blink LED device not ready");
        return 0;
    }
    blink_set_period_ms(blink, 1000u);

    const struct device *uart = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
    if (!device_is_ready(uart)) {
        return 0;
    }

    LOG_INF("Initializing fuel throttle valve");
    if (auto result = FuelValve::init(); !result) {
        LOG_ERR("Failed to initialize fuel throttle valve: %s",
            result.error().build_message().c_str());
        return 0;
    }

    LOG_INF("Initializing lox throttle valve");
    if (auto result = LoxValve::init(); !result) {
        LOG_ERR("Failed to initialize lox throttle valve: %s",
            result.error().build_message().c_str());
        return 0;
    }

    LOG_INF("Initializing PTs");
    if (auto result = pts_init(); !result) {
        LOG_ERR("Failed to initialize PTs: %s",
            result.error().build_message().c_str());
        return 0;
    }

    LOG_INF("Initializing Controller");
    if (auto result = Controller::controller_init(); !result) {
        LOG_ERR("Failed to initialize Controller: %s",
            result.error().build_message().c_str());
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


    k_sleep(K_FOREVER);
}
