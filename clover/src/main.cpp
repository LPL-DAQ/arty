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

static void send_str(const struct device *uart, const char *s)
{
    while (*s) {
        uart_poll_out(uart, *s++);
    }
}


int main(void)
{

    // Status LED
    const struct device* blink = DEVICE_DT_GET(DT_NODELABEL(blink_led));
    if (!device_is_ready(blink)) {
        return 0;
    }
    blink_set_period_ms(blink, 1000u);

    const struct device *uart = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
    if (!device_is_ready(uart)) {
        return 0;
    }


    while (1) {
        send_str(uart, "uart_poll_out test\r\n");
        LOG_INF("log inf test");
        k_sleep(K_SECONDS(1));
    }

    LOG_INF("Initializing fuel throttle valve");
    int err = FuelValve::init();
    if (err) {
        LOG_ERR("Failed to initialize fuel throttle valve");
        return 0;
    }

    LOG_INF("Initializing lox throttle valve");
    err = LoxValve::init();
    if (err) {
        LOG_ERR("Failed to initialize lox throttle valve");
        return 0;
    }

    LOG_INF("Initializing Pts");
    err = pts_init();
    if (err) {
        LOG_ERR("Failed to initialize PTs");
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

    LOG_INF("Initializing Controller");
    err = Controller::init();
    if (err) {
        LOG_ERR("Failed to initialize Controller");
        return 0;
    }

    k_sleep(K_FOREVER);
}
