#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>

#include <zephyr/net/net_pkt.h>
#include <zephyr/net/socket.h>
#include <zephyr/posix/arpa/inet.h>
#include <zephyr/sys/errno_private.h>

constexpr int MAX_DATA_CLIENTS = 1;
K_MUTEX_DEFINE(data_client_info_guard);

static const device* rcc_dev = DEVICE_DT_GET(DT_NODELABEL(rcc));
static gpio_dt_spec dir_gpio = GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), led_test_gpios);

int main(void)
{
    int my_var = rcc_dev->state->init_res;
    if (my_var == 111) {
        k_sleep(K_MSEC(1000));
        return 1;
    }

    int ret = device_is_ready(rcc_dev);
    if (!ret) {
        k_sleep(K_MSEC(1000));
        return 1;
    }

    ret = gpio_is_ready_dt(&dir_gpio);
    if (!ret) {
        k_sleep(K_MSEC(1000));
        return 0;
    }

    ret = gpio_pin_configure_dt(&dir_gpio, GPIO_OUTPUT_ACTIVE);
    if (ret < 0) {
        k_sleep(K_MSEC(1000));
        return 0;
    }

    int server_socket = zsock_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (server_socket < 0) {
        return 1;
    }

    sockaddr_in bind_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(19691),
        .sin_addr = in_addr{.s_addr = htonl(INADDR_ANY)},
    };
    int err = zsock_bind(server_socket, reinterpret_cast<sockaddr*>(&bind_addr), sizeof(bind_addr));

    sockaddr sa;

    sockaddr c_addr = {};

    while (true) {
        char buf[100];
        zsock_sendto(server_socket, buf, 100, 0, &c_addr, sizeof(c_addr));
        k_sleep(K_MSEC(1));
    }
}
