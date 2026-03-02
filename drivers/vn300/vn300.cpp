#include "vn300.hpp"
#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include <stdlib.h>

LOG_MODULE_REGISTER(vn300, LOG_LEVEL_INF);

#define VN_RX_BUF_SIZE 2048
static uint8_t _rx_buf[VN_RX_BUF_SIZE];
static volatile size_t _rx_head = 0;
static volatile size_t _rx_tail = 0;
static const struct device* _uart_dev = nullptr;

static void _uart_isr(const struct device* dev, void*)
{
    uart_irq_update(dev);
    while (uart_irq_rx_ready(dev)) {
        uint8_t b;
        if (uart_fifo_read(dev, &b, 1) == 1) {
            size_t next = (_rx_head + 1) % VN_RX_BUF_SIZE;
            if (next != _rx_tail) {
                _rx_buf[_rx_head] = b;
                _rx_head = next;
            }
        }
    }
}

bool vn300_init()
{
    _uart_dev = DEVICE_DT_GET(DT_NODELABEL(lpuart2));
    if (!device_is_ready(_uart_dev)) {
        LOG_ERR("VN-300 UART not ready");
        return false;
    }
    struct uart_config cfg = {
        .baudrate  = 115200,
        .parity    = UART_CFG_PARITY_NONE,
        .stop_bits = UART_CFG_STOP_BITS_1,
        .data_bits = UART_CFG_DATA_BITS_8,
        .flow_ctrl = UART_CFG_FLOW_CTRL_NONE,
    };
    uart_configure(_uart_dev, &cfg);
    uart_irq_callback_set(_uart_dev, _uart_isr);
    uart_irq_rx_enable(_uart_dev);
    LOG_INF("VN-300 UART ready");
    return true;
}

bool vn300_read_line(char* out_buf, size_t max_len)
{
    size_t i = 0;
    while (_rx_tail != _rx_head && i < max_len - 1) {
        char c = (char)_rx_buf[_rx_tail];
        _rx_tail = (_rx_tail + 1) % VN_RX_BUF_SIZE;
        if (c == '\n') {
            out_buf[i] = '\0';
            return i > 0;
        }
        out_buf[i++] = c;
    }
    return false;
}

bool vn300_parse_ymr(const char* line, float* yaw, float* pitch, float* roll)
{
    if (strncmp(line, "$VNYMR,", 7) != 0) return false;
    char buf[256];
    strncpy(buf, line + 7, sizeof(buf));
    char* token = strtok(buf, ",");
    if (!token) return false;
    *yaw = strtof(token, nullptr);
    token = strtok(nullptr, ",");
    if (!token) return false;
    *pitch = strtof(token, nullptr);
    token = strtok(nullptr, "*");
    if (!token) return false;
    *roll = strtof(token, nullptr);
    return true;
}