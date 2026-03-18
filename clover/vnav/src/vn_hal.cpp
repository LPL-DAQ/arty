#include "vn_hal.hpp"
#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include <stdlib.h>

LOG_MODULE_REGISTER(vn_hal, LOG_LEVEL_INF);

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

bool vn_init()
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

bool vn_read_line(char* out_buf, size_t max_len)
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

void vn_uart_send(const uint8_t* data, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        uart_poll_out(_uart_dev, data[i]);
    }
}

void vn_configure_outputs()
{
    // Enable YPR (yaw, pitch, roll) at 40Hz - already on by default but setting explicitly
    const char* ymr_cmd = "$VNWRG,75,1,8,01,0000000000000001*XX\r\n";
    
    // Enable IMU (accel + gyro) at 40Hz
    const char* imu_cmd = "$VNWRG,75,2,8,01,0000000000000006*XX\r\n";
    
    // Enable GPS position + velocity at 5Hz
    const char* gps_cmd = "$VNWRG,76,2,200,01,0000000000000008*XX\r\n";
    
    // Enable INS (fused position + velocity) at 40Hz
    const char* ins_cmd = "$VNWRG,77,2,8,01,0000000000000010*XX\r\n";

    // Enable magnetometer at 40Hz
    const char* mag_cmd = "$VNWRG,75,3,8,01,0000000000000002*XX\r\n";

    vn_uart_send((const uint8_t*)ymr_cmd, strlen(ymr_cmd));
    k_msleep(100);
    vn_uart_send((const uint8_t*)imu_cmd, strlen(imu_cmd));
    k_msleep(100);
    vn_uart_send((const uint8_t*)gps_cmd, strlen(gps_cmd));
    k_msleep(100);
    vn_uart_send((const uint8_t*)ins_cmd, strlen(ins_cmd));
    k_msleep(100);
    vn_uart_send((const uint8_t*)mag_cmd, strlen(mag_cmd));
    k_msleep(100);

    LOG_INF("VN-300 outputs configured");
}

bool vn_parse_ymr(const char* line, float* yaw, float* pitch, float* roll)
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

bool vn_parse_imu(const char* line, float* ax, float* ay, float* az,
                                     float* gx, float* gy, float* gz)
{
    if (strncmp(line, "$VNIMU,", 7) != 0) return false;
    char buf[256];
    strncpy(buf, line + 7, sizeof(buf));
    char* token = strtok(buf, ",");
    if (!token) return false; *ax = strtof(token, nullptr);
    token = strtok(nullptr, ","); if (!token) return false; *ay = strtof(token, nullptr);
    token = strtok(nullptr, ","); if (!token) return false; *az = strtof(token, nullptr);
    token = strtok(nullptr, ","); if (!token) return false; *gx = strtof(token, nullptr);
    token = strtok(nullptr, ","); if (!token) return false; *gy = strtof(token, nullptr);
    token = strtok(nullptr, "*"); if (!token) return false; *gz = strtof(token, nullptr);
    return true;
}

bool vn_parse_gps(const char* line, double* lat, double* lon, float* alt)
{
    if (strncmp(line, "$VNGPE,", 7) != 0) return false;
    char buf[256];
    strncpy(buf, line + 7, sizeof(buf));
    char* token = strtok(buf, ",");
    if (!token) return false; *lat = strtod(token, nullptr);
    token = strtok(nullptr, ","); if (!token) return false; *lon = strtod(token, nullptr);
    token = strtok(nullptr, "*"); if (!token) return false; *alt = strtof(token, nullptr);
    return true;
}

// Parse $VNINS packet: fused position and velocity
bool vn_parse_ins(const char* line, double* lat, double* lon, float* alt,
                                     float* vn, float* ve, float* vd)
{
    if (strncmp(line, "$VNINS,", 7) != 0) return false;
    char buf[256];
    strncpy(buf, line + 7, sizeof(buf));
    char* token = strtok(buf, ",");
    if (!token) return false; *lat = strtod(token, nullptr);
    token = strtok(nullptr, ","); if (!token) return false; *lon = strtod(token, nullptr);
    token = strtok(nullptr, ","); if (!token) return false; *alt = strtof(token, nullptr);
    token = strtok(nullptr, ","); if (!token) return false; *vn  = strtof(token, nullptr);
    token = strtok(nullptr, ","); if (!token) return false; *ve  = strtof(token, nullptr);
    token = strtok(nullptr, "*"); if (!token) return false; *vd  = strtof(token, nullptr);
    return true;
}

// Parse $VNMAG packet: magnetometer X, Y, Z (Gauss)
bool vn_parse_mag(const char* line, float* mx, float* my, float* mz)
{
    if (strncmp(line, "$VNMAG,", 7) != 0) return false;
    char buf[256];
    strncpy(buf, line + 7, sizeof(buf));
    char* token = strtok(buf, ",");
    if (!token) return false; *mx = strtof(token, nullptr);
    token = strtok(nullptr, ","); if (!token) return false; *my = strtof(token, nullptr);
    token = strtok(nullptr, "*"); if (!token) return false; *mz = strtof(token, nullptr);
    return true;
}