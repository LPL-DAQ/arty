#include "lidar.h"
#include "MutexGuard.h"
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/ring_buffer.h>

LOG_MODULE_REGISTER(LiDAR, CONFIG_LOG_DEFAULT_LEVEL);  // For logging to be put under the LiDAR tag

K_MUTEX_DEFINE(lidar_lock);  // Mutex to protect LiDAR data

K_SEM_DEFINE(lidar_ready, 0, 1);

RING_BUF_DECLARE(uart_ringbuf, 64);

// UART interrupt callback
static void uart_cb(const struct device* dev, void* user_data)
{
    uint8_t received_byte;
    if (!uart_irq_update(dev)) {
        return;
    }

    while (uart_irq_rx_ready(dev)) {
        uart_fifo_read(dev, &received_byte, 1);
        ring_buf_put(&uart_ringbuf, &received_byte, 1);
    }
}

static float current_distance = 0.0f;
static const struct device* lidar_1;
int decode(uint8_t*, bool);

static void lidar_thread(void*, void*, void*)
{
    int err = k_sem_take(&lidar_ready, K_FOREVER);
    if (err) {
        LOG_ERR("Failed to take LiDAR ready semaphore");
        return;
    }

    LOG_INF("LiDAR 1 thread started");

    uint8_t msg[9];
    int i = 0;

    while (1) {
        uint8_t byte;
        while (ring_buf_get(&uart_ringbuf, &byte, 1) > 0) {
            if (i == 0 && byte != 0x59)
                continue;
            if (i == 1 && byte != 0x59) {
                i = 0;
                continue;
            }

            msg[i++] = byte;
            if (i == 9) {
                i = 0;
                decode(msg, true);
            }
        }
    }
}

K_THREAD_DEFINE(lidar_tid, 2048, lidar_thread, nullptr, nullptr, nullptr, 5, 0, 0);

int lidar_init()
{
    lidar_1 = DEVICE_DT_GET(DT_NODELABEL(lpuart4));  // LiDAR is connected to UART4 (pins 7 & 8)
    if (!device_is_ready(lidar_1)) {
        LOG_ERR("LiDAR 1 device not ready");
        return -ENODEV;
    }

    uart_irq_callback_user_data_set(lidar_1, uart_cb, NULL);
    uart_irq_rx_enable(lidar_1);

    k_sem_give(&lidar_ready);
    return 0;
}

float lidar_get_distance()
{
    MutexGuard lidar_guard{&lidar_lock};
    float d = current_distance;
    return d;
}

int decode(uint8_t* msg, bool detailed)  // lod for level of detail
{
    uint16_t strength;
    uint16_t temp;
    uint16_t checksum;

    if (msg[0] != 0x59 || msg[1] != 0x59) {
        LOG_ERR("Invalid LiDAR message header");
        return -EINVAL;
    }

    uint16_t distance = msg[2] | (msg[3] << 8);

    if (detailed) {
        strength = msg[4] | (msg[5] << 8);
        temp = msg[6] | (msg[7] << 8);
        checksum = msg[8];

        uint8_t calculated_checksum = (msg[0] + msg[1] + msg[2] + msg[3] + msg[4] + msg[5] + msg[6] + msg[7]) & 0xFF;
        if (checksum != calculated_checksum) {
            LOG_ERR("LiDAR message checksum mismatch");
            return -EINVAL;
        }
    }

    float distance_meters = distance / 100.0f;

    {
        MutexGuard lidar_guard{&lidar_lock};
        current_distance = distance_meters;
    }

    if (detailed) {
        LOG_INF("LiDAR distance: %.2f m, strength: %u, temp: %u", distance_meters, strength, temp);
    }
    else {
        LOG_INF("LiDAR distance: %.2f m", distance_meters);
    }
    return 0;
}
