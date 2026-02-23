#include "lidar.h"
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>

LOG_MODULE_REGISTER(LiDAR, CONFIG_LOG_DEFAULT_LEVEL); //For logging to be put under the LiDAR tag

K_MUTEX_DEFINE(lidar_lock); //Mutex to protect LiDAR data

K_SEM_DEFINE(lidar_ready, 0, 1);

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
    int i=0;

    while (1) {
        uint8_t byte;
        if (uart_poll_in(lidar_1, &byte) == 0) {
            if (i == 0 && byte != 0x59) continue;
            if (i == 1 && byte != 0x59) { i = 0; continue; }

            msg[i++] = byte;
            if (i == 4) {
                i = 0;
                decode(msg, false);
            }
        }
        else{
            k_msleep(1);
        }
    }
}

K_THREAD_DEFINE(lidar_tid, 2048, lidar_thread, nullptr, nullptr, nullptr, 5, 0, 0);

int lidar_init()
{
    lidar_1 = DEVICE_DT_GET(DT_NODELABEL(lpuart4)); //LiDAR is connected to UART4 (pins 7 & 8)
    if (!device_is_ready(lidar_1)) {
        LOG_ERR("LiDAR 1 device not ready");
        return -ENODEV;
    }
    k_sem_give(&lidar_ready);
    return 0;
}

float lidar_get_distance()
{
    k_mutex_lock(&lidar_lock, K_FOREVER);
    float d = current_distance;
    k_mutex_unlock(&lidar_lock);
    return d;
}

int decode(uint8_t* msg, bool detailed) //lod for level of detail
{
    uint16_t strength;
    uint16_t temp;
    uint16_t checksum;

    if (msg[0] != 0x59 || msg[1] != 0x59) {
        LOG_ERR("Invalid LiDAR message header");
        return -EINVAL;
    }

    uint16_t distance = msg[2] | (msg[3] << 8);

    if(detailed){
        strength = msg[4] | (msg[5] << 8);
        temp     = msg[6] | (msg[7] << 8);
        checksum  = msg[8];


        uint8_t calculated_checksum = (msg[0] + msg[1] + msg[2] + msg[3] + msg[4] + msg[5] + msg[6] + msg[7]) & 0xFF;
        if (checksum != calculated_checksum) {
            LOG_ERR("LiDAR message checksum mismatch");
            return -EINVAL;
        }
    }

    float distance_meters = distance / 100.0f;

    k_mutex_lock(&lidar_lock, K_FOREVER);
    current_distance = distance_meters;
    k_mutex_unlock(&lidar_lock);

    if(detailed){
        LOG_INF("LiDAR distance: %.2f m, strength: %u, temp: %u", distance_meters, strength, temp);
    }
    else
    {
        LOG_INF("LiDAR distance: %.2f m", distance_meters);
    }
    return 0;
}
