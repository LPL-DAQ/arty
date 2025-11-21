#pragma once

#include <zephyr/kernel.h>



int imu_init();
int imu_read_raw(uint8_t *buf, size_t len);
int imu_write_read(const uint8_t *tx, size_t tx_len,
                   uint8_t *rx, size_t rx_len);
int imu_demo();


