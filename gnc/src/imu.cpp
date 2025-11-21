#include "imu.h"

#include <zephyr/logging/log.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>

LOG_MODULE_REGISTER(imu, CONFIG_LOG_DEFAULT_LEVEL);

#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)

/* Struct similar in spirit to pwm_dt_spec: “everything we need to talk to the IMU” */
struct imu_dt_spec {
    const struct device *bus;  // I2C bus device (e.g., &lpi2c1)
    uint16_t addr;             // I2C address of the IMU
};

/* Fill IMU from /zephyr,user properties */
static const imu_dt_spec IMU = {
    .bus  = DEVICE_DT_GET(DT_PROP(ZEPHYR_USER_NODE, imu_i2c_controller)),
    .addr = DT_PROP(ZEPHYR_USER_NODE, imu_i2c_addr),
};

/* Ready helper, analogous to pwm_ready(...) */
static inline bool imu_ready(const imu_dt_spec &imu)
{
    return device_is_ready(imu.bus);
}

/* Public API: initialize IMU */
int imu_init()
{
    if (!imu_ready(IMU)) {
        LOG_ERR("IMU I2C bus not ready");
        return -ENODEV;
    }

    /* Optional: simple presence check – try a 1-byte read */
    uint8_t dummy;
    int ret = i2c_read(IMU.bus, &dummy, 1, IMU.addr);
    if (ret < 0) {
        LOG_ERR("IMU not responding at 0x%02x (err=%d)", IMU.addr, ret);
        return ret;
    }

    LOG_INF("IMU present at 0x%02x", IMU.addr);
    return 0;
}

/* Basic “read N raw bytes” helper */
int imu_read_raw(uint8_t *buf, size_t len)
{
    if (!imu_ready(IMU)) {
        return -ENODEV;
    }
    return i2c_read(IMU.bus, buf, len, IMU.addr);
}

/* Typical “write, then read” transaction (useful for SHTP packets etc.) */
int imu_write_read(const uint8_t *tx, size_t tx_len,
                   uint8_t *rx, size_t rx_len)
{
    if (!imu_ready(IMU)) {
        return -ENODEV;
    }
    return i2c_write_read(IMU.bus, IMU.addr, tx, tx_len, rx, rx_len);
}

/* Example demo, similar idea to servotesting_demo() */
int imu_demo()
{
    int err = imu_init();
    if (err) {
        LOG_ERR("IMU init failed (%d)", err);
        return err;
    }

    uint8_t buf[16] = {0};
    err = imu_read_raw(buf, sizeof(buf));
    if (err) {
        LOG_ERR("IMU read failed (%d)", err);
        return err;
    }

    printk("IMU first 16 bytes:\n");
    for (int i = 0; i < 16; ++i) {
        printk("%02x ", buf[i]);
    }
    printk("\n");

    return 0;
}
