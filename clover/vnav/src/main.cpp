#include <zephyr/kernel.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/logging/log.h>
#include "vn_hal.hpp"

LOG_MODULE_REGISTER(vnav, LOG_LEVEL_INF);

BUILD_ASSERT(DT_NODE_HAS_COMPAT(DT_CHOSEN(zephyr_console), zephyr_cdc_acm_uart), "Console device is not ACM CDC UART device");

int main(void)
{
    usb_enable(NULL);
    k_msleep(2000);
    LOG_INF("VN-300 starting...");

    if (!vn_init()) {
        LOG_ERR("Failed to init VN-300 UART");
        return -1;
    }

    vn_configure_outputs();

    char line[256];
    float yaw, pitch, roll;
    float ax, ay, az, gx, gy, gz;
    double lat, lon;
    float alt;
    float vn, ve, vd;
    float mx, my, mz;

    while (1) {
        if (vn_read_line(line, sizeof(line))) {
            if (vn_parse_ymr(line, &yaw, &pitch, &roll)) {
                LOG_INF("YPR:  %.2f deg, %.2f deg, %.2f deg",
                    (double)yaw, (double)pitch, (double)roll);
            }
            else if (vn_parse_imu(line, &ax, &ay, &az, &gx, &gy, &gz)) {
                LOG_INF("ACCEL: %.3f %.3f %.3f m/s^2",
                    (double)ax, (double)ay, (double)az);
                LOG_INF("GYRO:  %.3f %.3f %.3f rad/s",
                    (double)gx, (double)gy, (double)gz);
            }
            else if (vn_parse_gps(line, &lat, &lon, &alt)) {
                LOG_INF("GPS: %.6f, %.6f, %.1f m",
                    lat, lon, (double)alt);
            }
            else if (vn_parse_ins(line, &lat, &lon, &alt, &vn, &ve, &vd)) {
                LOG_INF("INS POS: %.6f, %.6f, %.1f m",
                    lat, lon, (double)alt);
                LOG_INF("INS VEL: %.3f %.3f %.3f m/s",
                    (double)vn, (double)ve, (double)vd);
            }
            else if (vn_parse_mag(line, &mx, &my, &mz)) {
                LOG_INF("MAG: %.3f %.3f %.3f Gauss",
                    (double)mx, (double)my, (double)mz);
            }
        }
        k_msleep(1);
    }

    return 0;
}

// Run for USB port read troubleshooting

// #include <zephyr/kernel.h>
// int main(void)
// {
//     while (1) {
//         k_msleep(1000);
//     }
//     return 0;
// }