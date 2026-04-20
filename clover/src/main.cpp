#include <zephyr/kernel.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/sys/printk.h>

extern "C" {
#include "psram_test.h"
}

int main(void)
{
    usb_enable(nullptr);

    int ret = psram_boundary_test_run();
    if (ret != 0) {
        printk("PSRAM test failed: %d\n", ret);
    }

    while (1) {
        k_sleep(K_SECONDS(1));
    }

    return 0;
}
