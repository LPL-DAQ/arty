#include <zephyr/device.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/sys/printk.h>
#include <zephyr/kernel.h>
#include <stdint.h>

int main(void)
{
    usb_enable(nullptr);

    const struct device *dev = device_get_binding("FLEXSPI2");

    while (1) {
        if (dev == NULL) {
            printk("FLEXSPI2 device not found\n");
        } else if (!device_is_ready(dev)) {
            printk("FLEXSPI2 device not ready\n");
        } else {
            printk("FLEXSPI2 ready\n");

            /* Enable this after PSRAM is connected */
            volatile uint32_t *psram = (volatile uint32_t *)0x70000000;

            psram[0] = 0xdeadbeef;
            printk("PSRAM read: 0x%x\n", psram[0]);
        }

        k_sleep(K_SECONDS(1));
    }
}
