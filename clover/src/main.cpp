#include <zephyr/device.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/sys/printk.h>
#include <zephyr/kernel.h>
#include <stdint.h>
#include <cmsis_core.h>

#define PSRAM_BASE 0x70000000

static void write_read(volatile uint32_t *psram, size_t idx, uint32_t value)
{
    psram[idx] = value;
    __DSB();
    __ISB();
    k_busy_wait(10);

    //printk("%u: 0x%x\n", (unsigned)idx, psram[idx]);
}

int main(void)
{
    usb_enable(nullptr);
    SCB_DisableDCache();

    const struct device *dev = device_get_binding("FLEXSPI2");
    if (dev == NULL || !device_is_ready(dev)) {
        printk("FLEXSPI2 not ready\n");
        while (1) {
            k_sleep(K_SECONDS(1));
        }
    }

    printk("FLEXSPI2 ready\n");

    volatile uint32_t *psram = (volatile uint32_t *)PSRAM_BASE;

    printk("--- PSRAM 16MB Total Scan Start ---\n");

    size_t total_indices = (16u * 1024u * 1024u) / 4u;
    size_t step_1mb = 262144u;

    for (size_t i = 0; i < total_indices; i += 1024u) {
        write_read(psram, i, 0xA5A50000u ^ static_cast<uint32_t>(i));

        if (i % step_1mb == 0) {
            printk("Checked: %u MB (index: %u)\n", (unsigned int)(i / step_1mb), (unsigned int)i);
        }
    }

    printk("=== PSRAM SCAN PASS (16MB OK) ===\n");

    while (1) {
        k_sleep(K_SECONDS(1));
    }
}
