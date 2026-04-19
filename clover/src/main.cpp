#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/cache.h>
#include <stdint.h>

#define PSRAM_BASE   0x70000000u
#define TEST_WORDS   16
#define TEST_BYTES   (TEST_WORDS * sizeof(uint32_t))

int main(void)
{
    volatile uint32_t *psram = (volatile uint32_t *)PSRAM_BASE;
    uint32_t expected[TEST_WORDS];

    usb_enable(NULL);

    printk("PSRAM quick test start\n");

    for (size_t i = 0; i < TEST_WORDS; i++) {
        expected[i] = 0xA5A50000u ^ (uint32_t)i;
        psram[i] = expected[i];
    }

    __DSB();
    __ISB();

    sys_cache_data_flush_range((void *)psram, TEST_BYTES);
    sys_cache_data_invd_range((void *)psram, TEST_BYTES);

    __DSB();
    __ISB();

    for (size_t i = 0; i < TEST_WORDS; i++) {
        uint32_t readback = psram[i];
        printk("%u: wrote 0x%08x, read 0x%08x\n",
               (unsigned)i, expected[i], readback);

        if (readback != expected[i]) {
            printk("PSRAM FAILED at index %u\n", (unsigned)i);
            while (1) {
                k_sleep(K_SECONDS(1));
            }
        }
    }

    printk("PSRAM quick test pass\n");

    while (1) {
        k_sleep(K_SECONDS(1));
    }
}
