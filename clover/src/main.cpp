#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <stdint.h>
#include <zephyr/usb/usb_device.h>   // usb_enable()
#include <cmsis_core.h>             // SCB_DisableDCache()

#define PSRAM_BASE 0x70000000u
#define PSRAM_WORDS (16u * 1024u * 1024u / 4u)

static void write_read(volatile uint32_t *psram, size_t idx, uint32_t value)
{
    psram[idx] = value;
    __DSB();
}

int main(void)
{
    volatile uint32_t *psram = (volatile uint32_t *)PSRAM_BASE;

    usb_enable(nullptr);
    SCB_DisableDCache();

    printk("PSRAM scan start\n");

    for (size_t i = 0; i < PSRAM_WORDS; i += 1024u) {
        write_read(psram, i, 0xA5A50000u ^ (uint32_t)i);

        if ((i % 262144u) == 0) {
            printk("checked %u MB\n", (unsigned)(i / 262144u));
        }
    }

    printk("PSRAM scan done\n");

    while (1) {
        k_sleep(K_SECONDS(1));
    }
}
