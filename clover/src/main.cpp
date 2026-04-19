#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/cache.h>
#include <stdint.h>
#include <stddef.h>

#define PSRAM_BASE   0x70000000u
#define HALF_SIZE    (8u * 1024u * 1024u)
#define HALF_WORDS   (HALF_SIZE / sizeof(uint32_t))
#define TOTAL_SIZE   (2u * HALF_SIZE)

static inline uint32_t pattern_a(uint32_t i)
{
    return 0xA5A50000u ^ i;
}

static inline uint32_t pattern_b(uint32_t i)
{
    return 0x5A5A0000u ^ i;
}

static int write_region(volatile uint32_t *base,
                        uint32_t words,
                        uint32_t (*pattern_fn)(uint32_t))
{
    for (uint32_t i = 0; i < words; i++) {
        base[i] = pattern_fn(i);
    }

    __DSB();
    __ISB();
    sys_cache_data_flush_range((void *)base, words * sizeof(uint32_t));
    sys_cache_data_invd_range((void *)base, words * sizeof(uint32_t));
    __DSB();
    __ISB();

    return 0;
}

static int verify_region(volatile uint32_t *base,
                         uint32_t words,
                         uint32_t (*pattern_fn)(uint32_t),
                         const char *label)
{
    for (uint32_t i = 0; i < words; i++) {
        uint32_t expected = pattern_fn(i);
        uint32_t readback = base[i];

        if (readback != expected) {
            printk("FAIL in %s at word %u (addr 0x%08x): wrote 0x%08x read 0x%08x\n",
                   label, i, (uint32_t)((uintptr_t)base + (i * 4u)),
                   expected, readback);
            return -1;
        }
    }

    return 0;
}

int main(void)
{
    volatile uint32_t *psram = (volatile uint32_t *)PSRAM_BASE;
    volatile uint32_t *first_half = psram;
    volatile uint32_t *second_half = psram + HALF_WORDS;

    usb_enable(NULL);

    printk("PSRAM 16MB boundary test start\n");
    printk("Testing %u bytes total (%u bytes per half)\n", TOTAL_SIZE, HALF_SIZE);

    sys_cache_data_flush_range((void *)psram, TOTAL_SIZE);
    sys_cache_data_invd_range((void *)psram, TOTAL_SIZE);
    __DSB();
    __ISB();

    printk("Writing first 8MB...\n");
    if (write_region(first_half, HALF_WORDS, pattern_a) != 0) {
        printk("WRITE first half FAILED\n");
        while (1) {
            k_sleep(K_SECONDS(1));
        }
    }

    printk("Verifying first 8MB...\n");
    if (verify_region(first_half, HALF_WORDS, pattern_a, "first half") != 0) {
        printk("VERIFY first half FAILED\n");
        while (1) {
            k_sleep(K_SECONDS(1));
        }
    }

    printk("Writing second 8MB...\n");
    if (write_region(second_half, HALF_WORDS, pattern_b) != 0) {
        printk("WRITE second half FAILED\n");
        while (1) {
            k_sleep(K_SECONDS(1));
        }
    }

    printk("Verifying second 8MB...\n");
    if (verify_region(second_half, HALF_WORDS, pattern_b, "second half") != 0) {
        printk("VERIFY second half FAILED\n");
        while (1) {
            k_sleep(K_SECONDS(1));
        }
    }

    printk("Re-verifying first 8MB after second-half write...\n");
    if (verify_region(first_half, HALF_WORDS, pattern_a, "first half after second write") != 0) {
        printk("ALIASING DETECTED: second 8MB overwrote first 8MB\n");
        while (1) {
            k_sleep(K_SECONDS(1));
        }
    }

    printk("PSRAM 16MB boundary test PASS\n");

    while (1) {
        k_sleep(K_SECONDS(1));
    }
}
