#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/cache.h>
#include <stdint.h>
#include <stddef.h>

#define PSRAM_BASE     0x70000000u
#define PSRAM_SIZE     (16u * 1024u * 1024u)

#define SEG_WORDS      16u
#define SEG_BYTES      (SEG_WORDS * sizeof(uint32_t))
#define STEP_BYTES     4096u   /* */

static int test_segment(uint32_t *addr, uint32_t seed)
{
    uint32_t expected[SEG_WORDS];

    /* write */
    for (size_t i = 0; i < SEG_WORDS; i++) {
        expected[i] = seed ^ (uint32_t)i;
        addr[i] = expected[i];
    }

    __DSB();
    __ISB();

    /* ensure write to PSRAM */
    sys_cache_data_flush_range((void *)addr, SEG_BYTES);
    sys_cache_data_invd_range((void *)addr, SEG_BYTES);

    __DSB();
    __ISB();

    /* read back */
    for (size_t i = 0; i < SEG_WORDS; i++) {
        uint32_t readback = addr[i];
        if (readback != expected[i]) {
            printk("FAIL @ 0x%08x idx %u: wrote 0x%08x read 0x%08x\n",
                   (uint32_t)addr, (unsigned)i,
                   expected[i], readback);
            return -1;
        }
    }

    return 0;
}

int main(void)
{
    usb_enable(NULL);

    printk("PSRAM segment test start\n");

    uint32_t tested = 0;

    for (uint32_t offset = 0;
         offset + SEG_BYTES <= PSRAM_SIZE;
         offset += STEP_BYTES) {

        uint32_t *addr = (uint32_t *)(PSRAM_BASE + offset);
        uint32_t seed = 0xA5A50000u ^ offset;

        if (test_segment(addr, seed) != 0) {
            printk("PSRAM TEST FAILED at offset 0x%08x\n", offset);
            while (1) {
                k_sleep(K_SECONDS(1));
            }
        }

        tested++;
    }

    printk("PSRAM segment test PASS\n");
    printk("Tested segments: %u (step %u bytes)\n", tested, STEP_BYTES);

    while (1) {
        k_sleep(K_SECONDS(1));
    }
}
