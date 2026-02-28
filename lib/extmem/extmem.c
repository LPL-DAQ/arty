#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <extmem.h>

/* Place test data in EXTMEM */
__attribute__((section(".ext_rodata")))
__attribute__((used))
const uint8_t extmem_test_table[1024] = {1};

int extmem_enable_mmap(void)
{
    const struct device *flash =
        DEVICE_DT_GET(DT_NODELABEL(w25q128jv));

    if (!device_is_ready(flash)) {
        printk("QSPI not ready\n");
        return -ENODEV;
    }

    printk("QSPI ready\n");

    /* Self-test: verify EXTMEM mapping */
    volatile uint8_t v = extmem_test_table[0];
    printk("EXTMEM test = %d\n", v);

    return 0;
}
