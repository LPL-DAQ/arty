#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <cstdint>

int main(void)
{
    printk("PSRAM test start\n");

    volatile uint32_t* psram = reinterpret_cast<volatile uint32_t*>(0x70000000);

    // write
    psram[0] = 123;
    psram[1] = 456;

    // read
    uint32_t a = psram[0];
    uint32_t b = psram[1];

    printk("PSRAM readback: %u %u\n", a, b);

    while (true) {
        k_sleep(K_SECONDS(1));
    }
}
