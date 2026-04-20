#include <zephyr/kernel.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/sys/printk.h>

struct DataPacket {
    uint32_t id;
    uint32_t timestamp;
    uint8_t payload[32];
};

// 1. Define arrays for both PSRAM regions
__attribute__((section("PSRAM0")))
DataPacket data_packets_0[10000]; // Mapped to 0x70000000

__attribute__((section("PSRAM1")))
DataPacket data_packets_1[10000]; // Mapped to 0x70800000

int main(void)
{
    usb_enable(nullptr);
    k_sleep(K_MSEC(2500));
    printk("\n--- Dual PSRAM Linker Test ---\n");

    // 2. Verify PSRAM0
    printk("PSRAM0 Address: %p\n", (void*)data_packets_0);
    if (((uintptr_t)data_packets_0 & 0xFF800000) == 0x70000000) {
        data_packets_0[0].id = 0x11111111;
        if (data_packets_0[0].id == 0x11111111) {
            printk("PSRAM0 Test: PASSED\n");
        }
    } else {
        printk("PSRAM0 Test: FAILED\n");
    }

    // 3. Verify PSRAM1
    printk("PSRAM1 Address: %p\n", (void*)data_packets_1);
    if (((uintptr_t)data_packets_1 & 0xFF800000) == 0x70800000) {
        data_packets_1[0].id = 0x22222222;
        if (data_packets_1[0].id == 0x22222222) {
            printk("PSRAM1 Test: PASSED\n");
        }
    } else {
        printk("PSRAM1 Test: FAILED\n");
    }

    while (1) {
        k_sleep(K_SECONDS(1));
    }
    return 0;
}
