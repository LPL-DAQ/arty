#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/printk.h>
#include <zephyr/usb/usb_device.h>

#define U4_NODE DT_NODELABEL(u4_mcp23017)
#define U6_NODE DT_NODELABEL(u6_mcp23017)

static void test_mcp23017(const char *name, const struct device *i2c_dev, uint8_t addr)
{
    uint8_t val = 0;

    printk("Testing %s at 0x%02X\n", name, addr);

    int ret = i2c_reg_read_byte(i2c_dev, addr, 0x00, &val);
    printk("  read IODIRA ret=%d val=0x%02X\n", ret, val);

    ret = i2c_reg_write_byte(i2c_dev, addr, 0x00, 0xFE);
    printk("  write IODIRA ret=%d\n", ret);

    ret = i2c_reg_read_byte(i2c_dev, addr, 0x00, &val);
    printk("  readback IODIRA ret=%d val=0x%02X\n", ret, val);
}

int main(void)
{
    const struct device *u4 = DEVICE_DT_GET(U4_NODE);
    const struct device *u6 = DEVICE_DT_GET(U6_NODE);

    if (usb_enable(NULL) != 0) {
        printk("USB enable failed\n");
        return 0;
    }

    k_msleep(1000);

    if (!device_is_ready(u4)) {
        printk("U4 not ready\n");
    } else {
        test_mcp23017("U4", u4, 0x20);
    }

    if (!device_is_ready(u6)) {
        printk("U6 not ready\n");
    } else {
        test_mcp23017("U6", u6, 0x21);
    }

    printk("Done\n");
    return 0;
}
