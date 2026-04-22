#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <zephyr/usb/usb_device.h>

#define U4_NODE DT_NODELABEL(mcp23017_u4)
#define U6_NODE DT_NODELABEL(mcp23017_u6)

static int test_once(const struct device *dev, const char *name)
{
    int ret;

    ret = gpio_pin_configure(dev, 0, GPIO_OUTPUT_INACTIVE);
    printk("[%s] configure ret=%d\n", name, ret);
    if (ret) {
        return ret;
    }

    ret = gpio_pin_set(dev, 0, 1);
    printk("[%s] set HIGH ret=%d\n", name, ret);
    if (ret) {
        return ret;
    }

    k_msleep(200);

    ret = gpio_pin_set(dev, 0, 0);
    printk("[%s] set LOW ret=%d\n", name, ret);
    return ret;
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

    printk("=== MCP23017 dual test ===\n");

    if (!device_is_ready(u4)) {
        printk("[U4] NOT READY\n");
    } else {
        printk("[U4] READY\n");
    }

    if (!device_is_ready(u6)) {
        printk("[U6] NOT READY\n");
    } else {
        printk("[U6] READY\n");
    }

    while (1) {
        printk("---- cycle ----\n");

        if (device_is_ready(u4)) {
            int r = test_once(u4, "U4");
            if (r) {
                printk("[U4] FAIL (%d)\n", r);
            } else {
                printk("[U4] OK\n");
            }
        }

        if (device_is_ready(u6)) {
            int r = test_once(u6, "U6");
            if (r) {
                printk("[U6] FAIL (%d)\n", r);
            } else {
                printk("[U6] OK\n");
            }
        }

        k_msleep(1000);
    }

    return 0;
}
