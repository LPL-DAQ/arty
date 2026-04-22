#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <zephyr/usb/usb_device.h>

#define TEST_NODE DT_NODELABEL(mcp23017_u6)

int main(void)
{
    const struct device *mcp = DEVICE_DT_GET(TEST_NODE);
    int ret;

    if (usb_enable(NULL) != 0) {
        printk("USB enable failed\n");
        return 0;
    }

    k_msleep(1000);

    if (!device_is_ready(mcp)) {
        printk("MCP23017 not ready\n");
        return 0;
    }

    printk("Testing MCP23017 U6 only\n");

    ret = gpio_pin_configure(mcp, 0, GPIO_OUTPUT_INACTIVE);
    printk("gpio_pin_configure ret=%d\n", ret);
    if (ret) {
        return 0;
    }

    while (1) {
        ret = gpio_pin_set(mcp, 0, 1);
        printk("set high ret=%d\n", ret);
        k_msleep(500);

        ret = gpio_pin_set(mcp, 0, 0);
        printk("set low ret=%d\n", ret);
        k_msleep(500);
    }

    return 0;
}
