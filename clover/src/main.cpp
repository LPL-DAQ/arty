#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <zephyr/usb/usb_device.h>

#define U4_NODE DT_NODELABEL(u4_mcp23017)

static void mcp_blink_pin(const struct device *mcp, gpio_pin_t pin)
{
    int ret;

    ret = gpio_pin_configure(mcp, pin, GPIO_OUTPUT_INACTIVE);
    printk("gpio_pin_configure ret=%d\n", ret);
    if (ret) {
        return;
    }

    while (1) {
        ret = gpio_pin_set(mcp, pin, 1);
        printk("set high ret=%d\n", ret);
        k_msleep(500);

        ret = gpio_pin_set(mcp, pin, 0);
        printk("set low ret=%d\n", ret);
        k_msleep(500);
    }
}

int main(void)
{
    const struct device *u4 = DEVICE_DT_GET(U4_NODE);

    if (usb_enable(NULL) != 0) {
        printk("USB enable failed\n");
        return 0;
    }

    k_msleep(1000);

    if (!device_is_ready(u4)) {
        printk("U4 not ready\n");
        return 0;
    }

    printk("MCP23017 U4 GPIO test start\n");

    mcp_blink_pin(u4, 0);  // GPIO0

    return 0;
}
