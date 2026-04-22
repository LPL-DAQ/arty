#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <zephyr/usb/usb_device.h>

#define U4_NODE DT_NODELABEL(mcp23017_u4)
#define U6_NODE DT_NODELABEL(mcp23017_u6)

static void blink_pin(const struct device *mcp, gpio_pin_t pin, const char *name)
{
    int ret;

    ret = gpio_pin_configure(mcp, pin, GPIO_OUTPUT_INACTIVE);
    printk("%s gpio_pin_configure ret=%d\n", name, ret);
    if (ret) {
        return;
    }

    while (1) {
        ret = gpio_pin_set(mcp, pin, 1);
        printk("%s set high ret=%d\n", name, ret);
        k_msleep(500);

        ret = gpio_pin_set(mcp, pin, 0);
        printk("%s set low ret=%d\n", name, ret);
        k_msleep(500);
    }
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
        return 0;
    }

    if (!device_is_ready(u6)) {
        printk("U6 not ready\n");
        return 0;
    }

    printk("MCP23017 U4/U6 GPIO test start\n");

    while (1) {
        printk("Testing U4 pin0\n");
        gpio_pin_configure(u4, 0, GPIO_OUTPUT_INACTIVE);
        gpio_pin_set(u4, 0, 1);
        k_msleep(500);
        gpio_pin_set(u4, 0, 0);
        k_msleep(500);

        printk("Testing U6 pin0\n");
        gpio_pin_configure(u6, 0, GPIO_OUTPUT_INACTIVE);
        gpio_pin_set(u6, 0, 1);
        k_msleep(500);
        gpio_pin_set(u6, 0, 0);
        k_msleep(500);
    }

    return 0;
}
