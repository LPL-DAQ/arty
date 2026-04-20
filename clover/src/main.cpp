#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>

#define MCP23017_NODE DT_NODELABEL(mcp23017)

int main(void)
{
    const struct device *mcp = DEVICE_DT_GET(MCP23017_NODE);

    if (!device_is_ready(mcp)) {
        printk("MCP23017 not ready\n");
        return 0;
    }

    int ret = gpio_pin_configure(mcp, 0, GPIO_OUTPUT_INACTIVE);
    if (ret != 0) {
        printk("gpio_pin_configure failed: %d\n", ret);
        return 0;
    }

    while (1) {
        gpio_pin_toggle(mcp, 0);
        printk("toggled MCP23017 pin 0\n");
        k_sleep(K_SECONDS(1));
    }
}
