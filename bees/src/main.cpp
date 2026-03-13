#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

/* Place test data in external flash (EXTMEM) */
#include <extmem.h>

static const device* rcc_dev = DEVICE_DT_GET(DT_NODELABEL(rcc));
static const struct gpio_dt_spec dir_gpio = GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), led_test_gpios);

extern "C" int main(void)
{
    const struct device *flash_dev = DEVICE_DT_GET(DT_NODELABEL(mx25l25645g));

    if (!device_is_ready(flash_dev)) {
        printk("QSPI Flash device not ready!\n");
        return -1;
    }

    if (extmem_init() != 0) {
        printk("EXTMEM init failed\n");
    }

    while (1) {
        downloader_task_check_and_run();
        k_sleep(K_MSEC(500));
    }

    return 0;
}
