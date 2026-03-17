#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/flash.h>
/* Place test data in external flash (EXTMEM) */
#include <extmem.h>

static const struct device* rcc_dev = DEVICE_DT_GET(DT_NODELABEL(rcc));
static const struct gpio_dt_spec dir_gpio = GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), led_test_gpios);

extern "C" int main(void)
{
    /* 1. 只留最簡單的列印，確認能不能進到 main */
    printk("Hello! Entering main...\n");

    const struct device *flash_dev = DEVICE_DT_GET(DT_NODELABEL(mx25l25645g));


    int rc;
    uint32_t read_data = 0;

    if (!device_is_ready(flash_dev)) {
        printk("QSPI Flash device not ready!\n");
        return -1;
    }

    rc = flash_read(flash_dev, 0x0, &read_data, sizeof(read_data));

    if (rc == 0) {
        printk("QSPI Read Successful! Data at 0x0: 0x%08x\n", read_data);
    } else {
        printk("QSPI Read Failed! Error code: %d\n", rc);
    }

    /*if (extmem_init() != 0) {
        printk("EXTMEM init failed\n");
    }*/

    /*while (1) {
        downloader_task_check_and_run();
        k_sleep(K_MSEC(500));
    }*/

    return 0;
}
