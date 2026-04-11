#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/flash.h>
#include <extmem.h>

/* Get RCC device (not used here, but kept for future use) */
static const struct device* rcc_dev = DEVICE_DT_GET(DT_NODELABEL(rcc));

/* Get GPIO from device tree (for optional LED/debug use) */
static const struct gpio_dt_spec dir_gpio =
    GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), led_test_gpios);

extern "C" int main(void)
{
    printk("System start: entering main()\n");

    /* Get QSPI flash device from device tree */
    const struct device *flash_dev =
        DEVICE_DT_GET(DT_NODELABEL(mx25l25645g));

    int rc;
    uint32_t write_data = 0x12345678;
    uint32_t read_data = 0;

    /* Test offset (make sure this location is safe to use) */
    off_t offset = 0x1000;

    /* Check if flash device is ready */
    if (!device_is_ready(flash_dev)) {
        printk("Error: QSPI flash device not ready\n");
        return -1;
    }

    /* Get flash page info for erase operation */
    struct flash_pages_info info;
    rc = flash_get_page_info_by_offs(flash_dev, offset, &info);
    if (rc < 0) {
        printk("Error: Failed to get flash page info (%d)\n", rc);
        return rc;
    }

    /* Erase the flash sector before writing */
    rc = flash_erase(flash_dev, info.start_offset, info.size);
    if (rc < 0) {
        printk("Error: Flash erase failed (%d)\n", rc);
        return rc;
    }

    printk("Flash erase successful\n");

    /* Write data to flash */
    rc = flash_write(flash_dev, offset, &write_data, sizeof(write_data));
    if (rc < 0) {
        printk("Error: Flash write failed (%d)\n", rc);
        return rc;
    }

    printk("Flash write successful\n");

    /* Read back data from flash */
    rc = flash_read(flash_dev, offset, &read_data, sizeof(read_data));
    if (rc < 0) {
        printk("Error: Flash read failed (%d)\n", rc);
        return rc;
    }

    /* Verify data */
    if (read_data == write_data) {
        printk("Flash verification SUCCESS: 0x%08x\n", read_data);
    } else {
        printk("Flash verification FAILED: read=0x%08x expected=0x%08x\n",
               read_data, write_data);
    }

    return 0;
}
