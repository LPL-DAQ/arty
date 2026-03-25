#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>

/* Get SPI device from devicetree node label "spi1" */
#define SPI1_NODE DT_NODELABEL(spi1)

int main(void)
{
    /* 1. Get the SPI device binding */
    const struct device *spi_dev = DEVICE_DT_GET(SPI1_NODE);

    if (!device_is_ready(spi_dev)) {
        return -1;
    }

    /* 2. Configure Chip Select (CS) object */
    struct spi_cs_control cs_ctrl = {0};
    cs_ctrl.gpio = GPIO_DT_SPEC_GET_BY_IDX(SPI1_NODE, cs_gpios, 0);
    cs_ctrl.delay = 0;

    /* Check if the GPIO port for CS is ready (Requires CONFIG_GPIO=y) */
    if (cs_ctrl.gpio.port == NULL || !device_is_ready(cs_ctrl.gpio.port)) {
        return -1;
    }

    /* 3. Prepare data buffers
     * Using static/global or local arrays is fine now that CONFIG_PM=n
     */
    uint8_t tx_data[] = {0x18, 0x00}; // ADS7953 example command
    uint8_t rx_data[2] = {0, 0};

    struct spi_buf tx_b = { .buf = tx_data, .len = sizeof(tx_data) };
    struct spi_buf rx_b = { .buf = rx_data, .len = sizeof(rx_data) };

    struct spi_buf_set tx_set = { .buffers = &tx_b, .count = 1 };
    struct spi_buf_set rx_set = { .buffers = &rx_b, .count = 1 };

    volatile int status = 0;

    while (1) {
        /* 4. Define SPI configuration
         * Note: In your environment, .cs is an object, not a pointer.
         */
        struct spi_config spi_cfg = {0};
        spi_cfg.frequency = 1000000;

        /* Operation: Master, 8-bit, MSB First, Mode 0, Single Line */
        spi_cfg.operation = SPI_OP_MODE_MASTER | SPI_WORD_SET(8) |
                             SPI_TRANSFER_MSB | SPI_LINES_SINGLE;
        spi_cfg.slave     = 0;

        /* FIX: Direct object assignment (copying the struct)
         * This matches the compiler's expected type 'spi_cs_control'
         */
        spi_cfg.cs = cs_ctrl;

        /* 5. Perform the SPI transceiver call */
        status = spi_transceive(spi_dev, &spi_cfg, &tx_set, &rx_set);

        /*
         * If status is 0, transmission is successful.
         * If status is -22, double check the operation flags above.
         */

        k_sleep(K_MSEC(1000));
    }

    return 0;
}
