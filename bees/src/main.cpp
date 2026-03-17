#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>

static const struct device *spi_dev = DEVICE_DT_GET(DT_NODELABEL(spi1));

int main(void)
{
    if (!device_is_ready(spi_dev)) {
        return 0;
    }

    uint8_t tx_buf[2] = {0xAA, 0x55};
    uint8_t rx_buf[2] = {0};

    struct spi_buf tx_bufs[] = {
        {
            .buf = tx_buf,
            .len = sizeof(tx_buf)
        }
    };

    struct spi_buf rx_bufs[] = {
        {
            .buf = rx_buf,
            .len = sizeof(rx_buf)
        }
    };

    struct spi_buf_set tx = {
        .buffers = tx_bufs,
        .count = 1
    };

    struct spi_buf_set rx = {
        .buffers = rx_bufs,
        .count = 1
    };

    struct spi_config spi_cfg = {
        .frequency = 1000000,
        .operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB,
        .slave = 0,
    };

    while (1) {
        spi_transceive(spi_dev, &spi_cfg, &tx, &rx);
        k_sleep(K_MSEC(1000));
    }
}
