#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>

#define ADS7950_NODE DT_NODELABEL(ads7950)

/* Let devicetree provide CS, frequency, and SPI mode flags */
static const struct spi_dt_spec ads_spi = SPI_DT_SPEC_GET(
    ADS7950_NODE,
    SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_TRANSFER_MSB,
    0
);

volatile int g_init_ret;
volatile int g_ret;
volatile uint8_t g_tx_buf[2];
volatile uint8_t g_rx_buf[2];

static struct spi_buf tx_buf = {
    .buf = (void *)g_tx_buf,
    .len = sizeof(g_tx_buf),
};

static struct spi_buf rx_buf = {
    .buf = (void *)g_rx_buf,
    .len = sizeof(g_rx_buf),
};

static struct spi_buf_set tx_set = {
    .buffers = &tx_buf,
    .count = 1,
};

static struct spi_buf_set rx_set = {
    .buffers = &rx_buf,
    .count = 1,
};

int main(void)
{
    g_init_ret = 0;
    g_ret = 0;
    g_tx_buf[0] = 0;
    g_tx_buf[1] = 0;
    g_rx_buf[0] = 0;
    g_rx_buf[1] = 0;

    if (!spi_is_ready_dt(&ads_spi)) {
        g_init_ret = -1;
        return 0;
    }

    while (1) {
        /* Send a simple 2-byte frame */
        g_tx_buf[0] = 0x10;
        g_tx_buf[1] = 0x00;
        g_rx_buf[0] = 0x00;
        g_rx_buf[1] = 0x00;

        g_ret = spi_transceive_dt(&ads_spi, &tx_set, &rx_set);

        k_msleep(1000);
    }

    return 0;
}
