#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>

#define ADS7950_NODE DT_NODELABEL(ads7950)

/*
 * 關鍵修正：
 * 1. SPI_WORD_SET(16): 符合 ADS7950 16-bit 規格
 * 2. SPI_MODE_LOOP: 開啟內部環回，沒接硬體也能測試
 */
static const struct spi_dt_spec ads_spi = SPI_DT_SPEC_GET(
    ADS7950_NODE,
    SPI_OP_MODE_MASTER | SPI_WORD_SET(16) | SPI_TRANSFER_MSB | SPI_MODE_LOOP,
    0
);

volatile int g_init_ret = 0;
volatile int g_ret = 0;
volatile uint16_t g_tx_val = 0;
volatile uint16_t g_rx_val = 0;

static struct spi_buf tx_buf = { .buf = (void *)&g_tx_val, .len = 2 };
static struct spi_buf rx_buf = { .buf = (void *)&g_rx_val, .len = 2 };
static struct spi_buf_set tx_set = { .buffers = &tx_buf, .count = 1 };
static struct spi_buf_set rx_set = { .buffers = &rx_buf, .count = 1 };

int main(void)
{
    if (!spi_is_ready_dt(&ads_spi)) {
        g_init_ret = -1;
        return 0;
    }

    while (1) {
        /* 指令：Manual Mode, Channel 0 */
        g_tx_val = 0x1800;
        g_rx_val = 0;

        /* 執行傳輸：在 Loopback 模式下，g_rx_val 應該會變成 0x1800 */
        g_ret = spi_transceive_dt(&ads_spi, &tx_set, &rx_set);

        k_msleep(1000);
    }
    return 0;
}
