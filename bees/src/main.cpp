#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>

#define ADS7950_NODE DT_NODELABEL(ads7950)

/* SPI config: internal loopback */
static const struct spi_dt_spec ads_spi = SPI_DT_SPEC_GET(
	ADS7950_NODE,
	SPI_OP_MODE_MASTER | SPI_WORD_SET(16) | SPI_TRANSFER_MSB,
	0
);

/* Debug variables (watch these in debugger) */
volatile int g_init_ret = 0;
volatile int g_ret = 0;
volatile int g_match = 0;   // 1 = success, 0 = mismatch

volatile uint16_t g_tx_val[2] = {0};
volatile uint16_t g_rx_val[2] = {0};

/* SPI buffers */
static struct spi_buf tx_bufs[] = {
	{
		.buf = (void *)g_tx_val,
		.len = sizeof(g_tx_val),
	},
};

static struct spi_buf rx_bufs[] = {
	{
		.buf = (void *)g_rx_val,
		.len = sizeof(g_rx_val),
	},
};

static struct spi_buf_set tx_set = {
	.buffers = tx_bufs,
	.count = 1,
};

static struct spi_buf_set rx_set = {
	.buffers = rx_bufs,
	.count = 1,
};

int main(void)
{
	if (!spi_is_ready_dt(&ads_spi)) {
		g_init_ret = -1;
		while (1);
	}

	while (1) {
		/* Prepare test pattern */
		g_tx_val[0] = 0x1800;
		g_tx_val[1] = 0x1800;

		g_rx_val[0] = 0;
		g_rx_val[1] = 0;

		/* SPI transfer */
		g_ret = spi_transceive_dt(&ads_spi, &tx_set, &rx_set);

		/* Check result */
		if (g_ret == 0 &&
		    g_rx_val[0] == g_tx_val[0] &&
		    g_rx_val[1] == g_tx_val[1]) {
			g_match = 1;   // loopback OK
		} else {
			g_match = 0;   // failed
		}

		k_msleep(1000);
	}

	return 0;
}
