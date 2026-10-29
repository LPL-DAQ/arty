#pragma once

#include <zephyr/drivers/spi.h>

int fast_lpspi_transceive_dt(const struct spi_dt_spec* spec, const struct spi_buf_set* tx_bufs, const struct spi_buf_set* rx_bufs);
void fast_lpspi_lock_spi(const struct spi_dt_spec* spec);
void fast_lpspi_release_spi(const struct spi_dt_spec* spec);
