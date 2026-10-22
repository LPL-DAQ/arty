#include "LoRaRadio.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>

// 1. MUST be in .cpp to prevent duplicate symbol linker errors across translation units
LOG_MODULE_REGISTER(LoRaRadio, CONFIG_LOG_DEFAULT_LEVEL);

// 2. Devicetree macro references and static SPI spec live only inside this file
#define LORA_NODE DT_ALIAS(lora)

static const struct spi_dt_spec lora_spi = SPI_DT_SPEC_GET(
    LORA_NODE,
    SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_TRANSFER_MSB,
    0
);

std::expected<uint8_t, int> LoRaRadio::read_register(uint8_t reg_addr) {
    // Bit 7 = 0 indicates a read transaction on RFM9x/SX127x
    uint8_t tx_buf[2] = { static_cast<uint8_t>(reg_addr & 0x7F), 0x00 };
    uint8_t rx_buf[2] = { 0x00, 0x00 };

    const struct spi_buf tx_spi_buf = { .buf = tx_buf, .len = sizeof(tx_buf) };
    const struct spi_buf_set tx_set   = { .buffers = &tx_spi_buf, .count = 1 };

    const struct spi_buf rx_spi_buf = { .buf = rx_buf, .len = sizeof(rx_buf) };
    const struct spi_buf_set rx_set   = { .buffers = &rx_spi_buf, .count = 1 };

    int err = spi_transceive_dt(&lora_spi, &tx_set, &rx_set);
    if (err) {
        LOG_ERR("SPI read failed on reg 0x%02X: error %d", reg_addr, err);
        return std::unexpected(err);    
    }

    return rx_buf[1];
}

std::expected<void, int> LoRaRadio::write_register(uint8_t reg_addr, uint8_t value) {
    // Bit 7 = 1 indicates a write transaction on RFM9x/SX127x
    uint8_t tx_buf[2] = { static_cast<uint8_t>(reg_addr | 0x80), value };

    const struct spi_buf tx_spi_buf = { .buf = tx_buf, .len = sizeof(tx_buf) };
    const struct spi_buf_set tx_set   = { .buffers = &tx_spi_buf, .count = 1 };

    int err = spi_write_dt(&lora_spi, &tx_set);
    if (err) {
        LOG_ERR("SPI write failed on reg 0x%02X: error %d", reg_addr, err);
        return std::unexpected(err);
    }

    return {};
}

std::expected<void, int> LoRaRadio::init()
{
    LOG_INF("Initializing LoRa SPI bus ...");

    if (!spi_is_ready_dt(&lora_spi)) {
        LOG_ERR("SPI controller device &s is not ready", lora_spi.bus -> name);
        return std::unexpected(-EIO);
    }

    auto result = read_register(REG_VERSION);
    if (!result) {
        return std::unexpected(result.error());
    }

    uint8_t version = *result; 

    if (version != CHIP_VERSION_VAL) {
        LOG_ERR("LoRa silicon check failed! Got 0x%02X, expected 0x%02X", version, CHIP_VERSION_VAL);
        return std::unexpected(-ENODEV);
    }

    LOG_INF("LoRa SPI communication booted successfully. Silicon Version: 0x%02X", version);
    return {};
}