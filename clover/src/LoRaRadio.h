#pragma once

#include <cstdint>
#include <expected>

class LoRaRadio {
public:
    // Prevent instantiation if using a static driver class
    LoRaRadio() = delete;

    /// Initializes the SPI hardware bus and verifies the RFM9x silicon version.
    /// Returns std::unexpected with a negative POSIX error code (e.g. -EIO, -ENODEV) on failure.
    static std::expected<void, int> init();

    /// Reads a single byte from an RFM9x internal register.
    static std::expected<uint8_t, int> read_register(uint8_t reg_addr);

    /// Writes a single byte to an RFM9x internal register.
    static std::expected<void, int> write_register(uint8_t reg_addr, uint8_t value);

private: 
    // RFM9x hardware register definitions
    static constexpr uint8_t REG_VERSION = 0x42;
    static constexpr uint8_t CHIP_VERSION_VAL = 0x12;
};