#pragma once

#include "Error.h"
#include "MutexGuard.h"
#include "config.h"
#include <expected>
#include <optional>
#include <string.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/ring_buffer.h>

enum class VectornavKind { VECTORNAV };

consteval static const char* kind_to_prefix(VectornavKind kind)
{
    switch (kind) {
    case VectornavKind::VECTORNAV:
        return "[Vectornav]";
    default:
        return "[INVALID]";
    }
}

template <VectornavKind kind, const device* uart_dt_init, k_sem* ready_sem_ptr>
class Vectornav {
private:
    constexpr static int RING_BUF_SIZE = 2048;
    constexpr static int MAX_LINE_SIZE = 256;
    constexpr static uint32_t UART_BAUD_RATE = 115200;

    constexpr static const device* uart_dev = uart_dt_init;
    constexpr static k_sem* ready_sem = ready_sem_ptr;  // Must be static-initialized as the sense thread waits on this

    static inline ImuReading reading = ImuReading_init_default;
    static inline float sense_time_ns = 0.0f;
    static inline uint64_t last_reading_cycle = 0;
    static inline bool has_reading = false;

    // Initialized in init()
    static inline k_mutex reading_mutex;
    static inline k_sem data_ready_sem;
    static inline uint8_t vn_ring_buf_data[RING_BUF_SIZE];
    static inline ring_buf uart_ringbuf;

    // Interrupt-driven UART callback
    static void uart_handler(const device*, void*);

    // Send raw bytes over UART (poll mode - only used for configuration commands)
    static void uart_send(const uint8_t* data, size_t len);

    // Send output configuration commands to the VN-300
    static void configure_outputs();

    // Dispatch a single null-terminated NMEA-style line to the appropriate parser
    static std::expected<void, Error> decode_line(char line[MAX_LINE_SIZE]);

    // Parse a $VNQMR sentence into an ImuReading. Returns true on success.
    static bool parse_vnqmr(const char* line, ImuReading* out);

public:
    static std::expected<void, Error> init();
    static void start_sense();
    static std::optional<std::pair<ImuReading, float>> read();

    // Main sense loop thread, do not call directly.
    static void sense();
};

/// UART interrupt handler
template <VectornavKind kind, const device* uart_dt_init, k_sem* ready_sem_ptr>
void Vectornav<kind, uart_dt_init, ready_sem_ptr>::uart_handler(const device*, void*)
{
    LOG_MODULE_DECLARE(VectornavIMU);

    int err = uart_irq_update(uart_dev);
    if (err < 0) {
        LOG_ERR("%s Failed to update UART IRQ state: %s", kind_to_prefix(kind), Error::from_code(err).build_message().c_str());
        return;
    }

    while (true) {
        int is_ready = uart_irq_rx_ready(uart_dev);
        if (is_ready < 0) {
            LOG_ERR("%s UART RX ready check failed: %s", kind_to_prefix(kind), Error::from_code(is_ready).build_message().c_str());
            return;
        }
        if (!is_ready) {
            break;
        }

        uint8_t byte;
        int bytes_read = uart_fifo_read(uart_dev, &byte, 1);
        if (bytes_read < 0) {
            LOG_ERR("%s Failed to read from UART FIFO: %s", kind_to_prefix(kind), Error::from_code(bytes_read).build_message().c_str());
            return;
        }
        if (bytes_read == 0) {
            break;
        }
        if (ring_buf_put(&uart_ringbuf, &byte, 1) == 0) {
            LOG_WRN("%s Ring buffer full, byte dropped: 0x%02x", kind_to_prefix(kind), byte);
        }
    }
    k_sem_give(&data_ready_sem);
}


/// Send bytes over UART using poll mode. Only used during initialization for configuration commands.
template <VectornavKind kind, const device* uart_dt_init, k_sem* ready_sem_ptr>
void Vectornav<kind, uart_dt_init, ready_sem_ptr>::uart_send(const uint8_t* data, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        uart_poll_out(uart_dev, data[i]);
    }
}

/// Send NMEA configuration commands to enable YPR, IMU, GPS, INS, and magnetometer outputs.
template <VectornavKind kind, const device* uart_dt_init, k_sem* ready_sem_ptr>
void Vectornav<kind, uart_dt_init, ready_sem_ptr>::configure_outputs()
{
    LOG_MODULE_DECLARE(VectornavIMU);
    const char* type_cmd = "$VNWRG,06,08*XX\r\n"; // VNQMR: quat, mag, accel, gyro
    const char* rate_cmd = "$VNWRG,07,40*XX\r\n"; // 40 Hz

    uart_send((const uint8_t*)type_cmd, strlen(type_cmd)); k_msleep(100);
    uart_send((const uint8_t*)rate_cmd, strlen(rate_cmd)); k_msleep(100);

    LOG_INF("%s Outputs configured", kind_to_prefix(kind));
}

template <VectornavKind kind, const device* uart_dt_init, k_sem* ready_sem_ptr>
std::expected<void, Error> Vectornav<kind, uart_dt_init, ready_sem_ptr>::decode_line(char line[MAX_LINE_SIZE])
{
    LOG_MODULE_DECLARE(VectornavIMU);
    LOG_INF("%s rx: %.12s", kind_to_prefix(kind), line);
    ImuReading new_reading = ImuReading_init_default;
    if (parse_vnqmr(line, &new_reading)) {
        MutexGuard guard{&reading_mutex};
        uint64_t curr_cycle = k_cycle_get_64();
        sense_time_ns = static_cast<float>(curr_cycle - last_reading_cycle) / sys_clock_hw_cycles_per_sec() * 1e9f;
        last_reading_cycle = curr_cycle;
        reading = new_reading;
        has_reading = true;
    }
    return {};
}

/// Continuously accumulates sensor readings from the VN-300 over UART. Runs in dedicated thread.
template <VectornavKind kind, const device* uart_dt_init, k_sem* ready_sem_ptr>
void Vectornav<kind, uart_dt_init, ready_sem_ptr>::sense()
{
    LOG_MODULE_DECLARE(VectornavIMU);

    // Await initialization
    k_sem_take(ready_sem, K_FOREVER);
    LOG_INF("%s Sense loop initiated", kind_to_prefix(kind));

    char line[MAX_LINE_SIZE];
    int i = 0;

    while (1) {
        k_sem_take(&data_ready_sem, K_FOREVER);

        uint8_t byte;
        while (ring_buf_get(&uart_ringbuf, &byte, 1) > 0) {
            if (i == 0 && byte != '$'){
                continue;}
            if (byte == '\r') {
                continue;
            }
            if (byte == '\n') {
                if (i > 0) {
                    line[i] = '\0';
                    
                    decode_line(line);
                }
                i = 0;
                continue;
            }
            if (i < MAX_LINE_SIZE - 1) {
                line[i++] = static_cast<char>(byte);
            }
        }
    }
}

// $VNQMR field order: QuatW,QuatX,QuatY,QuatZ,MagX,MagY,MagZ,AccelX,AccelY,AccelZ,GyroX,GyroY,GyroZ
template <VectornavKind kind, const device* uart_dt_init, k_sem* ready_sem_ptr>
bool Vectornav<kind, uart_dt_init, ready_sem_ptr>::parse_vnqmr(const char* line, ImuReading* out)
{
    if (strncmp(line, "$VNQMR,", 7) != 0)
        return false;
    char buf[MAX_LINE_SIZE];
    strncpy(buf, line + 7, sizeof(buf));
    char* tok = strtok(buf, ",");  if (!tok) return false; out->quat_w  = strtof(tok, nullptr);
    tok = strtok(nullptr, ",");    if (!tok) return false; out->quat_x  = strtof(tok, nullptr);
    tok = strtok(nullptr, ",");    if (!tok) return false; out->quat_y  = strtof(tok, nullptr);
    tok = strtok(nullptr, ",");    if (!tok) return false; out->quat_z  = strtof(tok, nullptr);
    tok = strtok(nullptr, ",");    if (!tok) return false; out->mag_x   = strtof(tok, nullptr);
    tok = strtok(nullptr, ",");    if (!tok) return false; out->mag_y   = strtof(tok, nullptr);
    tok = strtok(nullptr, ",");    if (!tok) return false; out->mag_z   = strtof(tok, nullptr);
    tok = strtok(nullptr, ",");    if (!tok) return false; out->accel_x = strtof(tok, nullptr);
    tok = strtok(nullptr, ",");    if (!tok) return false; out->accel_y = strtof(tok, nullptr);
    tok = strtok(nullptr, ",");    if (!tok) return false; out->accel_z = strtof(tok, nullptr);
    tok = strtok(nullptr, ",");    if (!tok) return false; out->gyro_x  = strtof(tok, nullptr);
    tok = strtok(nullptr, ",");    if (!tok) return false; out->gyro_y  = strtof(tok, nullptr);
    tok = strtok(nullptr, "*");    if (!tok) return false; out->gyro_z  = strtof(tok, nullptr);
    return true;
}

/// Initialize VN-300: set up ring buffer, configure UART, attach IRQ, send output config commands.
template <VectornavKind kind, const device* uart_dt_init, k_sem* ready_sem_ptr>
std::expected<void, Error> Vectornav<kind, uart_dt_init, ready_sem_ptr>::init()
{
    LOG_MODULE_DECLARE(VectornavIMU);

    LOG_INF("%s Initializing", kind_to_prefix(kind));

    k_mutex_init(&reading_mutex);
    k_sem_init(&data_ready_sem, 0, 1);
    ring_buf_init(&uart_ringbuf, RING_BUF_SIZE, vn_ring_buf_data);

    LOG_INF("%s Checking UART readiness", kind_to_prefix(kind));
    if (!device_is_ready(uart_dev)) {
        return std::unexpected(Error::from_device_not_ready(uart_dev).context("%s UART is not ready", kind_to_prefix(kind)));
    }

    struct uart_config cfg = {
        .baudrate  = UART_BAUD_RATE,
        .parity    = UART_CFG_PARITY_NONE,
        .stop_bits = UART_CFG_STOP_BITS_1,
        .data_bits = UART_CFG_DATA_BITS_8,
        .flow_ctrl = UART_CFG_FLOW_CTRL_NONE,
    };

    LOG_INF("%s Configuring UART", kind_to_prefix(kind));
    int err = uart_configure(uart_dev, &cfg);
    if (err) {
        return std::unexpected(Error::from_code(err).context("%s Failed to configure UART", kind_to_prefix(kind)));
    }

    configure_outputs();

    LOG_INF("%s Setting UART IRQ", kind_to_prefix(kind));
    err = uart_irq_callback_user_data_set(uart_dev, uart_handler, nullptr);
    if (err) {
        return std::unexpected(Error::from_code(err).context("%s Failed to attach UART interrupt handler", kind_to_prefix(kind)));
    }

    LOG_INF("%s Enabling UART RX interrupt", kind_to_prefix(kind));
    uart_irq_rx_enable(uart_dev);
    
    LOG_INF("%s Initialized", kind_to_prefix(kind));

    return {};
}

template <VectornavKind kind, const device* uart_dt_init, k_sem* ready_sem_ptr>
void Vectornav<kind, uart_dt_init, ready_sem_ptr>::start_sense()
{
    k_sem_give(ready_sem);
}

/// Returns the latest accumulated sensor reading and the YMR cycle time in nanoseconds, if available.
template <VectornavKind kind, const device* uart_dt_init, k_sem* ready_sem_ptr>
std::optional<std::pair<ImuReading, float>> Vectornav<kind, uart_dt_init, ready_sem_ptr>::read()
{
    MutexGuard guard{&reading_mutex};

    if (!has_reading) {
        return std::nullopt;
    }

    // Consume reading
    has_reading = false;

    return {{reading, sense_time_ns}};
}

extern k_sem vectornav_ready_sem;

typedef Vectornav<VectornavKind::VECTORNAV, DEVICE_DT_GET(DT_ALIAS(imu_uart)), &vectornav_ready_sem> VectornavImu;