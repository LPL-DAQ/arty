#pragma once

#include "Error.h"
#include "MutexGuard.h"
#include "config.h"
#include <expected>
#include <optional>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/ring_buffer.h>

#ifdef CONFIG_LIDAR

enum class LidarKind { LIDAR_1, LIDAR_2 };

consteval static const char* kind_to_prefix(LidarKind name)
{
    switch (name) {
    case LidarKind::LIDAR_1:
        return "[Lidar1]";
    case LidarKind::LIDAR_2:
        return "[Lidar2]";
    default:
        return "[INVALID]";
    }
}

template <LidarKind kind, const device* uart_dt_init, k_sem* ready_sem_ptr> class Lidar {
private:
    constexpr static int RING_BUF_SIZE = 64;
    constexpr static int MAX_MSG_SIZE = 9;

    constexpr static const device* uart_dev = uart_dt_init;
    constexpr static k_sem* ready_sem = ready_sem_ptr;  // Must be static-initialized as the sense thread waits on this

    static inline LidarReading reading = LidarReading_init_default;
    static inline float sense_time_ns = 0.0f;
    static inline uint64_t last_reading_cycle = 0;
    static inline bool has_reading = false;

    // Initialized in init()
    static inline k_mutex reading_mutex;
    static inline k_sem data_ready_sem;
    static inline uint8_t lidar_ring_buf_data[RING_BUF_SIZE];
    static inline ring_buf uart_ringbuf;

    // Interrupt-driven UART callback
    static void uart_handler(const device*, void*);

    // Decode a UART message
    static std::expected<void, Error> decode(uint8_t msg[MAX_MSG_SIZE]);

public:
    static std::expected<void, Error> init();
    static void start_sense();
    static std::optional<LidarReading> read();

    // Main sense loop thread, do not call directly.
    static void sense();
};

/// UART interrupt handler
template <LidarKind kind, const device* uart_dt_init, k_sem* ready_sem_ptr> void Lidar<kind, uart_dt_init, ready_sem_ptr>::uart_handler(const device*, void*)
{
    LOG_MODULE_DECLARE(Lidar);
    int err = uart_irq_update(uart_dev);
    if (err < 0) {
        LOG_ERR("%s Failed to start processing UART interrupts: %s", kind_to_prefix(kind), Error::from_code(err).build_message().c_str());
        return;
    }

    // Populate ringbuf from UART fifo.
    while (uart_irq_rx_ready(uart_dev) > 0) {
        uint8_t received_byte;
        int bytes_read = uart_fifo_read(uart_dev, &received_byte, 1);
        if (bytes_read < 0) {
            LOG_ERR("%s Failed to read from UART fifo: %s", kind_to_prefix(kind), Error::from_code(bytes_read).build_message().c_str());
            return;
        }
        if (bytes_read == 0) {
            break;
        }
        ring_buf_put(&uart_ringbuf, &received_byte, 1);
    }
    k_sem_give(&data_ready_sem);
}

/// Continuously populates sensor reading from LiDAR over UART. Runs in dedicated thread.
template <LidarKind kind, const device* uart_dt_init, k_sem* read_sem_ptr> void Lidar<kind, uart_dt_init, read_sem_ptr>::sense()
{
    LOG_MODULE_DECLARE(Lidar);

    // Await initialization
    k_sem_take(ready_sem, K_FOREVER);
    LOG_INF("%s Sense loop initiated", kind_to_prefix(kind));
    LOG_INF("%s Running on thread %p", kind_to_prefix(kind), k_current_get());
    uint8_t msg[MAX_MSG_SIZE];
    int i = 0;

    while (1) {
        k_sem_take(&data_ready_sem, K_FOREVER);

        uint8_t byte;
        while (ring_buf_get(&uart_ringbuf, &byte, 1) > 0) {
            if (i == 0 && byte != 0x59)
                continue;
            if (i == 1 && byte != 0x59) {
                i = 0;
                continue;
            }

            msg[i++] = byte;
            if (i == 9) {
                i = 0;
                decode(msg);
            }
        }
    }
}

template <LidarKind kind, const device* uart_dt_init, k_sem* ready_sem_ptr>
std::expected<void, Error> Lidar<kind, uart_dt_init, ready_sem_ptr>::decode(uint8_t msg[MAX_MSG_SIZE])
{
    uint16_t strength;
    uint16_t temp;
    uint16_t checksum;

    if (msg[0] != 0x59 || msg[1] != 0x59) {
        return std::unexpected(Error::from_cause("Invalid LiDAR message header, byte 0: %d, byte 1: %d", msg[0], msg[1]));
    }

    uint16_t distance = msg[2] | (msg[3] << 8);

    strength = msg[4] | (msg[5] << 8);
    temp = msg[6] | (msg[7] << 8);
    checksum = msg[8];

    uint8_t calculated_checksum = (msg[0] + msg[1] + msg[2] + msg[3] + msg[4] + msg[5] + msg[6] + msg[7]) & 0xFF;
    if (checksum != calculated_checksum) {
        return std::unexpected(Error::from_cause("Invalid checksum, got %d but expected %d", calculated_checksum, checksum));
    }

    float distance_meters = distance / 100.0f;

    {
        MutexGuard reading_guard{&reading_mutex};
        uint64_t curr_cycle = k_cycle_get_64();
        sense_time_ns = static_cast<float>(curr_cycle - last_reading_cycle) / sys_clock_hw_cycles_per_sec() * 1e9f;
        last_reading_cycle = curr_cycle;
        reading = LidarReading{.distance_m = distance_meters, .strength = static_cast<float>(strength)};
        has_reading = true;
    }

    return {};
}

/// Initialize Lidar.
template <LidarKind kind, const device* uart_dt_init, k_sem* ready_sem_ptr> std::expected<void, Error> Lidar<kind, uart_dt_init, ready_sem_ptr>::init()
{
    LOG_MODULE_DECLARE(Lidar);

    LOG_INF("%s Initializing lidar", kind_to_prefix(kind));

    k_mutex_init(&reading_mutex);
    k_sem_init(&data_ready_sem, 0, 1);
    ring_buf_init(&uart_ringbuf, RING_BUF_SIZE, lidar_ring_buf_data);

    LOG_INF("%s Checking uart readiness", kind_to_prefix(kind));
    if (!device_is_ready(uart_dev)) {
        return std::unexpected(Error::from_device_not_ready(uart_dev).context("%s UART is not ready", kind_to_prefix(kind)));
    }

    LOG_INF("%s Setting UART IRQ", kind_to_prefix(kind));
    int err = uart_irq_callback_user_data_set(uart_dev, uart_handler, nullptr);
    if (err < 0) {
        LOG_INF("%s error", kind_to_prefix(kind));
        return std::unexpected(Error::from_code(err).context("%s Failed to attach UART interrupt handler", kind_to_prefix(kind)));
    }

    LOG_INF("%s Enabling UART RX interrupt", kind_to_prefix(kind));
    uart_irq_rx_enable(uart_dev);
    LOG_INF("%s Lidar initialized", kind_to_prefix(kind));

    return {};
}

template <LidarKind kind, const device* uart_dt_init, k_sem* ready_sem_ptr> void Lidar<kind, uart_dt_init, ready_sem_ptr>::start_sense()
{
    k_sem_give(ready_sem);
}

/// Returns a lidar reading and the time it took to acquire, if there's a reading.
template <LidarKind kind, const device* uart_dt_init, k_sem* ready_sem_ptr> std::optional<LidarReading> Lidar<kind, uart_dt_init, ready_sem_ptr>::read()
{
    MutexGuard reading_guard{&reading_mutex};

    if (!has_reading) {
        return std::nullopt;
    }

    // Consume reading
    has_reading = false;

    reading.sense_time_ns = sense_time_ns;
    return {reading};
}

extern k_sem lidar_1_ready_sem;
typedef Lidar<LidarKind::LIDAR_1, DEVICE_DT_GET(DT_ALIAS(lidar_1_uart)), &lidar_1_ready_sem> Lidar1;

extern k_sem lidar_2_ready_sem;
typedef Lidar<LidarKind::LIDAR_2, DEVICE_DT_GET(DT_ALIAS(lidar_2_uart)), &lidar_2_ready_sem> Lidar2;

#endif  // CONFIG_LIDAR
