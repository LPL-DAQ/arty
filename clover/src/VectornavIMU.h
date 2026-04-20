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

enum class VectornavKind { VECTORNAV_1 };

consteval static const char* kind_to_prefix(VectornavKind kind)
{
    switch (kind) {
    case VectornavKind::VECTORNAV_1:
        return "[Vectornav1]";
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

    // Per-sentence parsers — return true if the line matched and was successfully parsed
    static bool parse_ymr(const char* line, float* yaw, float* pitch, float* roll);
    static bool parse_imu(const char* line, float* ax, float* ay, float* az, float* gx, float* gy, float* gz);
    static bool parse_gps(const char* line, double* lat, double* lon, float* alt);
    static bool parse_ins(const char* line, double* lat, double* lon, float* alt, float* vn_vel, float* ve_vel, float* vd_vel);
    static bool parse_mag(const char* line, float* mx, float* my, float* mz);

public:
    static std::expected<void, Error> init();
    static std::expected<void, Error> handle_configure_analog_sensors(const ConfigureAnalogSensorsRequest& req);
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
            return;
        }

        uint8_t byte;
        int bytes_read = uart_fifo_read(uart_dev, &byte, 1);
        if (bytes_read < 0) {
            LOG_ERR("%s Failed to read from UART FIFO: %s", kind_to_prefix(kind), Error::from_code(bytes_read).build_message().c_str());
            return;
        }
        if (bytes_read == 0) {
            return;
        }

        ring_buf_put(&uart_ringbuf, &byte, 1);
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

    const char* ymr_cmd = "$VNWRG,75,1,8,01,0000000000000001*XX\r\n";   // YPR at 40 Hz
    const char* imu_cmd = "$VNWRG,75,2,8,01,0000000000000006*XX\r\n";   // Accel + gyro at 40 Hz
    const char* gps_cmd = "$VNWRG,76,2,200,01,0000000000000008*XX\r\n"; // GPS position + velocity at 5 Hz
    const char* ins_cmd = "$VNWRG,77,2,8,01,0000000000000010*XX\r\n";   // Fused INS at 40 Hz
    const char* mag_cmd = "$VNWRG,75,3,8,01,0000000000000002*XX\r\n";   // Magnetometer at 40 Hz

    uart_send((const uint8_t*)ymr_cmd, strlen(ymr_cmd)); k_msleep(100);
    uart_send((const uint8_t*)imu_cmd, strlen(imu_cmd)); k_msleep(100);
    uart_send((const uint8_t*)gps_cmd, strlen(gps_cmd)); k_msleep(100);
    uart_send((const uint8_t*)ins_cmd, strlen(ins_cmd)); k_msleep(100);
    uart_send((const uint8_t*)mag_cmd, strlen(mag_cmd)); k_msleep(100);

    LOG_INF("%s Outputs configured", kind_to_prefix(kind));
}

/// Dispatch a single NMEA-style line to the appropriate per-sentence parser and update the shared reading.
/// sense_time_ns is only updated on YMR packets (the primary 40 Hz attitude output) to give a stable timing signal.
/// All other sentence types silently update their sub-fields without marking a new has_reading.
/// Unrecognised lines (e.g. ACK responses) are silently discarded.
///
/// NOTE: field names on ImuReading below must match the protobuf definition.
template <VectornavKind kind, const device* uart_dt_init, k_sem* ready_sem_ptr>
std::expected<void, Error> Vectornav<kind, uart_dt_init, ready_sem_ptr>::decode_line(char line[MAX_LINE_SIZE])
{
    {
        float yaw, pitch, roll;
        if (parse_ymr(line, &yaw, &pitch, &roll)) {
            MutexGuard guard{&reading_mutex};
            uint64_t curr_cycle = k_cycle_get_64();
            sense_time_ns = static_cast<float>(curr_cycle - last_reading_cycle) / sys_clock_hw_cycles_per_sec() * 1e9f;
            last_reading_cycle = curr_cycle;
            reading.yaw = yaw;
            reading.pitch = pitch;
            reading.roll = roll;
            has_reading = true;
            return {};
        }
    }

    {
        float ax, ay, az, gx, gy, gz;
        if (parse_imu(line, &ax, &ay, &az, &gx, &gy, &gz)) {
            MutexGuard guard{&reading_mutex};
            reading.accel_x = ax;
            reading.accel_y = ay;
            reading.accel_z = az;
            reading.gyro_x = gx;
            reading.gyro_y = gy;
            reading.gyro_z = gz;
            return {};
        }
    }

    {
        double lat, lon;
        float alt;
        if (parse_gps(line, &lat, &lon, &alt)) {
            MutexGuard guard{&reading_mutex};
            reading.gps_lat = lat;
            reading.gps_lon = lon;
            reading.gps_alt = alt;
            return {};
        }
    }

    {
        double lat, lon;
        float alt, vn_vel, ve_vel, vd_vel;
        if (parse_ins(line, &lat, &lon, &alt, &vn_vel, &ve_vel, &vd_vel)) {
            MutexGuard guard{&reading_mutex};
            reading.ins_lat = lat;
            reading.ins_lon = lon;
            reading.ins_alt = alt;
            reading.vel_n = vn_vel;
            reading.vel_e = ve_vel;
            reading.vel_d = vd_vel;
            return {};
        }
    }

    {
        float mx, my, mz;
        if (parse_mag(line, &mx, &my, &mz)) {
            MutexGuard guard{&reading_mutex};
            reading.mag_x = mx;
            reading.mag_y = my;
            reading.mag_z = mz;
            return {};
        }
    }

    return {};  // Unrecognised sentence type (e.g. ACK) — silently discard
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

template <VectornavKind kind, const device* uart_dt_init, k_sem* ready_sem_ptr>
bool Vectornav<kind, uart_dt_init, ready_sem_ptr>::parse_ymr(const char* line, float* yaw, float* pitch, float* roll)
{
    if (strncmp(line, "$VNYMR,", 7) != 0)
        return false;
    char buf[MAX_LINE_SIZE];
    strncpy(buf, line + 7, sizeof(buf));
    char* token = strtok(buf, ",");
    if (!token) return false;
    *yaw = strtof(token, nullptr);
    token = strtok(nullptr, ",");
    if (!token) return false;
    *pitch = strtof(token, nullptr);
    token = strtok(nullptr, "*");
    if (!token) return false;
    *roll = strtof(token, nullptr);
    return true;
}

template <VectornavKind kind, const device* uart_dt_init, k_sem* ready_sem_ptr>
bool Vectornav<kind, uart_dt_init, ready_sem_ptr>::parse_imu(const char* line, float* ax, float* ay, float* az, float* gx, float* gy, float* gz)
{
    if (strncmp(line, "$VNIMU,", 7) != 0)
        return false;
    char buf[MAX_LINE_SIZE];
    strncpy(buf, line + 7, sizeof(buf));
    char* token = strtok(buf, ",");
    if (!token) return false;
    *ax = strtof(token, nullptr);
    token = strtok(nullptr, ",");
    if (!token) return false;
    *ay = strtof(token, nullptr);
    token = strtok(nullptr, ",");
    if (!token) return false;
    *az = strtof(token, nullptr);
    token = strtok(nullptr, ",");
    if (!token) return false;
    *gx = strtof(token, nullptr);
    token = strtok(nullptr, ",");
    if (!token) return false;
    *gy = strtof(token, nullptr);
    token = strtok(nullptr, "*");
    if (!token) return false;
    *gz = strtof(token, nullptr);
    return true;
}

template <VectornavKind kind, const device* uart_dt_init, k_sem* ready_sem_ptr>
bool Vectornav<kind, uart_dt_init, ready_sem_ptr>::parse_gps(const char* line, double* lat, double* lon, float* alt)
{
    if (strncmp(line, "$VNGPE,", 7) != 0)
        return false;
    char buf[MAX_LINE_SIZE];
    strncpy(buf, line + 7, sizeof(buf));
    char* token = strtok(buf, ",");
    if (!token) return false;
    *lat = strtod(token, nullptr);
    token = strtok(nullptr, ",");
    if (!token) return false;
    *lon = strtod(token, nullptr);
    token = strtok(nullptr, "*");
    if (!token) return false;
    *alt = strtof(token, nullptr);
    return true;
}

template <VectornavKind kind, const device* uart_dt_init, k_sem* ready_sem_ptr>
bool Vectornav<kind, uart_dt_init, ready_sem_ptr>::parse_ins(const char* line, double* lat, double* lon, float* alt, float* vn_vel, float* ve_vel, float* vd_vel)
{
    if (strncmp(line, "$VNINS,", 7) != 0)
        return false;
    char buf[MAX_LINE_SIZE];
    strncpy(buf, line + 7, sizeof(buf));
    char* token = strtok(buf, ",");
    if (!token) return false;
    *lat = strtod(token, nullptr);
    token = strtok(nullptr, ",");
    if (!token) return false;
    *lon = strtod(token, nullptr);
    token = strtok(nullptr, ",");
    if (!token) return false;
    *alt = strtof(token, nullptr);
    token = strtok(nullptr, ",");
    if (!token) return false;
    *vn_vel = strtof(token, nullptr);
    token = strtok(nullptr, ",");
    if (!token) return false;
    *ve_vel = strtof(token, nullptr);
    token = strtok(nullptr, "*");
    if (!token) return false;
    *vd_vel = strtof(token, nullptr);
    return true;
}

template <VectornavKind kind, const device* uart_dt_init, k_sem* ready_sem_ptr>
bool Vectornav<kind, uart_dt_init, ready_sem_ptr>::parse_mag(const char* line, float* mx, float* my, float* mz)
{
    if (strncmp(line, "$VNMAG,", 7) != 0)
        return false;
    char buf[MAX_LINE_SIZE];
    strncpy(buf, line + 7, sizeof(buf));
    char* token = strtok(buf, ",");
    if (!token) return false;
    *mx = strtof(token, nullptr);
    token = strtok(nullptr, ",");
    if (!token) return false;
    *my = strtof(token, nullptr);
    token = strtok(nullptr, "*");
    if (!token) return false;
    *mz = strtof(token, nullptr);
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

    LOG_INF("%s Setting UART IRQ", kind_to_prefix(kind));
    err = uart_irq_callback_user_data_set(uart_dev, uart_handler, nullptr);
    if (err) {
        return std::unexpected(Error::from_code(err).context("%s Failed to attach UART interrupt handler", kind_to_prefix(kind)));
    }

    LOG_INF("%s Enabling UART RX interrupt", kind_to_prefix(kind));
    uart_irq_rx_enable(uart_dev);

    LOG_INF("%s Configuring sensor outputs", kind_to_prefix(kind));
    configure_outputs();

    LOG_INF("%s Initialized", kind_to_prefix(kind));
    k_sem_give(ready_sem);

    return {};
}

/// Reconfigure sensor output rates/fields at runtime from a protobuf request.
/// TODO: map req fields to selective configure_outputs() calls as the proto definition warrants.
template <VectornavKind kind, const device* uart_dt_init, k_sem* ready_sem_ptr>
std::expected<void, Error> Vectornav<kind, uart_dt_init, ready_sem_ptr>::handle_configure_analog_sensors(const ConfigureAnalogSensorsRequest& req)
{
    configure_outputs();
    return {};
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

extern k_sem vectornav_1_ready_sem;

typedef Vectornav<VectornavKind::VECTORNAV_1, DEVICE_DT_GET(DT_ALIAS(imu_uart)), &vectornav_1_ready_sem> Vectornav1;