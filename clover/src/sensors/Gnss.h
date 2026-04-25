#pragma once

#include "Error.h"
#include "MutexGuard.h"
#include "config.h"
#include <expected>
#include <optional>
#include <stdint.h>
#include <string.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/ring_buffer.h>

struct GnssReading {
    // Position in local plane (from [mp])
    double north_m;
    double east_m;
    double up_m;
    float  pos_sigma_m;     // 3D position RMS [m]

    // Cartesian ECEF velocity (from [VE])
    float vx_ms;            // ECEF X velocity [m/s]
    float vy_ms;            // ECEF Y velocity [m/s]
    float vz_ms;            // ECEF Z velocity [m/s]
    float vel_sigma_ms;     // 3D velocity RMS [m/s]

    // RMS error breakdown (from [SG])
    float hrms_m;           // Horizontal position RMS [m]
    float vrms_m;           // Vertical position RMS [m]
    float hvel_rms_ms;      // Horizontal velocity RMS [m/s]
    float vvel_rms_ms;      // Vertical velocity RMS [m/s]

    // Timing
    uint32_t solution_time_ms;   // From [ST]: epoch time mod 1 day [ms]
    uint32_t receiver_time_ms;   // From [~~]: receiver time mod 1 day [ms]

    // Solution type (0=none, 1=standalone, 2=DGPS, 3=float, 4=fixed)
    uint8_t sol_type;
};

extern k_sem gnss_ready_sem;  // Must be static-initialized as the sense thread waits on this

namespace Gnss {

constexpr int RING_BUF_SIZE   = 512;
constexpr int MAX_BODY_LENGTH = 256;

inline const device* uart_dev;
inline k_sem* const ready_sem = &gnss_ready_sem;  // Must be static-initialized as the sense thread waits on this


inline GnssReading current_reading = {};
inline float        sense_time_ns  = 0.0f;
inline uint64_t     last_cycle     = 0;
inline bool         has_reading    = false;

// Initialized in init()
inline k_mutex  reading_mutex;
inline k_sem    data_ready_sem;
inline uint8_t  gnss_ring_buf_data[RING_BUF_SIZE];
inline ring_buf uart_ringbuf;

// UART interrupt callback
void uart_handler(const device*, void*);

// Little-endian binary reads
float    read_f4(const uint8_t* p);
double   read_f8(const uint8_t* p);
uint32_t read_u4(const uint8_t* p);

// GREIS message decoders
void decode_RT(const uint8_t* body, int length);  // [~~] Receiver Time
void decode_ST(const uint8_t* body, int length);  // [ST] Solution Time-Tag
void decode_mp(const uint8_t* body, int length);  // [mp] Position in Local Plane
void decode_VE(const uint8_t* body, int length);  // [VE] Cartesian Velocity
void decode_SG(const uint8_t* body, int length);  // [SG] Position/Velocity RMS Errors
void decode(const char id[2], const uint8_t* body, int body_length);

// GREIS checksum: rotate-left XOR over ID, 3-byte hex length, and body (excluding final cs byte).
uint8_t calculate_checksum(const char id[2], const char length_buf[4], const uint8_t* body, int body_length);

std::expected<void, Error> init();
void start_sense();
std::optional<std::pair<GnssReading, float>> read();

// Main sense loop thread, do not call directly.
void sense();

}

/// UART interrupt handler
inline void Gnss::uart_handler(const device* dev, void*)
{
    LOG_MODULE_DECLARE(Gnss);
    if (!uart_irq_update(dev)) {
        return;
    }
    uint8_t byte;
    while (uart_irq_rx_ready(dev) > 0) {
        int bytes_read = uart_fifo_read(dev, &byte, 1);
        if (bytes_read <= 0) {
            break;
        }
        ring_buf_put(&uart_ringbuf, &byte, 1);
    }
    k_sem_give(&data_ready_sem);
}

inline float Gnss::read_f4(const uint8_t* p)
{
    float v;
    memcpy(&v, p, 4);
    return v;
}

inline double Gnss::read_f8(const uint8_t* p)
{
    double v;
    memcpy(&v, p, 8);
    return v;
}

inline uint32_t Gnss::read_u4(const uint8_t* p)
{
    return (uint32_t)p[0]
         | ((uint32_t)p[1] << 8)
         | ((uint32_t)p[2] << 16)
         | ((uint32_t)p[3] << 24);
}

// [~~] Receiver Time {5}: u4 tod, u1 cs
inline void Gnss::decode_RT(const uint8_t* body, int length)
{
    LOG_MODULE_DECLARE(Gnss);
    if (length < 5) {
        LOG_WRN("[Gnss] [~~] bad length");
        return;
    }
    uint32_t tod = read_u4(body);
    MutexGuard g{&reading_mutex};
    uint64_t curr_cycle = k_cycle_get_64();
    sense_time_ns = static_cast<float>(curr_cycle - last_cycle) / sys_clock_hw_cycles_per_sec() * 1e9f;
    last_cycle = curr_cycle;
    current_reading.receiver_time_ms = tod;
    has_reading = true;
}

// [ST] Solution Time-Tag {6}: u4 time, u1 solType, u1 cs
inline void Gnss::decode_ST(const uint8_t* body, int length)
{
    LOG_MODULE_DECLARE(Gnss);
    if (length < 6) {
        LOG_WRN("[Gnss] [ST] bad length");
        return;
    }
    uint32_t t  = read_u4(body);
    uint8_t  st = body[4];
    MutexGuard g{&reading_mutex};
    current_reading.solution_time_ms = t;
    current_reading.sol_type         = st;
}

// [mp] Position in Local Plane {45}:
//   f8 n, f8 e, f8 u, f8 sep, f4 pSigma, u1 solType, ...
inline void Gnss::decode_mp(const uint8_t* body, int length)
{
    LOG_MODULE_DECLARE(Gnss);
    if (length < 45) {
        LOG_WRN("[Gnss] [mp] bad length");
        return;
    }
    double  n     = read_f8(body + 0);
    double  e     = read_f8(body + 8);
    double  u     = read_f8(body + 16);
    float   sigma = read_f4(body + 32);
    uint8_t st    = body[36];

    MutexGuard g{&reading_mutex};
    current_reading.north_m     = n;
    current_reading.east_m      = e;
    current_reading.up_m        = u;
    current_reading.pos_sigma_m = sigma;
    current_reading.sol_type    = st;
}

// [VE] Cartesian Velocity {18}: f4 x, f4 y, f4 z, f4 vSigma, u1 solType, u1 cs
inline void Gnss::decode_VE(const uint8_t* body, int length)
{
    LOG_MODULE_DECLARE(Gnss);
    if (length < 18) {
        LOG_WRN("[Gnss] [VE] bad length");
        return;
    }
    float   vx   = read_f4(body + 0);
    float   vy   = read_f4(body + 4);
    float   vz   = read_f4(body + 8);
    float   vsig = read_f4(body + 12);
    uint8_t st   = body[16];

    MutexGuard g{&reading_mutex};
    current_reading.vx_ms        = vx;
    current_reading.vy_ms        = vy;
    current_reading.vz_ms        = vz;
    current_reading.vel_sigma_ms = vsig;
    current_reading.sol_type     = st;
}

// [SG] Position and Velocity RMS Errors {18}: f4 hpos, f4 vpos, f4 hvel, f4 vvel, u1 solType, u1 cs
inline void Gnss::decode_SG(const uint8_t* body, int length)
{
    LOG_MODULE_DECLARE(Gnss);
    if (length < 18) {
        LOG_WRN("[Gnss] [SG] bad length");
        return;
    }
    float   hpos = read_f4(body + 0);
    float   vpos = read_f4(body + 4);
    float   hvel = read_f4(body + 8);
    float   vvel = read_f4(body + 12);
    uint8_t st   = body[16];

    MutexGuard g{&reading_mutex};
    current_reading.hrms_m      = hpos;
    current_reading.vrms_m      = vpos;
    current_reading.hvel_rms_ms = hvel;
    current_reading.vvel_rms_ms = vvel;
    current_reading.sol_type    = st;
}

inline void Gnss::decode(const char id[2], const uint8_t* body, int body_length)
{
    if (id[0] == '~' && id[1] == '~') { decode_RT(body, body_length); return; }
    if (id[0] == 'S' && id[1] == 'T') { decode_ST(body, body_length); return; }
    if (id[0] == 'm' && id[1] == 'p') { decode_mp(body, body_length); return; }
    if (id[0] == 'V' && id[1] == 'E') { decode_VE(body, body_length); return; }
    if (id[0] == 'S' && id[1] == 'G') { decode_SG(body, body_length); return; }
}

inline uint8_t Gnss::calculate_checksum(const char id[2], const char length_buf[4], const uint8_t* body, int body_length)
{
    uint8_t res = 0;
    auto rot_left = [](uint8_t val) -> uint8_t {
        return (uint8_t)((val << 2) | (val >> 6));
    };
    res = rot_left(res) ^ (uint8_t)id[0];
    res = rot_left(res) ^ (uint8_t)id[1];
    res = rot_left(res) ^ (uint8_t)length_buf[0];
    res = rot_left(res) ^ (uint8_t)length_buf[1];
    res = rot_left(res) ^ (uint8_t)length_buf[2];
    for (int i = 0; i < body_length - 1; i++) {
        res = rot_left(res) ^ body[i];
    }
    return rot_left(res);
}

/// Continuously accumulates GNSS readings from the receiver over UART. Runs in dedicated thread.
// GREIS frame format:
//   [2-char ASCII ID][3-char ASCII hex length][N bytes binary body][0x0A opt]
//
// States:
//   SYNC   — collecting 2 valid ASCII bytes as message ID
//   LENGTH — reading 3 ASCII hex chars for body length
//   BODY   — accumulating body bytes
inline void Gnss::sense()
{
    LOG_MODULE_DECLARE(Gnss);

    k_sem_take(ready_sem, K_FOREVER);
    LOG_INF("[Gnss] Sense loop initiated");

    enum { SYNC, LENGTH, BODY } state = SYNC;

    char    id[2]         = {0, 0};
    char    length_buf[4] = {0, 0, 0, 0};
    uint8_t body[MAX_BODY_LENGTH];
    int     sync_count    = 0;
    int     length_count  = 0;
    int     body_length   = 0;
    int     body_count    = 0;

    while (1) {
        k_sem_take(&data_ready_sem, K_MSEC(1000));

        uint8_t byte;
        while (ring_buf_get(&uart_ringbuf, &byte, 1) > 0) {
            if (byte == 0x0A || byte == 0x0D) {
                continue;
            }

            switch (state) {

            case SYNC:
                if (byte >= 0x30 && byte <= 0x7E) {
                    if (sync_count == 0) {
                        id[0] = (char)byte;
                        sync_count = 1;
                    } else {
                        id[1] = (char)byte;
                        sync_count   = 0;
                        length_count = 0;
                        state        = LENGTH;
                    }
                } else {
                    sync_count = 0;
                }
                break;

            case LENGTH:
                if ((byte >= '0' && byte <= '9') ||
                    (byte >= 'A' && byte <= 'F') ||
                    (byte >= 'a' && byte <= 'f')) {
                    length_buf[length_count++] = (char)byte;
                    if (length_count == 3) {
                        length_buf[3] = '\0';
                        body_length   = (int)strtol(length_buf, nullptr, 16);
                        body_count    = 0;
                        length_count  = 0;
                        if (body_length > 0 && body_length <= MAX_BODY_LENGTH) {
                            state = BODY;
                        } else {
                            LOG_WRN("[Gnss] invalid body length %d for id %c%c",
                                    body_length, id[0], id[1]);
                            state = SYNC;
                        }
                    }
                } else {
                    state      = SYNC;
                    sync_count = 0;
                }
                break;

            case BODY:
                body[body_count++] = byte;
                if (body_count == body_length) {
                    uint8_t calc_cs     = calculate_checksum(id, length_buf, body, body_length);
                    uint8_t expected_cs = body[body_length - 1];
                    if (calc_cs != expected_cs) {
                        LOG_WRN("[Gnss] [%c%c] bad checksum (calc 0x%02X, got 0x%02X)",
                                id[0], id[1], calc_cs, expected_cs);
                    } else {
                        decode(id, body, body_length);
                    }
                    state = SYNC;
                }
                break;
            }
        }
    }
}

/// Initialize GNSS: set up ring buffer, configure UART, attach IRQ.
inline std::expected<void, Error> Gnss::init()
{
    LOG_MODULE_DECLARE(Gnss);
    LOG_INF("[Gnss] Initializing");

    k_mutex_init(&reading_mutex);
    k_sem_init(&data_ready_sem, 0, 1);
    ring_buf_init(&uart_ringbuf, RING_BUF_SIZE, gnss_ring_buf_data);

    uart_dev = DEVICE_DT_GET(DT_ALIAS(gnss_uart));
    if (!device_is_ready(uart_dev)) {
        return std::unexpected(Error::from_device_not_ready(uart_dev).context("[Gnss] UART is not ready"));
    }

    int err = uart_irq_callback_user_data_set(uart_dev, uart_handler, nullptr);
    if (err < 0) {
        return std::unexpected(Error::from_code(err).context("[Gnss] Failed to attach UART interrupt handler"));
    }

    uart_irq_rx_enable(uart_dev);
    LOG_INF("[Gnss] Initialized");
    return {};
}

inline void Gnss::start_sense()
{
    k_sem_give(ready_sem);
}

/// Returns the latest complete GNSS epoch reading and sense cycle time [ns], if available.
inline std::optional<std::pair<GnssReading, float>> Gnss::read()
{
    MutexGuard g{&reading_mutex};
    if (!has_reading) {
        return std::nullopt;
    }
    has_reading = false;
    return {{current_reading, sense_time_ns}};
}
