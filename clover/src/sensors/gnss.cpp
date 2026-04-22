#include "gnss.h"
#include "../MutexGuard.h"
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/ring_buffer.h>
#include <string.h>

LOG_MODULE_REGISTER(GNSS, CONFIG_LOG_DEFAULT_LEVEL);

K_MUTEX_DEFINE(gnss_lock);

K_SEM_DEFINE(gnss_ready, 0, 1);
K_SEM_DEFINE(gnss_data_ready, 0, 1);

RING_BUF_DECLARE(gnss_ringbuf, 512);

static GnssReading current_reading = {};
static const struct device* gnss_dev;

static volatile uint32_t uart_bytes_received = 0;

// UART interrupt callback
static void gnss_uart_cb(const struct device* dev, void* user_data)
{
    uint8_t received_byte;
    if (!uart_irq_update(dev)) {
        return;
    }
    while (uart_irq_rx_ready(dev)) {
        int bytes_read = uart_fifo_read(dev, &received_byte, 1);
        if (bytes_read <= 0) {
            break;
        }
        ring_buf_put(&gnss_ringbuf, &received_byte, 1);
        uart_bytes_received++;
    }
    
    k_sem_give(&gnss_data_ready);
}

static float read_f4(const uint8_t* p)
{
    float v;
    memcpy(&v, p, 4);
    return v;
}

static double read_f8(const uint8_t* p)
{
    double v;
    memcpy(&v, p, 8);
    return v;
}

static uint32_t read_u4(const uint8_t* p)
{
    return (uint32_t)p[0]
         | ((uint32_t)p[1] << 8)
         | ((uint32_t)p[2] << 16)
         | ((uint32_t)p[3] << 24);
}

// Message decoders

// [~~] Reciever Time {5}: u4 tod, u1 cs
static void decode_RT(const uint8_t* body, int length)
{
    if (length < 5) {
        LOG_WRN("GNSS: [~~] bad length");
        return;
    }
    uint32_t tod = read_u4(body);
    MutexGuard g{&gnss_lock};
    current_reading.receiver_time_ms = tod;

    LOG_INF("GNSS sol_type=%u sol_time=%u ms rx_time=%u ms",
            current_reading.sol_type, current_reading.solution_time_ms, current_reading.receiver_time_ms);
    LOG_INF("GNSS pos  N=%.3f E=%.3f U=%.3f sigma=%.3f m",
            current_reading.north_m, current_reading.east_m, current_reading.up_m, (double)current_reading.pos_sigma_m);
    LOG_INF("GNSS vel  vx=%.3f vy=%.3f vz=%.3f sigma=%.3f m/s",
            (double)current_reading.vx_ms, (double)current_reading.vy_ms, (double)current_reading.vz_ms, (double)current_reading.vel_sigma_ms);
    LOG_INF("GNSS rms  hpos=%.3f vpos=%.3f hvel=%.3f vvel=%.3f",
            (double)current_reading.hrms_m, (double)current_reading.vrms_m, (double)current_reading.hvel_rms_ms, (double)current_reading.vvel_rms_ms);
}

// [ST] Solution Time-Tag {6}: u4 time, u1 solType, u1 cs
static void decode_ST(const uint8_t* body, int length)
{
    if (length < 6) {
        LOG_WRN("GNSS: [ST] bad length");
        return;
    }
    uint32_t t  = read_u4(body);
    uint8_t  st = body[4];
    MutexGuard g{&gnss_lock};
    current_reading.solution_time_ms = t;
    current_reading.sol_type = st;
}

// [mp] Position in Local Plane {45}:
//   f8 n, f8 e, f8 u, f8 sep, f4 pSigma,
//   u1 solType, u1 grid, u1 geoid, u2 prj, u1 gridZone, u2 chIssue, u1 cs
static void decode_mp(const uint8_t* body, int length)
{
    if (length < 45) {
        LOG_WRN("GNSS: [mp] bad length");
        return;
    }
    double n      = read_f8(body + 0);
    double e      = read_f8(body + 8);
    double u      = read_f8(body + 16);
    float  sigma  = read_f4(body + 32);
    uint8_t st    = body[36];

    MutexGuard g{&gnss_lock};
    current_reading.north_m     = n;
    current_reading.east_m      = e;
    current_reading.up_m        = u;
    current_reading.pos_sigma_m = sigma;
    current_reading.sol_type    = st;
}

// [VE] Cartesian Velocity {18}: f4 x, f4 y, f4 z, f4 vSigma, u1 solType, u1 cs
static void decode_VE(const uint8_t* body, int length)
{
    if (length < 18) {
        LOG_WRN("GNSS: [VE] bad length");
        return;
    }
    float vx    = read_f4(body + 0);
    float vy    = read_f4(body + 4);
    float vz    = read_f4(body + 8);
    float vsig  = read_f4(body + 12);
    uint8_t st  = body[16];

    MutexGuard g{&gnss_lock};
    current_reading.vx_ms        = vx;
    current_reading.vy_ms        = vy;
    current_reading.vz_ms        = vz;
    current_reading.vel_sigma_ms = vsig;
    current_reading.sol_type     = st;
}

// [SG] Position and Velocity RMS Errors {18}: f4 hpos, f4 vpos, f4 hvel, f4 vvel, u1 solType, u1 cs
static void decode_SG(const uint8_t* body, int length)
{
    if (length < 18) {
        LOG_WRN("GNSS: [SG] bad length");
        return;
    }

    float hpos  = read_f4(body + 0);
    float vpos  = read_f4(body + 4);
    float hvel  = read_f4(body + 8);
    float vvel  = read_f4(body + 12);
    uint8_t st  = body[16];

    MutexGuard g{&gnss_lock};
    current_reading.hrms_m      = hpos;
    current_reading.vrms_m      = vpos;
    current_reading.hvel_rms_ms = hvel;
    current_reading.vvel_rms_ms = vvel;
    current_reading.sol_type    = st;
}

// Use the correct decoder based on the message ID
static void decode(const char id[2], const uint8_t* body, int body_length)
{
    if (id[0] == '~' && id[1] == '~') { decode_RT(body, body_length); return; }
    if (id[0] == 'S' && id[1] == 'T') { decode_ST(body, body_length); return; }
    if (id[0] == 'm' && id[1] == 'p') { decode_mp(body, body_length); return; }
    if (id[0] == 'V' && id[1] == 'E') { decode_VE(body, body_length); return; }
    if (id[0] == 'S' && id[1] == 'G') { decode_SG(body, body_length); return; }
}

static uint8_t calculate_greis_checksum(const char id[2], const char length_buf[4], const uint8_t* body, int body_length)
{
    uint8_t res = 0;
    
    auto rot_left = [](uint8_t val) -> uint8_t {
        return (uint8_t)((val << 2) | (val >> 6));
    };

    // Process the 2-byte ID
    res = rot_left(res) ^ (uint8_t)id[0];
    res = rot_left(res) ^ (uint8_t)id[1];

    // Process the 3-byte length hex string
    res = rot_left(res) ^ (uint8_t)length_buf[0];
    res = rot_left(res) ^ (uint8_t)length_buf[1];
    res = rot_left(res) ^ (uint8_t)length_buf[2];

    // Process the body (excluding the final checksum byte itself)
    for (int i = 0; i < body_length - 1; i++) {
        res = rot_left(res) ^ body[i];
    }

    // Final rotation before returning
    return rot_left(res);
}

// GREIS frame format:
//   [2-char ASCII ID][3-char ASCII hex length][N bytes binary body][0x0A opt]
//
// States:
//   SYNC    — looking for 2 valid ASCII bytes as message ID
//   LENGTH  — reading 3 ASCII hex chars for body length
//   BODY    — accumulating body bytes

#define MAX_BODY_LENGTH 256  // largest message we expect is [mp] at 45 bytes

static void gnss_thread(void*, void*, void*)
{
    int err = k_sem_take(&gnss_ready, K_FOREVER);
    if (err) {
        LOG_ERR("GNSS: failed to take ready semaphore");
        return;
    }

    LOG_INF("GNSS thread started");

    uint32_t last_logged_bytes = 0;
    int64_t last_log_ms = k_uptime_get();

    // Parser state
    enum { SYNC, LENGTH, BODY } state = SYNC;

    char     id[2]         = {0, 0};
    char     length_buf[4]    = {0, 0, 0, 0};  // 3 chars + null terminator
    uint8_t  body[MAX_BODY_LENGTH];
    int      sync_count    = 0;  // how many valid ID bytes collected
    int      length_count     = 0;  // how many length chars collected
    int      body_length      = 0;  // expected body length
    int      body_count    = 0;  // bytes collected so far

    while (1) {
        k_sem_take(&gnss_data_ready, K_MSEC(1000));
        
        uint8_t byte;
        while (ring_buf_get(&gnss_ringbuf, &byte, 1) > 0) {
            // Skip CR/LF separators between messages regardless of state
            if (byte == 0x0A || byte == 0x0D) {
            continue;
            }

            switch (state) {

            case SYNC:
                // Valid GREIS ID bytes are printable ASCII 0x30–0x7E
                // Exception: [~~] uses 0x7E ('~') which is the upper boundary
                if (byte >= 0x30 && byte <= 0x7E) {
                    if (sync_count == 0) {
                        id[0] = (char)byte;
                        sync_count = 1;
                    } else {
                        id[1] = (char)byte;
                        sync_count = 0;
                        length_count  = 0;
                        state      = LENGTH;
                    }
                } else {
                    // Non-ASCII byte in sync — reset
                    sync_count = 0;
                }
                break;

            case LENGTH:
                // Collect 3 ASCII hex chars
                if ((byte >= '0' && byte <= '9') ||
                    (byte >= 'A' && byte <= 'F') ||
                    (byte >= 'a' && byte <= 'f')) {
                    length_buf[length_count++] = (char)byte;
                    if (length_count == 3) {
                        length_buf[3]  = '\0';
                        body_length    = (int)strtol(length_buf, nullptr, 16);
                        body_count  = 0;
                        length_count   = 0;
                        if (body_length > 0 && body_length <= MAX_BODY_LENGTH) {
                            state = BODY;
                        } else {
                            // Invalid length — probably not a real message start
                            LOG_WRN("GNSS: invalid body length %d for id %c%c",
                                    body_length, id[0], id[1]);
                            state = SYNC;
                        }
                    }
                } else {
                    // Not a hex digit — not a valid length field, resync
                    state      = SYNC;
                    sync_count = 0;
                }
                break;

            case BODY:
                body[body_count++] = byte;
                if (body_count == body_length) {
                    
                    // Calculate the official GREIS checksum
                    uint8_t calc_cs = calculate_greis_checksum(id, length_buf, body, body_length);
                    uint8_t expected_cs = body[body_length - 1];

                    // Validate
                    if (calc_cs != expected_cs) {
                        LOG_WRN("GNSS: [%c%c] bad checksum (calc 0x%02X, got 0x%02X)", 
                                id[0], id[1], calc_cs, expected_cs);
                    } else {
                        // Only decode if the checksum is perfectly valid!
                        decode(id, body, body_length);
                    }
                    
                    state = SYNC;
                }
                break;
            }
        }
        int64_t now = k_uptime_get();
            if (now - last_log_ms >= 5000) {
                LOG_INF("GNSS bytes received: %u", uart_bytes_received);
                last_logged_bytes = uart_bytes_received;
                last_log_ms = now;
            }
    }
}

K_THREAD_DEFINE(gnss_tid, 2048, gnss_thread, nullptr, nullptr, nullptr, 5, 0, 0);

int gnss_init()
{
    gnss_dev = DEVICE_DT_GET(DT_ALIAS(gnss_uart));
    if (!device_is_ready(gnss_dev)) {
        LOG_ERR("GNSS: device not ready");
        return -ENODEV;
    }

    uart_irq_callback_user_data_set(gnss_dev, gnss_uart_cb, NULL);
    uart_irq_rx_enable(gnss_dev);

    k_sem_give(&gnss_ready);
    return 0;
}

GnssReading gnss_read()
{
    MutexGuard g{&gnss_lock};
    return current_reading;
}