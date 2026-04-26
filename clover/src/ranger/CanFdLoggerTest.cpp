#include "CanFdLoggerTest.h"

#include <cmath>
#include <cstdio>
#include <cstring>

#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include "moteus/moteus_protocol.h"

namespace CanFdLoggerTest {
namespace {

// Depth of the receive message queue.  Each can_frame is ~72 bytes.
static constexpr int kQueueDepth = 8;

// Stack and thread for the receive loop.
static K_THREAD_STACK_DEFINE(s_stack, 2048);
static struct k_thread s_thread;

// Message queue — populated by the CAN driver ISR.
alignas(4) static struct can_frame s_q_buf[kQueueDepth];
static struct k_msgq s_queue;

// Persistent RX filter handle so we can remove it if needed.
static int s_filter_id = -1;

static const struct device *s_can_dev =
    DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus));

// ---------------------------------------------------------------------------
// Moteus reply detection
//
// A moteus reply has an extended arbitration_id of the form:
//   bits[6:0]   = dest id  (host = 0)
//   bits[14:8]  = source id (motor id, 1–127)
//   bit[15]     = reply_required — 0 in replies (set by host in commands)
//
// Motor replies: bit 15 clear, source in [1,127], dest == 0.
// ---------------------------------------------------------------------------
static bool is_moteus_reply(uint32_t arb_id) {
    const bool reply_required = (arb_id & 0x00008000u) != 0;
    const uint8_t source_id   = (arb_id >> 8) & 0x7Fu;
    const uint8_t dest_id     =  arb_id        & 0x7Fu;
    return !reply_required && (source_id >= 1) && (source_id <= 127) && (dest_id == 0);
}

// ---------------------------------------------------------------------------
// Log one raw frame (hex dump)
// ---------------------------------------------------------------------------
static void log_raw(const struct can_frame &f) {
    const bool is_fd  = (f.flags & CAN_FRAME_FDF) != 0;
    const bool is_brs = (f.flags & CAN_FRAME_BRS) != 0;
    const uint8_t len = can_dlc_to_bytes(f.dlc);

    printk("[CAN%s%s] id=0x%08X len=%u  ",
           is_fd  ? "-FD" : "   ",
           is_brs ? "+BRS" : "    ",
           (unsigned)f.id, len);

    for (uint8_t i = 0; i < len; i++) {
        printk("%02X ", f.data[i]);
    }
    printk("\n");
}

// ---------------------------------------------------------------------------
// Moteus command detection
//
// Command frames (host→motor): bit 15 set (reply_required), source=0 (host),
// dest in [1,127] (motor id).
// ---------------------------------------------------------------------------
static bool is_moteus_command(uint32_t arb_id) {
    const bool reply_required = (arb_id & 0x00008000u) != 0;
    const uint8_t dest_id     =  arb_id        & 0x7Fu;
    const uint8_t source_id   = (arb_id >> 8)  & 0x7Fu;
    return reply_required && (dest_id >= 1) && (dest_id <= 127) && (source_id == 0);
}

// ---------------------------------------------------------------------------
// Moteus command frame decoder
//
// The moteus multiplex wire format for writes (kWrite opcodes 0x00–0x0F):
//   cmd byte: bits[3:2] = resolution (0=i8,1=i16,2=i32,3=f32), bits[1:0] = count (0→next byte)
//   followed by varuint register address, then count values
// Read requests (kRead opcodes 0x10–0x1F) use the same layout but carry no values —
// they tell the motor which registers to include in its reply.
// ---------------------------------------------------------------------------

static uint16_t read_varuint_p(const uint8_t **p, const uint8_t *end) {
    uint16_t result = 0, shift = 0;
    while (*p < end) {
        uint8_t b = *(*p)++;
        result |= (uint16_t)(b & 0x7fu) << shift;
        shift += 7;
        if (!(b & 0x80u)) break;
    }
    return result;
}

static uint8_t res_size(uint8_t res) {
    return (res == 0) ? 1u : (res <= 2) ? (uint8_t)(1u << res) : 4u;
}

// Read one value according to resolution; scale factor applied; returns milli-units as int.
// s8/s16/s32 are the per-resolution scale factors from the moteus register map.
static int32_t read_milli(const uint8_t **p, uint8_t res,
                           float s8, float s16, float s32) {
    switch (res) {
        case 0: { int8_t  x; memcpy(&x, *p, 1); *p += 1;
                  return (x == -128)        ? INT32_MIN : (int32_t)((float)x * s8  * 1000.f); }
        case 1: { int16_t x; memcpy(&x, *p, 2); *p += 2;
                  return (x == -32768)      ? INT32_MIN : (int32_t)((float)x * s16 * 1000.f); }
        case 2: { int32_t x; memcpy(&x, *p, 4); *p += 4;
                  return (x == INT32_MIN)   ? INT32_MIN : (int32_t)((float)x * s32 * 1000.f); }
        case 3: { float   f; memcpy(&f, *p, 4); *p += 4;
                  return __builtin_isnan(f) ? INT32_MIN : (int32_t)(f * 1000.f); }
    }
    return INT32_MIN;
}

static void log_moteus_command(uint32_t arb_id, const uint8_t *data, uint8_t len) {
    const uint8_t dest = arb_id & 0x7Fu;
    printk("  [cmd->motor%u]\n", dest);

    const uint8_t *p   = data;
    const uint8_t *end = data + len;

    while (p < end) {
        const uint8_t cmd = *p++;
        if (cmd == 0x50u) continue;  // NOP

        const bool is_write = (cmd < 0x10u);
        const bool is_read  = (cmd >= 0x10u && cmd < 0x20u);
        if (!is_write && !is_read) break;  // unknown opcode

        const uint8_t base  = is_write ? 0x00u : 0x10u;
        const uint8_t res   = (cmd & 0x0Cu) >> 2;
        uint8_t       count = cmd & 0x03u;
        if (count == 0 && p < end) count = *p++;
        const uint16_t start_reg = read_varuint_p(&p, end);

        if (is_read) {
            // Read requests name the registers the motor should echo back.
            printk("    query %u reg(s) from 0x%03X\n", count, start_reg);
            // No payload bytes — skip.
            continue;
        }

        // Write: decode each register value.
        for (uint8_t i = 0; i < count && p + res_size(res) <= end; i++) {
            const uint16_t reg = start_reg + i;
            switch (reg) {
            case 0x000: {  // mode (always written as int8, scale=1)
                int32_t m = read_milli(&p, res, 1.f, 1.f, 1.f) / 1000;
                const char *name = (m == 10) ? "Position" :
                                   (m ==  0) ? "Stopped"  :
                                   (m ==  5) ? "PWM"      :
                                   (m == 12) ? "ZeroVel"  : "?";
                printk("    mode=%d (%s)\n", (int)m, name);
                break;
            }
            case 0x020: {  // kCommandPosition (rev)
                int32_t v = read_milli(&p, res, 0.01f, 0.0001f, 0.00001f);
                if (v == INT32_MIN) printk("    pos=NaN\n");
                else                printk("    pos=%d mrev\n", (int)v);
                break;
            }
            case 0x021: {  // kCommandVelocity (rev/s)
                int32_t v = read_milli(&p, res, 0.1f, 0.00025f, 0.00001f);
                if (v == INT32_MIN) printk("    vel=NaN\n");
                else                printk("    vel=%d mrps\n", (int)v);
                break;
            }
            case 0x022: {  // kCommandFeedforwardTorque (Nm)
                int32_t v = read_milli(&p, res, 0.5f, 0.01f, 0.001f);
                if (v == INT32_MIN) printk("    ff_torque=NaN\n");
                else                printk("    ff_torque=%d mNm\n", (int)v);
                break;
            }
            case 0x023:  // kCommandKpScale
            case 0x024: {// kCommandKdScale
                int32_t v = read_milli(&p, res, 1.f/127.f, 1.f/32767.f, 1.f/2147483647.f);
                printk("    %s=%d m%%\n", (reg == 0x023) ? "kp_scale" : "kd_scale", (int)v);
                break;
            }
            case 0x025: {  // kCommandPositionMaxTorque (Nm)
                int32_t v = read_milli(&p, res, 0.5f, 0.01f, 0.001f);
                printk("    max_torque=%d mNm\n", (int)v);
                break;
            }
            case 0x026: {  // kCommandStopPosition (rev)
                int32_t v = read_milli(&p, res, 0.01f, 0.0001f, 0.00001f);
                if (v == INT32_MIN) printk("    stop_pos=NaN\n");
                else                printk("    stop_pos=%d mrev\n", (int)v);
                break;
            }
            case 0x027: {  // kCommandTimeout (s)
                int32_t v = read_milli(&p, res, 0.01f, 0.001f, 0.000001f);
                if (v == INT32_MIN) printk("    timeout=NaN\n");
                else                printk("    timeout=%d ms\n", (int)v);
                break;
            }
            case 0x028: {  // kCommandVelocityLimit (rev/s)
                int32_t v = read_milli(&p, res, 0.1f, 0.00025f, 0.00001f);
                if (v == INT32_MIN) printk("    vel_limit=NaN\n");
                else                printk("    vel_limit=%d mrps\n", (int)v);
                break;
            }
            case 0x029: {  // kCommandAccelLimit (rev/s²)
                int32_t v = read_milli(&p, res, 0.05f, 0.001f, 0.00001f);
                if (v == INT32_MIN) printk("    accel_limit=NaN\n");
                else                printk("    accel_limit=%d mrev/s2\n", (int)v);
                break;
            }
            default:
                p += res_size(res);
                printk("    reg=0x%03X (unknown)\n", reg);
                break;
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Attempt moteus Query parse and log decoded fields
// ---------------------------------------------------------------------------
static void log_moteus(uint32_t arb_id,
                       const uint8_t *data, uint8_t len) {
    namespace mm = mjbots::moteus;

    const uint8_t motor_id  = (arb_id >> 8) & 0x7Fu;
    const uint8_t source_id =  arb_id        & 0x7Fu;

    mm::Query::Result r = mm::Query::Parse(data, len);

    printk("  [moteus] motor=%u src=%u  mode=%d fault=%d\n",
           motor_id, source_id,
           static_cast<int>(r.mode),
           r.fault);

    if (!std::isnan(r.position)) {
        printk("           pos=%.4f rev  vel=%.4f rev/s  torque=%.4f Nm\n",
               (double)r.position,
               (double)r.velocity,
               (double)r.torque);
    }
    if (!std::isnan(r.voltage)) {
        printk("           Vbus=%.2f V  temp=%.1f C\n",
               (double)r.voltage,
               (double)r.temperature);
    }
    if (!std::isnan(r.q_current)) {
        printk("           Iq=%.3f A  Id=%.3f A\n",
               (double)r.q_current,
               (double)r.d_current);
    }
}

// ---------------------------------------------------------------------------
// Receive thread body
// ---------------------------------------------------------------------------
static void recv_thread(void *, void *, void *) {
    printk("[CanFdLoggerTest] waiting for CAN-FD frames...\n");

    while (true) {
        struct can_frame f = {};
        // Block until a frame arrives (no timeout = wait forever).
        int rc = k_msgq_get(&s_queue, &f, K_FOREVER);
        if (rc != 0) continue;

        log_raw(f);

        const uint32_t arb_id = f.id & CAN_EXT_ID_MASK;
        const uint8_t  len    = can_dlc_to_bytes(f.dlc);

        if (is_moteus_reply(arb_id)) {
            log_moteus(arb_id, f.data, len);
        } else if (is_moteus_command(arb_id)) {
            log_moteus_command(arb_id, f.data, len);
        }
    }
}

}  // namespace

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void init() {
    if (!device_is_ready(s_can_dev)) {
        printk("[CanFdLoggerTest] ERROR: CAN device not ready\n");
        return;
    }

    can_stop(s_can_dev);
    can_set_mode(s_can_dev, CAN_MODE_FD);

    int ret = can_start(s_can_dev);
    if (ret != 0) {
        printk("[CanFdLoggerTest] ERROR: can_start failed (%d)\n", ret);
        return;
    }

    // Catch-all extended-ID filter (mask = 0 matches every ID).
    k_msgq_init(&s_queue,
                reinterpret_cast<char *>(s_q_buf),
                sizeof(struct can_frame),
                kQueueDepth);

    struct can_filter f = {};
    f.flags = CAN_FILTER_IDE;
    f.id    = 0;
    f.mask  = 0;

    s_filter_id = can_add_rx_filter_msgq(s_can_dev, &s_queue, &f);
    if (s_filter_id < 0) {
        printk("[CanFdLoggerTest] ERROR: can_add_rx_filter_msgq failed (%d)\n",
               s_filter_id);
        return;
    }

    printk("[CanFdLoggerTest] init OK (filter_id=%d)\n", s_filter_id);
}

void run() {
    k_thread_create(&s_thread, s_stack, K_THREAD_STACK_SIZEOF(s_stack),
                    recv_thread, nullptr, nullptr, nullptr,
                    K_PRIO_COOP(7), 0, K_NO_WAIT);
    k_thread_name_set(&s_thread, "can_fd_log");

    // Block the calling thread — call site can join or sleep forever.
    k_thread_join(&s_thread, K_FOREVER);
}

}  // namespace CanFdLoggerTest
