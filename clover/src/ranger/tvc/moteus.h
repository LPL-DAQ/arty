#pragma once

// Minimal moteus register ("multiplex") protocol over CAN-FD: frame encoding and reply decoding only. Pure and
// allocation-free (fixed 64-byte buffers), no Zephyr dependencies, so it is unit tested on the host. The CAN transport
// lives in moteus_can.h.
//
// Reference: mjbots/moteus docs/protocol/can.md and docs/protocol/registers.md.
//
// Encoding choice: every float register we write or read is sent as IEEE float32 rather than a scaled int16. int16
// position only spans +/-3.2767 rev at 0.0001 rev, which a lead-screw actuator can exceed over +/-12 deg depending on
// the (still unknown) screw lead, and float keeps full precision with no saturation edge cases. The cost is a few extra
// bytes: the position+query frame is 43 bytes (padded to 48), well within budget at 1 Mbps. Mode, home state and fault
// code are int8 since they are small enumerations.

#include <cstddef>
#include <cstdint>

namespace moteus {

constexpr size_t MAX_FRAME_BYTES = 64;

/// Subframe NOP; also the required padding byte.
constexpr uint8_t NOP = 0x50;

namespace reg {
constexpr uint16_t MODE = 0x000;
constexpr uint16_t POSITION = 0x001;
constexpr uint16_t VELOCITY = 0x002;
constexpr uint16_t TORQUE = 0x003;
constexpr uint16_t HOME_STATE = 0x00c;
constexpr uint16_t VOLTAGE = 0x00d;
constexpr uint16_t TEMPERATURE = 0x00e;
constexpr uint16_t FAULT = 0x00f;
constexpr uint16_t COMMAND_POSITION = 0x020;
constexpr uint16_t COMMAND_VELOCITY = 0x021;
constexpr uint16_t COMMAND_MAX_TORQUE = 0x025;
constexpr uint16_t COMMAND_WATCHDOG_TIMEOUT = 0x027;
constexpr uint16_t COMMAND_VELOCITY_LIMIT = 0x028;
constexpr uint16_t COMMAND_ACCEL_LIMIT = 0x029;
constexpr uint16_t SET_OUTPUT_EXACT = 0x131;
}  // namespace reg

/// Register 0x000 values (subset).
enum class Mode : int8_t {
    STOPPED = 0,
    FAULT = 1,
    POSITION = 10,
    TIMEOUT = 11,
};

/// Register 0x00c values.
enum class HomeState : int8_t {
    RELATIVE = 0,  // Not referenced to anything.
    ROTOR = 1,     // Referenced to the rotor.
    OUTPUT = 2,    // Referenced to the output (output encoder, or set output nearest/exact).
};

/// Data type of a register access; also the 2-bit type field in subframe headers.
enum class Type : uint8_t {
    INT8 = 0,
    INT16 = 1,
    INT32 = 2,
    FLOAT = 3,
};

/// 29-bit extended arbitration ID: prefix in bits 16-28, source in bits 8-14, bit 15 requests a reply, destination in
/// bits 0-6.
uint32_t arbitration_id(uint16_t prefix, uint8_t source, uint8_t destination, bool reply_required);

/// Smallest valid CAN-FD payload size (0-8, 12, 16, 20, 24, 32, 48, 64) that holds n bytes. Returns 0 if n > 64.
size_t fd_padded_size(size_t n);

/// Builds the payload of one CAN-FD frame from multiplex subframes. Fixed buffer; writes past 64 bytes set overflow()
/// and are dropped.
class FrameWriter {
public:
    void write_int8(uint16_t start_reg, const int8_t* values, size_t count);
    void write_int16(uint16_t start_reg, const int16_t* values, size_t count);
    void write_float(uint16_t start_reg, const float* values, size_t count);
    void read(Type type, uint16_t start_reg, size_t count);

    /// Pads with NOP to a valid CAN-FD size. Returns the padded size (0 on overflow).
    size_t finish();

    const uint8_t* data() const
    {
        return buf_;
    }
    size_t size() const
    {
        return len_;
    }
    bool overflow() const
    {
        return overflow_;
    }

private:
    void subframe_header(uint8_t base, size_t count, uint16_t start_reg);
    void put(uint8_t byte);
    void put_varuint(uint32_t value);
    void put_bytes(const void* src, size_t n);

    uint8_t buf_[MAX_FRAME_BYTES] = {};
    size_t len_ = 0;
    bool overflow_ = false;
};

/// Position-mode command. NaN in max_torque_nm means "use moteus' configured limit".
struct PositionCommand {
    float position_rev = 0.0f;
    float velocity_rev_s = 0.0f;
    float max_torque_nm = 0.0f;
    float watchdog_timeout_s = 0.0f;
    float velocity_limit_rev_s = 0.0f;
    float accel_limit_rev_s2 = 0.0f;
};

/// Frame builders. Each one also appends the standard telemetry query (see QueryResult), so every frame we send gets a
/// full status reply. Return the padded payload size written to `out` (which must hold MAX_FRAME_BYTES).
size_t make_position_frame(const PositionCommand& cmd, uint8_t* out);
size_t make_stop_frame(uint8_t* out);
size_t make_query_frame(uint8_t* out);
size_t make_set_output_exact_frame(float position_rev, uint8_t* out);

/// Decoded reply to the standard query. Fields not present in the reply are left at their defaults; `present` has bit
/// (1 << register) set for every register 0x000-0x01f that was decoded.
struct QueryResult {
    int8_t mode = -1;
    float position_rev = 0.0f;
    float velocity_rev_s = 0.0f;
    float torque_nm = 0.0f;
    int8_t home_state = -1;
    float voltage_v = 0.0f;
    float temperature_c = 0.0f;
    int8_t fault = 0;
    uint32_t present = 0;
    /// Set if the reply contained a read/write error subframe (0x30/0x31).
    bool register_error = false;

    bool has(uint16_t r) const
    {
        return r < 32 && (present & (1u << r));
    }
    /// True if every register in the standard query was decoded.
    bool complete() const;
};

/// Parses a reply payload. Returns false if the payload is malformed (unknown subframe, truncated data). Unknown
/// registers are skipped. Integer encodings are scaled per the moteus register mappings; the maximally negative integer
/// decodes to NaN.
bool parse_reply(const uint8_t* data, size_t len, QueryResult& out);

}  // namespace moteus
