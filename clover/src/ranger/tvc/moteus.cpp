#include "moteus.h"

#include <cmath>
#include <cstring>
#include <limits>

namespace moteus {

namespace {

// Subframe type bases (low 2 bits: register count 1-3, or 0 if a varuint count follows).
constexpr uint8_t WRITE_BASE = 0x00;
constexpr uint8_t READ_BASE = 0x10;
constexpr uint8_t REPLY_BASE = 0x20;
constexpr uint8_t WRITE_ERROR = 0x30;
constexpr uint8_t READ_ERROR = 0x31;

constexpr uint8_t type_bits(Type t)
{
    return static_cast<uint8_t>(t) << 2;
}

/// Appends the query every command frame carries:
///   mode (int8), position/velocity/torque (float), home state (int8), voltage/temperature (float), fault (int8).
void append_standard_query(FrameWriter& w)
{
    w.read(Type::INT8, reg::MODE, 1);
    w.read(Type::FLOAT, reg::POSITION, 3);
    w.read(Type::INT8, reg::HOME_STATE, 1);
    w.read(Type::FLOAT, reg::VOLTAGE, 2);
    w.read(Type::INT8, reg::FAULT, 1);
}

size_t emit(FrameWriter& w, uint8_t* out)
{
    const size_t n = w.finish();
    if (n == 0) {
        return 0;
    }
    std::memcpy(out, w.data(), n);
    return n;
}

/// Integer scale factors (int8, int16, int32) per register, from the "Mappings" section of registers.md.
struct Scale {
    float int8;
    float int16;
    float int32;
};
constexpr Scale RAW = {1.0f, 1.0f, 1.0f};
constexpr Scale POSITION_SCALE = {0.01f, 0.0001f, 0.00001f};
constexpr Scale VELOCITY_SCALE = {0.1f, 0.00025f, 0.00001f};
constexpr Scale TORQUE_SCALE = {0.5f, 0.01f, 0.001f};
constexpr Scale VOLTAGE_SCALE = {0.5f, 0.1f, 0.001f};
constexpr Scale TEMPERATURE_SCALE = {1.0f, 0.1f, 0.001f};

Scale scale_for(uint16_t r)
{
    switch (r) {
    case reg::POSITION:
        return POSITION_SCALE;
    case reg::VELOCITY:
        return VELOCITY_SCALE;
    case reg::TORQUE:
        return TORQUE_SCALE;
    case reg::VOLTAGE:
        return VOLTAGE_SCALE;
    case reg::TEMPERATURE:
        return TEMPERATURE_SCALE;
    default:
        return RAW;
    }
}

class Reader {
public:
    Reader(const uint8_t* data, size_t len) : data_(data), len_(len)
    {
    }

    bool done() const
    {
        return pos_ >= len_;
    }

    bool u8(uint8_t& out)
    {
        if (pos_ >= len_) {
            return false;
        }
        out = data_[pos_++];
        return true;
    }

    bool varuint(uint32_t& out)
    {
        out = 0;
        for (int i = 0; i < 5; i++) {
            uint8_t b;
            if (!u8(b)) {
                return false;
            }
            out |= static_cast<uint32_t>(b & 0x7f) << (7 * i);
            if (!(b & 0x80)) {
                return true;
            }
        }
        return false;
    }

    /// Reads one value of `type` and converts it to float using `scale`.
    bool value(Type type, const Scale& scale, float& out)
    {
        switch (type) {
        case Type::INT8: {
            int8_t v;
            if (!bytes(&v, sizeof(v))) {
                return false;
            }
            out = (v == std::numeric_limits<int8_t>::min()) ? NAN : v * scale.int8;
            return true;
        }
        case Type::INT16: {
            int16_t v;
            if (!bytes(&v, sizeof(v))) {
                return false;
            }
            out = (v == std::numeric_limits<int16_t>::min()) ? NAN : v * scale.int16;
            return true;
        }
        case Type::INT32: {
            int32_t v;
            if (!bytes(&v, sizeof(v))) {
                return false;
            }
            out = (v == std::numeric_limits<int32_t>::min()) ? NAN : v * scale.int32;
            return true;
        }
        case Type::FLOAT:
            return bytes(&out, sizeof(out));
        }
        return false;
    }

private:
    // moteus is little-endian, as are both of our targets (Cortex-M7, x86 native_sim).
    bool bytes(void* dst, size_t n)
    {
        if (pos_ > len_ || len_ - pos_ < n) {
            return false;
        }
        std::memcpy(dst, data_ + pos_, n);
        pos_ += n;
        return true;
    }

    const uint8_t* data_;
    size_t len_;
    size_t pos_ = 0;
};

int8_t to_int8(float v)
{
    return std::isnan(v) ? -1 : static_cast<int8_t>(v);
}

void store(QueryResult& out, uint16_t r, float v)
{
    switch (r) {
    case reg::MODE:
        out.mode = to_int8(v);
        break;
    case reg::POSITION:
        out.position_rev = v;
        break;
    case reg::VELOCITY:
        out.velocity_rev_s = v;
        break;
    case reg::TORQUE:
        out.torque_nm = v;
        break;
    case reg::HOME_STATE:
        out.home_state = to_int8(v);
        break;
    case reg::VOLTAGE:
        out.voltage_v = v;
        break;
    case reg::TEMPERATURE:
        out.temperature_c = v;
        break;
    case reg::FAULT:
        out.fault = to_int8(v);
        break;
    default:
        return;  // Not a register we track.
    }
    out.present |= 1u << r;
}

}  // namespace

uint32_t arbitration_id(uint16_t prefix, uint8_t source, uint8_t destination, bool reply_required)
{
    return (static_cast<uint32_t>(prefix & 0x1fff) << 16) | (static_cast<uint32_t>((source & 0x7f) | (reply_required ? 0x80 : 0x00)) << 8) |
        (destination & 0x7f);
}

size_t fd_padded_size(size_t n)
{
    static constexpr size_t SIZES[] = {12, 16, 20, 24, 32, 48, 64};
    if (n <= 8) {
        return n;
    }
    for (size_t s : SIZES) {
        if (n <= s) {
            return s;
        }
    }
    return 0;
}

void FrameWriter::put(uint8_t byte)
{
    if (len_ >= MAX_FRAME_BYTES) {
        overflow_ = true;
        return;
    }
    buf_[len_++] = byte;
}

void FrameWriter::put_varuint(uint32_t value)
{
    do {
        uint8_t b = value & 0x7f;
        value >>= 7;
        put(value ? (b | 0x80) : b);
    } while (value);
}

void FrameWriter::put_bytes(const void* src, size_t n)
{
    const auto* p = static_cast<const uint8_t*>(src);
    for (size_t i = 0; i < n; i++) {
        put(p[i]);
    }
}

void FrameWriter::subframe_header(uint8_t base, size_t count, uint16_t start_reg)
{
    if (count >= 1 && count <= 3) {
        put(base | static_cast<uint8_t>(count));
    }
    else {
        put(base);
        put_varuint(static_cast<uint32_t>(count));
    }
    put_varuint(start_reg);
}

void FrameWriter::write_int8(uint16_t start_reg, const int8_t* values, size_t count)
{
    subframe_header(WRITE_BASE | type_bits(Type::INT8), count, start_reg);
    put_bytes(values, count * sizeof(int8_t));
}

void FrameWriter::write_int16(uint16_t start_reg, const int16_t* values, size_t count)
{
    subframe_header(WRITE_BASE | type_bits(Type::INT16), count, start_reg);
    put_bytes(values, count * sizeof(int16_t));
}

void FrameWriter::write_float(uint16_t start_reg, const float* values, size_t count)
{
    subframe_header(WRITE_BASE | type_bits(Type::FLOAT), count, start_reg);
    put_bytes(values, count * sizeof(float));
}

void FrameWriter::read(Type type, uint16_t start_reg, size_t count)
{
    subframe_header(READ_BASE | type_bits(type), count, start_reg);
}

size_t FrameWriter::finish()
{
    if (overflow_) {
        return 0;
    }
    const size_t padded = fd_padded_size(len_);
    while (len_ < padded) {
        buf_[len_++] = NOP;
    }
    return len_;
}

size_t make_position_frame(const PositionCommand& cmd, uint8_t* out)
{
    FrameWriter w;
    // Mode must be the first register written in a command frame.
    const int8_t mode = static_cast<int8_t>(Mode::POSITION);
    w.write_int8(reg::MODE, &mode, 1);

    const float setpoint[] = {cmd.position_rev, cmd.velocity_rev_s};
    w.write_float(reg::COMMAND_POSITION, setpoint, 2);

    w.write_float(reg::COMMAND_MAX_TORQUE, &cmd.max_torque_nm, 1);

    // 0x026 (stop position) is skipped: deprecated, and moteus faults (code 45) if it is combined with limits.
    const float limits[] = {cmd.watchdog_timeout_s, cmd.velocity_limit_rev_s, cmd.accel_limit_rev_s2};
    w.write_float(reg::COMMAND_WATCHDOG_TIMEOUT, limits, 3);

    append_standard_query(w);
    return emit(w, out);
}

size_t make_stop_frame(uint8_t* out)
{
    FrameWriter w;
    const int8_t mode = static_cast<int8_t>(Mode::STOPPED);
    w.write_int8(reg::MODE, &mode, 1);
    append_standard_query(w);
    return emit(w, out);
}

size_t make_query_frame(uint8_t* out)
{
    FrameWriter w;
    append_standard_query(w);
    return emit(w, out);
}

size_t make_set_output_exact_frame(float position_rev, uint8_t* out)
{
    FrameWriter w;
    w.write_float(reg::SET_OUTPUT_EXACT, &position_rev, 1);
    append_standard_query(w);
    return emit(w, out);
}

bool QueryResult::complete() const
{
    constexpr uint32_t REQUIRED = (1u << reg::MODE) | (1u << reg::POSITION) | (1u << reg::VELOCITY) | (1u << reg::TORQUE) | (1u << reg::HOME_STATE) |
        (1u << reg::VOLTAGE) | (1u << reg::TEMPERATURE) | (1u << reg::FAULT);
    return (present & REQUIRED) == REQUIRED;
}

bool parse_reply(const uint8_t* data, size_t len, QueryResult& out)
{
    Reader r{data, len};
    while (!r.done()) {
        uint8_t header;
        r.u8(header);

        if (header == NOP) {
            continue;
        }

        if (header == WRITE_ERROR || header == READ_ERROR) {
            uint32_t err_reg, err_code;
            if (!r.varuint(err_reg) || !r.varuint(err_code)) {
                return false;
            }
            out.register_error = true;
            continue;
        }

        if ((header & 0xf0) != REPLY_BASE) {
            return false;
        }

        const Type type = static_cast<Type>((header >> 2) & 0x03);
        uint32_t count = header & 0x03;
        if (count == 0 && !r.varuint(count)) {
            return false;
        }
        uint32_t start;
        if (!r.varuint(start)) {
            return false;
        }

        for (uint32_t i = 0; i < count; i++) {
            const uint16_t reg_num = static_cast<uint16_t>(start + i);
            float v;
            if (!r.value(type, scale_for(reg_num), v)) {
                return false;
            }
            store(out, reg_num, v);
        }
    }
    return true;
}

}  // namespace moteus
