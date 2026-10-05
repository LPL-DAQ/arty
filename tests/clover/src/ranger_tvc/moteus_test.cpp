#include "ranger/tvc/moteus.h"

#include <cmath>
#include <cstring>
#include <zephyr/ztest.h>

using namespace moteus;

namespace {

size_t from_hex(const char* hex, uint8_t* out)
{
    size_t n = 0;
    for (; hex[0] && hex[1]; hex += 2) {
        auto nib = [](char c) -> uint8_t { return (c <= '9') ? c - '0' : (c | 0x20) - 'a' + 10; };
        out[n++] = static_cast<uint8_t>((nib(hex[0]) << 4) | nib(hex[1]));
    }
    return n;
}

void assert_bytes(const uint8_t* got, size_t got_len, const char* expected_hex)
{
    uint8_t expected[MAX_FRAME_BYTES];
    const size_t n = from_hex(expected_hex, expected);
    zassert_equal(got_len, n, "length %zu, expected %zu", got_len, n);
    for (size_t i = 0; i < n; i++) {
        zassert_equal(got[i], expected[i], "byte %zu: 0x%02x, expected 0x%02x", i, got[i], expected[i]);
    }
}

}  // namespace

ZTEST(Moteus_tests, test_arbitration_id)
{
    // Examples from can.md: 0x8001 = source 0 -> destination 1, reply requested. 0x100 = source 1 -> destination 0.
    zassert_equal(arbitration_id(0, 0, 1, true), 0x8001u);
    zassert_equal(arbitration_id(0, 1, 0, false), 0x0100u);
    zassert_equal(arbitration_id(0, 0, 2, true), 0x8002u);
    zassert_equal(arbitration_id(0x1234, 0, 1, true), 0x12348001u);
    // Prefix is 13 bits.
    zassert_equal(arbitration_id(0xffff, 0, 1, false), 0x1fff0001u);
}

ZTEST(Moteus_tests, test_fd_padded_size)
{
    zassert_equal(fd_padded_size(0), 0u);
    zassert_equal(fd_padded_size(8), 8u);
    zassert_equal(fd_padded_size(9), 12u);
    zassert_equal(fd_padded_size(13), 16u);
    zassert_equal(fd_padded_size(21), 24u);
    zassert_equal(fd_padded_size(25), 32u);
    zassert_equal(fd_padded_size(33), 48u);
    zassert_equal(fd_padded_size(49), 64u);
    zassert_equal(fd_padded_size(64), 64u);
    zassert_equal(fd_padded_size(65), 0u);
}

ZTEST(Moteus_tests, test_frame_writer_reproduces_can_md_example)
{
    // can.md "Example": mode=position, 3x int16 at 0x020, read 4x int16 from 0x000, read 3x int8 from 0x00d.
    FrameWriter w;
    const int8_t mode = 10;
    w.write_int8(reg::MODE, &mode, 1);
    const int16_t cmd[] = {0x0060, 0x0120, static_cast<int16_t>(0xff50)};
    w.write_int16(reg::COMMAND_POSITION, cmd, 3);
    w.read(Type::INT16, reg::MODE, 4);
    w.read(Type::INT8, reg::VOLTAGE, 3);
    zassert_false(w.overflow());
    assert_bytes(w.data(), w.size(), "01000a07206000200150ff140400130d");
}

ZTEST(Moteus_tests, test_parse_can_md_example_reply)
{
    // can.md reply: mode 10, position 80 (0.008 rev), velocity 256 (0.064 Hz), torque -144 (-1.44 Nm),
    // voltage 24 (12 V), temperature 20 C, fault 0.
    uint8_t data[MAX_FRAME_BYTES];
    const size_t n = from_hex("2404000a005000000170ff230d181400", data);
    QueryResult q;
    zassert_true(parse_reply(data, n, q));
    zassert_equal(q.mode, 10);
    zassert_within(q.position_rev, 0.008f, 1e-6f);
    zassert_within(q.velocity_rev_s, 0.064f, 1e-6f);
    zassert_within(q.torque_nm, -1.44f, 1e-5f);
    zassert_within(q.voltage_v, 12.0f, 1e-6f);
    zassert_within(q.temperature_c, 20.0f, 1e-6f);
    zassert_equal(q.fault, 0);
    zassert_false(q.has(reg::HOME_STATE));
    zassert_false(q.complete());
    zassert_false(q.register_error);
}

ZTEST(Moteus_tests, test_position_frame_bytes)
{
    // Command portion verified byte-for-byte against mjbots' moteus_protocol.h PositionMode::Make with the same
    // float format; query and NOP padding appended per moteus.h.
    PositionCommand cmd{
        .position_rev = 1.25f,
        .velocity_rev_s = 0.0f,
        .max_torque_nm = 0.5f,
        .watchdog_timeout_s = 0.1f,
        .velocity_limit_rev_s = 2.0f,
        .accel_limit_rev_s2 = 10.0f,
    };
    uint8_t out[MAX_FRAME_BYTES];
    const size_t n = make_position_frame(cmd, out);
    assert_bytes(
        out,
        n,
        "01000a"                        // write 1x int8 @0x000: mode 10 (position)
        "0e20" "0000a03f" "00000000"    // write 2x float @0x020: position 1.25, velocity 0
        "0d25" "0000003f"               // write 1x float @0x025: max torque 0.5
        "0f27" "cdcccc3d" "00000040" "00002041"  // write 3x float @0x027: watchdog 0.1, vel limit 2, accel limit 10
        "1100"                          // read 1x int8 @0x000: mode
        "1f01"                          // read 3x float @0x001: position, velocity, torque
        "110c"                          // read 1x int8 @0x00c: home state
        "1e0d"                          // read 2x float @0x00d: voltage, temperature
        "110f"                          // read 1x int8 @0x00f: fault
        "5050505050");                  // NOP padding 43 -> 48
}

ZTEST(Moteus_tests, test_position_frame_nan_max_torque)
{
    PositionCommand cmd{.max_torque_nm = NAN};
    uint8_t out[MAX_FRAME_BYTES];
    zassert_equal(make_position_frame(cmd, out), 48u);
    float torque;
    std::memcpy(&torque, out + 15, sizeof(torque));
    zassert_true(std::isnan(torque));
}

ZTEST(Moteus_tests, test_stop_frame_bytes)
{
    uint8_t out[MAX_FRAME_BYTES];
    const size_t n = make_stop_frame(out);
    assert_bytes(out, n, "010000" "11001f01110c1e0d110f" "505050");
}

ZTEST(Moteus_tests, test_query_frame_bytes)
{
    uint8_t out[MAX_FRAME_BYTES];
    const size_t n = make_query_frame(out);
    assert_bytes(out, n, "11001f01110c1e0d110f" "5050");
}

ZTEST(Moteus_tests, test_set_output_exact_frame_bytes)
{
    // Register 0x131 encodes as varuint b1 02.
    uint8_t out[MAX_FRAME_BYTES];
    const size_t n = make_set_output_exact_frame(0.0f, out);
    assert_bytes(out, n, "0db10200000000" "11001f01110c1e0d110f" "505050");
}

ZTEST(Moteus_tests, test_parse_standard_query_reply)
{
    // The reply moteus sends to our standard query: int8 mode, 3x float, int8 home state, 2x float, int8 fault.
    FrameWriter w;  // Reuse the writer to lay out reply bytes; patch the subframe headers from write (0x0_) to reply (0x2_).
    const int8_t mode = 10, home = 2, fault = 0;
    const float pvt[] = {-0.75f, 0.125f, 0.03f};
    const float vt[] = {24.5f, 31.0f};
    w.write_int8(reg::MODE, &mode, 1);
    w.write_float(reg::POSITION, pvt, 3);
    w.write_int8(reg::HOME_STATE, &home, 1);
    w.write_float(reg::VOLTAGE, vt, 2);
    w.write_int8(reg::FAULT, &fault, 1);
    uint8_t data[MAX_FRAME_BYTES];
    std::memcpy(data, w.data(), w.size());
    const size_t headers[] = {0, 3, 17, 20, 30};
    for (size_t h : headers) {
        data[h] |= 0x20;
    }
    const size_t n = fd_padded_size(w.size());
    std::memset(data + w.size(), NOP, n - w.size());

    QueryResult q;
    zassert_true(parse_reply(data, n, q));
    zassert_true(q.complete());
    zassert_equal(q.mode, 10);
    zassert_equal(q.position_rev, -0.75f);
    zassert_equal(q.velocity_rev_s, 0.125f);
    zassert_equal(q.torque_nm, 0.03f);
    zassert_equal(q.home_state, 2);
    zassert_equal(q.voltage_v, 24.5f);
    zassert_equal(q.temperature_c, 31.0f);
    zassert_equal(q.fault, 0);
}

ZTEST(Moteus_tests, test_parse_int32_and_nan)
{
    // int32 position 0x80000000 (max negative) -> NaN; int32 torque 1500 -> 1.5 Nm.
    uint8_t data[] = {0x29, 0x01, 0x00, 0x00, 0x00, 0x80, 0x29, 0x03, 0xdc, 0x05, 0x00, 0x00};
    QueryResult q;
    zassert_true(parse_reply(data, sizeof(data), q));
    zassert_true(std::isnan(q.position_rev));
    zassert_within(q.torque_nm, 1.5f, 1e-6f);
}

ZTEST(Moteus_tests, test_parse_fault_reply)
{
    // mode 1 (fault), fault code 33 (motor driver fault).
    uint8_t data[] = {0x21, 0x00, 0x01, 0x21, 0x0f, 0x21};
    QueryResult q;
    zassert_true(parse_reply(data, sizeof(data), q));
    zassert_equal(q.mode, static_cast<int8_t>(Mode::FAULT));
    zassert_equal(q.fault, 33);
}

ZTEST(Moteus_tests, test_parse_error_subframe)
{
    // Read error on register 0x00c, error code 2, followed by a normal reply.
    uint8_t data[] = {0x31, 0x0c, 0x02, 0x21, 0x00, 0x0a, 0x50, 0x50};
    QueryResult q;
    zassert_true(parse_reply(data, sizeof(data), q));
    zassert_true(q.register_error);
    zassert_equal(q.mode, 10);
}

ZTEST(Moteus_tests, test_parse_rejects_malformed)
{
    QueryResult q;
    const uint8_t truncated[] = {0x2f, 0x01, 0x00, 0x00};  // Says 3 floats, has 2 bytes.
    zassert_false(parse_reply(truncated, sizeof(truncated), q));

    const uint8_t unknown[] = {0x40, 0x00};
    zassert_false(parse_reply(unknown, sizeof(unknown), q));

    const uint8_t bad_varuint[] = {0x20, 0xff, 0xff, 0xff, 0xff, 0xff};
    zassert_false(parse_reply(bad_varuint, sizeof(bad_varuint), q));
}

ZTEST(Moteus_tests, test_frame_writer_overflow)
{
    FrameWriter w;
    float many[20] = {};
    w.write_float(reg::COMMAND_POSITION, many, 20);  // 2 + 1 + 80 bytes
    zassert_true(w.overflow());
    zassert_equal(w.finish(), 0u);
}

ZTEST_SUITE(Moteus_tests, NULL, NULL, NULL, NULL, NULL);
