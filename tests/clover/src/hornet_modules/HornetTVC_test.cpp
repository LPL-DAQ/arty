#include "../../../../clover/src/hornet/HornetTvc.h"
#include <zephyr/ztest.h>
#include <cmath>

// Independent copy of the linkage geometry, evaluated in double, so the firmware's float math is checked against it.
static constexpr double LINK_MM = 68.20;
static constexpr double MOUNT_MM = 128.25;
static constexpr double NEUTRAL_ANGLE_DEG = 69.96;
static constexpr double NEUTRAL_EXTENSION_MM = 25.0;
static constexpr double PULSE_US_PER_MM = 1000.0 / 50.0;

static constexpr float STEP_MM = 18.0f * 0.0015f;
// A full 50 mm stroke takes about 1852 ticks.
static constexpr int SETTLE_TICKS = 2500;

static constexpr HornetTvc::AxisCalibration DEFAULT_CAL{25.0f, 1000, 2000, 1.0f};

static double geometry_extension_mm(double gimbal_deg)
{
    const auto length = [](double deg) {
        return std::sqrt(LINK_MM * LINK_MM + MOUNT_MM * MOUNT_MM - 2.0 * LINK_MM * MOUNT_MM * std::cos((NEUTRAL_ANGLE_DEG + deg) * M_PI / 180.0));
    };
    return NEUTRAL_EXTENSION_MM + length(gimbal_deg) - length(0.0);
}

static uint32_t geometry_pulse_us(double gimbal_deg)
{
    return static_cast<uint32_t>(std::lround(1000.0 + geometry_extension_mm(gimbal_deg) * PULSE_US_PER_MM));
}

struct TickResult {
    uint32_t pitch_pulse_us;
    uint32_t yaw_pulse_us;
    HornetTvcMetrics metrics;
};

static TickResult tick(float pitch_deg, float yaw_deg)
{
    auto result = HornetTvc::tick_gimbal_deg(pitch_deg, yaw_deg);
    zassert_true(result.has_value(), "tick_gimbal_deg should succeed");
    auto [pitch_pulse, yaw_pulse, metrics] = *result;
    return {static_cast<uint32_t>(pitch_pulse), static_cast<uint32_t>(yaw_pulse), metrics};
}

static TickResult settle(float pitch_deg, float yaw_deg)
{
    TickResult last{};
    for (int i = 0; i < SETTLE_TICKS; ++i) {
        last = tick(pitch_deg, yaw_deg);
    }
    return last;
}

static void before_each(void*)
{
    HornetTvc::reset();
}

ZTEST(HornetTvc_tests, test_settled_pulses_match_expected_table)
{
    const struct {
        float deg;
        uint32_t pulse_us;
    } cases[] = {{0.0f, 1500}, {6.0f, 1639}, {-6.0f, 1359}, {12.0f, 1773}, {-12.0f, 1216}};

    for (const auto& c : cases) {
        HornetTvc::reset();
        const auto r = settle(c.deg, c.deg);
        zassert_within(r.pitch_pulse_us, c.pulse_us, 1, "pitch %d deg gave %u us, expected %u", (int)c.deg, r.pitch_pulse_us, c.pulse_us);
        zassert_within(r.yaw_pulse_us, c.pulse_us, 1, "yaw %d deg gave %u us, expected %u", (int)c.deg, r.yaw_pulse_us, c.pulse_us);
    }
}

ZTEST(HornetTvc_tests, test_settled_output_matches_geometry_every_degree)
{
    for (int deg = -12; deg <= 12; ++deg) {
        HornetTvc::reset();
        const auto r = settle(static_cast<float>(deg), static_cast<float>(deg));
        const double expected_mm = geometry_extension_mm(deg);
        zassert_within(static_cast<double>(r.metrics.commanded_pitch_extension_mm), expected_mm, 1e-3, "pitch extension at %d deg", deg);
        zassert_within(static_cast<double>(r.metrics.commanded_yaw_extension_mm), expected_mm, 1e-3, "yaw extension at %d deg", deg);
        zassert_equal(r.pitch_pulse_us, geometry_pulse_us(deg), "pitch pulse at %d deg", deg);
        zassert_equal(r.yaw_pulse_us, geometry_pulse_us(deg), "yaw pulse at %d deg", deg);
    }
}

ZTEST(HornetTvc_tests, test_commands_beyond_limit_settle_to_12_deg)
{
    const uint32_t pos_limit_us = settle(12.0f, 12.0f).pitch_pulse_us;
    HornetTvc::reset();
    const uint32_t neg_limit_us = settle(-12.0f, -12.0f).pitch_pulse_us;

    for (float deg : {45.0f, 120.0f}) {
        HornetTvc::reset();
        const auto pos = settle(deg, deg);
        zassert_equal(pos.pitch_pulse_us, pos_limit_us);
        zassert_equal(pos.yaw_pulse_us, pos_limit_us);
        zassert_within(pos.metrics.commanded_pitch_gimbal_deg, 12.0f, 1e-6f);

        HornetTvc::reset();
        const auto neg = settle(-deg, -deg);
        zassert_equal(neg.pitch_pulse_us, neg_limit_us);
        zassert_equal(neg.yaw_pulse_us, neg_limit_us);
        zassert_within(neg.metrics.commanded_pitch_gimbal_deg, -12.0f, 1e-6f);
    }
}

ZTEST(HornetTvc_tests, test_retract_side_needs_more_travel)
{
    const float up_mm = settle(12.0f, 0.0f).metrics.commanded_pitch_extension_mm - 25.0f;
    HornetTvc::reset();
    const float down_mm = 25.0f - settle(-12.0f, 0.0f).metrics.commanded_pitch_extension_mm;

    zassert_within(up_mm, 13.673f, 1e-3f, "+12 deg extends %f mm", (double)up_mm);
    zassert_within(down_mm, 14.187f, 1e-3f, "-12 deg retracts %f mm", (double)down_mm);
    zassert_true(up_mm < down_mm, "+12 deg travel should be smaller than -12 deg travel");
}

ZTEST(HornetTvc_tests, test_rate_limit_steps_toward_target)
{
    const auto first = tick(12.0f, -12.0f);
    zassert_within(first.metrics.commanded_pitch_extension_mm, 25.0f + STEP_MM, 1e-5f);
    zassert_within(first.metrics.commanded_yaw_extension_mm, 25.0f - STEP_MM, 1e-5f);

    constexpr int n = 200;
    TickResult r = first;
    for (int i = 1; i < n; ++i) {
        r = tick(12.0f, -12.0f);
    }
    zassert_within(r.metrics.commanded_pitch_extension_mm, 25.0f + n * STEP_MM, 1e-3f);
    zassert_within(r.metrics.commanded_yaw_extension_mm, 25.0f - n * STEP_MM, 1e-3f);

    r = settle(12.0f, -12.0f);
    zassert_equal(r.metrics.commanded_pitch_extension_mm, r.metrics.target_pitch_extension_mm, "pitch should stop exactly at target");
    zassert_equal(r.metrics.commanded_yaw_extension_mm, r.metrics.target_yaw_extension_mm, "yaw should stop exactly at target");

    const float held_mm = r.metrics.commanded_pitch_extension_mm;
    r = tick(12.0f, -12.0f);
    zassert_equal(r.metrics.commanded_pitch_extension_mm, held_mm, "settled extension should not move");
}

ZTEST(HornetTvc_tests, test_metrics_match_outputs)
{
    for (const auto& r : {tick(6.0f, -6.0f), settle(6.0f, -6.0f)}) {
        zassert_equal(r.metrics.pitch_pulse_us, r.pitch_pulse_us);
        zassert_equal(r.metrics.yaw_pulse_us, r.yaw_pulse_us);
        zassert_within(r.metrics.commanded_pitch_gimbal_deg, 6.0f, 1e-6f);
        zassert_within(r.metrics.commanded_yaw_gimbal_deg, -6.0f, 1e-6f);
        zassert_within(static_cast<double>(r.metrics.target_pitch_extension_mm), geometry_extension_mm(6.0), 1e-3);
        zassert_within(static_cast<double>(r.metrics.target_yaw_extension_mm), geometry_extension_mm(-6.0), 1e-3);
        zassert_equal(r.pitch_pulse_us, HornetTvc::extension_mm_to_pulse_us(r.metrics.commanded_pitch_extension_mm, DEFAULT_CAL));
        zassert_equal(r.yaw_pulse_us, HornetTvc::extension_mm_to_pulse_us(r.metrics.commanded_yaw_extension_mm, DEFAULT_CAL));
    }
}

ZTEST(HornetTvc_tests, test_reset_returns_to_neutral)
{
    settle(12.0f, -12.0f);
    HornetTvc::reset();
    const auto r = tick(0.0f, 0.0f);
    zassert_equal(r.pitch_pulse_us, 1500);
    zassert_equal(r.yaw_pulse_us, 1500);
    zassert_equal(r.metrics.commanded_pitch_extension_mm, 25.0f);
    zassert_equal(r.metrics.commanded_yaw_extension_mm, 25.0f);
}

ZTEST(HornetTvc_tests, test_command_neutral_returns_to_neutral)
{
    zassert_equal(HornetTvc::neutral_pitch_pulse_us(), 1500);
    zassert_equal(HornetTvc::neutral_yaw_pulse_us(), 1500);

    settle(-12.0f, 12.0f);
    auto [pitch_pulse, yaw_pulse, metrics] = HornetTvc::command_neutral();
    zassert_equal(pitch_pulse, 1500);
    zassert_equal(yaw_pulse, 1500);
    zassert_equal(metrics.commanded_pitch_gimbal_deg, 0.0f);
    zassert_equal(metrics.commanded_yaw_gimbal_deg, 0.0f);
    zassert_equal(metrics.target_pitch_extension_mm, 25.0f);
    zassert_equal(metrics.commanded_yaw_extension_mm, 25.0f);
    zassert_equal(metrics.pitch_pulse_us, 1500);
    zassert_equal(metrics.yaw_pulse_us, 1500);

    const auto r = tick(0.0f, 0.0f);
    zassert_equal(r.pitch_pulse_us, 1500);
    zassert_equal(r.yaw_pulse_us, 1500);
}

ZTEST(HornetTvc_tests, test_pitch_and_yaw_are_independent)
{
    const auto both = settle(12.0f, -6.0f);
    zassert_equal(both.pitch_pulse_us, 1773);
    zassert_equal(both.yaw_pulse_us, 1359);

    HornetTvc::reset();
    const auto pitch_only = settle(12.0f, 0.0f);
    zassert_equal(pitch_only.pitch_pulse_us, 1773);
    zassert_equal(pitch_only.yaw_pulse_us, 1500);

    HornetTvc::reset();
    const auto yaw_only = settle(0.0f, -6.0f);
    zassert_equal(yaw_only.pitch_pulse_us, 1500);
    zassert_equal(yaw_only.yaw_pulse_us, 1359);
}

ZTEST(HornetTvc_tests, test_pulse_always_within_range)
{
    for (float cmd : {-120.0f, -90.0f, -45.0f, 0.0f, 45.0f, 90.0f, 120.0f}) {
        HornetTvc::reset();
        for (int i = 0; i < SETTLE_TICKS; i += 100) {
            const auto r = tick(cmd, -cmd);
            zassert_true(r.pitch_pulse_us >= 1000 && r.pitch_pulse_us <= 2000);
            zassert_true(r.yaw_pulse_us >= 1000 && r.yaw_pulse_us <= 2000);
        }
    }
}

ZTEST(HornetTvc_tests, test_calibration_neutral_shifts_output)
{
    constexpr HornetTvc::AxisCalibration cal{30.0f, 1000, 2000, 1.0f};
    zassert_within(HornetTvc::gimbal_deg_to_extension_mm(0.0f, cal), 30.0f, 1e-5f);
    zassert_within(static_cast<double>(HornetTvc::gimbal_deg_to_extension_mm(6.0f, cal)), 30.0 + geometry_extension_mm(6.0) - NEUTRAL_EXTENSION_MM, 1e-3);
    zassert_equal(HornetTvc::extension_mm_to_pulse_us(HornetTvc::gimbal_deg_to_extension_mm(0.0f, cal), cal), 1600);
}

ZTEST(HornetTvc_tests, test_calibration_endpoints_scale_and_clamp)
{
    constexpr HornetTvc::AxisCalibration narrow{25.0f, 1100, 1900, 1.0f};
    zassert_equal(HornetTvc::extension_mm_to_pulse_us(0.0f, narrow), 1100);
    zassert_equal(HornetTvc::extension_mm_to_pulse_us(25.0f, narrow), 1500);
    zassert_equal(HornetTvc::extension_mm_to_pulse_us(50.0f, narrow), 1900);
    zassert_equal(HornetTvc::extension_mm_to_pulse_us(-10.0f, narrow), 1100);
    zassert_equal(HornetTvc::extension_mm_to_pulse_us(80.0f, narrow), 1900);

    constexpr HornetTvc::AxisCalibration reversed{25.0f, 2000, 1000, 1.0f};
    zassert_equal(HornetTvc::extension_mm_to_pulse_us(0.0f, reversed), 2000);
    zassert_equal(HornetTvc::extension_mm_to_pulse_us(10.0f, reversed), 1800);
    zassert_equal(HornetTvc::extension_mm_to_pulse_us(50.0f, reversed), 1000);
    zassert_equal(HornetTvc::extension_mm_to_pulse_us(-10.0f, reversed), 2000);
    zassert_equal(HornetTvc::extension_mm_to_pulse_us(80.0f, reversed), 1000);

    constexpr HornetTvc::AxisCalibration too_wide{25.0f, 900, 2100, 1.0f};
    zassert_equal(HornetTvc::extension_mm_to_pulse_us(0.0f, too_wide), 1000);
    zassert_equal(HornetTvc::extension_mm_to_pulse_us(50.0f, too_wide), 2000);
}

ZTEST(HornetTvc_tests, test_calibration_direction_mirrors_linkage)
{
    constexpr HornetTvc::AxisCalibration flipped{25.0f, 1000, 2000, -1.0f};
    for (float deg : {6.0f, 12.0f}) {
        zassert_equal(HornetTvc::gimbal_deg_to_extension_mm(deg, flipped), HornetTvc::gimbal_deg_to_extension_mm(-deg, DEFAULT_CAL));
        zassert_equal(HornetTvc::gimbal_deg_to_extension_mm(-deg, flipped), HornetTvc::gimbal_deg_to_extension_mm(deg, DEFAULT_CAL));
    }
    zassert_true(HornetTvc::gimbal_deg_to_extension_mm(12.0f, flipped) < 25.0f, "flipped axis should retract for positive angle");
}

ZTEST_SUITE(HornetTvc_tests, NULL, NULL, NULL, before_each, NULL);
