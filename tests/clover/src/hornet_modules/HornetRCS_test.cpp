#include "../../../../clover/src/hornet/HornetRcs.h"
#include <zephyr/ztest.h>
#include <cmath>

// Bang-bang roll control: effort = Kp * (command - measured roll), Kp = 2, both in degrees.
// A closed valve opens when |effort| > 0.15 and closes when |effort| < 0.10, and stays open
// for at least 0.10 s once opened.
static constexpr uint32_t THRUSTER_OFF = 1000;
static constexpr uint32_t THRUSTER_ON = 2000;

// Far from zero so that dt measured as time since boot would be obvious.
static constexpr int64_t T0_US = 10'000'000;
static constexpr int64_t TICK_US = 1'500;

struct Output {
    uint32_t cw;
    uint32_t ccw;
};

static EstimatedState rolled_state(float roll_deg)
{
    EstimatedState state = EstimatedState_init_default;
    const float half_rad = roll_deg * static_cast<float>(M_PI) / 360.0f;
    state.R_WB.qw = std::cos(half_rad);
    state.R_WB.qx = std::sin(half_rad);
    state.R_WB.qy = 0.0f;
    state.R_WB.qz = 0.0f;
    return state;
}

static Output tick_at(int64_t now_us, float command_deg, float roll_deg = 0.0f)
{
    HornetRcs::set_now_us_for_testing(now_us);
    auto result = HornetRcs::tick(rolled_state(roll_deg), command_deg);
    zassert_true(result.has_value(), "tick should succeed");
    auto [cw, ccw, metrics] = *result;
    return {static_cast<uint32_t>(cw), static_cast<uint32_t>(ccw)};
}

static int64_t ms(double t)
{
    return T0_US + static_cast<int64_t>(t * 1000.0);
}

static void before_each(void*)
{
    HornetRcs::reset();
}

ZTEST(HornetRcs_tests, test_zero_error_keeps_both_off)
{
    const auto out = tick_at(T0_US, 0.0f);
    zassert_equal(out.cw, THRUSTER_OFF);
    zassert_equal(out.ccw, THRUSTER_OFF);
}

ZTEST(HornetRcs_tests, test_positive_command_fires_ccw)
{
    const auto out = tick_at(T0_US, 30.0f);
    zassert_equal(out.ccw, THRUSTER_ON);
    zassert_equal(out.cw, THRUSTER_OFF);
}

ZTEST(HornetRcs_tests, test_negative_command_fires_cw)
{
    const auto out = tick_at(T0_US, -30.0f);
    zassert_equal(out.cw, THRUSTER_ON);
    zassert_equal(out.ccw, THRUSTER_OFF);
}

ZTEST(HornetRcs_tests, test_measured_roll_is_in_degrees)
{
    auto out = tick_at(T0_US, 10.0f, 10.0f);
    zassert_equal(out.cw, THRUSTER_OFF, "10 deg command at 10 deg roll is zero error");
    zassert_equal(out.ccw, THRUSTER_OFF, "10 deg command at 10 deg roll is zero error");

    HornetRcs::reset();
    out = tick_at(T0_US, 0.0f, 10.0f);
    zassert_equal(out.cw, THRUSTER_ON, "0 deg command at 10 deg roll should fire CW");
    zassert_equal(out.ccw, THRUSTER_OFF);
}

ZTEST(HornetRcs_tests, test_min_on_time_holds_for_100_ms)
{
    zassert_equal(tick_at(ms(0), 30.0f).ccw, THRUSTER_ON);
    zassert_equal(tick_at(ms(50), 0.0f).ccw, THRUSTER_ON, "held at 50 ms");
    zassert_equal(tick_at(ms(99), 0.0f).ccw, THRUSTER_ON, "held at 99 ms");
    zassert_equal(tick_at(ms(101), 0.0f).ccw, THRUSTER_ON, "timer expires on this tick, valve not re-evaluated yet");

    const auto out = tick_at(ms(102.5), 0.0f);
    zassert_equal(out.ccw, THRUSTER_OFF, "closes once the minimum on-time has passed");
    zassert_equal(out.cw, THRUSTER_OFF);
}

ZTEST(HornetRcs_tests, test_min_on_time_at_control_rate)
{
    zassert_equal(tick_at(T0_US, 30.0f).ccw, THRUSTER_ON);

    int ticks_on = 0;
    for (int i = 1; i <= 200; ++i) {
        if (tick_at(T0_US + i * TICK_US, 0.0f).ccw == THRUSTER_ON) {
            ticks_on = i;
        }
    }
    // 0.10 s at 1.5 ms per tick is 66.7 ticks, plus the tick that sees the timer at zero.
    zassert_true(ticks_on >= 67 && ticks_on <= 69, "valve stayed on for %d ticks", ticks_on);
}

ZTEST(HornetRcs_tests, test_dt_is_per_tick_delta)
{
    zassert_equal(tick_at(T0_US, 30.0f).ccw, THRUSTER_ON);
    for (int i = 0; i < 100; ++i) {
        zassert_equal(tick_at(T0_US, 0.0f).ccw, THRUSTER_ON, "no time has passed, so the timer must not advance");
    }
    zassert_equal(tick_at(ms(99), 0.0f).ccw, THRUSTER_ON);
    tick_at(ms(101), 0.0f);
    zassert_equal(tick_at(ms(102.5), 0.0f).ccw, THRUSTER_OFF);
}

ZTEST(HornetRcs_tests, test_min_on_time_holds_against_reversed_command)
{
    tick_at(ms(0), 30.0f);
    auto out = tick_at(ms(50), -30.0f);
    zassert_equal(out.ccw, THRUSTER_ON, "CCW held during minimum on-time");
    zassert_equal(out.cw, THRUSTER_OFF, "CW may not fire while CCW is held");

    tick_at(ms(101), -30.0f);
    out = tick_at(ms(102.5), -30.0f);
    zassert_equal(out.ccw, THRUSTER_OFF, "CCW closes after the minimum on-time");
    out = tick_at(ms(104), -30.0f);
    zassert_equal(out.cw, THRUSTER_ON, "CW fires on the next tick");
}

ZTEST(HornetRcs_tests, test_hysteresis_band_after_min_on_time)
{
    tick_at(ms(0), 30.0f);
    tick_at(ms(101), 0.06f);
    zassert_equal(tick_at(ms(150), 0.06f).ccw, THRUSTER_ON, "effort 0.12 is inside the band, valve stays open");
    zassert_equal(tick_at(ms(151.5), 0.04f).ccw, THRUSTER_OFF, "effort 0.08 is below the band, valve closes");
}

ZTEST(HornetRcs_tests, test_band_does_not_open_closed_valve)
{
    const auto out = tick_at(T0_US, 0.06f);
    zassert_equal(out.ccw, THRUSTER_OFF, "effort 0.12 is below the 0.15 opening threshold");
    zassert_equal(out.cw, THRUSTER_OFF);
}

ZTEST(HornetRcs_tests, test_reset_clears_valve_and_timer)
{
    tick_at(ms(0), 30.0f);
    HornetRcs::reset();
    const auto out = tick_at(ms(10), 0.05f);
    zassert_equal(out.cw, THRUSTER_OFF);
    zassert_equal(out.ccw, THRUSTER_OFF, "reset must clear the open valve and its timer");
}

ZTEST(HornetRcs_tests, test_reset_allows_opposite_direction)
{
    tick_at(ms(0), 30.0f);
    HornetRcs::reset();
    auto out = tick_at(ms(10), -30.0f);
    zassert_equal(out.cw, THRUSTER_ON);
    zassert_equal(out.ccw, THRUSTER_OFF);

    HornetRcs::reset();
    out = tick_at(ms(20), 30.0f);
    zassert_equal(out.ccw, THRUSTER_ON);
    zassert_equal(out.cw, THRUSTER_OFF);
}

ZTEST(HornetRcs_tests, test_reset_mid_hysteresis_disables_valve)
{
    tick_at(ms(0), 30.0f);
    tick_at(ms(101), 0.06f);
    zassert_equal(tick_at(ms(150), 0.06f).ccw, THRUSTER_ON);
    HornetRcs::reset();
    zassert_equal(tick_at(ms(160), 0.06f).ccw, THRUSTER_OFF, "after reset the valve needs a fresh deadzone cross");
}

static HornetRcsMetrics metrics_at(int64_t now_us, float command_deg, float roll_deg = 0.0f)
{
    HornetRcs::set_now_us_for_testing(now_us);
    auto result = HornetRcs::tick(rolled_state(roll_deg), command_deg);
    zassert_true(result.has_value(), "tick should succeed");
    return std::get<2>(*result);
}

ZTEST(HornetRcs_tests, test_metrics_report_controller_state)
{
    auto m = metrics_at(ms(0), 30.0f, 5.0f);
    zassert_within(m.measured_roll_deg, 5.0f, 1e-3f);
    zassert_within(m.control_effort, 2.0f * (30.0f - 5.0f), 1e-3f);
    zassert_equal(m.valve_state, 1);
    zassert_within(m.remaining_on_time_s, 0.10f, 1e-6f);

    m = metrics_at(ms(50), 30.0f, 5.0f);
    zassert_equal(m.valve_state, 1);
    zassert_within(m.remaining_on_time_s, 0.05f, 1e-4f);

    metrics_at(ms(101), 5.0f, 5.0f);
    m = metrics_at(ms(102.5), 5.0f, 5.0f);
    zassert_equal(m.valve_state, 0);
    zassert_equal(m.remaining_on_time_s, 0.0f);
    zassert_within(m.control_effort, 0.0f, 1e-3f);

    HornetRcs::reset();
    m = metrics_at(ms(200), -30.0f);
    zassert_equal(m.valve_state, -1);
}

ZTEST(HornetRcs_tests, test_bench_percent_maps_to_pulse_and_caps)
{
    zassert_equal(HornetRcs::bench_throttle_percent_to_pulse_us(0.0f), 1000);
    zassert_equal(HornetRcs::bench_throttle_percent_to_pulse_us(5.0f), 1050);
    zassert_equal(HornetRcs::bench_throttle_percent_to_pulse_us(10.0f), 1100);
    zassert_equal(HornetRcs::bench_throttle_percent_to_pulse_us(12.34f), 1123);
    zassert_equal(HornetRcs::bench_throttle_percent_to_pulse_us(12.36f), 1124, "should round, not truncate");
    zassert_equal(HornetRcs::bench_throttle_percent_to_pulse_us(15.0f), 1150);
    zassert_equal(HornetRcs::bench_throttle_percent_to_pulse_us(100.0f), 1150, "should cap at the bench limit");
    zassert_equal(HornetRcs::bench_throttle_percent_to_pulse_us(-20.0f), 1000);
    zassert_equal(HornetRcs::bench_throttle_percent_to_pulse_us(NAN), 1000);
}

ZTEST_SUITE(HornetRcs_tests, NULL, NULL, NULL, before_each, NULL);
