#include "ThrottleStateValveSeq.h"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(ThrottleStateValveSeq, LOG_LEVEL_INF);

static Trace fuel_trace;
static Trace lox_trace;
static bool has_fuel;
static bool has_lox;
static float fuel_total_time;
static float lox_total_time;

void ThrottleStateValveSeq::init(bool has_fuel_trace, bool has_lox_trace, float fuel_total_time_ms, float lox_total_time_ms)
{
    // Timer is reset in handle_start_sequence before entering

    has_fuel = has_fuel_trace;
    has_lox = has_lox_trace;
    fuel_total_time = fuel_total_time_ms;
    lox_total_time = lox_total_time_ms;
}

std::pair<ThrottleStateOutput, ThrottleValveSequenceData> ThrottleStateValveSeq::tick(int64_t current_time, int64_t start_time)
{
    ThrottleStateOutput out{};
    ThrottleValveSequenceData data{};
    out.power_on = true;
    out.next_state = ThrottleState_THROTTLE_STATE_VALVE_SEQ;  // Assume we stay in this state by default

    float dt = current_time - start_time;

    if (has_fuel) {
        auto f_target = fuel_trace.sample(dt);
        if (!f_target) {
            LOG_ERR("Failed to sample fuel trace: %s", f_target.error().build_message().c_str());
            out.next_state = ThrottleState_THROTTLE_STATE_IDLE;
            return {out, data};
        }
        out.has_fuel_pos = true;
        out.fuel_pos = *f_target;
    }

    if (has_lox) {
        auto l_target = lox_trace.sample(dt);
        if (!l_target) {
            LOG_ERR("Failed to sample lox trace: %s", l_target.error().build_message().c_str());
            out.next_state = ThrottleState_THROTTLE_STATE_IDLE;
            return {out, data};
        }
        out.has_lox_pos = true;
        out.lox_pos = *l_target;
    }

    bool done_fuel = has_fuel ? dt >= fuel_total_time : true;
    bool done_lox = has_lox ? dt >= lox_total_time : true;

    if (done_fuel && done_lox) {
        LOG_INF("Done open loop seq, dt was %f", static_cast<double>(dt));
        out.next_state = ThrottleState_THROTTLE_STATE_IDLE;
    }

    return std::make_pair(out, data);
}

Trace& ThrottleStateValveSeq::get_fuel_trace()
{
    return fuel_trace;
}

Trace& ThrottleStateValveSeq::get_lox_trace()
{
    return lox_trace;
}
