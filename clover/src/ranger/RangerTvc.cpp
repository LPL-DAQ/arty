#include "RangerTvc.h"
#include "moteus/moteus.h"

#include <cmath>
#include <cstdio>
#include <optional>
#include <string>

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(RangerTVC);

namespace RangerTvc {
namespace {

namespace moteus = mjbots::moteus;

// What every Controller::Set*() call returns: empty if the motor didn't reply.
using MotorReply = moteus::Optional<moteus::Controller::Result>;

static const struct device *s_can_dev =
    DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus));

static std::shared_ptr<moteus::ZephyrCanTransport> s_transport;
static std::optional<moteus::Controller> s_motor;

// Travel range found by the last successful home(); empty until then.
static std::optional<TravelRange> s_travel;

// ---------------------------------------------------------------------------
// Homing tunables — start conservative and tune on the bench.
// ---------------------------------------------------------------------------
constexpr float kHomeVelocity     = 0.1f;   // rev/s creep speed toward a stop
constexpr float kHomeMaxTorque    = 0.25f;  // N·m — enough to beat friction, gentle on the stop
constexpr float kStallVelocity    = 0.01f;  // rev/s — below this counts as stalled
constexpr int   kStallTimeMs      = 200;    // must stay stalled this long
constexpr int   kStartGraceMs     = 300;    // ignore stall detection while starting to move
constexpr int   kDirTimeoutMs     = 10000;  // give up finding a stop after this long
constexpr int   kHomeLoopMs       = 10;     // command/poll period during homing
constexpr float kLimitMargin      = 0.03f;  // rev — soft limits sit this far inside the stops
constexpr float kMinExpectedRange = 0.1f;   // rev — smaller than this means a false stall
constexpr float kMoveVelocity     = 0.2f;   // rev/s for the move off the stop
constexpr float kMoveTolerance    = 0.01f;  // rev — "arrived" threshold for the move
constexpr int   kMoveTimeoutMs    = 10000;
constexpr float kWatchdogTimeout  = 0.5f;   // s — motor safe-stops if commands stop arriving

// Which direction's hard stop becomes position 0. The other stop is +range.
constexpr int kZeroDirection = -1;

static moteus::Controller::Options make_motor_opts(
    int id, std::shared_ptr<moteus::ZephyrCanTransport> transport) {
    moteus::Controller::Options opts;
    opts.id        = id;
    opts.transport = transport;

    // Query format — produces the same 3 query groups as the working test bench:
    //   mode(int8) | pos+vel+torque+Iq+Id(float×5) | voltage+temp+fault(int8×3)
    // mode/position/velocity/torque/voltage/temperature/fault are already the
    // right defaults in Query::Format; only q_current and d_current need adding.
    opts.query_format.q_current = moteus::kFloat;
    opts.query_format.d_current = moteus::kFloat;
    // Lets us confirm SetOutputExact took effect (2 = output-referenced).
    opts.query_format.home_state = moteus::kInt8;

    // Position command format — include the three NaN fields so they appear
    // in the frame exactly as the test bench sends them.
    opts.position_format.stop_position      = moteus::kFloat;
    opts.position_format.feedforward_torque = moteus::kFloat;
    opts.position_format.velocity_limit     = moteus::kFloat;
    opts.position_format.accel_limit        = moteus::kFloat;
    opts.position_format.watchdog_timeout   = moteus::kFloat;

    return opts;
}

// Position-mode format used only while homing (passed as command_override, so
// tick()'s format is untouched): adds a torque cap and the per-command flag
// that bypasses servopos limits.
static moteus::PositionMode::Format homing_format() {
    moteus::PositionMode::Format fmt;
    fmt.maximum_torque         = moteus::kFloat;
    fmt.velocity_limit         = moteus::kFloat;
    fmt.watchdog_timeout       = moteus::kFloat;
    fmt.ignore_position_bounds = moteus::kInt8;
    return fmt;
}

// Runs a moteus diagnostic-console command (e.g. "conf set ...") and returns
// true only if the motor answered "OK". Controller::DiagnosticCommand() drops
// the error code, so call the async form with a BlockingCallback instead — the
// Zephyr transport completes synchronously, so Wait() returns immediately.
static bool diag_command(const char *cmd) {
    moteus::BlockingCallback cbk;
    std::string response;
    s_motor->AsyncDiagnosticCommand(cmd, &response, cbk.callback());
    const int err = cbk.Wait();
    if (err != 0) {
        LOG_ERR("diag '%s' failed: err=%d response='%s'", cmd, err, response.c_str());
        return false;
    }
    LOG_INF("diag '%s' -> OK", cmd);
    return true;
}

static bool set_position_limits(float min_rev, float max_rev) {
    char line[64];
    if (std::isnan(min_rev)) {
        snprintf(line, sizeof(line), "conf set servopos.position_min nan");
    } else {
        snprintf(line, sizeof(line), "conf set servopos.position_min %.4f", (double)min_rev);
    }
    if (!diag_command(line)) { return false; }

    if (std::isnan(max_rev)) {
        snprintf(line, sizeof(line), "conf set servopos.position_max nan");
    } else {
        snprintf(line, sizeof(line), "conf set servopos.position_max %.4f", (double)max_rev);
    }
    return diag_command(line);
}

// Drives toward a hard stop at low torque until the motor stalls there.
// direction: -1 or +1. Returns the motor position at the stall.
static std::expected<float, Error> creep_to_stop(int direction) {
    const moteus::PositionMode::Format fmt = homing_format();

    moteus::PositionMode::Command cmd;
    cmd.position                   = NaN;  // NaN = start from current position, integrate velocity
    cmd.velocity                   = direction * kHomeVelocity;
    cmd.feedforward_torque         = kHomeMaxTorque;
    cmd.watchdog_timeout           = kWatchdogTimeout;
    cmd.ignore_position_bounds     = 1;

    LOG_INF("home: creeping %s at %.2f rev/s, max torque %.2f Nm",
            direction < 0 ? "negative" : "positive",
            (double)kHomeVelocity, (double)kHomeMaxTorque);

    const int64_t start = k_uptime_get();
    int64_t stall_since = -1;

    //Dying some where here, stack overflow...
    while (true) {
        const int64_t now = k_uptime_get();
        if (now - start > kDirTimeoutMs) {
            s_motor->SetStop();
            return std::unexpected(Error::from_cause("home: no stop found within timeout"));
        }

        MotorReply result = s_motor->SetPosition(cmd, &fmt);
        if (!result) {
            s_motor->SetStop();
            return std::unexpected(Error::from_cause("home: no motor reply"));
        }

        const moteus::Query::Result &v = result->values;
        if (v.mode == moteus::Mode::kFault) {
            LOG_ERR("home: motor fault %d at pos=%.3f", (int)v.fault, v.position);
            s_motor->SetStop();
            return std::unexpected(Error::from_cause("home: motor faulted while creeping"));
        }

        const bool past_grace = (now - start) > kStartGraceMs;
        if (past_grace && std::fabs(v.velocity) < kStallVelocity) {
            if (stall_since < 0) { stall_since = now; }
            if (now - stall_since >= kStallTimeMs) {
                LOG_INF("home: stall at pos=%.4f rev (torque=%.3f Nm)",
                        v.position, v.torque);
                return static_cast<float>(v.position);
            }
        } else {
            stall_since = -1;
        }

        k_msleep(kHomeLoopMs);
    }
}

// Moves to an absolute position (ignoring servopos limits) and waits to arrive.
static std::expected<void, Error> move_to(float target) {
    const moteus::PositionMode::Format fmt = homing_format();

    moteus::PositionMode::Command cmd;
    cmd.position                   = target;
    cmd.velocity                   = 0.0;
    cmd.velocity_limit             = kMoveVelocity;
    cmd.feedforward_torque         = kHomeMaxTorque;
    cmd.watchdog_timeout           = kWatchdogTimeout;
    cmd.ignore_position_bounds     = 1;

    const int64_t start = k_uptime_get();
    while (k_uptime_get() - start < kMoveTimeoutMs) {
        MotorReply result = s_motor->SetPosition(cmd, &fmt);
        if (!result) {
            return std::unexpected(Error::from_cause("home: no motor reply during move"));
        }
        const moteus::Query::Result &v = result->values;
        if (v.mode == moteus::Mode::kFault) {
            LOG_ERR("home: motor fault %d during move", (int)v.fault);
            return std::unexpected(Error::from_cause("home: motor faulted during move"));
        }
        if (std::fabs(v.position - target) < kMoveTolerance &&
            std::fabs(v.velocity) < kStallVelocity) {
            return {};
        }
        k_msleep(kHomeLoopMs);
    }
    return std::unexpected(Error::from_cause("home: move did not arrive in time"));
}

}  // anonymous namespace

std::expected<TravelRange, Error> home() {
    LOG_INF("Starting Home");
    if (!s_motor) {
        return std::unexpected(Error::from_cause("RangerTvc not initialized"));
    }

    s_travel.reset();

    // 1. Clear any fault/timeout.
    s_motor->SetStop();
    k_msleep(50);

    // 2. Disable limits left over from a previous boot so entering position
    //    mode can't fault with "start outside limit".
    // if (!set_position_limits(NaN, NaN)) {
    //     return std::unexpected(Error::from_cause("home: failed to clear servopos limits"));
    // }

    // 3. Find the zero-side stop and define it as position 0.
    std::expected<float, Error> zero_stop = creep_to_stop(kZeroDirection);
    if (!zero_stop) { 
        return std::unexpected(zero_stop.error()); 
    }

    s_motor->SetStop();
    k_msleep(50);

    MotorReply exact = s_motor->SetOutputExact({0.0});
    if (!exact) {
        return std::unexpected(Error::from_cause("home: SetOutputExact got no reply"));
    }
    // Query once more so the reply reflects the new reference.
    MotorReply after_zero = s_motor->SetQuery();
    if (after_zero) {
        LOG_INF("home: zero set, pos=%.4f home_state=%d",
                after_zero->values.position, (int)after_zero->values.home_state);
    }

    // 4. Find the other stop; its position is the full travel.
    std::expected<float, Error> far_stop = creep_to_stop(-kZeroDirection);

    if (!far_stop) { 
        return std::unexpected(far_stop.error()); 
    }

    const float range = std::fabs(*far_stop);

    s_motor->SetStop();
    k_msleep(50);

    // 5. Sanity check — a tiny range means it stalled on friction, not a stop.
    if (range < kMinExpectedRange) {
        LOG_ERR("home: range %.4f rev is implausibly small", (double)range);
        return std::unexpected(Error::from_cause("home: measured range too small"));
    }

    // With kZeroDirection = -1 the far stop is at +range; with +1 it's at -range.
    const float lo = (kZeroDirection < 0) ? 0.0f : -range;
    const float hi = (kZeroDirection < 0) ? range : 0.0f;
    const TravelRange travel{lo + kLimitMargin, hi - kLimitMargin};

    // 6. Move off the stop to inside the limits *before* enabling them, so
    //    the next position-mode entry starts in bounds.
    const float park = (kZeroDirection < 0) ? travel.max_rev : travel.min_rev;
    std::expected<void, Error> moved = move_to(park);

    s_motor->SetStop();

    if (!moved) { 
        return std::unexpected(moved.error()); 
    }

    // 7. Let the moteus firmware enforce the limits (RAM only, no conf write —
    //    they're re-measured every boot).
    // if (!set_position_limits(travel.min_rev, travel.max_rev)) {
    //     return std::unexpected(Error::from_cause("home: failed to set servopos limits"));
    // }

    LOG_INF("home: done, travel [%.4f, %.4f] rev (stops at %.4f / %.4f)",
            (double)travel.min_rev, (double)travel.max_rev, (double)lo, (double)hi);

    s_travel = travel;
    return travel;
}

std::optional<TravelRange> travel() {
    return s_travel;
}

void reset() {
    if (!device_is_ready(s_can_dev)) {
        LOG_ERR("CAN device not ready");
        return;
    }

    can_stop(s_can_dev);

    int ret = can_set_mode(s_can_dev, CAN_MODE_FD);
    if (ret != 0) {
        LOG_ERR("can_set_mode(FD) failed: %d", ret);
        return;
    }

    ret = can_start(s_can_dev);
    if (ret != 0) {
        LOG_ERR("can_start failed: %d", ret);
        return;
    }

    LOG_INF("CAN started in FD mode");

    moteus::ZephyrCanTransport::Options t_opts;
    t_opts.can_dev         = s_can_dev;
    t_opts.brs_enabled     = false;  // match test bench — no BRS until confirmed working
    t_opts.recv_timeout_ms = 100;    // 100 ms; motor reply typically arrives within a few ms
    s_transport = std::make_shared<moteus::ZephyrCanTransport>(t_opts);
    LOG_INF("transport created");

    s_motor.emplace(make_motor_opts(1, s_transport));
    LOG_INF("controller ready (motor id=1)");

    // Clear any fault/timeout state left from a previous session.
    s_motor->SetStop();
    k_msleep(50);
}

//TODO: Test tick without the blokers for setup to see if SetPosition works even when pos has not been reached
std::expected<std::tuple<TvcActuatorCommand, TvcActuatorCommand, RangerTvcMetrics>, Error>
tick(float position) {
    if (!s_motor) {
        return std::unexpected(Error::from_cause("RangerTvc not initialized"));
    }

    if (!s_travel) {
        return std::unexpected(Error::from_cause("RangerTvc not homed"));
    }

    moteus::PositionMode::Command cmd;
    cmd.position            = position; //position relative to the motor position
    cmd.velocity            = 0.0f;
    cmd.stop_position       = NaN;
    cmd.feedforward_torque  = 1.0f;
    cmd.velocity_limit      = 0.75f; // temp setting speed to 0.75f
    cmd.watchdog_timeout    = kWatchdogTimeout; // motor safe-stops if Teensy hangs

    LOG_INF("tx: pos=%.3f rev", (double)cmd.position);

    MotorReply result = s_motor->SetPosition(cmd);

    //Does this work even when pos has not been reached?
    if (result) {
        LOG_INF("rx: pos=%.3f rev  vel=%d mrot/s  torq=%d mNm  mode=%d  fault=%d",
            result->values.position,
            (int)(result->values.velocity * 1000),
            (int)(result->values.torque   * 1000),
            (int)result->values.mode,
            (int)result->values.fault);
    } else {
        LOG_WRN("rx: no reply");
    }

    if (!result) {
        return std::unexpected(Error::from_cause("RangerTvc: no motor reply"));
    }

    return std::make_tuple(TvcActuatorCommand{}, TvcActuatorCommand{}, RangerTvcMetrics{});
}

}  // namespace RangerTvc
