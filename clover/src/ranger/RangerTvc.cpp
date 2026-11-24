#include "RangerTvc.h"
#include "moteus/moteus.h"

#include <optional>

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(RangerTVC);

namespace RangerTvc {
namespace {

static const struct device *s_can_dev =
    DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus));

static std::shared_ptr<mjbots::moteus::ZephyrCanTransport> s_transport;
static std::optional<mjbots::moteus::Controller> s_motor;

static std::optional<TravelRange> s_travel;

// ---------------------------------------------------------------------------
// Homing tunables — start conservative and tune on the bench.
// ---------------------------------------------------------------------------
constexpr float kHomeVelocity     = 0.75f;   // rev/s creep speed toward a stop
constexpr float kHomeMaxTorque    = 0.1f;  // N·m — enough to beat friction, gentle on the stop
constexpr float kStallVelocity    = 0.1f;  // rev/s — below this counts as stalled
constexpr int   kStallTimeMs      = 200;    // must stay stalled this long
constexpr int   kStartGraceMs     = 300;    // ignore stall detection while starting to move
constexpr int   kDirTimeoutMs     = 30000;  // give up finding a stop after this long
constexpr int   kHomeLoopMs       = 20;     // command/poll period during homing
constexpr float kLimitMargin      = 0.03f;  // rev — soft limits sit this far inside the stops
constexpr float kMinExpectedRange = 0.1f;   // rev — smaller than this means a false stall
constexpr float kMoveVelocity     = 0.2f;   // rev/s for the move off the stop
constexpr float kMoveTolerance    = 0.01f;  // rev — "arrived" threshold for the move
constexpr int   kMoveTimeoutMs    = 10000;
constexpr float kWatchdogTimeout  = 0.5f;   // s — motor safe-stops if commands stop arriving
constexpr int kZeroDirection      = -1;     // which direction's hard stop becomes position 0 

static mjbots::moteus::Controller::Options make_motor_opts(
    int id, std::shared_ptr<mjbots::moteus::ZephyrCanTransport> transport) {
    mjbots::moteus::Controller::Options opts;
    opts.id        = id;
    opts.transport = transport;

    // Query format — produces the same 3 query groups as the working test bench:
    //   mode(int8) | pos+vel+torque+Iq+Id(float×5) | voltage+temp+fault(int8×3)
    // mode/position/velocity/torque/voltage/temperature/fault are already the
    // right defaults in Query::Format; only q_current and d_current need adding.
    opts.query_format.q_current = mjbots::moteus::kFloat;
    opts.query_format.d_current = mjbots::moteus::kFloat;

    // Position command format — include the three NaN fields so they appear
    // in the frame exactly as the test bench sends them.
    opts.position_format.stop_position      = mjbots::moteus::kFloat;
    opts.position_format.feedforward_torque = mjbots::moteus::kFloat;
    opts.position_format.velocity_limit     = mjbots::moteus::kFloat;
    opts.position_format.accel_limit        = mjbots::moteus::kFloat;
    opts.position_format.watchdog_timeout   = mjbots::moteus::kFloat;

    return opts;
}

static std::expected<float, Error> creep_to_stop(int direction){
    
    mjbots::moteus::PositionMode::Command cmd;
    cmd.position                    = NaN;
    cmd.velocity                   = direction * kHomeVelocity;
    cmd.feedforward_torque          = kHomeMaxTorque;
    cmd.watchdog_timeout            = kWatchdogTimeout;
    cmd.ignore_position_bounds     = 1;
    
    LOG_INF("home: creeping %s at %.2f rev/s, max torque %.2f Nm",
    direction < 0 ? "negative" : "positive",
    kHomeVelocity, kHomeMaxTorque);

    const int64_t start = k_uptime_get();
    int64_t stall_since = -1;

    while(true){
        const int64_t now = k_uptime_get();

        if (now - start > kDirTimeoutMs){
            s_motor->SetStop();
            return std::unexpected(Error::from_cause("home: not stop found within timeout"));
        }

        mjbots::moteus::Optional<mjbots::moteus::Controller::Result> result = s_motor->SetPosition(cmd);

        if(!result){
            s_motor->SetStop();
            return std::unexpected(Error::from_cause("home: no motor reply"));
        }

        const mjbots::moteus::Query::Result &v = result->values;

        LOG_INF("rx: pos=%.3f mrot  vel=%d mrot/s  torq=%d mNm  mode=%d fault=%d",
            result->values.position,
            (int)(result->values.velocity * 1000),
            (int)(result->values.torque   * 1000),
            (int)(result->values.mode),
            (int)(result->values.fault));

        if (v.mode == mjbots::moteus::Mode::kFault){
            LOG_ERR("home: motor fault %d at pos%.3f", (int)v.fault, v.position);
            s_motor->SetStop();
            return std::unexpected(Error::from_cause("home: motor faulted while creeping"));
        }

        const bool past_grace = (now - start) > kStartGraceMs;

        if (past_grace && std::fabs((float)v.velocity) < kStallVelocity){
            
            if (stall_since < 0){
                stall_since = now;
            }

            if (now - stall_since >= kStallTimeMs){
                LOG_INF("home: stall at pos=%.4f rev (torque=%.3f Nm)", v.position, v.torque);
                
                return static_cast<float>(v.position);
            }
        }
        else{
            stall_since = -1;
        }

        k_msleep(kHomeLoopMs);
    }
}

}  // namespace

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

    mjbots::moteus::ZephyrCanTransport::Options t_opts;
    t_opts.can_dev         = s_can_dev;
    t_opts.brs_enabled     = false;  // match test bench — no BRS until confirmed working
    t_opts.recv_timeout_ms = 100;    // 100 ms; motor reply typically arrives within a few ms
    s_transport = std::make_shared<mjbots::moteus::ZephyrCanTransport>(t_opts);
    LOG_INF("transport created");

    s_motor.emplace(make_motor_opts(1, s_transport));
    LOG_INF("controller ready (motor id=1)");

    // Clear any fault/timeout state left from a previous session.
    s_motor->SetStop();
    k_msleep(50);
}

std::expected<TravelRange, Error> home(){
    LOG_INF("Starting home");
    if(!s_motor){
        return std::unexpected(Error::from_cause("RangerTvc not initialized"));
    }

    s_travel.reset();

    s_motor->SetStop();
    k_msleep(50);

    std::expected<float, Error> zero_stop = creep_to_stop(kZeroDirection);

    if (!zero_stop){
        return std::unexpected(zero_stop.error());
    }

    s_motor->SetStop();
    k_msleep(50);

    LOG_INF("Figured out position 0");

    std::expected<float, Error> far_stop = creep_to_stop(-kZeroDirection);

    if (!far_stop){
        return std::unexpected(far_stop.error());
    }    

    s_motor->SetStop();
    k_msleep(50);

    LOG_INF("Figured out far stop position");

    TravelRange temp = {0.0f, 0.0f};

    return temp;
}

std::expected<std::tuple<TvcActuatorCommand, TvcActuatorCommand, RangerTvcMetrics>, Error>
tick(float position) {
    if (!s_motor) {
        return std::unexpected(Error::from_cause("RangerTvc not initialized"));
    }

    mjbots::moteus::PositionMode::Command cmd;
    cmd.position            = position; //position relative to the motor position
    cmd.velocity            = 0.0f;
    cmd.stop_position       = NaN;
    cmd.feedforward_torque  = 1.0f;
    cmd.velocity_limit      = 0.75f; // temp setting speed to 0.75f
    cmd.watchdog_timeout    = 0.5f; // 500 ms — motor safe-stops if Teensy hangs

    LOG_INF("tx: pos=%d mrot", (int)cmd.position);

    auto result = s_motor->SetPosition(cmd);

    if (result) {
        LOG_INF("rx: pos=%.3f mrot  vel=%d mrot/s  torq=%d mNm  mode=%d fault=%d",
            result->values.position,
            (int)(result->values.velocity * 1000),
            (int)(result->values.torque   * 1000),
            (int)(result->values.mode),
            (int)(result->values.fault));
    } else {
        LOG_WRN("rx: no reply");
    }

    if (!result) {
        return std::unexpected(Error::from_cause("RangerTvc: no motor reply"));
    }

    return std::make_tuple(TvcActuatorCommand{}, TvcActuatorCommand{}, RangerTvcMetrics{});
}

}  // namespace RangerTvc
