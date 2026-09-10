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
    opts.position_format.stop_position    = mjbots::moteus::kFloat;
    opts.position_format.velocity_limit  = mjbots::moteus::kFloat;
    opts.position_format.accel_limit     = mjbots::moteus::kFloat;
    opts.position_format.watchdog_timeout = mjbots::moteus::kFloat;

    return opts;
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

std::expected<std::tuple<TvcActuatorCommand, TvcActuatorCommand, RangerTvcMetrics>, Error>
tick(float pitch_command_deg) {
    if (!s_motor) {
        return std::unexpected(Error::from_cause("RangerTvc not initialized"));
    }

    mjbots::moteus::PositionMode::Command cmd;
    cmd.position         = pitch_command_deg / 360.0f;
    cmd.velocity         = 0.0f;
    cmd.stop_position    = NaN;
    cmd.velocity_limit   = 0.13f;  // ~0.13 rev/s → covers 46° in ~1 s
    cmd.accel_limit      = 4.0;
    cmd.watchdog_timeout = 0.5f;  // 500 ms — motor safe-stops if Teensy hangs

    LOG_INF("tx: pos=%d mrot", (int)(cmd.position * 1000));

    auto result = s_motor->SetPosition(cmd);

    if (result) {
        LOG_INF("rx: pos=%d mrot  vel=%d mrot/s  torq=%d mNm  mode=%d",
            (int)(result->values.position * 1000),
            (int)(result->values.velocity * 1000),
            (int)(result->values.torque   * 1000),
            (int)result->values.mode);
    } else {
        LOG_WRN("rx: no reply");
    }

    if (!result) {
        return std::unexpected(Error::from_cause("RangerTvc: no motor reply"));
    }

    return std::make_tuple(TvcActuatorCommand{}, TvcActuatorCommand{}, RangerTvcMetrics{});
}

}  // namespace RangerTvc
