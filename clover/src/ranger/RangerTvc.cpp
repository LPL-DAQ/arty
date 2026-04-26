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
    opts.query_format.position = mjbots::moteus::kFloat;
    opts.query_format.velocity = mjbots::moteus::kFloat;
    opts.query_format.torque   = mjbots::moteus::kFloat;
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
    t_opts.can_dev = s_can_dev;
    s_transport = std::make_shared<mjbots::moteus::ZephyrCanTransport>(t_opts);

    s_motor.emplace(make_motor_opts(1, s_transport));

    mjbots::moteus::PositionMode::Command init_cmd;
    init_cmd.position = 1.0f;
    init_cmd.velocity = 1.0f;
    s_motor->SetPosition(init_cmd);

    LOG_INF("controller ready (motor id=1)");
}

std::expected<std::tuple<TvcActuatorCommand, TvcActuatorCommand, RangerTvcMetrics>, Error>
tick(float pitch_command_deg) {
    if (!s_motor) {
        return std::unexpected(Error::from_cause("RangerTvc not initialized"));
    }

    mjbots::moteus::PositionMode::Command cmd;
    cmd.position = pitch_command_deg / 360.0f;

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
