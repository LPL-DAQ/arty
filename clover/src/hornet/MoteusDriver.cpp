#include "MoteusDriver.h"
#include "moteus/moteus_transport.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <map>
#include <memory>

LOG_MODULE_REGISTER(moteus, CONFIG_LOG_DEFAULT_LEVEL);

namespace {

static const struct device *s_can_dev =
    DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus));

// One Controller per motor ID, created in init().
std::map<uint8_t, mjbots::moteus::Controller> s_controllers;
std::shared_ptr<mjbots::moteus::ZephyrCanTransport> s_transport;

}  // namespace

std::expected<void, Error> MoteusDriver::init()
{
    if (!device_is_ready(s_can_dev)) {
        return std::unexpected(Error::from_cause("MoteusDriver: CAN device not ready"));
    }

    int ret = can_set_mode(s_can_dev, CAN_MODE_FD);
    if (ret != 0) {
        return std::unexpected(Error::from_cause("MoteusDriver: can_set_mode(FD) failed: %d", ret));
    }

    ret = can_start(s_can_dev);
    if (ret != 0 && ret != -EALREADY) {
        return std::unexpected(Error::from_cause("MoteusDriver: can_start failed"));
    }

    mjbots::moteus::ZephyrCanTransport::Options t_opts;
    t_opts.can_dev         = s_can_dev;
    t_opts.send_timeout_ms = 10;
    t_opts.recv_timeout_ms = 10;
    s_transport = std::make_shared<mjbots::moteus::ZephyrCanTransport>(t_opts);

    LOG_INF("MoteusDriver: CAN started, transport ready");
    return {};
}

// Internal helper: get (or lazily create) the Controller for motor_id.
static mjbots::moteus::Controller& get_controller(uint8_t motor_id)
{
    auto it = s_controllers.find(motor_id);
    if (it != s_controllers.end()) return it->second;

    mjbots::moteus::Controller::Options opts;
    opts.id        = motor_id;
    opts.transport = s_transport;

    // Request position, velocity, and torque in every reply.
    opts.query_format.position = mjbots::moteus::kFloat;
    opts.query_format.velocity = mjbots::moteus::kFloat;
    opts.query_format.torque   = mjbots::moteus::kFloat;
    opts.query_format.mode     = mjbots::moteus::kInt8;

    s_controllers.emplace(motor_id, mjbots::moteus::Controller(opts));
    return s_controllers.at(motor_id);
}

std::expected<void, Error> MoteusDriver::stop(uint8_t motor_id)
{
    try {
        get_controller(motor_id).SetStop();
        return {};
    } catch (const std::exception& e) {
        return std::unexpected(Error::from_cause("MoteusDriver::stop: %s", e.what()));
    }
}

std::expected<mjbots::moteus::Optional<MoteusDriver::Reply>, Error>
MoteusDriver::set_position(uint8_t motor_id,
                            double position_rev,
                            double velocity_rev_s,
                            double max_torque_Nm)
{
    mjbots::moteus::PositionMode::Command cmd;
    cmd.position      = position_rev;
    cmd.velocity      = velocity_rev_s;
    cmd.maximum_torque = max_torque_Nm;

    try {
        auto result = get_controller(motor_id).SetPosition(cmd);
        return result;
    } catch (const std::exception& e) {
        return std::unexpected(
            Error::from_cause("MoteusDriver::set_position: %s", e.what()));
    }
}
