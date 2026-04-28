#include "Valves.h"
#include "clover.pb.h"
#include <MutexGuard.h>
#include <optional>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(Valves, CONFIG_LOG_DEFAULT_LEVEL);

K_MUTEX_DEFINE(valves_lock);

/// Valve GPIOs.
static constexpr int NUM_VALVE_GPIOS = DT_PROP_LEN(DT_PATH(zephyr_user), valve_gpios);
static constexpr gpio_dt_spec VALVE_GPIOS[NUM_VALVE_GPIOS] = {DT_FOREACH_PROP_ELEM_SEP(DT_PATH(zephyr_user), valve_gpios, GPIO_DT_SPEC_GET_BY_IDX, (, ))};

std::array<std::optional<ValveConfig>, _Valve_ARRAYSIZE> valve_configs;
std::array<ValveState, _Valve_ARRAYSIZE> valve_states;

/// Configure valve GPIOs.
std::expected<void, Error> Valves::init()
{
    MutexGuard valves_guard{&valves_lock};

    LOG_INF("Configuring valve GPIOs (%d in total)", NUM_VALVE_GPIOS);
    for (int i = 0; i < NUM_VALVE_GPIOS; ++i) {
        const auto& gpio_cfg = VALVE_GPIOS[i];
        if (int err = gpio_pin_configure_dt(&gpio_cfg, GPIO_OUTPUT_ACTIVE); err) {
            return std::unexpected(Error::from_code(err).context("failed to configure valve gpio %s (channel %d)", gpio_cfg.port->name, i));
        }

        // Ensure all GPIOs are in off state.
        if (int err = gpio_pin_set_dt(&gpio_cfg, 0); err) {
            return std::unexpected(Error::from_code(err).context("failed to initialize gpio %s (channel %d) as off", gpio_cfg.port->name, i));
        }
    }

    valve_configs.fill(std::nullopt);
    valve_states.fill(ValveState_UNKNOWN_VALVE_STATE);

    LOG_INF("Valve GPIOs ready.");
    return {};
}

/// Report all valve states.
std::expected<ValveStates, Error> Valves::get_valve_states()
{
    MutexGuard valves_guard{&valves_lock};

    ValveStates out;

    for (int i = 0; i < _Valve_ARRAYSIZE; ++i) {
        const auto& valve_config = valve_configs[i];
        const auto& valve_state = valve_states[i];

        if (!valve_config) {
            continue;
        }

        // Populate output with valve assignment.
        switch (valve_config->assignment) {
        // Vehicle

        // Pressurant
        case Valve_SV001: {
            out.has_sv001 = true;
            out.sv001 = valve_state;
            break;
        }
        case Valve_SV002: {
            out.has_sv002 = true;
            out.sv002 = valve_state;
            break;
        }
        case Valve_SV003: {
            out.has_sv003 = true;
            out.sv003 = valve_state;
            break;
        }
        case Valve_SV004: {
            out.has_sv004 = true;
            out.sv004 = valve_state;
            break;
        }
        case Valve_SV005: {
            out.has_sv005 = true;
            out.sv005 = valve_state;
            break;
        }
        case Valve_PBV006: {
            out.has_pbv006 = true;
            out.pbv006 = valve_state;
            break;
        }

        // LOx
        case Valve_PBV101: {
            out.has_pbv101 = true;
            out.pbv101 = valve_state;
            break;
        }

        // Fuel
        case Valve_PBV201: {
            out.has_pbv201 = true;
            out.pbv201 = valve_state;
            break;
        }

        // Purge
        case Valve_SV301: {
            out.has_sv301 = true;
            out.sv301 = valve_state;
            break;
        }

        // RCS
        case Valve_SVR001: {
            out.has_svr001 = true;
            out.svr001 = valve_state;
            break;
        }
        case Valve_SVR002: {
            out.has_svr002 = true;
            out.svr002 = valve_state;
            break;
        }
        case Valve_SVR003: {
            out.has_svr003 = true;
            out.svr003 = valve_state;
            break;
        }
        case Valve_SVR004: {
            out.has_svr004 = true;
            out.svr004 = valve_state;
            break;
        }

        default:
            return std::unexpected(Error::from_cause("invalid valve assignment in supposedly sanitized configs: %d", valve_config->assignment));
        }
    }

    return out;
}

/// Load a new set of valve configs. All valves must be powered off before the reconfiguration is accepted.
std::expected<void, Error> Valves::handle_configure_valves_request(const ConfigureValvesRequest& req)
{
    MutexGuard valves_guard{&valves_lock};

    // Enforce that all valves are in their normal states before we allow valve reconfiguration.
    for (int i = 0; i < _Valve_ARRAYSIZE; ++i) {
        const auto& config = valve_configs[i];
        const auto& state = valve_states[i];

        if (!config) {
            continue;
        }
        if (config->normally_closed && state != ValveState_CLOSED) {
            return std::unexpected(
                Error::from_cause("valve at channel %d (assigned %d) must be in its normal state of CLOSED before reconfiguration", config->assignment, i));
        }
        if (!config->normally_closed && state != ValveState_OPEN) {
            return std::unexpected(
                Error::from_cause("valve at channel %d (assigned %d) must be in its normal state of OPEN before reconfiguration", config->assignment, i));
        }
    }

    // Sanity check -- all GPIOs should be powered off now.
    for (int i = 0; i < NUM_VALVE_GPIOS; ++i) {
        int ret = gpio_pin_get_dt(&VALVE_GPIOS[i]);
        if (ret < 0) {
            return std::unexpected(Error::from_code(ret).context("failed to check state of gpio at channel %d", i));
        }
        if (ret != 0) {
            return std::unexpected(Error::from_cause("gpio at channel %d is not off (actual: %d)", ret));
        }
    }

    valve_configs.fill(std::nullopt);

    for (int i = 0; i < req.configs_count; ++i) {
        const auto& config = req.configs[i];

        if (config.channel >= NUM_VALVE_GPIOS) {
            return std::unexpected(Error::from_cause("invalid gpio channel %d, must be from 0 to %d", config.channel, NUM_VALVE_GPIOS - 1));
        }
        if (config.assignment < _Valve_MIN || config.assignment > _Valve_MAX) {
            return std::unexpected(Error::from_cause("invalid valve assignment %d, must be from %d to %d", config.assignment, _Valve_MIN, _Valve_MAX));
        }

        valve_configs[config.assignment] = std::make_optional(config);

        // Normalize normally_closed property.
        if (!valve_configs[config.assignment]->has_normally_closed) {
            valve_configs[config.assignment]->normally_closed = false;
        }

        // Populate states -- we just checked that all GPIOs are off, so configured valves should be in their normal states.
        if (valve_configs[config.assignment]->normally_closed) {
            valve_states[config.assignment] = ValveState_CLOSED;
        }
        else {
            valve_states[config.assignment] = ValveState_OPEN;
        }
    }

    return {};
}

/// Actuate a single valve.
std::expected<void, Error> Valves::handle_actuate_valve_request(const ActuateValveRequest& req)
{
    MutexGuard valves_guard{&valves_lock};

    if (req.valve < _Valve_MIN || req.valve > _Valve_MAX) {
        return std::unexpected(Error::from_cause("invalid valve assignment %d, must be from %d to %d", req.valve, _Valve_MIN, _Valve_MAX));
    }

    if (req.state != ValveState_OPEN && req.state != ValveState_CLOSED) {
        return std::unexpected(Error::from_cause("request must specify either OPEN or CLOSED, got %d", req.state));
    }

    // Ensure valve is configured to a GPIO
    const auto& config = valve_configs[req.valve];
    if (!config) {
        return std::unexpected(Error::from_cause("valve %d is not configured to any GPIO channel", req.valve));
    }

    // Check valve state
    const auto& valve_state = valve_states[req.valve];
    if (valve_state != ValveState_OPEN && valve_state != ValveState_CLOSED) {
        return std::unexpected(Error::from_cause("valve %d is not in any valid state, got: %d", req.valve, valve_state));
    }

    // Check if we even need to do any work
    if (valve_state == req.state) {
        return {};
    }

    // We in fact must actuate this valve.
    int target_state;
    if ((req.state == ValveState_CLOSED && config->normally_closed) || (req.state == ValveState_OPEN && !config->normally_closed)) {
        target_state = 0;
    }
    else {
        target_state = 1;
    }

    if (int err = gpio_pin_set_dt(&VALVE_GPIOS[config->channel], target_state); err) {
        return std::unexpected(
            Error::from_code(err).context("failed to set gpio %s (channel %d) as %d", VALVE_GPIOS[config->channel].port->name, config->channel, target_state));
    }
    valve_states[req.valve] = req.state;

    return {};
}
