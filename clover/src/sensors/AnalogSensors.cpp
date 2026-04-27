#include "AnalogSensors.h"
#include "../MutexGuard.h"
#include "clover.pb.h"
#include "../config.h"
#include "../lut/tc_k_type_v_to_deg_c_lut.h"
#include "../lut/tc_t_type_v_to_deg_c_lut.h"
#include <array>
#include <optional>
#include <tuple>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

// Array of each ADC device
static constexpr int NUM_ADCS = DT_PROP_LEN(DT_PATH(zephyr_user), analog_sensor_adcs);
static constexpr std::array<const device*, NUM_ADCS> adc_devices = {
    DT_FOREACH_PROP_ELEM_SEP(DT_PATH(zephyr_user), analog_sensor_adcs, DEVICE_DT_GET_BY_IDX, (, ))};

static constexpr int OVERSAMPLING = DT_PROP(DT_PATH(zephyr_user), analog_sensor_adc_oversampling);
static constexpr int RESOLUTION = DT_PROP(DT_PATH(zephyr_user), analog_sensor_adc_resolution);

// Subset of ADC channels we make available to be assigned to PTs/TCs.
static constexpr int NUM_ANALOG_CHANNELS = DT_PROP_LEN(DT_PATH(zephyr_user), io_channels);
#define LAMBDA(node_id, prop, idx) ADC_DT_SPEC_GET_BY_IDX(node_id, idx)
// For some ungodly reason we cannot use a std::array here.
static constexpr adc_dt_spec adc_channels[NUM_ANALOG_CHANNELS] = {DT_FOREACH_PROP_ELEM_SEP(DT_PATH(zephyr_user), io_channels, LAMBDA, (, ))};
#undef LAMBDA

// Spec and buffers used for ADC reads.

// Generates an ADC channel bitmask from the devicetree config.
static consteval uint32_t channels_bitmask(const device* adc_dev)
{
    uint32_t mask = 0;
    for (int i = 0; i < NUM_ANALOG_CHANNELS; ++i) {
        const auto& channel = adc_channels[i];
        if (channel.dev == adc_dev) {
            mask |= (1 << channel.channel_id);
        }
    }
    return mask;
}

/// Buffer into which raw readings are written. Only accessed by the analog_sensors thread.
static std::array<std::array<uint16_t, DT_PROP(DT_PATH(zephyr_user), analog_sensor_max_adc_channels)>, NUM_ADCS> raw_readings;

/// Maps ADC and ADC channel into overall channel index
static std::array<std::array<int, DT_PROP(DT_PATH(zephyr_user), analog_sensor_max_adc_channels)>, NUM_ADCS> adc_channel_to_input_channel = []() consteval {
    std::array<std::array<int, DT_PROP(DT_PATH(zephyr_user), analog_sensor_max_adc_channels)>, NUM_ADCS> out;
    for (int i = 0; i < NUM_ADCS; ++i) {
        const device* adc_dev = adc_devices[i];
        int reading_index = 0;

        for (int j = 0; j < NUM_ANALOG_CHANNELS; ++j) {
            const auto& channel = adc_channels[j];
            if (channel.dev == adc_dev) {
                out[i][reading_index] = j;
                ++reading_index;
            }
            else {
                out[i][reading_index] = -1;
            }
        }
    }
    return out;
}();

/// Sequence read options
static adc_sequence_options adc_read_options = {
    .interval_us = 0,
    .extra_samplings = 0,  // No multi-sampling
};

/// Sequence spec used to read from ADC. Only accessed by the analog_sensors thread.
static std::array<adc_sequence, NUM_ADCS> adc_read_seqs = []() consteval {
    std::array<adc_sequence, NUM_ADCS> out;
    for (int i = 0; i < NUM_ADCS; ++i) {
        const device* adc_dev = adc_devices[i];

        // Determine ADC channels we want to read from
        size_t used_channels = 0;
        uint32_t mask = 0;
        for (int j = 0; j < NUM_ANALOG_CHANNELS; ++j) {
            const auto& channel = adc_channels[j];
            if (channel.dev == adc_dev) {
                mask |= (1 << channel.channel_id);
                used_channels++;
            }
        }

        out[i] = {
            .options = &adc_read_options,
            .channels = mask,
            .buffer = raw_readings[i].data(),
            .buffer_size = used_channels * sizeof(uint16_t),
            .resolution = RESOLUTION,
            .oversampling = OVERSAMPLING};
    }
    return out;
}();

/// Signals upon which ADCs will notify when their read is complete
std::array<k_poll_signal, NUM_ADCS> read_signals;
std::array<k_poll_event, NUM_ADCS> read_events;
#undef LAMBDA

/// Signals readiness
K_SEM_DEFINE(ready_sem, 0, 1);

/// Taken every sensor read to coordinate read times with controller ticks.
K_SEM_DEFINE(allow_sense_sem, 0, 1);

/// Sensor configs. Each AnalogSensorConfig is validated, and corresponds to the channel of the corresponding index.
constexpr int MAX_SENSOR_CONFIG_LEN = sizeof(static_cast<ConfigureAnalogSensorsRequest*>(nullptr)->configs);
static std::array<std::optional<AnalogSensorConfig>, NUM_ANALOG_CHANNELS> sensor_configs;
K_MUTEX_DEFINE(config_mutex);

/// Sensor outputs
static bool has_reading = false;
static AnalogSensorReadings sensor_readings = AnalogSensorReadings_init_default;
static float sense_time_ns = 0.0f;
K_MUTEX_DEFINE(reading_mutex);

LOG_MODULE_REGISTER(AnalogSensors, CONFIG_LOG_DEFAULT_LEVEL);

/// Continuously sense, coordinating with control ticks so the reading is ready at the start of each one.
static void sense()
{
    // Await initialization
    k_sem_take(&ready_sem, K_FOREVER);
    LOG_INF("Sense loop initiated");

    while (true) {
        // Single read is allowed at the end of a controller tick.
        k_sem_take(&allow_sense_sem, K_FOREVER);

        // Read from all ADCs concurrently
        uint64_t start_read_cycle = k_cycle_get_64();
        for (int i = 0; i < NUM_ADCS; ++i) {
            k_poll_signal_init(&read_signals[i]);
            int err = adc_read_async(adc_devices[i], &adc_read_seqs[i], &read_signals[i]);
            if (err) {
                LOG_ERR("Error initiating async read of ADC %s: %s", adc_devices[i]->name, Error::from_code(err).build_message().c_str());
            }
        }

        int num_complete = 0;
        while (true) {
            // Wait for an ADC to finish their read.
            k_poll(read_events.data(), NUM_ADCS, K_FOREVER);

            // Check for which ADCs have finished a read.
            for (int i = 0; i < NUM_ADCS; ++i) {
                uint32_t signaled;
                int result;
                k_poll_signal_check(&read_signals[i], &signaled, &result);
                if (signaled == 0) {
                    continue;
                }

                // Reset signal to prevent it from re-triggering poll.
                read_events[i].state = K_POLL_STATE_NOT_READY;
                k_poll_signal_reset(&read_signals[i]);
                num_complete++;

                // Inspect result of read call.
                if (result != 0) {
                    LOG_ERR("Error during async read of ADC %s: %s", adc_devices[i]->name, Error::from_code(result).build_message().c_str());
                }
            }

            // Check if everybody has signaled completion.
            if (num_complete == NUM_ADCS) {
                break;
            }
        }

        // Write output reading
        {
            MutexGuard reading_guard{&reading_mutex};
            MutexGuard config_guard{&config_mutex};

            sense_time_ns = static_cast<float>(k_cycle_get_64() - start_read_cycle) / sys_clock_hw_cycles_per_sec() * 1e9f;

            has_reading = true;

            for (int i = 0; i < NUM_ADCS; ++i) {
                for (int j = 0; j < static_cast<int>(adc_read_seqs[i].buffer_size); ++j) {
                    int input_channel = adc_channel_to_input_channel[i][j];

                    if (input_channel == -1) {
                        // No more reading left for this ADC
                        break;
                    }

                    auto maybe_config = sensor_configs[input_channel];
                    if (!maybe_config) {
                        // Channel was not configured
                        continue;
                    }
                    // This config is pre-validated.
                    const AnalogSensorConfig& config = *maybe_config;

                    float raw_reading = raw_readings[i][j];
                    float curr_reading;
                    // Sensor is a PT
                    if (config.has_pt_bias_psig) {
                        curr_reading = raw_reading / static_cast<float>(1 << RESOLUTION) * config.pt_range_psig + config.pt_bias_psig;
                    }

                    // Sensor is a TC. Note that this code is specialized for our own TC amp modules.
                    else if (config.has_tc_type) {
                        // Using NIST conversion from here: https://www.analog.com/en/resources/app-notes/an-1087.html
                        // TC signal is amplified by AD8495.

                        // Convert raw reading to 5V amplified signal
                        float reading_v = raw_reading / static_cast<float>(1 << RESOLUTION) * 5.0f;

                        // Convert amplified signal back to unamplified TC signal
                        constexpr float V_REF = 1.25f;
                        float unamplified_v = (reading_v - V_REF - 0.00125f) / 122.4f;

                        switch (config.tc_type) {
                        case TCType_K_TYPE: {
                            // Reading is from -200 to 400 deg C
                            curr_reading = TcKTypeVToDegCLut::sample(unamplified_v);
                            break;
                        }
                        case TCType_T_TYPE: {
                            // Reading is from -200 to 50 deg C
                            curr_reading = TcTTypeVToDegCLut::sample(unamplified_v);
                            break;
                        }
                        default: {
                            // Impossible as we pre-validated the configs.
                            LOG_ERR("Config has no valid TC type set despite it being validated, got: %d", config.tc_type);
                            curr_reading = -9999.0f;
                            break;
                        }
                        }
                    }

                    // Sensor is raw
                    else if (config.has_raw_bias_v) {
                        curr_reading = static_cast<float>(raw_reading) / static_cast<float>(1 << RESOLUTION) * config.raw_range_v + config.raw_bias_v;
                    }

                    else {
                        LOG_ERR("Config has no sensor type set despite it being validated");
                        continue;
                    }

                    // Write sensor reading, configs should be validated to prevent overwriting.
                    switch (config.assignment) {
                    // Vehicle pressurant
                    case AnalogSensor_PT001: {
                        sensor_readings.has_pt001 = true;
                        sensor_readings.pt001 = curr_reading;
                        break;
                    }
                    case AnalogSensor_PT002: {
                        sensor_readings.has_pt002 = true;
                        sensor_readings.pt002 = curr_reading;
                        break;
                    }
                    case AnalogSensor_PT003: {
                        sensor_readings.has_pt003 = true;
                        sensor_readings.pt003 = curr_reading;
                        break;
                    }
                    case AnalogSensor_PT004: {
                        sensor_readings.has_pt004 = true;
                        sensor_readings.pt004 = curr_reading;
                        break;
                    }
                    case AnalogSensor_PT005: {
                        sensor_readings.has_pt005 = true;
                        sensor_readings.pt005 = curr_reading;
                        break;
                    }
                    case AnalogSensor_PT006: {
                        sensor_readings.has_pt006 = true;
                        sensor_readings.pt006 = curr_reading;
                        break;
                    }
                    // Vehicle fuel side
                    case AnalogSensor_PT103: {
                        sensor_readings.has_pt103 = true;
                        sensor_readings.pt103 = curr_reading;
                        break;
                    }
                    // Vehicle LOx side
                    case AnalogSensor_PT203: {
                        sensor_readings.has_pt203 = true;
                        sensor_readings.pt203 = curr_reading;
                        break;
                    }
                    // Purge
                    case AnalogSensor_PT301: {
                        sensor_readings.has_pt301 = true;
                        sensor_readings.pt301 = curr_reading;
                        break;
                    }
                    // Engine
                    case AnalogSensor_PTF401: {
                        sensor_readings.has_ptf401 = true;
                        sensor_readings.ptf401 = curr_reading;
                        break;
                    }
                    case AnalogSensor_PTO401: {
                        sensor_readings.has_pto401 = true;
                        sensor_readings.pto401 = curr_reading;
                        break;
                    }
                    case AnalogSensor_PTC401: {
                        sensor_readings.has_ptc401 = true;
                        sensor_readings.ptc401 = curr_reading;
                        break;
                    }
                    case AnalogSensor_PTC402: {
                        sensor_readings.has_ptc402 = true;
                        sensor_readings.ptc402 = curr_reading;
                        break;
                    }
                    // TCs
                    case AnalogSensor_TC002: {
                        sensor_readings.has_tc002 = true;
                        sensor_readings.tc002 = curr_reading;
                        break;
                    }
                    case AnalogSensor_TC102: {
                        sensor_readings.has_tc102 = true;
                        sensor_readings.tc102 = curr_reading;
                        break;
                    }
                    case AnalogSensor_TC102_5: {
                        sensor_readings.has_tc102_5 = true;
                        sensor_readings.tc102_5 = curr_reading;
                        break;
                    }
                    case AnalogSensor_TCF401: {
                        sensor_readings.has_tcf401 = true;
                        sensor_readings.tcf401 = curr_reading;
                        break;
                    }
                    case AnalogSensor_TCO401: {
                        sensor_readings.has_tco401 = true;
                        sensor_readings.tco401 = curr_reading;
                        break;
                    }
                    // GSE pressurant
                    case AnalogSensor_PTG001: {
                        sensor_readings.has_ptg001 = true;
                        sensor_readings.ptg001 = curr_reading;
                        break;
                    }
                    case AnalogSensor_PTG002: {
                        sensor_readings.has_ptg002 = true;
                        sensor_readings.ptg002 = curr_reading;
                        break;
                    }
                    // GSE LOx
                    case AnalogSensor_PTG101: {
                        sensor_readings.has_ptg101 = true;
                        sensor_readings.ptg101 = curr_reading;
                        break;
                    }
                    // Hornet
                    case AnalogSensor_BATTERY_VOLTAGE: {
                        sensor_readings.has_battery_voltage = true;
                        sensor_readings.battery_voltage = curr_reading;
                        break;
                    }

                    // Impossible as we validated configs beforehand.
                    default: {
                        LOG_ERR("Got invalid sensor assignment %d despite us validating configs beforehand", config.assignment);
                        break;
                    }
                    }
                }
            }
        }
    }
}

K_THREAD_DEFINE(analog_sensors, 2048, sense, nullptr, nullptr, nullptr, ANALOG_SENSORS_THREAD_PRIORITY, 0, 0);

/// Check ADC readiness and initialize all ADC channels we're using.
std::expected<void, Error> AnalogSensors::init()
{
    LOG_INF("Checking ADC readiness");
    for (const device* dev : adc_devices) {
        if (!device_is_ready(dev)) {
            return std::unexpected(Error::from_device_not_ready(dev).context("AnalogSensors ADC is not ready"));
        }
    }

    // Configure ADC channels.
    for (int i = 0; i < NUM_ANALOG_CHANNELS; ++i) {
        const auto& channel = adc_channels[i];
        LOG_INF("Initializing channel %d (ADC channel %d) on device %s", i, channel.channel_id, channel.dev->name);

        if (int err = adc_channel_setup_dt(&channel)) {
            return std::unexpected(Error::from_code(err).context("failed to set up ADC channel %d", channel.channel_id));
        }
    }

    // Initialize read_events
    for (int i = 0; i < NUM_ADCS; ++i) {
        k_poll_event_init(&read_events[i], K_POLL_TYPE_SIGNAL, K_POLL_MODE_NOTIFY_ONLY, &read_signals[i]);
    }

    LOG_INF("Initiating sense loop");
    k_sem_give(&ready_sem);

    return {};
}

/// Sets configs for all analog sensors, validating them to ensure its correct.
std::expected<void, Error> AnalogSensors::handle_configure_analog_sensors(const ConfigureAnalogSensorsRequest& req)
{
    std::array<bool, NUM_ANALOG_CHANNELS> assigned_channels{};
    assigned_channels.fill(false);
    std::array<bool, _AnalogSensor_ARRAYSIZE> assigned_sensors{};
    assigned_sensors.fill(false);

    for (int i = 0; i < req.configs_count; ++i) {
        const AnalogSensorConfig& config = req.configs[i];

        // Check channel
        if (config.channel >= NUM_ANALOG_CHANNELS) {
            return std::unexpected(Error::from_cause("invalid channel %d", config.channel));
        }
        if (assigned_channels[config.channel]) {
            return std::unexpected(Error::from_cause("channel %d is assigned multiple times", config.channel));
        }
        assigned_channels[config.channel] = true;

        // Check assignment
        if (config.assignment < _AnalogSensor_MIN || config.assignment > _AnalogSensor_MAX) {
            return std::unexpected(Error::from_cause("invalid sensor assignment %d", config.assignment));
        }
        if (assigned_sensors[config.assignment]) {
            return std::unexpected(Error::from_cause("sensor %d is assigned multiple times", config.channel));
        }
        assigned_sensors[config.assignment] = true;

        bool has_pt_config = config.has_pt_bias_psig || config.has_pt_range_psig;
        bool has_tc_config = config.has_tc_type;
        bool has_raw_config = config.has_raw_range_v || config.has_raw_bias_v;

        // Only one sensor type should be set
        if (has_pt_config + has_tc_config + has_raw_config != 1) {
            return std::unexpected(Error::from_cause("sensor config must configure only one config type (pt, tc, or raw)"));
        }

        // Check PT config
        if (has_pt_config) {
            if (!(config.has_pt_range_psig && config.has_pt_bias_psig)) {
                return std::unexpected(Error::from_cause("all pt config fields must be filled"));
            }
            if (config.pt_range_psig == 0.0f) {
                return std::unexpected(Error::from_cause("pt range cannot be 0"));
            }
        }

        // Check TC config
        if (has_tc_config) {
            if (config.tc_type < _TCType_MIN || config.tc_type > _TCType_MAX) {
                return std::unexpected(Error::from_cause("invalid tc type %d", config.tc_type));
            }
        }

        // Check raw config
        if (has_raw_config) {
            if (!(config.has_raw_bias_v && config.has_raw_range_v)) {
                return std::unexpected(Error::from_cause("all raw config fields must be filled"));
            }
            if (config.raw_range_v == 0.0f) {
                return std::unexpected(Error::from_cause("raw range cannot be 0"));
            }
        }
    }

    {
        MutexGuard config_guard{&config_mutex};

        // Reset config assignments
        sensor_configs.fill(std::nullopt);

        // For assigned channels, put the provided config
        for (int i = 0; i < req.configs_count; ++i) {
            sensor_configs[req.configs[i].channel] = std::make_optional(req.configs[i]);
        }
    }

    return {};
}

/// Called from at the end of a control tick to allow AnalogSensors to initiate a sensor reading.
void AnalogSensors::start_sense()
{
    // Semaphore count is only non-zero if sense is taking longer than a controller tick and is
    // still in-progress. In this case, we guard against giving multiple semaphores and just allow
    // it to immediately re-sense when it finishes its current job.
    if (k_sem_count_get(&allow_sense_sem) == 0) {
        k_sem_give(&allow_sense_sem);
    }
}

/// Returns the last read PTs, TCs, and sense time, if it is ready.
std::optional<std::pair<AnalogSensorReadings, float>> AnalogSensors::read()
{
    MutexGuard reading_guard{&reading_mutex};

    if (!has_reading) {
        return std::nullopt;
    }

    // Consume reading
    has_reading = false;

    return {{sensor_readings, sense_time_ns}};
}
