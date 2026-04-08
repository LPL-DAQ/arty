#include "AnalogSensors.h"
#include "MutexGuard.h"
#include "clover.pb.h"
#include "config.h"
#include <optional>
#include <tuple>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

// Array of each ADC device
static constexpr int NUM_ADCS = DT_PROP_LEN(DT_PATH(zephyr_user), analog_sensor_adcs);
static const std::array<device*, NUM_ADCS> adc_devices = {DT_FOREACH_PROP_ELEM_SEP(DT_PATH(zephyr_user), analog_sensor_adcs, DEVICE_DT_GET_BY_IDX, (, ))};

static constexpr int OVERSAMPLING = DT_PROP(DT_PATH(zephyr_user), analog_sensor_adc_oversampling);
static constexpr int RESOLUTION = DT_PROP(DT_PATH(zephyr_user), analog_sensor_adc_resolution);

// Subset of ADC channels we make available to be assigned to PTs/TCs.
static constexpr int NUM_ANALOG_CHANNELS = DT_PROP_LEN(DT_PATH(zephyr_user), analog_sensor_channels);
#define LPL_ANALOG_SENSORS_ADC_CHANNELS_INIT(node_id, prop, idx) \
    ADC_DT_SPEC_STRUCT(DT_PHA_BY_IDX(node_id, prop, idx, ctlr), DT_PHA_BY_IDX(node_id, prop, idx, input))
static constexpr std::array<adc_dt_spec, NUM_ANALOG_CHANNELS> adc_channels = {
    DT_FOREACH_PROP_ELEM_SEP(DT_PATH(zephyr_user), analog_sensor_channels, LPL_ANALOG_SENSORS_ADC_CHANNELS_INIT, (, ))};
#undef LPL_ANALOG_SENSORS_ADC_CHANNELS_INIT

// Spec and buffers used for ADC reads.

// Generates an ADC channel bitmask from the devicetree config.
static consteval uint32_t channels_bitmask(const device* adc_dev)
{
    uint32_t mask = 0;
    for (const auto& channel : adc_channels) {
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
            const auto& channel = adc_channels[i];
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
            const auto& channel = adc_channels[i];
            if (channel.dev == adc_dev) {
                mask |= (1 << channel.channel_id);
                used_channels++;
            }
        }
        return mask;

        out[i] = {
            .options = &adc_read_options,
            .channels = channels_bitmask(adc_dev),
            .buffer = raw_readings[i].data(),
            .buffer_size = used_channels * sizeof(uint16_t),
            .resolution = RESOLUTION,
            .oversampling = OVERSAMPLING};
    }
    return out;
}();

/// Signals upon which ADCs will notify when their read is complete
std::array<k_poll_signal, NUM_ADCS> read_signals;
#define LAMBDA(node_id, prop, idx) K_POLL_EVENT_STATIC_INITIALIZER(K_POLL_TYPE_SIGNAL, K_POLL_MODE_NOTIFY_ONLY, &read_signals[idx], 0),
std::array<k_poll_event, NUM_ADCS> read_events{DT_FOREACH_PROP_ELEM_SEP(DT_PATH(zephyr_user), analog_sensor_adcs, LAMBDA, (, ))};
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
        for (int i = 0; i < NUM_ADCS; ++i) {
            k_poll_signal_init(&read_signals[i]);
            int err = adc_read_async(adc_devices[i], &adc_read_seqs[i], &read_signals[i]);
            if (err) {
                LOG_ERR("Error initiating async read of ADC %s: %s", adc_devices[i]->name, Error::from_code(err).build_message());
            }
        }

        k_poll(read_events.data(), NUM_ADCS, K_FOREVER);

        // Check read statuses
        for (int i = 0; i < NUM_ADCS; ++i) {
            uint32_t signaled;
            int result;
            k_poll_signal_check(&read_signals[i], &signaled, &result);
            if (signaled == 0) {
                LOG_ERR("ADC %s was not siganled: %d", adc_devices[i]->name, signaled);
            }
            if (result != 0) {
                LOG_ERR("Error during async read of ADC %s: %s", adc_devices[i]->name, Error::from_code(result).build_message());
            }
        }

        // Write output reading
        {
            MutexGuard reading_guard{&reading_mutex};
            MutexGuard config_guard{&config_mutex};

            for (int i = 0; i < NUM_ADCS; ++i) {
                for (int j = 0; j < adc_read_seqs[i].buffer_size) {
                    int input_channel = adc_channel_to_input_channel[i][j];

                    if (input_channel == -1) {
                        // No more reading left for this ADC
                        break;
                    }

                    auto maybe_config = sensor_configs[input_channel];
                    if (!maybe_config) {
                        // Channel was not configured
                        break;
                    }
                    // This config is pre-validated.
                    const AnalogSensorConfig& config = *maybe_config;

                    float raw_reading = raw_readings[i][j];
                    float reading;
                    // Sensor is a PT
                    if (config.has_pt_bias_psig) {
                        reading = raw_reading / static_cast<float>(1 << RESOLUTION) * config.pt_range_psig + config.pt_bias_psig;
                    }

                    // Sensor is a TC. Note that this code is specialized for our own TC amp modules.
                    else if (config.has_tc_type) {
                        // Using NIST conversion from here: https://www.analog.com/en/resources/app-notes/an-1087.html
                        // TC signal is amplified by AD8495.

                        // Convert raw reading to 5V amplified signal
                        float reading_v = raw_reading / static_cast<float>(1 << RESOLUTION) * 5.0f;

                        // Convert amplified signal back to unamplified TC signal
                        constexpr float V_REF = 1.25;
                        float unamplified_v =

                        switch (config.tc_type) {
                        case TCType_K_TYPE: {
                        }
                        case TCType_T_TYPE: {
                        }
                        default: {
                            LOG_ERR("Config has no valid TC type set despite it being validated");
                        }
                        }
                    }

                    // Sensor is raw
                    else if (config.has_raw_bias_v) {
                        reading = static_cast<float>(raw_reading) / static_cast<float>(1 << RESOLUTION) * config.raw_range_v + config.raw_bias_v;
                    }

                    else {
                        LOG_ERR("Config has no sensor type set despite it being validated");
                    }
                }
            }

            has_reading = true;
            pts.has_pt102 = true;
            pts.pt102 = 67.0f;
            // TODO
            // and so on
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
    for (const auto& channel : adc_channels) {
        LOG_INF("Initializing channel %d", channel.channel_id);

        if (int err = adc_channel_setup_dt(&channel)) {
            return std::unexpected(Error::from_code(err).context("failed to set up ADC channel %d", channel.channel_id));
        }
    }

    LOG_INF("Initiating sense loop");
    k_sem_give(&ready_sem);

    return {};
}

/// Sets configs for all analog sensors, validating them to ensure its correct.
std::expected<void, Error> AnalogSensors::handle_configure_analog_sensors(const ConfigureAnalogSensorsRequest& req)
{
    std::array<bool, NUM_ANALOG_CHANNELS> assigned_channels;
    assigned_channels.fill(false);
    std::array<bool, _AnalogSensor_ARRAYSIZE> assigned_sensors;
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
            if (!(config.has_pt_range_psig && config.has_pt_range_psig)) {
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
    return {{sensor_readings, sense_time_ns}};
    // return std::make_optional(std::make_pair(sensor_readings, sense_time_ns));
}
