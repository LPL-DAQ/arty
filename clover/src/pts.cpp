#include "pts.h"

#include <array>
#include <zephyr/drivers/adc.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#define USER_NODE DT_PATH(zephyr_user)

// Validate devicetree
#if !DT_NODE_HAS_PROP(USER_NODE, pt_names)
#error "pts: Missing `pt-names` property from `zephyr-user` node."
#endif

#if !DT_NODE_HAS_PROP(USER_NODE, io_channels)
#error "pts: Missing `io-channels` property from `zephyr-user` node."
#endif

#if DT_PROP_LEN(USER_NODE, pt_names) != DT_PROP_LEN(USER_NODE, io_channels)
#error "pts: `pt-names` and `io-channels` must have the same length."
#endif

#define CONFIG_PT_SAMPLES 1

// Trailing comma needed as we are using preprocessor to instantiate each element of an array.
#define ARTY_PTS_DT_SPEC_AND_COMMA(node_id, prop, idx) ADC_DT_SPEC_GET_BY_IDX(node_id, idx),
static constexpr struct adc_dt_spec adc_channels[NUM_PTS] = {DT_FOREACH_PROP_ELEM(USER_NODE, io_channels, ARTY_PTS_DT_SPEC_AND_COMMA)};

static adc_sequence_options sequence_options = {
    .interval_us = 0,
    .extra_samplings = CONFIG_PT_SAMPLES - 1,
};
static adc_sequence sequence;
uint16_t raw_readings[CONFIG_PT_SAMPLES][NUM_PTS];

static pt_readings last_reading;
static int64_t last_read_uptime;

LOG_MODULE_REGISTER(pts, CONFIG_LOG_DEFAULT_LEVEL);

consteval std::array<float, NUM_PTS> pts_adc_ranges() {
    std::array<float, NUM_PTS> ranges;
    for (int i = 0; i < NUM_PTS; ++i) {
        ranges[i] = static_cast<float>(1 << static_cast<uint32_t>(adc_channels[i].resolution));
    }
    return ranges;
}

pt_config pt_configs[NUM_PTS] = {
    {.scale = 1000.0f / pts_adc_ranges()[0], .bias = -13.0f, .range = 1000.0f},
    {.scale = 1000.0f / pts_adc_ranges()[1], .bias = -14.0f, .range = 1000.0f},
    {.scale = 2000.0f / pts_adc_ranges()[2], .bias = -38.0f, .range = 2000.0f},
    {.scale = 2000.0f / pts_adc_ranges()[3], .bias = -32.0f, .range = 2000.0f},
    {.scale = 2000.0f / pts_adc_ranges()[4], .bias = -30.0f, .range = 2000.0f},
    {.scale = 1000.0f / pts_adc_ranges()[5], .bias = -18.7f, .range = 1000.0f},
    {.scale = 2000.0f / pts_adc_ranges()[6], .bias = -28.0f, .range = 2000.0f},
    {.scale = 1000.0f / pts_adc_ranges()[7], .bias = -13.0f, .range = 1000.0f},
};

/// Initialize PT sensors by initializing the ADC they're all connected to.
int pts_init() {
    // Initializes resolution and oversampling from device tree. Let's assume all channels share those properties. Also
    // initializes `channels` with just one channel, so we overwrite that later to sample all channels at once.
    LOG_INF("Initializing ADC sequence");
    adc_sequence_init_dt(&adc_channels[0], &sequence);
    sequence.buffer = raw_readings;
    sequence.buffer_size = sizeof raw_readings;
    sequence.options = &sequence_options;

    // Configure ADC channels.
    for (int i = 0; i < NUM_PTS; i++) {
        LOG_INF("pt %d: Checking readiness", i);
        if (!adc_is_ready_dt(&adc_channels[i])) {
            LOG_ERR("pt %d: ADC controller device %s not ready", i, adc_channels[i].dev->name);
            return 1;
        }

        LOG_INF("pt %d: Initializing channel", i);
        int err = adc_channel_setup_dt(&adc_channels[i]);
        if (err) {
            LOG_ERR("pt %d: Failed to set up channel: err %d", i, err);
            return 1;
        }

        // Request reading for this channel in sequence.
        sequence.channels |= BIT(adc_channels[i].channel_id);
    }

    return 0;
}

/// Update PT sample readings.
pt_readings pts_sample() {
    int err = adc_read(adc_channels[0].dev, &sequence);
    if (err) {
        LOG_ERR("Failed to read from ADC: err %d", err);
        return pt_readings{};
    }

    // Averaging readings
    float readings_by_idx[NUM_PTS] = {0};
    for (int i = 0; i < NUM_PTS; ++i) {
        for (int j = 0; j < CONFIG_PT_SAMPLES; ++j) {
            readings_by_idx[i] += static_cast<float>(raw_readings[j][i]);
        }
        readings_by_idx[i] = readings_by_idx[i] / CONFIG_PT_SAMPLES * pt_configs[i].scale + pt_configs[i].bias;
    }

    // Assign each PT name as fields to initialize pt_readings
#define ARTY_PTS_DT_TO_READINGS_ASSIGNMENT(node_id, prop, idx) .DT_STRING_TOKEN_BY_IDX(node_id, prop, idx) = readings_by_idx[idx],
    auto readings = pt_readings{DT_FOREACH_PROP_ELEM(USER_NODE, pt_names, ARTY_PTS_DT_TO_READINGS_ASSIGNMENT)};
    last_reading = readings;
    last_read_uptime = k_uptime_get();
    return readings;
}

/// Gets the last sample if it's sufficiently recent, or resample manually.
pt_readings pts_get_last_reading() {
    // If last PT reading was less than 100 ms ago
    if (k_uptime_get() - last_read_uptime < 100) {
        return last_reading;
    }
    return pts_sample();
}

int pts_set_bias(int index, float bias) {
    if (index < 0 || index >= NUM_PTS) {
        LOG_ERR("Invalid PT index: %d", index);
        return 1;
    }

    pt_configs[index].bias = bias;

    return 0;
}

int pts_set_range(int index, float range) {
    if (index < 0 || index >= NUM_PTS) {
        LOG_ERR("Invalid PT index: %d", index);
        return 1;
    }

    pt_configs[index].range = range;
    pt_configs[index].scale = range / pts_adc_ranges()[index];

    return 0;
}
