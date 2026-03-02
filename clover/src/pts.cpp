#include "pts.h"

#include <array>
#include <zephyr/drivers/adc.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#define USER_NODE DT_PATH(zephyr_user)

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

#define ARTY_PTS_DT_SPEC_AND_COMMA(node_id, prop, idx) ADC_DT_SPEC_GET_BY_IDX(node_id, idx),
static constexpr struct adc_dt_spec adc_channels[NUM_PTS] = {
    DT_FOREACH_PROP_ELEM(USER_NODE, io_channels, ARTY_PTS_DT_SPEC_AND_COMMA)
};

static adc_sequence_options sequence_options = {
    .interval_us = 0,
    .extra_samplings = CONFIG_PT_SAMPLES - 1,
};
static adc_sequence sequence;
uint16_t raw_readings[CONFIG_PT_SAMPLES][NUM_PTS];

static pt_readings last_reading;
static int64_t last_read_uptime;

static struct k_poll_signal adc_async_signal = K_POLL_SIGNAL_INITIALIZER(adc_async_signal);
static bool async_pending = false;
static uint32_t async_start_cycles = 0;
static pts_metrics current_metrics = {};

LOG_MODULE_REGISTER(pts, CONFIG_LOG_DEFAULT_LEVEL);

consteval std::array<float, NUM_PTS> pts_adc_ranges()
{
    std::array<float, NUM_PTS> ranges;
    for (int i = 0; i < NUM_PTS; ++i) {
        ranges[i] = static_cast<float>(1 << static_cast<uint32_t>(adc_channels[i].resolution));
    }
    return ranges;
}

pt_config pt_configs[NUM_PTS] = {
    {.scale = 1000.0f / pts_adc_ranges()[0], .bias = -13.0f,  .range = 1000.0f},
    {.scale = 1000.0f / pts_adc_ranges()[1], .bias = -14.0f,  .range = 1000.0f},
    {.scale = 2000.0f / pts_adc_ranges()[2], .bias = -38.0f,  .range = 2000.0f},
    {.scale = 2000.0f / pts_adc_ranges()[3], .bias = -32.0f,  .range = 2000.0f},
    {.scale = 2000.0f / pts_adc_ranges()[4], .bias = -30.0f,  .range = 2000.0f},
    {.scale = 1000.0f / pts_adc_ranges()[5], .bias = -18.7f,  .range = 1000.0f},
    {.scale = 2000.0f / pts_adc_ranges()[6], .bias = -28.0f,  .range = 2000.0f},
    {.scale = 1000.0f / pts_adc_ranges()[7], .bias = -13.0f,  .range = 1000.0f},
};

static pt_readings pts_process_raw()
{
    float readings_by_idx[NUM_PTS] = {0};
    for (int i = 0; i < NUM_PTS; ++i) {
        for (int j = 0; j < CONFIG_PT_SAMPLES; ++j) {
            readings_by_idx[i] += static_cast<float>(raw_readings[j][i]);
        }
        readings_by_idx[i] = readings_by_idx[i] / CONFIG_PT_SAMPLES
                             * pt_configs[i].scale + pt_configs[i].bias;
    }

#define ARTY_PTS_DT_TO_READINGS_ASSIGNMENT(node_id, prop, idx) \
    .DT_STRING_TOKEN_BY_IDX(node_id, prop, idx) = readings_by_idx[idx],

    pt_readings readings = {
        DT_FOREACH_PROP_ELEM(USER_NODE, pt_names, ARTY_PTS_DT_TO_READINGS_ASSIGNMENT)
    };

    last_reading = readings;
    last_read_uptime = k_uptime_get();
    return readings;
}

std::expected<void, Error> pts_init()
{
    LOG_INF("Initializing ADC sequence");
    adc_sequence_init_dt(&adc_channels[0], &sequence);
    sequence.buffer = raw_readings;
    sequence.buffer_size = sizeof raw_readings;
    sequence.options = &sequence_options;

    for (int i = 0; i < NUM_PTS; i++) {
        LOG_INF("pt %d: Checking readiness", i);
        if (!adc_is_ready_dt(&adc_channels[i])) {
            LOG_ERR("pt %d: ADC controller device %s not ready", i, adc_channels[i].dev->name);
            return std::unexpected(Error::from_device_not_ready(adc_channels[i].dev)
                .context("ADC channel %d not ready during pts_init", i));
        }

        LOG_INF("pt %d: Initializing channel", i);
        int err = adc_channel_setup_dt(&adc_channels[i]);
        if (err) {
            LOG_ERR("pt %d: Failed to set up channel: err %d", i, err);
            return std::unexpected(Error::from_code(err)
                .context("failed to set up ADC channel %d during pts_init", i));
        }

        sequence.channels |= BIT(adc_channels[i].channel_id);
    }

    return {};
}

pt_readings pts_sample()
{
    int err = adc_read(adc_channels[0].dev, &sequence);
    if (err) {
        LOG_ERR("Failed to read from ADC: err %d", err);
        return pt_readings{};
    }
    return pts_process_raw();
}

pt_readings pts_get_last_reading()
{
    if (k_uptime_get() - last_read_uptime < 100) {
        return last_reading;
    }
    return pts_sample();
}

std::expected<void, Error> pts_set_bias(int index, float bias)
{
    if (index < 0 || index >= NUM_PTS) {
        LOG_ERR("Invalid PT index: %d", index);
        return std::unexpected(Error::from_cause("invalid PT index %d in pts_set_bias (valid: 0-%d)", index, NUM_PTS - 1));
    }
    pt_configs[index].bias = bias;
    return {};
}

std::expected<void, Error> pts_set_range(int index, float range)
{
    if (index < 0 || index >= NUM_PTS) {
        LOG_ERR("Invalid PT index: %d", index);
        return std::unexpected(Error::from_cause("invalid PT index %d in pts_set_range (valid: 0-%d)", index, NUM_PTS - 1));
    }
    pt_configs[index].range = range;
    pt_configs[index].scale = range / pts_adc_ranges()[index];
    return {};
}

void pts_request_async()
{
    if (async_pending) {
        LOG_WRN("pts_request_async: previous read still pending, skipping");
        return;
    }

    k_poll_signal_reset(&adc_async_signal);
    async_start_cycles = k_cycle_get_32();
    async_pending = true;

    int err = adc_read_async(adc_channels[0].dev, &sequence, &adc_async_signal);
    if (err) {
        LOG_ERR("pts_request_async: adc_read_async failed: %d", err);
        async_pending = false;
    }
}

bool pts_try_collect(pt_readings* out)
{
    if (!async_pending) {
        return false;
    }

    unsigned int signaled; // FIX: Changed from int to unsigned int
    int result;
    k_poll_signal_check(&adc_async_signal, &signaled, &result);

    if (!signaled) {
        return false;
    }

    async_pending = false;

    uint32_t end_cycles = k_cycle_get_32();
    current_metrics.last_latency_us = k_cyc_to_us_near32(end_cycles - async_start_cycles);

    if (result != 0) {
        LOG_ERR("pts_try_collect: async ADC read returned error: %d", result);
        return false;
    }

    *out = pts_process_raw();
    return true;
}

pts_metrics pts_get_metrics()
{
    return current_metrics;
}
