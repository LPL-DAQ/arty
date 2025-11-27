#include "pts.h"

#include <zephyr/sys/util.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <array>

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
#define CLOVER_PTS_DT_SPEC_AND_COMMA(node_id, prop, idx) ADC_DT_SPEC_GET_BY_IDX(node_id, idx),
static constexpr struct adc_dt_spec adc_channels[NUM_PTS] = {
        DT_FOREACH_PROP_ELEM(USER_NODE, io_channels, CLOVER_PTS_DT_SPEC_AND_COMMA)
};

static adc_sequence_options sequence_options = {
        .interval_us = 0,
        .extra_samplings = CONFIG_PT_SAMPLES - 1,
};
static adc_sequence sequence;
uint16_t raw_readings[CONFIG_PT_SAMPLES][NUM_PTS];

LOG_MODULE_REGISTER(pts, CONFIG_LOG_DEFAULT_LEVEL);

constexpr std::array<float, NUM_PTS> pts_adc_ranges() {
    std::array<float, NUM_PTS> ranges;
    for (int i = 0; i < NUM_PTS; ++i) {
        ranges[i] = static_cast<float>(1 << static_cast<uint32_t>(adc_channels[i].resolution));
    }
    return ranges;
}

pt_config pt_configs[NUM_PTS] = {
        {
                .scale = 2000.0f / pts_adc_ranges()[0],
                .bias = 0.0f,
                .range =2000.0f
        },
        {
                .scale = 2000.0f / pts_adc_ranges()[1],
                .bias = 0.0f,
                .range = 2000.0f
        },
        {
                .scale = 2000.0f / pts_adc_ranges()[2],
                .bias = 0.0f,
                .range = 2000.0f
        },
        {
                .scale = 2000.0f / pts_adc_ranges()[3],
                .bias = 0.0f,
                .range = 2000.0f
        },
};

// --- Moving-average support -------------------------------------------------

// Max number of samples we keep for moving-average calculations.
constexpr int PTS_MOVING_AVG_MAX_SAMPLES = 128;
static int pts_mavg_window = 10;

// pts_history[sample_index][pt_index]
static std::array<std::array<float, NUM_PTS>, PTS_MOVING_AVG_MAX_SAMPLES> pts_history{};
static int pts_history_size = 0;   // number of valid samples in history
static int pts_history_index = 0;  // index of next slot to overwrite in ring buffer

// Low-level helper: read from ADC and convert into scaled readings_by_idx[]
static int pts_read_and_convert(float readings_by_idx[NUM_PTS]) {
    int err = adc_read(adc_channels[0].dev, &sequence);
    if (err) {
        LOG_ERR("Failed to read from ADC: err %d", err);
        return err;
    }

    for (int i = 0; i < NUM_PTS; ++i) {
        float sum = 0.0f;
        for (int j = 0; j < CONFIG_PT_SAMPLES; ++j) {
            sum += static_cast<float>(raw_readings[j][i]);
        }
        readings_by_idx[i] = sum / static_cast<float>(CONFIG_PT_SAMPLES)
                             * pt_configs[i].scale + pt_configs[i].bias;
    }

    return 0;
}

// Push a new sample into the ring buffer
static void pts_add_to_history(const float readings_by_idx[NUM_PTS]) {
    for (int i = 0; i < NUM_PTS; ++i) {
        pts_history[pts_history_index][i] = readings_by_idx[i];
    }

    pts_history_index = (pts_history_index + 1) % PTS_MOVING_AVG_MAX_SAMPLES;
    if (pts_history_size < PTS_MOVING_AVG_MAX_SAMPLES) {
        ++pts_history_size;
    }
}

// Compute moving average over the last `window` samples into avg_by_idx[]
static void pts_compute_mavg(int window, float avg_by_idx[NUM_PTS]) {
    if (pts_history_size == 0) {
        for (int i = 0; i < NUM_PTS; ++i) {
            avg_by_idx[i] = 0.0f;
        }
        return;
    }

    if (window <= 0) {
        window = 1;
    }

    if (window > pts_history_size) {
        window = pts_history_size;
    }

    for (int i = 0; i < NUM_PTS; ++i) {
        float sum = 0.0f;

        // Walk backwards from the most recent sample
        for (int n = 0; n < window; ++n) {
            int idx = pts_history_index - 1 - n;
            if (idx < 0) {
                idx += PTS_MOVING_AVG_MAX_SAMPLES;
            }
            sum += pts_history[idx][i];
        }

        avg_by_idx[i] = sum / static_cast<float>(window);
    }
}


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

//int pts_configure(int pt_index, ) {
//
//}

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
#define CLOVER_PTS_DT_TO_READINGS_ASSIGNMENT(node_id, prop, idx) .DT_STRING_TOKEN_BY_IDX(node_id, prop, idx) = readings_by_idx[idx],
    return pt_readings{
            DT_FOREACH_PROP_ELEM(USER_NODE, pt_names, CLOVER_PTS_DT_TO_READINGS_ASSIGNMENT)
    };
}


/// Take a new PT sample, update history, and return a moving average over the last `window` samples.
pt_readings pts_sample_mavg(int window) {
    // Clamp the requested window
    if (window <= 0) {
        window = 1;
    }
    if (window > PTS_MOVING_AVG_MAX_SAMPLES) {
        window = PTS_MOVING_AVG_MAX_SAMPLES;
    }

    // Remember this as the "current" window for logging
    pts_mavg_window = window;

    float readings_by_idx[NUM_PTS] = {0};

    // Take a fresh reading and push into history
    if (pts_read_and_convert(readings_by_idx) != 0) {
        return pt_readings{};
    }

    pts_add_to_history(readings_by_idx);

    // Compute moving average from history
    float avg_by_idx[NUM_PTS] = {0};
    pts_compute_mavg(window, avg_by_idx);

    // Map averaged values into pt_readings
#define CLOVER_PTS_DT_TO_MAVG_ASSIGNMENT(node_id, prop, idx) \
.DT_STRING_TOKEN_BY_IDX(node_id, prop, idx) = avg_by_idx[idx],

    return pt_readings{
        DT_FOREACH_PROP_ELEM(USER_NODE, pt_names, CLOVER_PTS_DT_TO_MAVG_ASSIGNMENT)
    };
}


/// Log PT readings for debug purposes
void pts_log_readings(const pt_readings &readings) {
#define CLOVER_PTS_DT_TO_LOG(node_id, prop, idx) LOG_INF(DT_PROP_BY_IDX(node_id, prop, idx) ": %f psig", static_cast<double>(readings.DT_STRING_TOKEN_BY_IDX(node_id, prop, idx)));
    DT_FOREACH_PROP_ELEM(USER_NODE, pt_names, CLOVER_PTS_DT_TO_LOG)


}

void pts_log_readings_mavg() {
    // Take a moving-average sample using the current window
    pt_readings avg = pts_sample_mavg(pts_mavg_window);

#define CLOVER_PTS_DT_TO_LOG_MAVG(node_id, prop, idx)
    LOG_INF("PTS_MAVG (N=%d) " STRINGIFY(DT_STRING_TOKEN_BY_IDX(node_id, prop, idx))
        " = %f",
        pts_mavg_window,
        static_cast<double>(avg.DT_STRING_TOKEN_BY_IDX(node_id, prop, idx)));
    DT_FOREACH_PROP_ELEM(USER_NODE, pt_names, CLOVER_PTS_DT_TO_LOG_MAVG)
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
