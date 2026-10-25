#include <errno.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include "valve_sequence.h"
#include "test_trace.h"

static const struct gpio_dt_spec valves[] = {
    DT_FOREACH_PROP_ELEM_SEP(DT_PATH(zephyr_user), valve_gpios,
                             GPIO_DT_SPEC_GET_BY_IDX, (,))
};

BUILD_ASSERT(SEQUENCE_REQUIRED_CHANNELS <= ARRAY_SIZE(valves),
             "Sequence uses a valve channel absent from this board");
BUILD_ASSERT(ARRAY_SIZE(valves) <= 32, "Sequence masks support at most 32 channels");

/* Keep trying every output, even if one write fails. */
static int all_off(void)
{
    int first_error = 0;
    for (size_t i = 0; i < ARRAY_SIZE(valves); ++i) {
        if (!gpio_is_ready_dt(&valves[i])) {
            if (!first_error) {
                first_error = -ENODEV;
            }
            continue;
        }
        int err = gpio_pin_set_dt(&valves[i], 0);
        if (err) {
            trace_event("OFF FAILED channel=%u error=%d", (unsigned)i, err);
        }
        if (err && !first_error) {
            first_error = err;
        }
    }
    return first_error;
}

int main(void)
{
    trace_start("embedded sequence");
    trace_event("Initializing %u GPIO channels", (unsigned)ARRAY_SIZE(valves));
    bool ready = true;
    for (size_t i = 0; i < ARRAY_SIZE(valves); ++i) {
        const struct gpio_dt_spec *pin = &valves[i];
        int err = gpio_is_ready_dt(pin) ?
            gpio_pin_configure_dt(pin, GPIO_OUTPUT_INACTIVE) : -ENODEV;
        trace_event("INIT channel=%u port=%s pin=%u active_%s result=%d",
                    (unsigned)i, pin->port->name, pin->pin,
                    pin->dt_flags & GPIO_ACTIVE_LOW ? "low" : "high", err);
        if (err) {
            ready = false;
        }
    }
    if (!ready) {
        trace_event("STOP: GPIO initialization failed; sequence will not run");
        all_off();
        return -ENODEV;
    }

    for (size_t step = 0; step < ARRAY_SIZE(sequence_steps); ++step) {
        trace_event("STEP %u/%u mask=0x%08x duration=%u ms",
                    (unsigned)step + 1, (unsigned)ARRAY_SIZE(sequence_steps),
                    (unsigned)sequence_steps[step].mask,
                    (unsigned)sequence_steps[step].duration_ms);
        int err = 0;
        for (size_t channel = 0; channel < ARRAY_SIZE(valves); ++channel) {
            if (sequence_steps[step].mask & BIT(channel)) {
                err = gpio_pin_set_dt(&valves[channel], 1);
                trace_event("ON channel=%u result=%d raw_readback=%d",
                            (unsigned)channel, err,
                            gpio_pin_get_raw(valves[channel].port, valves[channel].pin));
                if (err) {
                    break;
                }
            }
        }
        if (!err) {
            k_msleep(sequence_steps[step].duration_ms);
        }
        int off_err = all_off();
        trace_event("STEP %u OFF result=%d", (unsigned)step + 1, off_err);
        if (err || off_err) {
            trace_event("STOP: activation=%d deactivation=%d", err, off_err);
            return err ? err : off_err;
        }
    }
    trace_event("COMPLETE: all outputs commanded inactive");
    return 0;
}
