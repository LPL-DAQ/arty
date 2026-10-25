
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include "test_trace.h"

static const struct gpio_dt_spec valves[] = {
    DT_FOREACH_PROP_ELEM_SEP(DT_PATH(zephyr_user), valve_gpios,
                             GPIO_DT_SPEC_GET_BY_IDX, (,))
};

BUILD_ASSERT(CONFIG_VALVE_TEST_CHANNEL < ARRAY_SIZE(valves),
             "Selected valve channel is not present on this board");

int main(void)
{
    trace_start("automatic single-channel");
    bool ready = true;
    /* Initialize every valve output inactive before allowing a pulse. */
    for (size_t i = 0; i < ARRAY_SIZE(valves); ++i) {
        int err = gpio_is_ready_dt(&valves[i]) ?
            gpio_pin_configure_dt(&valves[i], GPIO_OUTPUT_INACTIVE) : -ENODEV;
        trace_event("INIT channel=%u port=%s pin=%u result=%d",
                    (unsigned)i, valves[i].port->name, valves[i].pin, err);
        if (err) {
            ready = false;
        }
    }
    if (!ready) {
        trace_event("STOP: initialization failed");
        return -ENODEV;
    }

    const struct gpio_dt_spec *valve = &valves[CONFIG_VALVE_TEST_CHANNEL];
    trace_event("Waiting 15 seconds before channel %d pulse", CONFIG_VALVE_TEST_CHANNEL);
    k_sleep(K_SECONDS(15));
    /* Hold one output active long enough to position and read a multimeter. */
    int err = gpio_pin_set_dt(valve, 1);
    trace_event("ON channel=%d result=%d raw_readback=%d", CONFIG_VALVE_TEST_CHANNEL,
                err, gpio_pin_get_raw(valve->port, valve->pin));
    if (err) {
        int off_err = gpio_pin_set_dt(valve, 0);
        trace_event("STOP: activation failed; cleanup result=%d", off_err);
        return err;
    }
    k_sleep(K_SECONDS(60));
    err = gpio_pin_set_dt(valve, 0);
    trace_event("OFF channel=%d result=%d", CONFIG_VALVE_TEST_CHANNEL, err);
    return err;
}
