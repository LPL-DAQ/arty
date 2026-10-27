#include <errno.h>
#include <string.h>
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

enum sequence_state { INITIALIZING, READY, RUNNING, COMPLETE, FAULTED };
static enum sequence_state state = INITIALIZING;
static const struct valve_sequence *selected;
K_MUTEX_DEFINE(state_lock);
K_SEM_DEFINE(run_requested, 0, 1);

static int run_sequence(const struct valve_sequence *sequence)
{
    trace_event("START sequence=%s", sequence->name);
    for (size_t step = 0; step < sequence->count; ++step) {
        if (sequence->steps[step].comment[0]) {
            trace_event("NEXT: %s", sequence->steps[step].comment);
        }
        trace_event("STEP %u/%u mask=0x%08x duration=%u ms",
                    (unsigned)step + 1, (unsigned)sequence->count,
                    (unsigned)sequence->steps[step].mask,
                    (unsigned)sequence->steps[step].duration_ms);
        int err = 0;
        for (size_t channel = 0; channel < ARRAY_SIZE(valves); ++channel) {
            if (sequence->steps[step].mask & BIT(channel)) {
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
            k_msleep(sequence->steps[step].duration_ms);
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

static int cmd_list(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);
    for (size_t i = 0; i < ARRAY_SIZE(sequences); ++i) {
        shell_print(sh, "%s (%u steps)", sequences[i].name, (unsigned)sequences[i].count);
    }
    return 0;
}

static int cmd_status(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);
    static const char *const names[] = {"initializing", "ready", "running", "complete", "faulted"};
    k_mutex_lock(&state_lock, K_FOREVER);
    shell_print(sh, "%s; sequence=%s", names[state], selected ? selected->name : "none");
    k_mutex_unlock(&state_lock);
    return 0;
}

static int cmd_run(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    const struct valve_sequence *sequence = NULL;
    for (size_t i = 0; i < ARRAY_SIZE(sequences); ++i) {
        if (!strcmp(argv[1], sequences[i].name)) {
            sequence = &sequences[i];
            break;
        }
    }
    if (!sequence) {
        shell_error(sh, "Unknown sequence; use sequence list");
        return -ENOENT;
    }
    k_mutex_lock(&state_lock, K_FOREVER);
    if (state == INITIALIZING || state == RUNNING || state == FAULTED) {
        shell_error(sh, "Cannot start: initializing, running, or faulted (faults require reset)");
        k_mutex_unlock(&state_lock);
        return -EBUSY;
    }
    selected = sequence;
    state = RUNNING;
    k_mutex_unlock(&state_lock);
    shell_print(sh, "Queued sequence %s", sequence->name);
    k_sem_give(&run_requested);
    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sequence_commands,
    SHELL_CMD_ARG(list, NULL, "List embedded sequences", cmd_list, 1, 0),
    SHELL_CMD_ARG(run, NULL, "Run a named sequence", cmd_run, 2, 0),
    SHELL_CMD_ARG(status, NULL, "Show sequence state", cmd_status, 1, 0),
    SHELL_SUBCMD_SET_END
);
SHELL_CMD_REGISTER(sequence, &sequence_commands, "Embedded valve sequences", NULL);

int main(void)
{
    trace_start("named embedded sequences (manual start)");
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
        k_mutex_lock(&state_lock, K_FOREVER);
        state = FAULTED;
        k_mutex_unlock(&state_lock);
        return -ENODEV;
    }

    k_mutex_lock(&state_lock, K_FOREVER);
    state = READY;
    k_mutex_unlock(&state_lock);
    trace_event("READY: use sequence list, then sequence run <name>");
    for (;;) {
        k_sem_take(&run_requested, K_FOREVER);
        /* Only the shell writes selected, and RUNNING prevents changes during a run. */
        int err = run_sequence(selected);
        k_mutex_lock(&state_lock, K_FOREVER);
        state = err ? FAULTED : COMPLETE;
        k_mutex_unlock(&state_lock);
        trace_event("State=%s", err ? "faulted (reset required)" : "complete; ready for another run");
    }
}
