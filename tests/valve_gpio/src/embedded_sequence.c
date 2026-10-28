#include <errno.h>
#include <string.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include "valve_sequence.h"
#include "sequence_engine.h"
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

enum sequence_state { INITIALIZING, READY, RUNNING, COMPLETE, FAULTED, ABORTED };
static enum sequence_state state = INITIALIZING;
static const struct valve_sequence *selected;
K_MUTEX_DEFINE(state_lock);
K_SEM_DEFINE(run_requested, 0, 1);
K_SEM_DEFINE(abort_wakeup, 0, 1);
static bool aborting;
static uint32_t current_step;
static int last_error;

size_t sequence_count(void) { return ARRAY_SIZE(sequences); }
const char *sequence_name(size_t index) { return index < sequence_count() ? sequences[index].name : NULL; }

struct sequence_snapshot sequence_status(void)
{
    static const char *const names[] = {"initializing", "ready", "running", "complete", "faulted", "aborted"};
    k_mutex_lock(&state_lock, K_FOREVER);
    struct sequence_snapshot out = {
        .state = aborting && state == RUNNING ? "aborting" : names[state],
        .name = selected ? selected->name : "",
        .comment = selected && current_step ? selected->steps[current_step - 1].comment : "",
        .step = current_step,
        .total_steps = selected ? selected->count : 0,
        .error = last_error,
    };
    k_mutex_unlock(&state_lock);
    return out;
}

int sequence_abort(void)
{
    k_mutex_lock(&state_lock, K_FOREVER);
    int err = 0;
    if (state == INITIALIZING) {
        err = -EBUSY;
    } else if (state == RUNNING) {
        aborting = true;
        k_sem_give(&abort_wakeup);
    } else {
        /* An idle/faulted abort also attempts to de-energize every output.
         * Never clear a latched GPIO fault as a side effect of abort. */
        err = all_off();
        if (err) {
            last_error = err;
            state = FAULTED;
        } else if (state != FAULTED) {
            state = ABORTED;
        }
    }
    k_mutex_unlock(&state_lock);
    return err;
}

static int run_sequence(const struct valve_sequence *sequence)
{
    trace_event("START sequence=%s", sequence->name);
    for (size_t step = 0; step < sequence->count; ++step) {
        k_mutex_lock(&state_lock, K_FOREVER);
        if (aborting) {
            k_mutex_unlock(&state_lock);
            int off_err = all_off();
            return off_err;
        }
        current_step = step + 1;
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
        k_mutex_unlock(&state_lock);
        if (!err) {
            /* ABORT wakes this wait; timing and cleanup remain owned by this thread. */
            k_sem_take(&abort_wakeup, K_MSEC(sequence->steps[step].duration_ms));
        }
        int off_err = all_off();
        trace_event("STEP %u OFF result=%d", (unsigned)step + 1, off_err);
        if (err || off_err) {
            trace_event("STOP: activation=%d deactivation=%d", err, off_err);
            return err ? err : off_err;
        }
        k_mutex_lock(&state_lock, K_FOREVER);
        bool cancelled = aborting;
        k_mutex_unlock(&state_lock);
        if (cancelled) {
            trace_event("ABORTED: all outputs commanded inactive");
            return 0;
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
    struct sequence_snapshot status = sequence_status();
    shell_print(sh, "%s; sequence=%s step=%u/%u error=%d comment=%s", status.state,
                status.name, (unsigned)status.step, (unsigned)status.total_steps,
                status.error, status.comment);
    return 0;
}

int sequence_start(const char *name)
{
    const struct valve_sequence *sequence = NULL;
    for (size_t i = 0; i < ARRAY_SIZE(sequences); ++i) {
        if (!strcmp(name, sequences[i].name)) {
            sequence = &sequences[i];
            break;
        }
    }
    if (!sequence) {
        return -ENOENT;
    }
    k_mutex_lock(&state_lock, K_FOREVER);
    if (state == INITIALIZING || state == RUNNING || state == FAULTED) {
        k_mutex_unlock(&state_lock);
        return -EBUSY;
    }
    selected = sequence;
    state = RUNNING;
    aborting = false;
    current_step = 0;
    last_error = 0;
    k_sem_reset(&abort_wakeup);
    /* Publish selection and wakeup together so USB/TCP starts cannot race. */
    k_sem_give(&run_requested);
    k_mutex_unlock(&state_lock);
    return 0;
}

static int cmd_run(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    int err = sequence_start(argv[1]);
    if (err) {
        shell_error(sh, "Cannot start (%d): check name/status; faults require reset", err);
    } else {
        shell_print(sh, "Queued sequence %s", argv[1]);
    }
    return err;
}

static int cmd_abort(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);
    int err = sequence_abort();
    if (err) {
        shell_error(sh, "Abort failed (%d); check sequence status", err);
        return err;
    }
    shell_print(sh, "Abort requested; use sequence status to confirm outputs-off completion");
    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sequence_commands,
    SHELL_CMD_ARG(list, NULL, "List embedded sequences", cmd_list, 1, 0),
    SHELL_CMD_ARG(run, NULL, "Run a named sequence", cmd_run, 2, 0),
    SHELL_CMD_ARG(status, NULL, "Show sequence state", cmd_status, 1, 0),
    SHELL_CMD_ARG(abort, NULL, "Abort and deactivate outputs", cmd_abort, 1, 0),
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
        last_error = -ENODEV;
        k_mutex_unlock(&state_lock);
        return -ENODEV;
    }

    k_mutex_lock(&state_lock, K_FOREVER);
    state = READY;
    k_mutex_unlock(&state_lock);
    trace_event("READY: use sequence list, then sequence run <name>");
    for (;;) {
        k_sem_take(&run_requested, K_FOREVER);
        /* RUNNING prevents USB/TCP commands from changing selected during a run. */
        int err = run_sequence(selected);
        k_mutex_lock(&state_lock, K_FOREVER);
        last_error = err;
        state = last_error ? FAULTED : (aborting ? ABORTED : COMPLETE);
        k_mutex_unlock(&state_lock);
        trace_event("State=%s", sequence_status().state);
    }
}
