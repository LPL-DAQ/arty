#include <errno.h>
#include <stdlib.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/usb/usb_device.h>

static const struct gpio_dt_spec valves[] = {
    DT_FOREACH_PROP_ELEM_SEP(DT_PATH(zephyr_user), valve_gpios,
                             GPIO_DT_SPEC_GET_BY_IDX, (,))
};
static bool ready;

static int number(const char *text, unsigned long maximum, unsigned long *value)
{
    char *end;
    errno = 0;
    *value = strtoul(text, &end, 10);
    return errno || end == text || *end || *value > maximum ? -EINVAL : 0;
}

static void report(const struct shell *sh, unsigned long channel)
{
    const struct gpio_dt_spec *pin = &valves[channel];
    int raw = gpio_pin_get_raw(pin->port, pin->pin);
    if (raw < 0) {
        shell_error(sh, "Channel %lu read failed: %d", channel, raw);
        return;
    }
    shell_print(sh, "Channel %lu: %s pin %u, active-%s, raw=%d (%s), logical=%d",
                channel, pin->port->name, pin->pin,
                pin->dt_flags & GPIO_ACTIVE_LOW ? "low" : "high",
                raw, raw ? "HIGH" : "LOW",
                raw ^ !!(pin->dt_flags & GPIO_ACTIVE_LOW));
}

static int cmd_list(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);
    if (!ready) {
        shell_error(sh, "GPIO initialization failed; reset after checking hardware.");
        return -ENODEV;
    }
    for (size_t i = 0; i < ARRAY_SIZE(valves); ++i) {
        report(sh, i);
    }
    return 0;
}

static int cmd_pulse(const struct shell *sh, size_t argc, char **argv)
{
    unsigned long channel, duration = 1000;
    if (!ready) {
        shell_error(sh, "GPIO initialization failed; reset after checking hardware.");
        return -ENODEV;
    }
    if (number(argv[1], ARRAY_SIZE(valves) - 1, &channel) ||
        (argc == 3 && number(argv[2], 10000, &duration)) || duration == 0) {
        shell_error(sh, "Use valve pulse <channel 0..%u> [duration 1..10000 ms]",
                    (unsigned int)ARRAY_SIZE(valves) - 1);
        return -EINVAL;
    }
    const struct gpio_dt_spec *pin = &valves[channel];
    shell_print(sh, "Before:");
    report(sh, channel);
    int err = gpio_pin_set_dt(pin, 1);
    if (!err) {
        shell_print(sh, "Active for %lu ms:", duration);
        report(sh, channel);
        k_msleep(duration);
    }
    /* Always attempt to turn off, including after an activation error. */
    int off_err = gpio_pin_set_dt(pin, 0);
    shell_print(sh, "After:");
    report(sh, channel);
    if (err || off_err) {
        shell_error(sh, "GPIO write failed: activation=%d, deactivation=%d", err, off_err);
    }
    return err ? err : off_err;
}

/* Machine-readable completion includes a caller token to reject stale replies. */
static int cmd_group(const struct shell *sh, size_t argc, char **argv)
{
    unsigned long token, duration, channels[ARRAY_SIZE(valves)];
    if (number(argv[1], 2147483647UL, &token)) {
        return -EINVAL;
    }
    int err = 0;
    if (!ready) {
        err = -ENODEV;
        goto done;
    }
    if (number(argv[2], 10000, &duration) || duration == 0 ||
        argc - 3 > ARRAY_SIZE(valves)) {
        err = -EINVAL;
        goto done;
    }
    /* Validate the entire group before touching hardware. */
    for (size_t i = 3; i < argc; ++i) {
        if (number(argv[i], ARRAY_SIZE(valves) - 1, &channels[i - 3])) {
            err = -EINVAL;
            goto done;
        }
        for (size_t j = 3; j < i; ++j) {
            if (channels[j - 3] == channels[i - 3]) {
                err = -EINVAL;
                goto done;
            }
        }
    }
    for (size_t i = 3; i < argc; ++i) {
        err = gpio_pin_set_dt(&valves[channels[i - 3]], 1);
        if (err) {
            break;
        }
    }
    if (!err) {
        k_msleep(duration);
    }
    /* Attempt every deactivation even if activation or another off write failed. */
    for (size_t i = 3; i < argc; ++i) {
        int off_err = gpio_pin_set_dt(&valves[channels[i - 3]], 0);
        if (!err) {
            err = off_err;
        }
    }
done:
    shell_print(sh, "VALVE_DONE %lu %d", token, err);
    return err;
}

SHELL_STATIC_SUBCMD_SET_CREATE(valve_commands,
    SHELL_CMD_ARG(list, NULL, "List channels and digital pin readback", cmd_list, 1, 0),
    SHELL_CMD_ARG(pulse, NULL, "Pulse <channel> [milliseconds, default 1000]", cmd_pulse, 2, 1),
    SHELL_CMD_ARG(group, NULL, "Group <token> <milliseconds> <channel> [channels...]", cmd_group, 4, 31),
    SHELL_SUBCMD_SET_END);
SHELL_CMD_REGISTER(valve, &valve_commands, "Valve GPIO bench test", NULL);

int main(void)
{
    bool ok = true;
    for (size_t i = 0; i < ARRAY_SIZE(valves); ++i) {
        if (!gpio_is_ready_dt(&valves[i]) ||
            gpio_pin_configure_dt(&valves[i], GPIO_OUTPUT_INACTIVE)) {
            ok = false;
        }
    }
    ready = ok;
    return usb_enable(NULL);
}
