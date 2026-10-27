"""Host checks: python -m unittest discover -s tests/valve_gpio/test -v."""
import json
from pathlib import Path
import re
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class SequencesTest(unittest.TestCase):
    def generate(self, source, output, success=True):
        result = subprocess.run([sys.executable, str(ROOT / 'generate_sequence.py'),
                                 str(source), str(output)], capture_output=True, text=True)
        self.assertEqual(result.returncode == 0, success, result.stderr)

    def test_generator_validation(self):
        with tempfile.TemporaryDirectory() as directory:
            directory = Path(directory)
            inputs = directory / 'inputs'
            inputs.mkdir()
            header = directory / 'valve_sequence.h'
            self.generate(inputs, header, False)
            path = inputs / 'valid.json'
            step = {'duration_ms': 1, 'channels': [31], 'comment': 'Valve "A" \\ 123 café %'}
            path.write_text(json.dumps([step]))
            self.generate(inputs, header)
            self.assertIn('0x80000000U', header.read_text())
            self.assertIn('SEQUENCE_REQUIRED_CHANNELS 32', header.read_text())
            # Single-file builds and optional comments remain supported.
            self.generate(path, header)
            for change in [{'channels': [0, 0]}, {'channels': [32]}, {'duration_ms': 0},
                           {'comment': '\n'}, {'comment': 'x' * 101}, {'comment': 42},
                           {'unexpected': True}]:
                path.write_text(json.dumps([dict(step, **change)]))
                self.generate(inputs, header, False)
            path.unlink()
            (inputs / 'bad name.json').write_text('[{"duration_ms":1,"channels":[]}]')
            self.generate(inputs, header, False)

    def test_firmware_with_mock_gpio(self):
        """Execute actual firmware control flow with mocked GPIO, clock and shell."""
        with tempfile.TemporaryDirectory() as directory:
            directory = Path(directory)
            inputs = directory / 'inputs'
            inputs.mkdir()
            (inputs / 'a.json').write_text(json.dumps([
                {'duration_ms': 10, 'channels': [0, 1], 'comment': 'Before "on" café %'},
                {'duration_ms': 5, 'channels': []},
                {'duration_ms': 20, 'channels': [2]},
            ]))
            (inputs / 'b.json').write_text('[{"duration_ms":7,"channels":[1]}]')
            self.generate(inputs, directory / 'valve_sequence.h')
            firmware = (ROOT / 'src/embedded_sequence.c').read_text()
            firmware = re.sub(r'#include <zephyr/[^>]+>\n|#include "test_trace.h"\n', '', firmware)
            firmware = firmware.replace('int main(void)', 'int firmware_main(void)')
            harness = directory / 'test.c'
            harness.write_text(MOCKS + firmware + CHECKS)
            binary = directory / 'test'
            subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror', str(harness),
                            '-o', str(binary)], check=True)
            subprocess.run([str(binary)], check=True)


MOCKS = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <setjmp.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#define BUILD_ASSERT(c, m) _Static_assert(c, m)
#define BIT(n) (1U << (n))
#define ARG_UNUSED(a) (void)(a)
#define GPIO_ACTIVE_LOW 1
#define GPIO_OUTPUT_INACTIVE 0
#define K_FOREVER 0
#define K_MUTEX_DEFINE(n) static int n
#define K_SEM_DEFINE(n, a, b) static int n
#define k_mutex_lock(a, b) ((void)(a), (void)(b))
#define k_mutex_unlock(a) ((void)(a))
#define SHELL_CMD_ARG(...) 0
#define SHELL_SUBCMD_SET_END 0
#define SHELL_STATIC_SUBCMD_SET_CREATE(n, ...) static int n[] = {__VA_ARGS__}
#define SHELL_CMD_REGISTER(n, sub, help, handler) static void *registered = sub
struct shell { int unused; };
struct device { const char *name; };
static struct device port = {"mock"};
struct gpio_dt_spec { struct device *port; unsigned pin; unsigned dt_flags; };
#define DT_FOREACH_PROP_ELEM_SEP(...) {&port,0,0},{&port,1,0},{&port,2,0}
static int values[3], on_count, off_count, elapsed, init_fail, fail_on, fail_off;
static int pending, take_count, scenario;
static char output[8192];
static jmp_buf idle;
static void print_message(const char *fmt, va_list args) {
    size_t used = strlen(output);
    vsnprintf(output + used, sizeof(output) - used, fmt, args);
}
static void trace_event(const char *fmt, ...) {
    va_list args; va_start(args, fmt); print_message(fmt, args); va_end(args);
}
static void trace_start(const char *mode) { trace_event("%s", mode); }
static void shell_print(const struct shell *sh, const char *fmt, ...) {
    (void)sh;
    va_list args; va_start(args, fmt); print_message(fmt, args); va_end(args);
}
#define shell_error shell_print
static bool gpio_is_ready_dt(const struct gpio_dt_spec *p) { (void)p; return true; }
static int gpio_pin_configure_dt(const struct gpio_dt_spec *p, int flags) {
    assert(flags == GPIO_OUTPUT_INACTIVE); values[p->pin] = 0; return init_fail ? -5 : 0;
}
static int gpio_pin_set_dt(const struct gpio_dt_spec *p, int value) {
    if (value) {
        ++on_count;
        if (fail_on && p->pin == 1) return -5;
        if (on_count == 1) assert(strstr(output, "NEXT: Before \"on\" café %"));
    } else {
        ++off_count;
        if (fail_off && p->pin == 1) return -5;
    }
    values[p->pin] = value;
    return 0;
}
static int gpio_pin_get_raw(const struct device *d, unsigned pin) { (void)d; return values[pin]; }
static void k_msleep(int ms) { elapsed += ms; }
static void k_sem_give(int *s) { (void)s; ++pending; }
static void k_sem_take(int *s, int timeout);
'''

CHECKS = r'''
static void k_sem_take(int *s, int timeout) {
    (void)s; (void)timeout;
    char *a[] = {"run", "a"};
    char *b[] = {"run", "b"};
    assert(!pending);
    if (take_count++ == 0) {
        assert(state == READY);
        assert(on_count == 0 && elapsed == 0); /* No boot actuation. */
        assert(cmd_run(NULL, 2, a) == 0);
        assert(state == RUNNING && selected == &sequences[0]);
        assert(cmd_run(NULL, 2, b) == -EBUSY);
        assert(selected == &sequences[0]);
    } else if (scenario == 0 && take_count <= 3) {
        assert(state == COMPLETE);
        assert(!values[0] && !values[1] && !values[2]);
        /* Run a second file, then repeat it without reset. */
        assert(cmd_run(NULL, 2, b) == 0);
    } else {
        assert(state == (scenario ? FAULTED : COMPLETE));
        if (scenario) assert(cmd_run(NULL, 2, b) == -EBUSY);
        longjmp(idle, 1);
    }
    assert(pending == 1);
    --pending;
}
static void reset(void) {
    memset(values, 0, sizeof(values));
    output[0] = 0;
    on_count = off_count = elapsed = pending = take_count = 0;
    init_fail = fail_on = fail_off = 0;
    state = INITIALIZING;
    selected = NULL;
}
int main(void) {
    assert(registered);
    char *a[] = {"run", "a"};
    char *bad[] = {"run", "missing"};
    assert(cmd_run(NULL, 2, a) == -EBUSY);
    assert(cmd_run(NULL, 2, bad) == -ENOENT);
    assert(cmd_list(NULL, 0, NULL) == 0);
    assert(strstr(output, "a (3 steps)") && strstr(output, "b (1 steps)"));
    reset();
    scenario = 0;
    if (!setjmp(idle)) firmware_main();
    assert(on_count == 5 && elapsed == 49 && off_count == 15);
    assert(cmd_status(NULL, 0, NULL) == 0);
    assert(strstr(output, "complete; sequence=b"));
    reset(); scenario = 1; fail_on = 1;
    if (!setjmp(idle)) firmware_main();
    assert(on_count == 2 && elapsed == 0 && off_count == 3);
    assert(!values[0] && !values[1] && !values[2]);
    reset(); scenario = 2; fail_off = 1;
    if (!setjmp(idle)) firmware_main();
    assert(on_count == 2 && elapsed == 10 && off_count == 3);
    reset(); init_fail = 1;
    assert(firmware_main() == -ENODEV);
    assert(state == FAULTED && on_count == 0 && off_count == 3);
    assert(cmd_run(NULL, 2, a) == -EBUSY);
    puts("Mock firmware checks passed: idle boot, named/repeated runs, busy, errors, cleanup, faults, logs");
    return 0;
}
'''
