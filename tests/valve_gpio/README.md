# Ranger valve GPIO bench test

For a test that runs automatically without serial commands, follow
[AUTOMATIC_TEST.md](AUTOMATIC_TEST.md).

Standalone firmware: no flight controller, network server, or valve assignment
configuration is needed. Channel numbers are zero-based indices into
`zephyr,user/valve-gpios` in the board DTS. This tests the GPIO path directly,
not Clover's valve assignment or command protocol.

Build and flash from the repository root in the development container:

```sh
west build -s tests/valve_gpio -b ranger_1/mimxrt1062 -d tests/valve_gpio/build
west flash -d tests/valve_gpio/build
```

Enter the Teensy's bootloader when flashing, as for Clover. Open its USB serial
port with a terminal (for example `python -m serial.tools.miniterm /dev/ttyACM0
115200`; adjust the port for your host). Enable DTR if your terminal requires it.
Run:

```text
valve list
valve pulse 0
valve pulse 17 5000
```

`list` shows each expander device, pin, polarity, and digital readback. `pulse`
activates one channel for 1 second by default (maximum 10 seconds), then turns
it off. It prints before, active, and after readings. All channels are configured
inactive at startup. An unavailable expander or GPIO configuration error disables
pulse commands. A failed read is reported as an error, never as a voltage.

For the current Ranger mapping:

| Channels | Expander | Pins |
| --- | --- | --- |
| 0–7 | mcp23017_1 | 8–15 (GPB0–GPB7) |
| 8–15 | mcp23017_1 | 0–7 (GPA0–GPA7) |
| 16–23 | mcp23017_2 | 8–15 (GPB0–GPB7) |
| 24–31 | mcp23017_2 | 0–7 (GPA0–GPA7) |

These are expander pins, not Teensy header pin numbers. Both expanders have I2C
address 0x20 but are on different buses.

## Measure the electrical output

Start with valve loads disconnected. Connect a multimeter's reference lead to
board ground and probe the selected expander output. For example, use
`valve pulse 0 5000` while probing expander 1 GPB0. Ranger declares these signals
**active-high**, so expect LOW → HIGH → LOW at the expander control pin. The
actual HIGH voltage depends on the board's GPIO supply. If you expect voltage to
increase, probe the corresponding valve driver output using the board schematic
to identify the correct output terminals and measurement reference. The DTS does
not describe the driver output wiring or voltage.

GPIO readback is a digital threshold reading, not a voltage measurement or proof
that a connected valve actuated. An automatic measurement in volts requires an
ADC or external instrument and a defined, appropriately scaled feedback path;
no valve-to-ADC feedback mapping is provided here. Do not connect a valve supply
output directly to a Teensy input.

The timeout runs in firmware and still completes if the serial terminal closes;
it cannot guarantee deactivation after a firmware crash or an I2C write failure.
Restore the normal Clover firmware after testing.

## Run a sequence file

`run_sequence.py` reads a JSON array of steps; see `sequence.example.json`.
Each step supplies `duration_ms` (1–10000) and a list of GPIO `channels` (0–31).
An empty list waits with outputs inactive. Each nonempty step activates its
channels together, holds them for the duration, and deactivates them before
acknowledging completion. Consecutive steps have an off interval and host/USB
latency between them; this is not a precise timestamp scheduler. Group activation
uses consecutive GPIO writes, so rising edges have I2C skew. Channels describe
energized outputs, not necessarily mechanically open valves.

Flash updated manual firmware first (the automatic firmware cannot run this):

```sh
west build -p always -s tests/valve_gpio -b ranger_1/mimxrt1062 \
  -d tests/valve_gpio/build-manual -- -DCONFIG_VALVE_TEST_AUTOMATIC=n
west flash -d tests/valve_gpio/build-manual
```

Validate the file without connecting hardware:

```sh
python tests/valve_gpio/run_sequence.py tests/valve_gpio/sequence.example.json --dry-run
```

Install pyserial into the selected Python environment if necessary, then run:

```sh
uv pip install --python "$(command -v python)" pyserial
python tests/valve_gpio/run_sequence.py tests/valve_gpio/sequence.example.json \
  --port /dev/ttyACM0
```

On a Mac, substitute the actual `/dev/cu.usbmodem...` device. Close other serial
terminals first. Docker Desktop must have USB forwarding or a separately running
Mac serial bridge; a container showing only `/dev/ttyS*` cannot reach the Teensy.
The runner accepts `--port socket://host.docker.internal:7000` when a bridge has
already been configured at that address. It does not create a bridge itself.

The runner validates the whole file before sending anything and waits for a
matching `VALVE_DONE` token after each pulse. A failure or missing acknowledgement
stops later steps. The board attempts to deactivate every channel in a group
after a write error. Disconnecting the host or interrupting the runner does not
cancel an in-progress pulse: the board finishes its timed hold. Firmware crashes
and failed off writes still require independent removal of valve power. Test
with loads disconnected first. This tests standalone GPIO firmware, not Clover's
production valve API.

## Store named sequences in firmware (manual start over USB)

Put separate JSON files in `tests/valve_gpio/sequences/`. Each filename without
`.json` becomes its command name. Included examples are `single_valve` and
`multi_valve`. Steps use the same JSON format as the host runner, with optional
comments. Names must be 1–48 characters: letters, digits, underscores, dots or
hyphens; the first character must be a letter, digit or underscore.

Build and flash from the repository root:

```sh
west build -p always -s tests/valve_gpio -b ranger_1/mimxrt1062 \
  -d tests/valve_gpio/build-sequence -- -DEXTRA_CONF_FILE=sequence.conf
west flash -d tests/valve_gpio/build-sequence
```

If reusing a build configured with the old `VALVE_SEQUENCE_FILE` setting, clear
that override by adding `-UVALVE_SEQUENCE_FILE` after `--`, or use a fresh build
directory. The normal build embeds every `*.json` directly in `sequences/`.
`sequence.example.json` remains an example for the host runner; editing it does
not change the named sequences in the default firmware build.

Boot initializes all outputs inactive and waits; it never automatically runs a
sequence. Open the USB serial monitor at 115200 baud, press Enter, and use:

```text
sequence list
sequence status
sequence run single_valve
sequence run multi_valve
trace
```

Wait for completion before starting another run. The shell remains responsive
while the main thread executes the sequence. `sequence status` reports
`initializing`, `ready`, `running`, `complete`, or `faulted`, plus the selected
name. Both `ready` and `complete` allow another run. A second start while running
is rejected. Unknown names do not activate any output. `trace` retrieves recent
logs if you connected after startup. Use a generic USB serial terminal if your
monitor does not recognize the board's USB CDC device.

An initialization, activation, or deactivation error prevents further runs until
reset. After an activation or deactivation error the firmware attempts to turn
every output off. Disconnecting the monitor does not cancel a running sequence:
it completes on the board. There is no stop command. Firmware crashes and failed
I2C off writes still require independent removal of valve power.

Switching among embedded sequences needs no flash or reset. Adding, removing or
editing a JSON file requires rebuilding and flashing; CMake tracks directory
membership and file changes. To embed files from another directory:

```sh
west build -d tests/valve_gpio/build-sequence -- \
  -DVALVE_SEQUENCE_DIR=/absolute/path/to/sequences
```

For a single-file build, the explicit `-DVALVE_SEQUENCE_FILE=/absolute/path/test.json`
override is still supported; its command name is `test`. Clear it with
`-UVALVE_SEQUENCE_FILE` to return to directory mode.

Each step energizes its channels, waits, then commands every output inactive.
An empty channel list waits with outputs inactive. Group edges use sequential
I2C writes, not hardware synchronization. Comments and logs add timing overhead.
The build rejects invalid steps, names, and channels absent from the board.
Keep `automatic.conf` for the separate old automatic single-channel test;
`sequence.conf` now selects named sequences with manual start. Verify first with
loads disconnected and restore normal Clover firmware after testing.

Host validation and mocked firmware behavior tests (no board required):

```sh
python -m unittest discover -s tests/valve_gpio/test -v
```

These exercise the actual C control flow with mock GPIO, shell and timing; they
do not verify physical outputs, USB enumeration, or real scheduling on hardware.

## Execution logging

The test's `app.overlay` routes the shell and its log backend to USB CDC ACM.
Setting only `zephyr,console` leaves the inherited Teensy shell on hardware UART.
When updating an existing build created before this overlay was added, rebuild
with `west build -p always` so CMake discovers the new overlay.

Each JSON step may include an optional `"comment"` string, for example
`"comment": "Energizing valves 0 and 1 for 10 seconds"`. Embedded firmware logs
`NEXT: <comment>` before that step's GPIO writes or inactive wait. Comments must
be printable text, at most 100 UTF-8 bytes, with no newlines or control characters.
Steps without comments still work. Rebuild and flash after editing comments.
The host runner also accepts comments and includes them in its terminal output.

Named-sequence and automatic single-channel firmware enable USB CDC and emit
Zephyr INFO logs to the USB shell. Connect a serial terminal on the Mac's actual
`/dev/cu.usbmodem...` port at 115200 baud. Named sequences wait for an explicit
run command. Early startup logs may be missed if the monitor connects after boot.
Enter `trace` to retrieve the
latest 128 retained events, with milliseconds since boot. This includes the mode,
USB initialization result, each GPIO's device/pin and configuration result,
activation, deactivation failures, and sequence completion or failure.

The embedded sequence additionally logs each step's mask/duration and raw digital
readback after activation. A negative readback is a driver error, not LOW; a
successful write or digital readback is not proof of output voltage or mechanical
movement. `trace` can be run again while execution is in progress. Trace history
is lost on power loss/reset. Logs add execution overhead and group timing skew.
A container still needs serial forwarding to receive these logs; flashing through
flasherd alone does not provide a serial terminal.
