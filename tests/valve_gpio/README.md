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

## Embed the sequence (no serial connection)

This mode reads `sequence.example.json` at build time and compiles the steps into
firmware. It needs no pyserial, USB terminal, or serial forwarding. The JSON
format and per-step behavior are the same as the host runner above.

From the repository root:

```sh
west build -s tests/valve_gpio -b ranger_1/mimxrt1062 \
  -d tests/valve_gpio/build-sequence -- -DEXTRA_CONF_FILE=sequence.conf
west flash -d tests/valve_gpio/build-sequence
```

Every successful boot initializes all outputs inactive, immediately executes
the sequence once, then leaves the outputs inactive.
An empty channel list adds an inactive wait. Power cycling runs the sequence
again; the Teensy Program button enters its bootloader rather than starting a
normal reset. Flashing may start the sequence immediately when firmware boots.

With the current example, measured after GPIO initialization:

| Time | Energized channels |
| --- | --- |
| 0–1 seconds | None (first JSON wait) |
| 1–11 seconds | 0 |
| 11–16 seconds | None |
| 16–26 seconds | 0 and 1 |
| After 26 seconds | None |

GPIO writes add small delays. Group edges use sequential I2C writes, not hardware
synchronization. All outputs are commanded inactive between steps. If any GPIO
fails initialization, the sequence does not run. An activation or deactivation
write failure stops the sequence after attempting to deactivate all outputs.

Edit the JSON and rebuild/reflash to change the sequence. CMake tracks the JSON
as a build dependency and regenerates the embedded header when it changes.
To select another file, supply an absolute path:

```sh
west build -d tests/valve_gpio/build-sequence -- \
  -DVALVE_SEQUENCE_FILE=/absolute/path/to/my-sequence.json
```

The build rejects invalid JSON steps, duplicate/out-of-range channels, and
channels absent from the board. Keep `automatic.conf` for the separate old
single-channel test; use `sequence.conf` for this mode. Disconnect loads for the
first verification and restore normal firmware after testing. Firmware timing
cannot guarantee shutdown after a crash or failed I2C write. A failed flash does
not confirm that this sequence firmware is running.

## Execution logging

Embedded-sequence and automatic single-channel firmware enable USB CDC and emit
Zephyr INFO logs to the USB shell. Connect a serial terminal on the Mac's actual
`/dev/cu.usbmodem...` port at 115200 baud. No terminal connection is required to
start execution, so early live logs may be missed. Enter `trace` to retrieve the
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
