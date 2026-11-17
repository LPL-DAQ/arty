# Ranger valve GPIO bench test

The default `prj.conf` builds named sequences and TCP manual valve control.
No extra configuration file is required:

```sh
west build -p always -s tests/valve_gpio -b ranger_1/mimxrt1062 \
  -d tests/valve_gpio/build-sequence
west flash -d tests/valve_gpio/build-sequence
```

Use a pristine build when switching from an older build directory.

Standalone firmware: no flight controller, network server, or valve assignment
configuration is needed. Channel numbers are zero-based indices into
`zephyr,user/valve-gpios` in the board DTS. This tests the GPIO path directly,
not Clover's valve assignment or command protocol.

## Tracing execution in the source

`CMakeLists.txt` selects one firmware entry point: `src/main.c` for USB
`valve pulse/group` commands or `src/embedded_sequence.c` for named sequences. The latter also builds
`src/sequence_server.c` and `src/sequence_protocol.c` for TCP control.

For named sequences, follow these paths:

1. At build time, `generate_sequence.py` calls `run_sequence.py:load_sequence`
   to validate JSON and resolve names through `valve_map.json`. It emits
   `valve_sequence.h`, containing immutable step masks, durations and comments.
2. On the host, `scripts/client-new.py` routes `sequence ...` commands to
   `scripts/named_sequences.py`. The `vlvseq` menu uses
   `scripts/valve_sequence_cli.py`, which serializes prompt commands and status
   polling and manual commands over the same socket inside `vlvseq`.
3. On the board, `sequence_server.c:serve` assembles each length-prefixed TCP
   frame, decodes the shared Clover protobuf, and calls
   `sequence_protocol.c:sequence_handle_request`. USB `sequence` commands call
   the same engine functions directly.
4. `embedded_sequence.c:sequence_start` checks state and queues work. Its `main`
   loop wakes and calls `run_sequence`, which energizes each mask, waits for the
   duration or abort wakeup, and clears the sequence request after each step.
   Manual overrides are applied independently. Command
   acceptance is asynchronous; use status to observe completion or faults.
5. `sequence_status` copies progress under the state mutex. `test_trace.h`
   retains recent events for the USB `trace` command and also sends them to
   Zephyr logging. GPIO faults latch until restart. Manual overrides persist across sequence transitions and client disconnects.

The alternative host USB runner, `run_sequence.py`, sends one `valve group`
command at a time to `src/main.c` and waits for its matching `VALVE_DONE` token.
An empty step delays on the host. This path does not use the named-sequence
engine or TCP server.

`test/test_sequences.py` checks validation and firmware control flow with mock
GPIO. `test/test_ethernet.py` compiles the engine, protocol and server with
`test/host_zephyr.h` adapters to exercise real TCP and nanopb framing. Generated
protobuf bindings, JSON examples and captured logs are data artifacts; edit
`api/clover.proto` and `api/clover.options` to change the wire schema.

For the legacy USB `valve pulse/group` commands below, explicitly disable the
default named-sequence engine and build from the repository root:

```sh
west build -p always -s tests/valve_gpio -b ranger_1/mimxrt1062 -d tests/valve_gpio/build -- -DCONFIG_VALVE_TEST_EMBEDDED_SEQUENCE=n
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
activates one channel for 1 second by default (maximum 2147483647 ms), then turns
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

## Valve-name mapping

`valve_map.json` maps valve names to zero-based channels in
`ranger_1.dts`'s `valve-gpios` list. **These are provisional sequential
assignments requested for setup, not verified physical wiring.**

**12 V valves**

| Valve | Channel | Expander | GPIO pin |
| --- | --- | --- | --- |
| SV001 | 0 | mcp23017_1 | 8 / GPB0 |
| SV002 | 1 | mcp23017_1 | 9 / GPB1 |
| PBV002 | 5 | mcp23017_1 | 13 / GPB5 |
| PBV001 | 4 | mcp23017_1 | 12 / GPB4 |

**24 V valves**

| Valve | Channel | Expander | GPIO pin |
| --- | --- | --- | --- |
| SV003 | 2 | mcp23017_1 | 10 / GPB2 |
| SV004 | 3 | mcp23017_1 | 11 / GPB3 |
| SV201 | 6 | mcp23017_1 | 14 / GPB6 |
| SV301 | 7 | mcp23017_1 | 15 / GPB7 |
| PBV101 | 8 | mcp23017_1 | 0 / GPA0 |

The mapping file lists the 12 V group first, then the 24 V group, separated
by a blank line. Channel assignments are unchanged. These voltage labels
describe the valve supplies; GPIO commands do not select the supply voltage.

Edit the channel numbers in this one file when the wiring is confirmed. The
mapping is specific to standalone valve tests; it does not configure Clover's
valve-assignment API or require its valve enum names.

Sequence steps can now use names:

```json
[
  {"duration_ticks": 1000, "valves": ["SV001", "PBV101"], "comment": "Energizing SV001 and PBV101"},
  {"duration_ticks": 1000, "valves": [], "comment": "All outputs off"}
]
```

`"channels": ["SV001", "PBV101"]` also accepts names. Existing numeric channels
still work. Use either `valves` or `channels` per step. `valves` accepts only
names; an empty list waits with all outputs inactive. Names are case sensitive.
The loader validates every referenced name and resolves it to a GPIO channel
before running or generating firmware. Unknown/unassigned names, duplicate
resolved channels, invalid channel numbers and duplicate mapping assignments
are rejected. Set an unknown assignment to `null` to prevent its use.

The included `sequences/single_valve.json` and `sequences/multi_valve.json` use
names now, with their existing durations and provisional channel selections
preserved. `sequence.named.example.json` is another example outside the compiled
sequence directory. The host runner supports names through the same loader:

```sh
python tests/valve_gpio/run_sequence.py tests/valve_gpio/sequences/multi_valve.json --dry-run
```

Embedded builds track mapping changes automatically. After editing the mapping
or a sequence, rebuild and flash:

```sh
west build -d tests/valve_gpio/build-sequence
west flash -d tests/valve_gpio/build-sequence
```

A custom mapping can be selected with `--valve-map /path/map.json` in the host
runner/generator, or `-DVALVE_MAP_FILE=/path/map.json` in the CMake arguments.
The board still executes compiled channel masks; its USB/Ethernet `vlvseq`
commands and timing behavior are unchanged.

## Run a sequence file

`run_sequence.py` reads a JSON array of steps; see `sequence.example.json`.
Each step supplies `duration_ticks` (1–2147483647 milliseconds) and a list of GPIO `channels` (0–31).
An empty list waits with outputs inactive. Each nonempty step activates its
channels together, holds them for the duration, and deactivates them before
acknowledging completion. Consecutive steps have an off interval and host/USB
latency between them; this is not a precise timestamp scheduler. Group activation
uses consecutive GPIO writes, so rising edges have I2C skew. Channels describe
energized outputs, not necessarily mechanically open valves.

Flash the USB manual firmware first:

```sh
west build -p always -s tests/valve_gpio -b ranger_1/mimxrt1062 \
  -d tests/valve_gpio/build-manual -- -DCONFIG_VALVE_TEST_EMBEDDED_SEQUENCE=n
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
comments. Up to 16 files are supported. Names must be 1–48 characters: letters, digits, underscores, dots or
hyphens; the first character must be a letter, digit or underscore.

Build and flash from the repository root:

```sh
west build -p always -s tests/valve_gpio -b ranger_1/mimxrt1062 \
  -d tests/valve_gpio/build-sequence
west flash -d tests/valve_gpio/build-sequence
```

### Persistent manual control over Ethernet

With the updated standalone sequence firmware flashed, enter `vlvseq` in
`scripts/client-new.py`, then `v` for the valve selector and ON/OFF
prompt. Use `r` to release overrides. The selected valve appears in a bordered box. Both selectors use the same valve mapping. Every
action returns to `VLVSEQ>`; only `back` or `exit` leaves this mode.

Manual overrides take priority over sequence requests, including manual OFF.
Sequences may start with overrides present, and manual commands work during a
running sequence. `r` removes the selected override and immediately follows
the current sequence request (OFF when idle). Sequence completion and disconnect
do not clear overrides. Only one sequence may run at a time.

`abort` cancels a pending client countdown, clears all sequence requests and manual
overrides, and attempts to deactivate every output. GPIO faults also clear control
requests, attempt every output OFF, and remain latched until reset. Commands are
never replayed automatically after a lost acknowledgement. Rebuild and flash the
firmware for override support; the client refuses manual override commands on
older firmware. Keep the client mapping consistent with the firmware build.

Status tables show GPIO readback, sequence requests, global manual overrides and
this client's last accepted ON/OFF commands independently. `release` clears the
session indicator for that channel. Neither a command nor GPIO readback proves
mechanical valve position.

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
sequence abort
trace
```

Wait for completion before starting another run. The shell remains responsive
while the main thread executes the sequence. `sequence status` reports
`initializing`, `ready`, `running`, `aborting`, `aborted`, `complete`, or `faulted`,
plus the selected name, current step, total steps, comment and error code.
`ready`, `complete` and `aborted` allow another run. A second start while running
is rejected. Unknown names do not activate any output. `trace` retrieves recent
logs if you connected after startup. Use a generic USB serial terminal if your
monitor does not recognize the board's USB CDC device.

An initialization, activation, or deactivation error prevents further runs until
reset. After an activation or deactivation error the firmware attempts to turn
every output off. Disconnecting the monitor does not cancel a running sequence:
it completes on the board. `sequence abort` interrupts the timed wait and requests
outputs off; wait for `aborted` or `faulted` in status before starting again.
Firmware crashes and failed I2C off writes still require independent removal
of valve power.

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

Each step requests its channels ON, waits, then clears its sequence requests.
Manual overrides take priority throughout. An empty step leaves only manual
overrides active. Group edges use sequential
I2C writes, not hardware synchronization. Comments and logs add timing overhead.
The build rejects invalid steps, names, and channels absent from the board.
The base `prj.conf` selects named sequences with manual start. The separate
mode configuration files are no longer needed. Verify first with
loads disconnected and restore normal Clover firmware after testing.

Host validation and mocked firmware behavior tests (no board required):

```sh
python -m unittest discover -s tests/valve_gpio/test -v
```

These exercise the actual C control flow with mock GPIO and timing, plus the C
TCP server and nanopb encoding over real localhost sockets with the Python client.
The Ethernet tests require a west workspace with nanopb and permission to open
loopback sockets. They do not verify physical outputs, USB enumeration, Ethernet
hardware, or real scheduling on the board.

## Ethernet connection

The test firmware enables Ethernet with Clover's direct-connection settings:
static IPv4 `169.254.99.99`, netmask `255.255.0.0`. Connect the Ethernet cable
and give your computer's Ethernet adapter a different address on the same subnet
(for example `169.254.99.1` with that netmask). Check connectivity from the host:

```sh
ping 169.254.99.99
```

The test overlay selects a device-derived unicast MAC instead of the board's
fixed multicast MAC. If flashing fails while requesting a reset, press the
Teensy Program button to enter its bootloader, then retry `west flash`. A failed
upload does not confirm the new firmware is running.

In the USB shell, `net iface` and `net ipv4` show interface and IPv4 state.
Networking initialization does not wait for a cable before starting the USB
shell. Valve sequences still require an explicit USB command.

The default named-sequence build (`prj.conf`) also runs a TCP command server on
port **19690**, with Clover's length-prefixed protobuf Request/Response framing.
It accepts named-sequence requests and the existing `AbortRequest`; ordinary Clover control and
telemetry requests return an explicit unsupported-command error. USB and TCP
share one sequence engine, so they cannot start overlapping tests. Up to two
TCP clients can connect at once. A disconnected client does not cancel a run.

Flash the named-sequence build, then from the repository root run:

```sh
python scripts/client-new.py --no-data --host 169.254.99.99
```

At `CMD>` enter:

```text
vlvseq
```

Inside the `VLVSEQ>` prompt, use `list`, `run single_valve`, `v`,
`v`, `r`, `status`, or `abort`. `back` or `exit` returns to the main client prompt without
stopping the board. Ctrl+C shows a reminder to use `abort`; it does not send a
abort automatically.

The mode displays sequence names in a table and timestamped state/step changes
with the current JSON comment. Progress is polled every 200 ms and printed above
the prompt while it remains available for commands. Unchanged status is not
repeated. This reads status snapshots, so steps shorter than the polling interval
may not appear; full retained GPIO events remain available through USB `trace`.
The combined mode requires the updated firmware and client.

Or send a single command and exit, without subscribing to telemetry:

```sh
python scripts/client-new.py --host 169.254.99.99 --command "sequence list"
python scripts/client-new.py --host 169.254.99.99 --command "sequence run single_valve"
python scripts/client-new.py --host 169.254.99.99 --command "sequence status"
python scripts/client-new.py --host 169.254.99.99 --command "sequence abort"
```

RUN acknowledges queuing, not sequence completion. The `vlvseq` mode automatically
shows progress; one-shot commands can poll status manually. ABORT acknowledges
the request; confirm `aborted` or
`faulted` before another run. GPIO faults require reset. `abort` sends the same protobuf `AbortRequest` used by
Clover's top-level abort command (not `NamedSequenceRequest.STOP`). In this
standalone firmware its handler cancels a run and de-energizes all valve GPIOs,
including when idle; it does not link or execute Clover's `Controller::handle_abort()`.
A fault remains latched even if a later abort successfully writes all outputs off.
A successful abort allows another test run. The legacy STOP wire operation is
still accepted for older clients, but the new CLI and USB command are `abort`.
Sequence commands are
never automatically resent after a lost acknowledgement; reconnect and check
status before attempting another run. `--port` overrides the TCP port if needed.
Sequence names are case sensitive.

The wire extension is in `api/clover.proto` (`Request.named_sequence`, tag 40;
`Response.named_sequence`, tag 2). Existing field numbers are unchanged. This
implements Ethernet commands in the standalone bench firmware, not production
Clover's controller or valve-assignment API. The client can continue to use its
existing commands with normal Clover firmware; named sequences need this test
firmware. Integrating them into production Clover remains separate work.

For a routed/DAQ network, change the IPv4 address, netmask and gateway settings
in `prj.conf` to match that network, then rebuild and flash.

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

Named-sequence firmware enables USB CDC and emits
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

Interactive `vlvseq` → `run <name>` schedules a ten-second countdown while the
prompt remains available for manual commands. Space or Q at an empty prompt cancels the pending start.
`sequence run <name>` at the top-level prompt retains its blocking countdown. Press Space or Q (without
Enter, at an empty prompt) to cancel with the connection open. Ctrl+C
also cancels during the countdown. Once RUN is sent, use `abort` to stop the
board. Redirected/nonterminal input skips the countdown for automation. The
countdown appears in client output, not board logs; direct USB commands are
unchanged.

The client `status` dashboard retains telemetry and polls valve readback alongside
it. `sequence status` and `vlvseq` also show every valve in `valve_map.json` with
ON/OFF logical GPIO readback, UNKNOWN for unavailable readings or older firmware,
and UNASSIGNED for null mappings. Readback does not confirm mechanical position.
Rebuild/flash embedded sequence firmware and restart the client for this protocol
extension.

Sequence JSON uses the field name `duration_ticks`, with values interpreted as
milliseconds (1–2147483647 per step): `5000` means 5 seconds. This is a field rename
only; it does not use scheduler tick units. Embedded
sequences use `K_MSEC`; the USB runner also accepts milliseconds.

The valve tables also show “This client: last accepted command” for manual GPIO
commands acknowledged during the current connection. Requested ON/OFF is kept
separate from GPIO readback, including when they disagree. Valves powered by
other means remain visible without a client command indicator. This history
resets on reconnect and does not establish ownership or mechanical position.

Inside `vlvseq`, `r` opens a numbered valve selector with a highlighted
selection box. Choose `all` to release every override reported by the board,
including unmapped channels. Release
returns valves to the current sequence request, or OFF when idle, and stays in
`vlvseq`. The all operation sends one command per overridden channel; it stops
on failure without retrying. Overrides added by other clients during the batch
may remain; use status to inspect the result.

To release all from the selector, enter `r` at `VLVSEQ>`, then type `all` at
the release selection prompt. You remain in `vlvseq` afterward.
