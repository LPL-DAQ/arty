# Valve Sequencing and Manual Testing

## Overview

- Built a standalone Ranger valve-testing application in `tests/valve_gpio`.
- Supports individual GPIO checks, timed valve groups, named sequences, and persistent manual ON/OFF control.
- Added USB and Ethernet commands, with sequence and manual controls integrated into the Python client.
- Sequences execute on the board after an explicit start; the laptop can monitor progress without controlling step timing.

## 1. GPIO Testing and Firmware Modes

**Files:**

- `tests/valve_gpio/src/main.c`
- `tests/valve_gpio/CMakeLists.txt`
- `tests/valve_gpio/Kconfig`
- `tests/valve_gpio/prj.conf`

**What it does:**

- Two selectable firmware modes:
  - Interactive USB commands: list channels, pulse one channel, or activate a group.
  - Named sequences: initialize inactive and wait for an explicit USB or Ethernet run command.
- Read channel definitions from the board's `valve-gpios` devicetree property.
- Initialize every output inactive; block testing if initialization fails.
- Validate channel numbers and durations before changing outputs.
- For group commands, reject duplicate channels and attempt every requested deactivation even after a write error.
- Return a caller-specific `VALVE_DONE` token after group cleanup so the host can identify the correct completion.
- CMake selects the default named-sequence entry point or the legacy USB manual entry point.

## 2. Sequence format and valve mapping

**Files:**

- `tests/valve_gpio/run_sequence.py`
- `tests/valve_gpio/valve_map.json`
- `tests/valve_gpio/sequences/single_valve.json`
- `tests/valve_gpio/sequences/multi_valve.json`
- `tests/valve_gpio/sequence.example.json`
- `tests/valve_gpio/sequence.named.example.json`

**What it does:**

- Define sequences as JSON steps containing a duration, valve names or channel numbers, and an optional comment.
- Resolve valve names through one mapping file instead of repeating channel assignments in each sequence.
- Share `load_sequence()` between the USB runner and firmware generator.
- Validate the complete input before execution or code generation:
  - Nonempty sequence; duration of 1–10000 ms per step.
  - Exactly one of `valves` or `channels` per step.
  - Unique resolved channels within 0–31.
  - Valid valve names and assignments; reject unknown names and duplicate mappings.
  - Printable comments limited to 100 UTF-8 bytes.
- Allow `null` mapping entries for valves whose wiring is not assigned yet.
- Support empty lists for delays without energizing outputs.

## 3. Host runner and embedded sequence generation

**Files:**

- `tests/valve_gpio/run_sequence.py`
- `tests/valve_gpio/generate_sequence.py`
- `tests/valve_gpio/CMakeLists.txt`

**What it does:**

- Added a USB runner that sends one group command at a time and waits for its matching completion token.
  - Stops subsequent steps on an error or missing acknowledgement.
  - Supports `--dry-run` validation without hardware.
  - Implements empty steps as host-side delays.
- Added a generator that converts validated JSON into `valve_sequence.h`.
  - Each step becomes a duration, a 32-bit channel mask, and a comment.
  - Each JSON filename becomes a selectable sequence name.
  - Supports up to 16 sequences, with names up to 48 characters.
- Track sequence files, directory membership, mapping changes, and generator/loader changes in CMake.
- Check generated channel requirements against the board at compile time.

**Implementation tradeoff:**

- Embedded sequences need no JSON parser on the board and run independently of the host.
- Editing a sequence or mapping requires rebuilding and flashing; selecting an already compiled sequence does not.
- The USB runner still depends on the host to advance between steps.

## 4. On-board sequence execution

**Files:**

- `tests/valve_gpio/src/embedded_sequence.c`
- `tests/valve_gpio/src/sequence_engine.h`

**What it does:**

- Put execution behind a shared engine used by both USB and Ethernet commands.
- Track initialization, readiness, running, completion, abort, and fault states.
- Queue accepted runs for the main thread rather than running timed steps inside command handlers.
- Use a mutex for command admission and consistent status snapshots.
- Use separate semaphores to queue a run and interrupt a timed wait for abort.
- Execute each step by activating its mask, waiting, and attempting to turn every output off.
- Publish sequence name, step number, total steps, comment, and error status.
- Reject overlapping runs and latch GPIO faults until restart.

**Points to explain:**

- A successful RUN response means accepted/queued; status shows completion.
- Every step ends with outputs commanded inactive, so consecutive steps have an off interval.
- Group activation uses sequential I2C writes, so edges are not simultaneous.
- Timing includes GPIO, logging, and scheduling overhead; this is not an exact absolute-time scheduler.

## 5. Manual control and abort behavior

**Files:**

- `tests/valve_gpio/src/embedded_sequence.c`
- `tests/valve_gpio/src/sequence_engine.h`
- `tests/valve_gpio/src/sequence_protocol.c`
- `scripts/client-new.py`

**What it does:**

- Added persistent manual ON/OFF control through the client's `v` command.
- Read valve choices from `valve_map.json` and send the selected channel and power state.
- Track successful manual commands in a bitmask and show manually powered outputs, including changes from other clients.
- Prevent manual commands during a run and prevent sequence starts while manual outputs are ON.
- Make abort handle both running sequences and idle manual outputs:
  - Running: wake the timed wait and let the worker clean up.
  - Idle: attempt to deactivate every output directly.
- Route the shared `AbortRequest` and legacy sequence STOP operation to the same engine abort function.
- Keep faults latched even if a later abort succeeds.

## 6. Ethernet and shared protocol

**Files:**

- `tests/valve_gpio/src/sequence_server.c`
- `tests/valve_gpio/src/sequence_protocol.c`
- `tests/valve_gpio/src/sequence_protocol.h`
- `tests/valve_gpio/prj.conf`
- `tests/valve_gpio/app.overlay`
- `api/clover.proto`
- `api/clover.options`
- `scripts/clover_pb2.py` — generated bindings
- `clover/src/server.cpp`

**What it does:**

- Added a TCP server on port 19690 using Clover's length-prefixed protobuf framing.
- Separate socket/framing handling from request dispatch and GPIO execution.
- Support two clients with independent partial-frame state.
- Bound request size, time out incomplete frames, and limit slow-reader stalls.
- Add named-sequence requests/status and manual-valve requests to the shared schema.
- Dispatch list, run, status, manual control, and abort to the engine; reject unrelated commands in bench firmware.
- Explicitly reject named GPIO sequences in production Clover, which does not contain the bench engine.
- Configure static IPv4 `169.254.99.99`, a device-derived unicast MAC, and USB CDC shell/logging.

## 7. Python client and progress display

**Files:**

- `scripts/client-new.py`
- `scripts/named_sequences.py`
- `scripts/valve_sequence_cli.py`

**What it does:**

- Add one-shot `sequence list`, `run`, `status`, and `abort` commands.
- Add `vlvseq`, an interactive mode with a sequence list and automatic progress updates.
- Poll status every 200 ms and print changes without repeatedly printing identical snapshots.
- Keep the command prompt usable while progress appears above it.
- Lock the full request/response transaction so polling and user commands cannot consume each other's replies.
- Stop and join the polling thread before returning socket ownership to the main client.
- Read complete length-prefixed replies and distinguish server rejections from transport/framing failures.
- Close a broken connection and avoid automatically replaying commands after a lost acknowledgement.

## 8. Diagnostics and supporting changes

**Files:**

- `tests/valve_gpio/src/test_trace.h`
- `tests/valve_gpio/README.md`
- `flasherd/client.py`
- `scripts/pyproject.toml`

**What it does:**

- Log initialization, GPIO activity, sequence steps/comments, cleanup errors, and final state.
- Retain the latest 128 events with time since boot for retrieval through USB `trace`.
- Document build/flash commands, wiring assumptions, measurement procedures, and client usage.
- Propagate nonzero remote flasherd exit codes to the calling process.
- Remove the temporary `rich` workspace example; retain the client UI's `rich` dependency.

## 9. Validation

**Files:**

- `tests/valve_gpio/test/test_sequences.py`
- `tests/valve_gpio/test/test_ethernet.py`
- `tests/valve_gpio/test/host_zephyr.h`

**Coverage:**

- Sequence generation, input validation, valve-name resolution, and mapping errors.
- Actual embedded C control flow with mocked GPIO and timing.
- Engine, protocol adapter, and TCP server exercised through localhost sockets.
- Repeated runs, busy rejection, abort, cleanup, and latched faults.
- Manual persistence, multiple clients, fragmented messages, and lost acknowledgements.
- Client command routing and progress display.

## 10. Example autonomous execution of sequence

```sh
west build -p always -s tests/valve_gpio -b ranger_1/mimxrt1062 \
  -d tests/valve_gpio/build-sequence
west flash -d tests/valve_gpio/build-sequence
python scripts/client-new.py --no-data --host 169.254.99.99
```

- At `CMD>`, enter `vlvseq`.
- At `VLVSEQ>`, enter `list`, then `run single_valve`.
- Watch progress; use `status` for an explicit snapshot.
- For an interrupted run, enter `abort` and wait for `aborted` or `faulted`.

### Tracing execution

| Stage | File / function | What happens |
| --- | --- | --- |
| Sequence input | `tests/valve_gpio/sequences/single_valve.json` | Defines eight 5-second steps, alternating inactive waits and individual powered outputs. |
| Name resolution | `tests/valve_gpio/valve_map.json` | Resolves SV001, SV002, SV003, and SV004 to channels 0, 1, 6, and 7 in the current mapping. |
| Validation | `tests/valve_gpio/run_sequence.py` → `load_sequence()` | Checks durations, names, comments, and channels; returns normalized steps. |
| Code generation | `tests/valve_gpio/generate_sequence.py` | Emits static step tables and masks into the build's generated `valve_sequence.h`. |
| Firmware build | `tests/valve_gpio/CMakeLists.txt` | Compiles the generated tables with the engine, protocol adapter, and TCP server. |
| Board startup | `tests/valve_gpio/src/embedded_sequence.c` → `main()` | Configures outputs inactive, publishes ready state, and waits on `run_requested`. |

### During the run: command and execution path

1. **Enter sequence mode — `scripts/client-new.py`.**
   - `cmd_valve_sequences()` creates `ValveSequenceMode` using the existing command socket.
2. **Submit the command — `scripts/valve_sequence_cli.py`.**
   - `execute('run single_valve')` asks `named_sequences.make_request()` to create a RUN request.
   - Locks the socket transaction so background status polling cannot interfere.
3. **Send the request — `scripts/named_sequences.py`.**
   - `exchange()` serializes the protobuf and sends its length prefix and payload once.
4. **Receive and dispatch — firmware transport.**
   - `src/sequence_server.c` → `serve()` assembles the frame and decodes the request.
   - `src/sequence_protocol.c` → `sequence_handle_request()` routes RUN to `sequence_start()`.
5. **Accept the run — `src/embedded_sequence.c`.**
   - `sequence_start()` checks the name, state, faults, and manual-output mask.
   - Records the selection, marks running, and gives `run_requested`.
   - The reply reports acceptance and a status snapshot; the worker can already be progressing by the time it arrives.
6. **Execute independently — `main()` → `run_sequence()`.**
   - Main wakes, walks the compiled steps, and publishes the current step/comment.
   - Activates each mask, waits on `abort_wakeup` with the step duration as timeout, then calls `all_off()`.
   - At the end, publishes complete; GPIO errors instead produce a latched fault.
7. **Display progress — back through the response path.**
   - The client watcher requests STATUS about every 200 ms over the same transport.
   - `sequence_status()` takes a consistent snapshot; the protocol adapter encodes it in the response.
   - `show_status()` prints state/step changes. `test_trace.h` also retains firmware events for USB `trace`.

### Expected output sequence to talk through

| Approximate interval after execution begins | Commanded outputs |
| --- | --- |
| 0–5 s | All inactive |
| 5–10 s | SV001 / channel 0 active |
| 10–15 s | All inactive |
| 15–20 s | SV002 / channel 1 active |
| 20–25 s | All inactive |
| 25–30 s | SV003 / channel 6 active |
| 30–35 s | All inactive |
| 35–40 s | SV004 / channel 7 active |
| Completion | All outputs commanded inactive; state complete |

## 11. Run of manual valve ON/OFF testing


### Commands and expected behavior

1. At the main `CMD>` prompt, enter `v`.
2. Select SV001 from the displayed list; the current mapping resolves it to channel 0.
3. Select **Power ON**.
   - The board energizes channel 0 and the client lists SV001 among the manually powered outputs.
   - There is no timed automatic OFF in this mode.
4. Enter `v` again, select SV001, and choose **Power OFF**.
   - The board de-energizes channel 0 and clears its manual-output bit.
   - If no other outputs are manually powered, the client reports none.
5. Optional interaction check:
   - With a manual output ON, try `sequence run single_valve`; the start is rejected.
   - Use `sequence abort` to request all outputs OFF, then inspect `sequence status`.
   - With a sequence running, `v` requests are rejected until the run ends or abort cleanup finishes.

### Execution path

| Stage | File / function | What happens |
| --- | --- | --- |
| Prompt and selection | `scripts/client-new.py` → `cmd_actuate_valve()` | Reads the mapping, displays assigned valves, and asks for ON/OFF. |
| Mapping input | `tests/valve_gpio/valve_map.json` | Supplies the selected valve's channel on the host. The board receives a channel number rather than resolving the valve name. |
| Request construction | `cmd_actuate_valve()` | Sets `Request.manual_valve.channel` and `.powered`. |
| Wire exchange | `scripts/named_sequences.py` → `exchange()` | Sends one length-prefixed protobuf request and waits for its response without automatic retry. |
| TCP reception | `src/sequence_server.c` → `serve()` | Reassembles and decodes the complete frame. |
| Request routing | `src/sequence_protocol.c` → `sequence_handle_request()` | Recognizes the manual payload and calls `sequence_manual_valve(channel, powered)`. |
| GPIO operation | `src/embedded_sequence.c` → `sequence_manual_valve()` | Checks channel/state under the mutex, writes the GPIO, and updates the manual mask on success. |
| Fault handling | `sequence_manual_valve()` → `all_off()` | On a GPIO write error, attempts to deactivate all outputs and latches a fault. |
| Response | `sequence_status()` → protocol adapter → TCP server | Returns engine state and the manual-output mask when idle and not faulted. |
| Client display | `cmd_actuate_valve()` | Maps returned bits back to valve names and prints the manually powered outputs. |
