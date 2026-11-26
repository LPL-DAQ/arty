# Ranger valve sequencing

The Ranger 1 Clover build includes named sequences and manual valve control.
Follow the [Clover build and flash instructions](../../README.md). The firmware
starts with valve outputs inactive and waits for an explicit command.

## Mapping and sequences

`valve_map.json` maps case-sensitive valve names to zero-based indices in
`zephyr,user/valve-gpios` in the board DTS. The assignments follow the Ranger
harness table: channel = GPIO_N − 1, where GPIO_N is the table's "GPIO Expander
Channel" (PCB net). All named valves are on U4, expander 1:

| Valve | Channel | PCB net | U4 pin | Expander pin | Supply |
| --- | --- | --- | --- | --- | --- |
| SV003 | 0 | GPIO_1 | U4.1 | 8 / GPB0 | 24 V |
| SV301 | 2 | GPIO_3 | U4.3 | 10 / GPB2 | 24 V |
| SV201 | 4 | GPIO_5 | U4.5 | 12 / GPB4 | 24 V |
| SV001 | 12 | GPIO_13 | U4.25 | 4 / GPA4 | 12 V |
| SV002 | 10 | GPIO_11 | U4.23 | 2 / GPA2 | 12 V |
| PBV003 | 8 | GPIO_9 | U4.21 | 0 / GPA0 | 12 V |

Connector pins and the unnamed outputs are listed in
`tests/valve_gpio/README.md`. Set an unknown assignment to `null` to prevent its
use. The client reads this file, and `tests/valve_gpio/valve_map.json` must stay
identical to it.

Put named sequence JSON files directly in `sequences/`. The filename without
`.json` is the command name. Each file contains an array of steps, for example:

```json
[
  {"duration_ticks": 1000, "valves": ["SV001"], "comment": "Power SV001"},
  {"duration_ticks": 1000, "valves": [], "off_valves": ["SV001"]}
]
```

`duration_ticks` means milliseconds and must be between 1 and 2147483647.
Use `valves` for named channels or `channels` for numeric channels. Optional
`off_valves` or `off_channels` explicitly hold outputs off; ON and OFF lists
must not overlap. An empty step delays without claiming valves. Comments may
contain up to 100 UTF-8 bytes of printable text. Group writes are sequential,
so their edges are not simultaneous.

Each step takes control of the valves it names, clearing their manual settings;
other manual outputs remain unchanged. Consecutive steps can keep a valve on.
Manual commands are rejected while a sequence is running. GPIO faults latch
until reset. Abort clears sequence requests and manual overrides and attempts
to turn every output off; confirm the reported state before another run.
A TCP abort also triggers Clover's controller abort.

Adding, removing, or editing JSON files or changing the mapping requires a
rebuild and flash. The normal build embeds every `*.json` in `sequences/`.
`-DVALVE_SEQUENCE_DIR=/absolute/path` selects another directory;
`-DVALVE_SEQUENCE_FILE=/absolute/path/file.json` selects one file, and
`-DVALVE_MAP_FILE=/absolute/path/map.json` selects a custom map. Pass CMake
options after `--` in the `west build` command. Clear a previous single-file
override with `-UVALVE_SEQUENCE_FILE` after `--` to return to directory mode.

To validate a sequence without a board:

```sh
python clover/src/sequencing/run_sequence.py clover/src/sequencing/sequences/multi_valve.json --dry-run
```

## Control and status

Connect to port 19690 with `python client/client-new.py --no-data --host
169.254.99.99`. Enter `vlvseq` for `list`, `run NAME`, manual valve controls,
`status`, and `abort`. The interactive run uses a countdown; once a run is sent,
use `abort` to stop it. A RUN acknowledgement means queued, not complete. Do not
automatically retry after a lost acknowledgement; reconnect and check status.

The USB shell also accepts `sequence list`, `sequence run NAME`, `sequence
status`, `sequence abort`, and `trace`. The last 32 valve events are retained in
Clover; automatic shell log output is disabled. A client disconnect does not
stop a running sequence. The status dashboard displays GPIO readback and the
board's control source. Neither readback nor a successful command confirms
mechanical valve position. Test first with loads disconnected; hardware power
removal remains necessary if firmware or an I2C off write fails.

## Host checks

```sh
python -m unittest discover -s tests/clover/sequencing -v
```

The suite includes generator and mocked GPIO checks, client display and command
checks, and localhost TCP/protobuf tests using a test-only server harness. TCP
tests require the west nanopb module and permission to open loopback sockets.
These tests do not verify real GPIO, Ethernet hardware, or timing on the board.
