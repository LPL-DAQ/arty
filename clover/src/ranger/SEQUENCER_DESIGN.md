# Sequencer

The Ranger sequencer runs timed autonomous valve events from ignition `.log` files. `.log` is the only supported file format; JSON and YAML definitions are not accepted.

## Loading a sequence

The controller accepts a sequence filename through `RunAutonomousValveSequenceRequest` and reads it from `/SD:/sequences/`. Filenames must end in lowercase `.log`, contain only letters, digits, `_`, `-`, and `.`, must not contain `..`, and must fit the request/file-name size limit. The file is limited to 8 KiB.

The optional `run_index` selects a zero-based ignition run and defaults to `0`. A log can contain up to 64 runs, and each selected run can contain up to 64 valve events.

## Log format

The parser recognizes ignition markers, valve event lines, and termination markers:

```text
[USER] IGNITION 10.000
[IGNITION] PBV001 CLOSED -> OPEN 10.250
[IGNITION] SV001 OPEN -> CLOSE 10.500
[USER] IGNITION_TERMINATED 11.000
```

Timestamps are seconds with an optional decimal fraction. Event times are normalized relative to the selected run's ignition marker. The event target after `->` is executed; the preceding valve state is required by the log format but does not determine the command. The termination marker determines the sequence duration.

Other nonempty lines outside a run and non-ignition lines inside a run are ignored. A selected run must have a start, a matching termination, and at least one supported valve event. Malformed ignition event lines and invalid timing are rejected.

## Controller integration

The controller starts a loaded sequence immediately when it receives the run request, and only from IDLE. Each control tick advances the scheduler and actuates events whose timestamps have passed. Before starting, the controller verifies that each referenced valve reports an unpowered safe state. On completion, halt, abort, or command failure, it attempts to restore those initial safe states.
