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

<<<<<<< HEAD
The controller starts a loaded sequence immediately when it receives the run request, and only from IDLE. Each control tick advances the scheduler and actuates events whose timestamps have passed. Before starting, the controller verifies that each referenced valve reports an unpowered safe state. On completion, halt, abort, or command failure, it attempts to restore those initial safe states.
=======
Example YAML:

```yaml
name: ignition_sequence
duration_ms: 5000
events:
  - time_ms: 0
    action: valve_open
    value: 1.0
  - time_ms: 1000
    action: throttle
    value: 12.5
```

Example JSON:

```json
{
  "name": "ignition_sequence",
  "duration_ms": 5000,
  "events": [
    { "time_ms": 0, "action": "valve_open", "value": 1.0 },
    { "time_ms": 1000, "action": "throttle", "value": 12.5 }
  ]
}
```

Both should be normalized into the same in-memory event structure before the controller sees them. That lets us support multiple source formats without branching logic through the control state machine.

The parser should reject malformed or unsupported definitions early. I would rather fail fast on a bad sequence than accidentally run a partially decoded or ambiguous sequence.

## Controller State Machine Integration

The controller already has a clear state pattern, so the sequence feature should fit into that pattern instead of creating a separate ad hoc execution path.

I’d add a new sequence state in the protobuf state enum:

```proto
enum SystemState {
  STATE_UNKNOWN = 0;
  STATE_IDLE = 1;
  STATE_ABORT = 2;
  STATE_SEQUENCE_PRIMED = 19;
  STATE_SEQUENCE = 20;
}
```

This gives the following flow:
- IDLE -> load sequence
- SEQUENCE_PRIMED -> start sequence
- SEQUENCE -> active sequence execution
- abort/halt -> return to idle or abort flow

I’d also add a corresponding request pair:

```proto
message LoadSequenceRequest {
  required string sequence_name = 1;
  optional string sequence_file = 2;
}

message StartSequenceRequest {}
```

This matches the existing "load then start" pattern already used elsewhere in the controller API.

## Controller Behavior

Once a sequence is loaded, the controller should treat it as a runtime object with deterministic timing. During each control tick, it would sample the active sequence at the current time and map the output into the correct actuator command.

Conceptually:

```cpp
if (current_state == SystemState_STATE_SEQUENCE) {
    auto sample = active_sequence.sample_at_ms(data.trace_time_msec);
    if (!sample.has_value()) {
        return std::unexpected(sample.error().context("failed to sample active sequence"));
    }

    // map sample to valve / throttle / TVC / other command
}
```

That keeps the logic consistent with the existing trace-driven control patterns already in the controller.

## Separation of Responsibilities

The important design principle is to keep responsibilities clean:

- Ranger owns parsing and runtime sequence behavior
- Controller owns state transitions and command mapping
- proto owns the external interface and state model

This keeps the controller from becoming a parser, a state machine, and an actuator mapper all in one file.

## Open Decisions

The main details still to settle are:
- exact schema for sequence files
- whether YAML support should be a limited subset or a more general parser
- exact list of supported action types (valves, throttle, TVC, RCS, etc.)
- how to handle invalid timing / out-of-order events / missing values
- whether the controller should accept sequence files from external storage or only from a known internal path

Those are the main open design questions before implementation.

## Summary

The basic direction is straightforward:
- parse YAML/JSON into a normalized runtime format
- validate before execution
- add a sequence state in the controller state machine
- sample the sequence during control ticks and map each sample to the correct actuator output

This keeps the design modular and consistent with the rest of the codebase, while still leaving room for the exact event schema and runtime behavior to be finalized.

test 3
>>>>>>> 89c1802f2919ca6830d1aacac962789b436f859b
