# Sequencer Plan

The goal is to add a reusable sequence runner in the Ranger layer that can load YAML or JSON definitions, validate them, normalize them into a single runtime format, and then drive control behavior through the existing controller state machine.

## Baseline

The current controller model already handles discrete control modes like idle, primed, active, and abort. The missing piece is a way to load a sequence from a file or external definition and run it in the same pattern as the existing traced commands.

The sequence runner should give us a clean separation between:
- file parsing / input format handling
- sequence validation
- runtime sampling and timing
- control-state transitions and actuator mapping

This is important because sequence definitions likely need to be editable outside the firmware, while the runtime behavior still needs to be deterministic and safe inside the controller.

## Implementation

I would add a dedicated sequence utility under the Ranger module:

- clover/src/ranger/Sequencer.h
- clover/src/ranger/Sequencer.cpp

The runtime object should not expose YAML or JSON specifics. It should only expose a normalized sequence representation that the controller consumes.

Conceptually:

```cpp
namespace ranger {

struct SequenceEvent {
    uint32_t time_ms;
    std::string action;
    float value;
};

class Sequencer {
public:
    bool load_yaml(std::string_view path);
    bool load_json(std::string_view path);
    bool load_from_buffer(std::string_view text, bool is_json);

    bool validate() const;
    std::expected<float, Error> sample_at_ms(uint32_t time_ms) const;
    const std::vector<SequenceEvent>& events() const;

private:
    std::vector<SequenceEvent> events_;
};

} // namespace ranger
```

This keeps the sequence parser independent from the control logic. The controller can just ask for the command at the current time instead of caring about file format or parsing logic.

## Parsing/Normalization

The system should accept a controlled schema, not arbitrary loose data.

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