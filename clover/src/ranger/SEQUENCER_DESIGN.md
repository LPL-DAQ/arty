# Autonomous Valve Sequencer

The Ranger sequencer executes timed valve-position events parsed from ignition `.log` files. JSON and YAML are not supported.

## Loading and running sequences

Upload a laptop file to device RAM without starting it:

```shell
uv run scripts/upload-sequence.py --ip 169.254.99.99 upload path/to/ignition.log
```

Then explicitly request the run:

```shell
uv run scripts/upload-sequence.py --ip 169.254.99.99 run ignition.log
```

Uploads are sent in ordered 512-byte chunks. The controller validates the complete selected run before replacing the last valid upload. The upload remains in volatile device RAM until reboot; it is not written to SD. Alternatively, the controller can load a file from `/SD:/sequences/`. Upload and run requests are accepted only in IDLE.

The run index is zero-based and defaults to `0`. When uploading another run with `--run-index N`, pass the same `--run-index N` to the run command. Files must be ASCII `.log` basenames, at most 48 characters, containing only letters, digits, `_`, `-`, and `.`; `..` is disallowed. Maximum file size is 8 KiB, with at most 64 ignition runs and 64 events per run.

## Completion behavior

By default, normal completion restores the captured unpowered starting states for the valves used by the sequence. The controller retains its existing safe-state checks in both modes. For testing, pass `--continue-valve-states` to `run`; upon normal completion, valves remain at the final positions commanded by the sequence instead of being restored.

The continue option applies only to normal completion. Abort and valve-command failure still attempt to restore captured starting states regardless of the selected mode. No valve states are inferred from one another: the normal safe-return mode uses each referenced valve's own captured unpowered state, while continue mode explicitly leaves the last requested position.

## Event timing and acknowledgment

An event is a position command, not a one-millisecond pulse. On each control tick, the scheduler reports at most the next due event. The controller issues the corresponding valve command and acknowledges the event only after that command succeeds. Until acknowledgment, later calls return the same event; a command failure therefore cannot silently consume the event.

If a tick is late and multiple events are overdue, the controller issues them in log order, one event per tick. This preserves every transition and avoids collapsing catch-up commands into one burst, but events may occur later than their timestamps when the controller is behind.

## Log format

The parser recognizes ignition markers, valve event lines, and termination markers:

```text
[USER] IGNITION 10.000
[IGNITION] PBV001 CLOSED -> OPEN 10.250
[IGNITION] SV001 OPEN -> CLOSE 10.500
[USER] IGNITION_TERMINATED 11.000
```

Timestamps are seconds with an optional decimal fraction. Event times are normalized relative to the selected run's ignition marker. The target after `->` determines the commanded position; the preceding state is required by the log format but does not determine the command. The termination marker sets the sequence duration.

Nonempty lines outside a run and non-ignition lines inside a run are ignored. A selected run must have a start, a matching termination, and at least one supported valve event. Malformed ignition event lines and invalid timing are rejected before execution.
