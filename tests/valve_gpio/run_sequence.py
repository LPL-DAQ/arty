#!/usr/bin/env python3
"""Run timed GPIO bench-test groups through the manual firmware's USB shell."""
import argparse
import json
from pathlib import Path
import re
import secrets
import time


DEFAULT_VALVE_MAP = Path(__file__).with_name("valve_map.json")


def load_valve_map(path):
    def unique_names(pairs):
        result = {}
        for name, channel in pairs:
            if name in result:
                raise ValueError(f"duplicate valve name in mapping: {name}")
            result[name] = channel
        return result

    mapping = json.loads(Path(path).read_text(), object_pairs_hook=unique_names)
    if not isinstance(mapping, dict):
        raise ValueError("valve map must be a JSON object of name: channel assignments")
    assigned = {}
    for name, channel in mapping.items():
        if not re.fullmatch(r"[A-Z][A-Z0-9_]*", name):
            raise ValueError(f"invalid valve name in mapping: {name!r}")
        if channel is None:
            continue  # Unknown wiring is never guessed; referencing it will fail.
        if type(channel) is not int or not 0 <= channel <= 31:
            raise ValueError(f"{name}: mapped channel must be an integer 0..31 or null")
        if channel in assigned:
            raise ValueError(f"{name} and {assigned[channel]} map to the same channel {channel}")
        assigned[channel] = name
    return mapping


def load_sequence(path, valve_map=DEFAULT_VALVE_MAP):
    mapping = None
    steps = json.loads(Path(path).read_text())
    if not isinstance(steps, list) or not steps:
        raise ValueError("sequence must be a nonempty JSON array")
    for index, step in enumerate(steps, 1):
        if (not isinstance(step, dict) or
                "duration_ms" not in step or
                ("channels" in step) == ("valves" in step) or
                set(step) - {"duration_ms", "channels", "valves", "comment"}):
            raise ValueError(f"step {index}: expected duration_ms, either channels or valves, and optional comment")
        comment = step.get("comment", "")
        if (not isinstance(comment, str) or
                any(not char.isprintable() for char in comment) or
                len(comment.encode("utf-8")) > 100):
            raise ValueError(f"step {index}: comment must be printable text of at most 100 UTF-8 bytes")
        duration = step["duration_ms"]
        entries = step.get("valves", step.get("channels"))
        if not isinstance(entries, list):
            raise ValueError(f"step {index}: channels/valves must be a list")
        channels = []
        for entry in entries:
            if isinstance(entry, str):
                if mapping is None:
                    mapping = load_valve_map(valve_map)
                if entry not in mapping:
                    raise ValueError(f"step {index}: unknown valve {entry!r}; check {valve_map}")
                channel = mapping[entry]
                if channel is None:
                    raise ValueError(f"step {index}: valve {entry} has no channel assigned in {valve_map}")
                channels.append(channel)
            elif "valves" not in step:
                channels.append(entry)
            else:
                raise ValueError(f"step {index}: valves must contain valve names, not channel numbers")
        if type(duration) is not int or not 1 <= duration <= 10000:
            raise ValueError(f"step {index}: duration_ms must be 1..10000")
        if (not isinstance(channels, list) or
                any(type(c) is not int or not 0 <= c <= 31 for c in channels) or
                len(set(channels)) != len(channels)):
            raise ValueError(f"step {index}: resolved channels must be unique integers 0..31")
        # Both the host runner and embedded generator use this validated GPIO list.
        step["channels"] = channels
        step.pop("valves", None)
    return steps


def pulse(connection, token, step):
    channels = " ".join(map(str, step["channels"]))
    command = f"valve group {token} {step['duration_ms']} {channels}\r"
    connection.write(command.encode("ascii"))
    connection.flush()
    deadline = time.monotonic() + step["duration_ms"] / 1000 + 5
    pattern = re.compile(rb"VALVE_DONE " + str(token).encode() + rb" (-?\d+)\r?\n")
    received = b""
    while time.monotonic() < deadline:
        received = (received + connection.read(1024))[-16384:]
        match = pattern.search(received)
        if match:
            result = int(match[1])
            if result:
                raise RuntimeError(f"board reported GPIO/command error {result}")
            return
    raise RuntimeError("no completion from board; sequence stopped. Check firmware and connection")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("sequence", type=Path)
    parser.add_argument("--valve-map", type=Path, default=DEFAULT_VALVE_MAP)
    parser.add_argument("--port", help="serial device or socket://host:port")
    parser.add_argument("--dry-run", action="store_true", help="validate and print without hardware")
    args = parser.parse_args()
    try:
        steps = load_sequence(args.sequence, args.valve_map)
        if args.dry_run:
            for index, step in enumerate(steps, 1):
                print(f"{index}: {step['duration_ms']} ms, channels {step['channels']}")
                if step.get("comment"):
                    print(f"  NEXT: {step['comment']}")
            return
        if not args.port:
            parser.error("--port is required unless --dry-run is used")
        import serial
        with serial.serial_for_url(args.port, baudrate=115200, timeout=0.1,
                                   write_timeout=2) as connection:
            connection.dtr = True
            time.sleep(1)
            connection.reset_input_buffer()
            # Clear any partially entered shell command before starting.
            connection.write(b"\x15\r")
            time.sleep(0.2)
            for index, step in enumerate(steps, 1):
                print(f"Step {index}/{len(steps)}: {step}", flush=True)
                if step["channels"]:
                    pulse(connection, secrets.randbelow(2147483647), step)
                else:
                    time.sleep(step["duration_ms"] / 1000)
        print("Sequence complete; all requested deactivation writes succeeded.")
    except (ValueError, OSError, RuntimeError, ImportError) as exc:
        parser.exit(1, f"Error: {exc}\n")
    except KeyboardInterrupt:
        parser.exit(130, "Stopped. An in-progress board pulse still runs to its firmware timeout.\n")


if __name__ == "__main__":
    main()
