#!/usr/bin/env python3
"""Run timed GPIO bench-test groups through the manual firmware's USB shell."""
import argparse
import json
from pathlib import Path
import re
import secrets
import time


def load_sequence(path):
    steps = json.loads(Path(path).read_text())
    if not isinstance(steps, list) or not steps:
        raise ValueError("sequence must be a nonempty JSON array")
    for index, step in enumerate(steps, 1):
        if not isinstance(step, dict) or set(step) != {"duration_ms", "channels"}:
            raise ValueError(f"step {index}: expected duration_ms and channels")
        duration, channels = step["duration_ms"], step["channels"]
        if type(duration) is not int or not 1 <= duration <= 10000:
            raise ValueError(f"step {index}: duration_ms must be 1..10000")
        if (not isinstance(channels, list) or
                any(type(c) is not int or not 0 <= c <= 31 for c in channels) or
                len(set(channels)) != len(channels)):
            raise ValueError(f"step {index}: channels must be unique integers 0..31")
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
    parser.add_argument("--port", help="serial device or socket://host:port")
    parser.add_argument("--dry-run", action="store_true", help="validate and print without hardware")
    args = parser.parse_args()
    try:
        steps = load_sequence(args.sequence)
        if args.dry_run:
            for index, step in enumerate(steps, 1):
                print(f"{index}: {step['duration_ms']} ms, channels {step['channels']}")
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
