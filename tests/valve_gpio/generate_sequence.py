#!/usr/bin/env python3
"""Validate a valve JSON sequence and generate a firmware header."""
import argparse
from pathlib import Path
from run_sequence import load_sequence


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("sequence", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    try:
        steps = load_sequence(args.sequence)
    except (ValueError, OSError) as exc:
        parser.error(str(exc))
    used = {channel for step in steps for channel in step["channels"]}
    lines = ["/* Generated from JSON; do not edit. */", "#pragma once",
             "#include <stdint.h>",
             f"#define SEQUENCE_REQUIRED_CHANNELS {max(used, default=-1) + 1}",
             "static const struct { uint32_t duration_ms; uint32_t mask; } sequence_steps[] = {"]
    for step in steps:
        mask = sum(1 << channel for channel in step["channels"])
        lines.append(f"    {{{step['duration_ms']}U, 0x{mask:08x}U}},")
    lines.append("};\n")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text("\n".join(lines))


if __name__ == "__main__":
    main()
