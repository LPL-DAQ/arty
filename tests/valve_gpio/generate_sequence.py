#!/usr/bin/env python3
"""Validate a JSON file or directory of named sequences and generate a firmware header."""
import argparse
import re
from pathlib import Path
from run_sequence import DEFAULT_VALVE_MAP, load_sequence


def c_string(value):
    # Fixed-width octal escapes preserve UTF-8 and cannot consume following digits.
    return '"' + ''.join(f"\\{byte:03o}" for byte in value.encode("utf-8")) + '"'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("sequence", type=Path, help="JSON file or directory of *.json files")
    parser.add_argument("output", type=Path)
    parser.add_argument("--valve-map", type=Path, default=DEFAULT_VALVE_MAP)
    args = parser.parse_args()
    try:
        files = sorted(args.sequence.glob("*.json")) if args.sequence.is_dir() else [args.sequence]
        if not files:
            raise ValueError("sequence directory contains no JSON files")
        if len(files) > 16:
            raise ValueError("at most 16 embedded sequences are supported")
        sequences = []
        for path in files:
            if not re.fullmatch(r"[A-Za-z0-9_][A-Za-z0-9_.-]{0,47}", path.stem):
                raise ValueError(f"{path}: name must be 1..48 letters, digits, underscores, dots or hyphens")
            sequences.append((path.stem, load_sequence(path, args.valve_map)))
    except (ValueError, OSError) as exc:
        parser.error(str(exc))
    used = {channel for _, steps in sequences for step in steps for channel in step["channels"]}
    lines = ["/* Generated from JSON; do not edit. */", "#pragma once",
             "#include <stdint.h>", "#include <stddef.h>",
             f"#define SEQUENCE_REQUIRED_CHANNELS {max(used, default=-1) + 1}",
             "struct sequence_step { uint32_t duration_ms; uint32_t mask; const char *comment; };",
             "struct valve_sequence { const char *name; const struct sequence_step *steps; size_t count; };"]
    for index, (_, steps) in enumerate(sequences):
        lines.append(f"static const struct sequence_step sequence_steps_{index}[] = {{")
        for step in steps:
            mask = sum(1 << channel for channel in step["channels"])
            comment = c_string(step.get("comment", ""))
            lines.append(f"    {{{step['duration_ms']}U, 0x{mask:08x}U, {comment}}},")
        lines.append("};")
    lines.append("static const struct valve_sequence sequences[] = {")
    for index, (name, steps) in enumerate(sequences):
        lines.append(f"    {{{c_string(name)}, sequence_steps_{index}, {len(steps)}U}},")
    lines.append("};\n")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text("\n".join(lines))


if __name__ == "__main__":
    main()
