#!/usr/bin/env python3
"""
generate_tables.py

Build tables.bin from tables.txt

Usage:
    python3 generate_tables.py
    python3 generate_tables.py --input tables.txt --output tables.bin
    python3 generate_tables.py -m 33554432   (set max data size)

Features:
 - Supports multiple [TABLE] blocks
 - Supported types: u8, i16, u16, u32, f32
 - Validates required fields (ID, TYPE)
 - Detects duplicate IDs
 - Validates numeric ranges
 - Optional total data size limit
"""

import struct
import sys
import argparse
import os

TABLE_MAGIC = 0x54424C45  # "TBLE"
SUPPORTED_TYPES = {"u8", "i16", "u16", "u32", "f32"}


# -------------------------------------------------
# Pack a single value according to TYPE
# -------------------------------------------------
def pack_value(dtype, value_str, lineno=None):
    try:
        if dtype == "u8":
            v = int(value_str)
            if not (0 <= v <= 0xFF):
                raise ValueError("out of range for u8")
            return struct.pack("<B", v)

        if dtype == "i16":
            v = int(value_str)
            if not (-0x8000 <= v <= 0x7FFF):
                raise ValueError("out of range for i16")
            return struct.pack("<h", v)

        if dtype == "u16":
            v = int(value_str)
            if not (0 <= v <= 0xFFFF):
                raise ValueError("out of range for u16")
            return struct.pack("<H", v)

        if dtype == "u32":
            v = int(value_str)
            if not (0 <= v <= 0xFFFFFFFF):
                raise ValueError("out of range for u32")
            return struct.pack("<I", v)

        if dtype == "f32":
            v = float(value_str)
            return struct.pack("<f", v)

    except ValueError as e:
        if lineno is None:
            raise
        raise ValueError(
            f"Line {lineno}: cannot pack value '{value_str}' as {dtype}: {e}"
        )


# -------------------------------------------------
# Parse tables.txt
# -------------------------------------------------
def parse_tables_file(path):
    tables = []
    current = None
    current_data = []
    lineno = 0

    with open(path, "r", encoding="utf-8") as f:
        for raw in f:
            lineno += 1
            line = raw.strip()

            # Ignore empty lines and comments
            if not line or line.startswith("#"):
                continue

            # Start a new table block
            if line == "[TABLE]":
                if current:
                    current["data"] = current_data
                    tables.append(current)
                current = {"__start_line": lineno}
                current_data = []
                continue

            # Parse key=value lines (ID, NAME, TYPE)
            if "=" in line:
                if current is None:
                    raise RuntimeError(
                        f"Line {lineno}: key=value found outside a [TABLE] block"
                    )
                key, value = line.split("=", 1)
                current[key.strip().upper()] = value.strip()
                continue

            # Otherwise treat as data line
            if current is None:
                raise RuntimeError(
                    f"Line {lineno}: data found outside [TABLE] block"
                )

            current_data.append((line, lineno))

    # Append last table
    if current:
        current["data"] = current_data
        tables.append(current)

    return tables


# -------------------------------------------------
# Validate tables and pack data
# -------------------------------------------------
def validate_and_pack(tables, max_total_data_bytes):
    if not tables:
        raise RuntimeError("No tables found in input file.")

    ids = []

    for t in tables:
        if "ID" not in t:
            raise RuntimeError(
                f"Table starting at line {t.get('__start_line')} missing ID"
            )

        if "TYPE" not in t:
            raise RuntimeError(
                f"Table starting at line {t.get('__start_line')} missing TYPE"
            )

        try:
            tid = int(t["ID"])
        except Exception:
            raise RuntimeError(
                f"Invalid ID value at line {t.get('__start_line')}: {t['ID']}"
            )

        ids.append(tid)

    # Detect duplicate IDs
    if len(ids) != len(set(ids)):
        duplicates = set([x for x in ids if ids.count(x) > 1])
        raise RuntimeError(f"Duplicate table ID(s) detected: {duplicates}")

    packed_tables = []
    total_data = 0

    for t in tables:
        tid = int(t["ID"])
        dtype = t["TYPE"]

        if dtype not in SUPPORTED_TYPES:
            raise RuntimeError(
                f"Table ID={tid}: unsupported TYPE '{dtype}'"
            )

        name = t.get("NAME", "Unnamed")
        values = t.get("data", [])

        blob = bytearray()

        for val_str, lineno in values:
            blob += pack_value(dtype, val_str, lineno)

        print(
            f"Adding table: ID={tid}, NAME={name}, "
            f"TYPE={dtype}, SIZE={len(blob)} bytes"
        )

        packed_tables.append((tid, blob))
        total_data += len(blob)

        if total_data > max_total_data_bytes:
            raise RuntimeError(
                f"Total data size {total_data} exceeds limit {max_total_data_bytes}"
            )

    return packed_tables, total_data


# -------------------------------------------------
# Build final binary blob
# -------------------------------------------------
def build_blob(packed_tables):
    table_count = len(packed_tables)
    header_size = 24
    directory_size = table_count * 12
    data_offset = header_size + directory_size

    header = struct.pack(
        "<6I",
        TABLE_MAGIC,
        1,  # version
        table_count,
        header_size,
        directory_size,
        data_offset
    )

    directory = bytearray()
    data_region = bytearray()
    offset = 0

    for tid, blob in packed_tables:
        directory += struct.pack("<3I", tid, offset, len(blob))
        data_region += blob
        offset += len(blob)

    return header + directory + data_region


# -------------------------------------------------
# Main entry
# -------------------------------------------------
def main():
    parser = argparse.ArgumentParser(
        description="Generate tables.bin from tables.txt"
    )
    parser.add_argument(
        "--input", "-i",
        default="tables.txt",
        help="Input table definition file"
    )
    parser.add_argument(
        "--output", "-o",
        default="tables.bin",
        help="Output binary file"
    )
    parser.add_argument(
        "--max-data-bytes", "-m",
        type=int,
        default=16 * 1024 * 1024,
        help="Maximum total table data size (bytes)"
    )

    args = parser.parse_args()

    if not os.path.exists(args.input):
        print(f"Error: input file '{args.input}' not found.", file=sys.stderr)
        sys.exit(2)

    try:
        tables = parse_tables_file(args.input)
        packed_tables, total = validate_and_pack(
            tables, args.max_data_bytes
        )
        blob = build_blob(packed_tables)

        with open(args.output, "wb") as f:
            f.write(blob)

        print("tables.bin generated successfully.")
        print("Total tables:", len(packed_tables))
        print("Total data bytes:", total)
        print("Output file:", args.output)

    except Exception as e:
        print("Error:", e, file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
