#!/usr/bin/env python3
"""Convert an experimental PT CAN binary log to the standard logger CSV."""

import argparse
import csv
from pathlib import Path
import struct
import sys

HEADER = b"PTCAN\x01\x11\x00"
RECORD = struct.Struct("<IIB8s")
CSV_HEADER = ["Time Stamp", "ID", "Extended", "Dir", "Bus", "LEN"] + [
    f"D{i}" for i in range(1, 9)
]


def convert(source, output):
    with source.open("rb") as src:
        if src.read(len(HEADER)) != HEADER:
            raise ValueError("Invalid or unsupported binary log header")
        dst = output.open("x", newline="", encoding="ascii")
        completed = False
        try:
            with dst:
                writer = csv.writer(dst, lineterminator="\r\n")
                writer.writerow(CSV_HEADER)
                while True:
                    data = src.read(RECORD.size)
                    if not data:
                        break
                    if len(data) != RECORD.size:
                        raise ValueError("Incomplete final binary record")
                    timestamp, tagged_id, length, payload = RECORD.unpack(data)
                    extended = bool(tagged_id & 0x80000000)
                    identifier = tagged_id & 0x7FFFFFFF
                    if length > 8 or identifier > (0x1FFFFFFF if extended else 0x7FF):
                        raise ValueError("Invalid identifier or length in binary record")
                    if any(payload[length:]):
                        raise ValueError("Nonzero unused payload bytes in binary record")
                    writer.writerow([timestamp * 1000, f"{identifier:03X}",
                                     str(extended).lower(), "Rx", 0, length] +
                                    [f"{value:02X}" for value in payload[:length]])
            completed = True
        finally:
            if not completed:
                output.unlink()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    try:
        convert(args.input, args.output)
    except (OSError, ValueError) as exc:
        print(f"Conversion failed: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
