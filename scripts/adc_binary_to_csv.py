#!/usr/bin/env python3
"""Convert a PT ADC/tach binary capture to an UltraLog-style CSV."""

import argparse
import csv
from pathlib import Path
import struct
import sys

HEADER = b"PTADC\x01\x0e\x00" + struct.pack("<I", 10000)
HEADER_V2 = b"PTADC\x02\x0e\x00" + struct.pack("<III", 10000, 3000000, 1)
SAMPLE = struct.Struct("<H")
TACH_SAMPLE = struct.Struct("<HI")
INTERVAL_US = 100
CHUNK_BYTES = 8192


def convert(source, output):
    with source.open("rb") as src:
        header = src.read(len(HEADER))
        has_tach = header == HEADER_V2[:len(HEADER)]
        if has_tach:
            if header + src.read(len(HEADER_V2) - len(HEADER)) != HEADER_V2:
                raise ValueError("Invalid or unsupported ADC/tach binary header")
        elif header != HEADER:
            raise ValueError("Invalid or unsupported ADC binary header")
        record = TACH_SAMPLE if has_tach else SAMPLE
        dst = output.open("x", newline="", encoding="ascii")
        completed = False
        try:
            with dst:
                writer = csv.writer(dst, lineterminator="\r\n")
                writer.writerow(["Time", "MAP", "RPM"] if has_tach else ["Time", "MAP"])
                index = 0
                while True:
                    data = src.read((CHUNK_BYTES // record.size) * record.size)
                    if not data:
                        break
                    if len(data) % record.size:
                        raise ValueError("Incomplete final ADC sample")
                    for sample in record.iter_unpack(data):
                        value = sample[0]
                        if value > 16383:
                            raise ValueError(f"Invalid 14-bit ADC sample at index {index}")
                        row = [index * INTERVAL_US, value]
                        if has_tach:
                            period = sample[1]
                            if period >= 900000:
                                raise ValueError(f"Tach period exceeds timeout at index {index}")
                            rpm = 180000000 / period if period else 0
                            writer.writerow(row + [f"{rpm:.3f}"])
                        else:
                            writer.writerow(row)
                        index += 1
            completed = True
        finally:
            if not completed:
                output.unlink()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path, help="ADC binary capture")
    parser.add_argument("output", type=Path, help="new CSV file (will not overwrite)")
    args = parser.parse_args()
    try:
        convert(args.input, args.output)
    except (OSError, ValueError) as exc:
        print(f"Conversion failed: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
