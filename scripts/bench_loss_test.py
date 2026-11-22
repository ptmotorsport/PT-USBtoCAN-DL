#!/usr/bin/env python3
"""Automated bench loss test using PCAN send and logger SD export."""

import argparse
import csv
import datetime as dt
import sys
import time
from collections import Counter, defaultdict
from dataclasses import dataclass
from pathlib import Path

import serial
from serial.tools import list_ports

from download_sd_logs import ExportError, SDClient

import ctypes
from ctypes import c_ubyte, c_uint, c_ulong, c_ushort


@dataclass
class Sample:
    offset_ms: float
    can_id: int
    counter: int
    payload: bytes


def crc32_bytes(data: bytes) -> int:
    import zlib
    return zlib.crc32(data) & 0xFFFFFFFF


class TPCANMsg(ctypes.Structure):
    _fields_ = [
        ("ID", c_uint),
        ("MSGTYPE", c_ubyte),
        ("LEN", c_ubyte),
        ("DATA", c_ubyte * 8),
    ]


class PCAN:
    USBBUS1 = 0x51
    BAUD_1M = 0x0014
    MSG_STD = 0x00
    STATUS_OK = 0x00000
    STATUS_QXMTFULL = 0x00080
    STATUS_BUSHEAVY = 0x00008
    STATUS_BUSOFF = 0x00010

    def __init__(self):
        self.lib = ctypes.WinDLL("PCANBasic.dll")
        self.channel = c_ushort(self.USBBUS1)
        self.lib.CAN_Initialize.argtypes = [c_ushort, c_ushort, c_uint, c_uint, c_ushort]
        self.lib.CAN_Initialize.restype = c_ulong
        self.lib.CAN_Write.argtypes = [c_ushort, ctypes.POINTER(TPCANMsg)]
        self.lib.CAN_Write.restype = c_ulong
        self.lib.CAN_Uninitialize.argtypes = [c_ushort]
        self.lib.CAN_Uninitialize.restype = c_ulong

    def open(self):
        status = self.lib.CAN_Initialize(self.channel, c_ushort(self.BAUD_1M), 0, 0, 0)
        if status != self.STATUS_OK:
            raise RuntimeError(f"PCAN init failed with status 0x{status:X}")

    def close(self):
        self.lib.CAN_Uninitialize(self.channel)

    def send(self, can_id: int, counter: int) -> int:
        msg = TPCANMsg()
        msg.ID = can_id
        msg.MSGTYPE = self.MSG_STD
        msg.LEN = 8
        payload = counter.to_bytes(4, "little") + b"\x00\x00\x00\x00"
        for i, value in enumerate(payload):
            msg.DATA[i] = value
        return self.lib.CAN_Write(self.channel, ctypes.byref(msg))


def parse_trace(trace_path: Path) -> list[Sample]:
    samples = []
    with trace_path.open() as f:
        for line in f:
            if ":" not in line or line.startswith(";"):
                continue
            number, offset, direction, can_id, dlc, *payload = line.split()
            if direction != "Rx" or dlc != "8":
                continue
            data = bytes.fromhex("".join(payload))
            samples.append(
                Sample(
                    offset_ms=float(offset),
                    can_id=int(can_id, 16),
                    counter=int.from_bytes(data[:4], "little"),
                    payload=data,
                )
            )
    if not samples:
        raise RuntimeError(f"No trace frames in {trace_path}")
    return samples


def parse_csv(csv_path: Path) -> list[Sample]:
    samples = []
    with csv_path.open(newline="") as f:
        rows = csv.reader(f)
        header = next(rows)
        expected = [
            "Time Stamp",
            "ID",
            "Extended",
            "Dir",
            "Bus",
            "LEN",
            "D1",
            "D2",
            "D3",
            "D4",
            "D5",
            "D6",
            "D7",
            "D8",
        ]
        if header != expected:
            raise RuntimeError(f"Unexpected header in {csv_path.name}: {header}")
        for row in rows:
            if len(row) != 14 or row[2:6] != ["false", "Rx", "0", "8"]:
                raise RuntimeError(f"Unexpected row in {csv_path.name}: {row}")
            data = bytes.fromhex("".join(row[6:14]))
            if len(data) != 8:
                raise RuntimeError(f"Invalid payload in {csv_path.name}: {row}")
            samples.append(
                Sample(
                    offset_ms=int(row[0]) / 1000,
                    can_id=int(row[1], 16),
                    counter=int.from_bytes(data[:4], "little"),
                    payload=data,
                )
            )
    if not samples:
        raise RuntimeError(f"No logger frames in {csv_path}")
    return samples


def analyze(reference: list[Sample], logger: list[Sample]) -> dict:
    ref_map = {(s.can_id, s.counter): s.payload for s in reference}
    if len(ref_map) != len(reference):
        raise RuntimeError("Reference contains duplicate frame keys")
    ref_order = {(s.can_id, s.counter): i for i, s in enumerate(reference)}
    log_keys = [(s.can_id, s.counter) for s in logger]
    unique = set(log_keys)
    duplicates = len(log_keys) - len(unique)
    missing = sorted(set(ref_map) - unique)
    extra = sorted(unique - set(ref_map))
    payload_mismatch = sum(1 for s in logger if ref_map.get((s.can_id, s.counter), s.payload) != s.payload)
    positions = [ref_order[key] for key in log_keys if key in ref_order]
    out_of_order = sum(b < a for a, b in zip(positions, positions[1:]))
    by_id_missing = Counter(can_id for can_id, _ in missing)
    by_id_ref = Counter(s.can_id for s in reference)
    by_id_log = Counter(s.can_id for s in logger)
    return {
        "reference_frames": len(reference),
        "logger_frames": len(logger),
        "missing_total": len(missing),
        "missing_by_id": dict(sorted(by_id_missing.items())),
        "duplicates": duplicates,
        "extra": len(extra),
        "payload_mismatch": payload_mismatch,
        "out_of_order": out_of_order,
        "reference_by_id": dict(sorted(by_id_ref.items())),
        "logger_by_id": dict(sorted(by_id_log.items())),
        "missing_examples": missing[:20],
    }


def analyze_without_reference(logger: list[Sample], sent_counts: Counter[int]) -> dict:
    by_id = defaultdict(list)
    for sample in logger:
        by_id[sample.can_id].append(sample)
    missing_total = 0
    missing_by_id = {}
    duplicate_total = 0
    duplicate_by_id = {}
    out_of_order = 0
    for can_id, expected in sorted(sent_counts.items()):
        samples = by_id.get(can_id, [])
        counters = [s.counter for s in samples]
        if counters:
            duplicate_total += len(counters) - len(set(counters))
            duplicate_by_id[can_id] = len(counters) - len(set(counters))
            out_of_order += sum(b < a for a, b in zip(counters, counters[1:]))
            expected_set = set(range(1, expected + 1))
            actual_set = set(counters)
            missing = len(expected_set - actual_set)
        else:
            missing = expected
        missing_total += missing
        missing_by_id[can_id] = missing
    extras = sum(
        1 for sample in logger
        if sample.can_id not in sent_counts or not 1 <= sample.counter <= sent_counts[sample.can_id]
    )
    payload_mismatch = sum(
        1 for sample in logger
        if sample.payload[4:] != b"\x00\x00\x00\x00"
    )
    return {
        "submitted_frames_not_independent_reference": int(sum(sent_counts.values())),
        "logger_frames": len(logger),
        "missing_total": missing_total,
        "missing_by_id": dict(sorted(missing_by_id.items())),
        "duplicates": duplicate_total,
        "duplicate_by_id": dict(sorted(duplicate_by_id.items())),
        "extra": extras,
        "payload_mismatch": payload_mismatch,
        "out_of_order_per_id": out_of_order,
        "out_of_order": out_of_order,
        "submitted_by_id": dict(sorted((k, int(v)) for k, v in sent_counts.items())),
        "logger_by_id": dict(sorted((k, len(v)) for k, v in by_id.items())),
        "missing_examples": [],
    }


def pick_new_csvs(before: set[str], after: list[str]) -> list[str]:
    return [name for name in after if name not in before]

def segment_key(path: Path):
    return tuple(int(part) for part in path.stem.split("_"))


def await_start(logger: SDClient):
    logger.command("LOG START")
    deadline = time.monotonic() + 15
    while True:
        line = logger.line(deadline)
        if line == b"OK: Logging enabled":
            return
        if line.startswith(b"ERR:"):
            raise ExportError(line.decode("ascii", errors="replace"))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--logger-port", default="COM22")
    parser.add_argument("--external-trace", type=Path, help="Existing ECUMaster .trace to compare against")
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--duration-sec", type=int, default=600)
    parser.add_argument("--id500-ms", type=float, default=1.0)
    parser.add_argument("--id501-ms", type=float, default=3.0)
    args = parser.parse_args()
    if args.duration_sec <= 0 or args.id500_ms <= 0 or args.id501_ms <= 0:
        parser.error("duration and intervals must be positive")

    args.out.mkdir(parents=True, exist_ok=True)
    run_dir = args.out / dt.datetime.now().strftime("run-%Y%m%d-%H%M%S")
    run_dir.mkdir()
    trace_path = args.external_trace
    csv_dir = run_dir / "logger_csv"
    csv_dir.mkdir()
    report = run_dir / "report.txt"

    ports = {p.device: p for p in list_ports.comports()}
    if args.logger_port not in ports:
        raise RuntimeError(f"Logger port {args.logger_port} not present")

    with serial.Serial(args.logger_port, 115200, timeout=0.25, write_timeout=2) as serial_port:
        serial_port.reset_input_buffer()
        logger = SDClient(serial_port)
        logger.stop_logging()
        before = set(logger.list_files())
        pcan = PCAN()
        sent = Counter()
        status_counts = Counter()
        pcan_open = False
        logging_started = False
        try:
            pcan.open()
            pcan_open = True
            logging_started = True
            await_start(logger)
            time.sleep(0.5)
            start = time.perf_counter()
            next_500 = start
            next_501 = start
            while True:
                now = time.perf_counter()
                if now - start >= args.duration_sec:
                    break
                if now >= next_500:
                    sent[0x500] += 1
                    status_counts[pcan.send(0x500, sent[0x500])] += 1
                    next_500 += args.id500_ms / 1000.0
                if now >= next_501:
                    sent[0x501] += 1
                    status_counts[pcan.send(0x501, sent[0x501])] += 1
                    next_501 += args.id501_ms / 1000.0
                sleep_for = min(next_500, next_501) - time.perf_counter()
                if sleep_for > 0:
                    time.sleep(min(0.0005, sleep_for))
        finally:
            if pcan_open:
                pcan.close()
            if logging_started:
                logger.stop_logging()
        if sent:
            logger.command("SD")
            sd_diag = []
            deadline = time.monotonic() + 8
            while True:
                line = logger.line(deadline).decode(errors="replace")
                sd_diag.append(line)
                if line.strip() == "====================================":
                    break
            (run_dir / "sd_diag.txt").write_text("\n".join(sd_diag) + "\n", encoding="utf-8")
            after = logger.list_files()
            new_files = pick_new_csvs(before, after)
            if not new_files:
                raise RuntimeError("No new CSV files found after test run")
            for name in new_files:
                logger.download(name, csv_dir)

    reference = parse_trace(trace_path) if trace_path else None
    if reference:
        reference_check = analyze_without_reference(reference, sent)
        if any(reference_check[key] for key in
               ("missing_total", "duplicates", "extra", "payload_mismatch", "out_of_order")):
            raise RuntimeError("Reference trace is incomplete, inconsistent, or does not match submitted frames")
    logger_rows = []
    for name in sorted(csv_dir.glob("*.csv"), key=segment_key):
        logger_rows.extend(parse_csv(name))
    result = analyze(reference, logger_rows) if reference else analyze_without_reference(logger_rows, sent)

    with report.open("w", encoding="utf-8") as f:
        f.write("Automated bench CAN loss test\n")
        f.write(f"Duration: {args.duration_sec}s\n")
        f.write(f"Sent counters: {dict(sent)}\n")
        f.write(f"PCAN status counts: {dict(sorted(status_counts.items()))}\n")
        f.write("CAN_Write success is not an independent on-bus reference.\n")
        f.write("Without a reference, global cross-ID ordering is not measured.\n")
        for key, value in result.items():
            f.write(f"{key}: {value}\n")
        f.write(f"new_csv_files: {[p.name for p in sorted(csv_dir.glob('*.csv'), key=segment_key)]}\n")

    print(report)
    if any(status != 0 for status in status_counts) or result["missing_total"] or result["duplicates"] or result["extra"] or result["payload_mismatch"] or result["out_of_order"]:
        return 2
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (RuntimeError, OSError, ExportError, ValueError) as exc:
        print(f"Bench test failed: {exc}", file=sys.stderr)
        raise SystemExit(1)
