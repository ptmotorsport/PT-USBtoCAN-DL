#!/usr/bin/env python3
"""List or download SD logger files over USB serial (requires pyserial)."""

import argparse
import binascii
from pathlib import Path
import re
import sys
import tempfile
import time
import zlib

import serial


LOG_NAME = re.compile(r"[0-9][0-9_]{0,58}\.(?:csv|bin)", re.IGNORECASE | re.ASCII)


class ExportError(Exception):
    pass


class SDClient:
    def __init__(self, port):
        self.port = port
        self.buffer = bytearray()

    def line(self, deadline):
        while True:
            end = self.buffer.find(b"\n")
            if end >= 0:
                line = bytes(self.buffer[:end]).rstrip(b"\r")
                del self.buffer[:end + 1]
                return line
            if len(self.buffer) > 1024:
                raise ExportError("Overlong serial response")
            if time.monotonic() >= deadline:
                raise ExportError("Timed out waiting for the logger's reply")
            self.buffer.extend(self.port.read(min(self.port.in_waiting or 1, 4096)))

    def command(self, command):
        self.port.write(command.encode("ascii") + b"\n")

    def stop_logging(self):
        self.command("LOG STOP")
        deadline = time.monotonic() + 15
        while True:
            line = self.line(deadline)
            if line.startswith(b"OK: Logging stopped;"):
                return
            if line.startswith(b"ERR:"):
                raise ExportError(line.decode("ascii", errors="replace"))

    def export_line(self, deadline):
        line = self.line(deadline)
        if line.startswith(b"SDX ERR "):
            raise ExportError(line[8:].decode("ascii", errors="replace"))
        return line

    def header(self, command, prefix):
        self.command(command)
        deadline = time.monotonic() + 15
        while True:
            line = self.export_line(deadline)
            if line.startswith(prefix):
                return line
            if line.startswith(b"SDX "):
                raise ExportError("Unexpected export header")

    def list_files(self):
        if self.header("SD LIST", b"SDX LIST") != b"SDX LIST":
            raise ExportError("Invalid listing header")
        names = []
        while True:
            line = self.export_line(time.monotonic() + 10)
            if line == b"SDX END":
                return names
            if not line.startswith(b"SDX FILE "):
                raise ExportError("Invalid directory listing")
            name = line[9:].decode("ascii")
            if not LOG_NAME.fullmatch(name):
                raise ExportError(f"Invalid log filename: {name!r}")
            names.append(name)

    def download(self, name, directory):
        if not LOG_NAME.fullmatch(name):
            raise ExportError(f"Invalid log filename: {name!r}")
        target = directory / name
        if target.exists():
            raise ExportError(f"Not overwriting {target}")
        header = self.header(f"SD GET {name}", b"SDX GET ")
        if not re.fullmatch(rb"[0-9]{1,10}", header[8:]):
            raise ExportError("Invalid file length")
        size = int(header[8:])
        if size > 0xFFFFFFFF:
            raise ExportError("Invalid file length")

        temp_path = None
        try:
            with tempfile.NamedTemporaryFile(dir=directory, suffix=".part", delete=False) as out:
                temp_path = Path(out.name)
                count = crc = 0
                while True:
                    line = self.export_line(time.monotonic() + 10)
                    if re.fullmatch(rb"SDX CRC [0-9A-Fa-f]{1,8}", line):
                        if count != size or int(line[8:], 16) != crc:
                            raise ExportError(f"{name}: byte count or CRC-32 mismatch")
                        break
                    if not line.startswith(b":") or not 1 < len(line) <= 513:
                        raise ExportError(f"{name}: invalid data record")
                    try:
                        chunk = binascii.unhexlify(line[1:])
                    except binascii.Error as exc:
                        raise ExportError(f"{name}: invalid hex payload") from exc
                    count += len(chunk)
                    if count > size:
                        raise ExportError(f"{name}: more data than advertised")
                    crc = zlib.crc32(chunk, crc)
                    out.write(chunk)
            if target.exists():
                raise ExportError(f"Not overwriting {target}")
            temp_path.rename(target)
            return count, crc
        finally:
            if temp_path is not None:
                temp_path.unlink(missing_ok=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True, help="Logger USB serial port, e.g. COM22")
    parser.add_argument("--stop", action="store_true", help="Send LOG STOP first; leave logging stopped")
    commands = parser.add_subparsers(dest="action", required=True)
    commands.add_parser("list", help="Print log filenames, one per line")
    pull = commands.add_parser("pull", help="Download and verify log files")
    pull.add_argument("--output", type=Path, required=True)
    pull.add_argument("files", nargs="*", help="Filenames; all log files if omitted")
    args = parser.parse_args()
    try:
        with serial.Serial(args.port, 115200, timeout=0.25, write_timeout=2) as port:
            port.reset_input_buffer()
            client = SDClient(port)
            if args.stop:
                client.stop_logging()
            if args.action == "list":
                for name in client.list_files():
                    print(name)
            else:
                args.output.mkdir(parents=True, exist_ok=True)
                for name in args.files or client.list_files():
                    size, crc = client.download(name, args.output)
                    print(f"{name}: {size} bytes, CRC-32 {crc:08X}")
    except (OSError, ExportError, UnicodeError) as exc:
        print(f"SD export failed: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
