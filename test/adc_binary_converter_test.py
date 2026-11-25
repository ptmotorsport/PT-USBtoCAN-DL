import csv
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from adc_binary_to_csv import CHUNK_BYTES, HEADER, HEADER_V2, convert


class AdcBinaryConverterTest(unittest.TestCase):
    def test_tach_capture(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "log.bin"
            output = Path(directory) / "log.csv"
            values = [(639, 0), (16383, 30000), (42, 600000)] * 500
            source.write_bytes(HEADER_V2 + b"".join(struct.pack("<HI", *row) for row in values))
            convert(source, output)
            with output.open(newline="", encoding="ascii") as stream:
                rows = list(csv.reader(stream))
            self.assertEqual(rows[0], ["Time", "MAP", "RPM"])
            self.assertEqual(rows[1:4], [
                ["0", "639", "0.000"], ["100", "16383", "6000.000"],
                ["200", "42", "300.000"],
            ])
            self.assertEqual(rows[-1], ["149900", "42", "300.000"])

    def test_round_trip_and_chunk_boundary(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "ADC0000.BIN"
            output = Path(directory) / "converted.csv"
            values = [0, 16383, 639] + [42] * (CHUNK_BYTES // 2)
            source.write_bytes(HEADER + struct.pack(f"<{len(values)}H", *values))
            convert(source, output)
            with output.open(newline="", encoding="ascii") as stream:
                rows = list(csv.reader(stream))
            self.assertEqual(rows[0], ["Time", "MAP"])
            self.assertEqual(rows[1:4], [["0", "0"], ["100", "16383"], ["200", "639"]])
            self.assertEqual(rows[-1], [str((len(values) - 1) * 100), "42"])
            with self.assertRaises(FileExistsError):
                convert(source, output)

    def test_invalid_input_leaves_no_output(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "log.bin"
            output = Path(directory) / "log.csv"
            for content in (
                b"bad",
                HEADER[:-1],
                HEADER + b"\x01",
                HEADER + struct.pack("<H", 16384),
                HEADER + b"\x00\x00" * (CHUNK_BYTES // 2) + b"\x01",
                HEADER_V2[:-1],
                HEADER_V2 + b"\x01",
                HEADER_V2 + struct.pack("<HI", 16384, 0),
                HEADER_V2 + struct.pack("<HI", 42, 900000),
                HEADER_V2 + struct.pack("<HI", 42, 0) * 1400 + b"\x01",
            ):
                with self.subTest(content_length=len(content)):
                    source.write_bytes(content)
                    with self.assertRaises(ValueError):
                        convert(source, output)
                    self.assertFalse(output.exists())

    def test_command_line(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "log.bin"
            output = Path(directory) / "log.csv"
            source.write_bytes(HEADER + struct.pack("<HH", 12, 34))
            script = Path(__file__).resolve().parents[1] / "scripts" / "adc_binary_to_csv.py"
            result = subprocess.run(
                [sys.executable, str(script), str(source), str(output)],
                capture_output=True, text=True, check=False,
            )
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertEqual(output.read_bytes(), b"Time,MAP\r\n0,12\r\n100,34\r\n")
            result = subprocess.run(
                [sys.executable, str(script), str(source), str(output)],
                capture_output=True, text=True, check=False,
            )
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("Conversion failed", result.stderr)
            self.assertEqual(output.read_bytes(), b"Time,MAP\r\n0,12\r\n100,34\r\n")


if __name__ == "__main__":
    unittest.main()
