import csv
from pathlib import Path
import struct
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from binary_log_to_csv import HEADER, convert


class BinaryLogConverterTest(unittest.TestCase):
    def test_round_trip(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "log.bin"
            output = Path(directory) / "log.csv"
            source.write_bytes(HEADER + struct.pack("<IIB8s", 123, 0x8001ABCD, 2,
                                                    b"\xab\xcd" + b"\0" * 6) +
                               struct.pack("<IIB8s", 456, 0x123, 0, b"\0" * 8))
            convert(source, output)
            with output.open(newline="") as stream:
                rows = list(csv.reader(stream))
            self.assertEqual(rows[1], ["123000", "1ABCD", "true", "Rx", "0", "2", "AB", "CD"])
            self.assertEqual(rows[2], ["456000", "123", "false", "Rx", "0", "0"])
            with self.assertRaises(FileExistsError):
                convert(source, output)

    def test_invalid_input_leaves_no_output(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "log.bin"
            output = Path(directory) / "log.csv"
            for content in (b"bad", HEADER + b"x",
                            HEADER + struct.pack("<IIB8s", 0, 0x80000000, 9, b"\0" * 8),
                            HEADER + struct.pack("<IIB8s", 0, 0x800, 0, b"\0" * 8)):
                source.write_bytes(content)
                with self.assertRaises(ValueError):
                    convert(source, output)
                self.assertFalse(output.exists())


if __name__ == "__main__":
    unittest.main()
