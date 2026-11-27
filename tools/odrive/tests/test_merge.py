import contextlib
import io
import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from extract_nvm_config import ihex_record
from merge import Logger, merge_hex_files, read_hex_file


class MergeHexTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.directory = Path(self.temp.name)
        self.logger = Logger()
        self.output = self.directory / "merged.hex"

    def write_hex(self, name, records):
        path = self.directory / name
        path.write_text("\n".join(records + [ihex_record(0, 1)]) + "\n", encoding="ascii")
        return path

    def merge(self, inputs, threshold=0xffffffff):
        with contextlib.redirect_stdout(io.StringIO()):
            return merge_hex_files(*inputs, self.output, threshold, self.logger)

    def test_merge_uses_absolute_addresses_and_bootloader_entry(self):
        boot = self.write_hex("boot.hex", [
            ihex_record(0, 4, b"\x08\x00"),
            ihex_record(0, 0, b"\x01\x02"),
            ihex_record(0, 5, b"\x08\x00\x00\x01"),
        ])
        app = self.write_hex("app.hex", [
            ihex_record(0, 4, b"\x08\x01"),
            ihex_record(0, 0, b"\x03\x04"),
            ihex_record(0, 5, b"\x08\x01\x00\x01"),
        ])
        config = self.write_hex("config.hex", [
            ihex_record(0, 2, b"\x12\x34"),
            ihex_record(0x10, 0, b"\x05\x06"),
        ])
        self.assertTrue(self.merge([boot, app, config]))
        data, entry = read_hex_file(self.output)
        self.assertEqual(data, {0x08000000: 1, 0x08000001: 2,
                                0x08010000: 3, 0x08010001: 4,
                                0x12350: 5, 0x12351: 6})
        self.assertEqual(entry, (5, b"\x08\x00\x00\x01"))
        self.assertEqual(self.output.read_text(encoding="ascii").count(":00000001FF"), 1)

    def test_threshold_trims_partial_record(self):
        boot = self.write_hex("boot.hex", [ihex_record(0, 0, b"\x01\x02\x03")])
        app = self.write_hex("app.hex", [ihex_record(3, 0, b"\x04\x05")])
        config = self.write_hex("config.hex", [ihex_record(5, 0, b"\x06\x07")])
        self.assertTrue(self.merge([boot, app, config], 5))
        self.assertEqual(read_hex_file(self.output)[0], dict(enumerate(range(1, 7))))

    def test_overlap_and_bad_checksum_do_not_replace_output(self):
        boot = self.write_hex("boot.hex", [ihex_record(0, 0, b"\x01")])
        app = self.write_hex("app.hex", [ihex_record(0, 0, b"\x02")])
        config = self.write_hex("config.hex", [ihex_record(2, 0, b"\x03")])
        self.output.write_text("existing", encoding="ascii")
        self.assertFalse(self.merge([boot, app, config]))
        self.assertEqual(self.output.read_text(encoding="ascii"), "existing")
        app.write_text(":010001000200\n:00000001FF\n", encoding="ascii")
        self.assertFalse(self.merge([boot, app, config]))
        self.assertEqual(self.output.read_text(encoding="ascii"), "existing")

    def test_main_resolves_paths_from_script_directory(self):
        root = self.directory / "project"
        script_dir = root / "tools" / "odrive"
        config_dir = root / "configFiles" / "\u7248\u672c\u8bf4\u660e\u6587\u4ef6"
        build_dir = root / "Firmware" / "build"
        script_dir.mkdir(parents=True)
        config_dir.mkdir(parents=True)
        build_dir.mkdir(parents=True)
        original_dir = Path(__file__).resolve().parents[1]
        for name in ("merge.py", "extract_nvm_config.py"):
            shutil.copyfile(original_dir / name, script_dir / name)
        for name, address in (("boot.hex", 0), ("app.hex", 1), ("config.hex", 2)):
            (build_dir / name).write_text(
                ihex_record(address, 0, bytes([address])) + "\n" + ihex_record(0, 1) + "\n",
                encoding="ascii",
            )
        (config_dir / "config.txt").write_text(
            "delete_hex = False\n"
            "bootloader_file = ../../Firmware/build/boot.hex\n"
            "firmware_file = ../../Firmware/build/app.hex\n"
            "datafile_paths = ../../Firmware/build/config.hex\n",
            encoding="utf-8",
        )
        result = subprocess.run([sys.executable, str(script_dir / "merge.py")],
                                cwd=self.directory, capture_output=True, text=True,
                                encoding="utf-8", errors="replace")
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        outputs = list(build_dir.glob("configv0.0.0.hex"))
        self.assertEqual(len(outputs), 1)
        self.assertEqual(read_hex_file(outputs[0])[0], {0: 0, 1: 1, 2: 2})


if __name__ == "__main__":
    unittest.main()
