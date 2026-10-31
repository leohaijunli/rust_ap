import tempfile
import unittest
from pathlib import Path

from build_report import ROOT, Elf32, source_identity


class BuildReportTests(unittest.TestCase):
    def test_rejects_non_elf_and_truncated_headers(self):
        for data in (b"", b"\x7fELF", bytes(52), b"\x7fELF\x01\x01\x01" + bytes(45)):
            with self.subTest(data=data), self.assertRaises(ValueError):
                Elf32(data)

    def test_source_id_is_repeatable_and_feature_sensitive(self):
        with tempfile.TemporaryDirectory(dir=ROOT / "target") as directory:
            root = Path(directory)
            (root / "src").mkdir()
            (root / ".cargo").mkdir()
            for name in ("Cargo.toml", "Cargo.lock", "build.rs", "memory.x", "rust-toolchain.toml", ".cargo/config.toml", "src/lib.rs"):
                (root / name).write_text(name)
            first = source_identity(root)
            self.assertEqual(first, source_identity(root))
            self.assertNotEqual(first[0], source_identity(root, ["fault-injection"])[0])
            self.assertNotEqual(first[0], source_identity(root, ["fault-dump"])[0])
            self.assertNotEqual(
                source_identity(root, ["fault-dump"])[0],
                source_identity(root, ["fault-injection"])[0],
            )
            (root / "src/lib.rs").write_text("changed")
            self.assertNotEqual(first, source_identity(root))
