#!/usr/bin/env python3
"""Validate ELF/bin/PX4 agreement and record P1 source/artifact SHA-256."""

import argparse
import base64
import hashlib
import json
import os
import struct
import subprocess
import tomllib
import zlib
from pathlib import Path

from package import APP_SIZE, APP_START, RAM_END, make_package

ROOT = Path(__file__).resolve().parents[1]


def require(condition, message):
    if not condition:
        raise ValueError(message)


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def source_identity(root, features=()):
    # Sort by the relative posix path so the order matches build.rs's
    # case-sensitive PathBuf ordering on every host platform.
    paths = [
        root / name for name in (
            "Cargo.toml", "Cargo.lock", "build.rs", "memory.x",
            "rust-toolchain.toml", ".cargo/config.toml",
        )
    ] + [p for p in (root / "src").rglob("*") if p.is_file()]
    paths.sort(key=lambda p: p.relative_to(root).as_posix())
    identity = 0xCBF29CE484222325
    for path in paths:
        data = path.relative_to(root).as_posix().encode() + b"\0" + path.read_bytes() + b"\0"
        for byte in data:
            identity = ((identity ^ byte) * 0x100000001B3) & 0xFFFFFFFFFFFFFFFF
    features = sorted(features)
    marker = b"".join(b"feature:" + name.encode("ascii") + b"\0" for name in features) or b"default"
    for byte in marker:
        identity = ((identity ^ byte) * 0x100000001B3) & 0xFFFFFFFFFFFFFFFF
    return f"{identity:016x}", {p.relative_to(root).as_posix(): sha256(p) for p in paths}


class Elf32:
    """Small bounded ELF32 reader; no third-party host dependency."""

    def __init__(self, data):
        self.data = data
        require(len(data) >= 52 and data[:7] == b"\x7fELF\x01\x01\x01", "expected ELF32 little-endian")
        h = struct.unpack_from("<HHIIIIIHHHHHH", data, 16)
        require(h[0:2] == (2, 40), "expected an ARM executable")
        _, _, _, self.entry, phoff, shoff, _, _, phsize, phnum, shsize, shnum, shstr = h
        require(phsize == 32 and shsize == 40 and shnum > 0 and shstr < shnum, "unsupported ELF tables")
        self.headers = [struct.unpack("<IIIIIIII", self.slice(phoff + i * phsize, phsize)) for i in range(phnum)]
        raw = [struct.unpack("<IIIIIIIIII", self.slice(shoff + i * shsize, shsize)) for i in range(shnum)]
        names = self.slice(raw[shstr][4], raw[shstr][5])
        self.sections = {}
        for s in raw:
            name = self.string(names, s[0])
            self.sections[name] = dict(type=s[1], flags=s[2], addr=s[3], offset=s[4], size=s[5])
            if s[1] != 8:
                self.slice(s[4], s[5])
        self.symbols = {}
        for s in raw:
            if s[1] != 2:
                continue
            require(s[9] == 16 and s[5] % 16 == 0 and s[6] < shnum, "invalid symbol table")
            strings = self.slice(raw[s[6]][4], raw[s[6]][5])
            for offset in range(s[4], s[4] + s[5], 16):
                name, value, size, info, _, index = struct.unpack("<IIIBBH", self.slice(offset, 16))
                if name and index:
                    self.symbols[self.string(strings, name)] = dict(value=value, size=size, info=info)

    def slice(self, offset, size):
        require(0 <= offset <= len(self.data) and 0 <= size <= len(self.data) - offset, "truncated ELF")
        return self.data[offset:offset + size]

    @staticmethod
    def string(data, offset):
        require(offset < len(data), "invalid ELF string offset")
        end = data.find(b"\0", offset)
        require(end >= 0, "unterminated ELF string")
        return data[offset:end].decode("utf-8")


def validate(elf, image, package, source_id):
    require(package == make_package(image), "PX4 container does not match binary/partition metadata")
    payload = zlib.decompress(base64.b64decode(package["image"], validate=True))
    require(payload[:len(image)] == image, "container image mismatch")
    symbols = {name: value["value"] for name, value in elf.symbols.items()}
    expected = {
        "_stack_start": RAM_END, "_stack_end": 0x24070000,
        "__fault_stack_bottom": 0x2406FC00, "__fault_stack_top": 0x24070000,
        "__sheap": 0x24070000,
    }
    for name, value in expected.items():
        require(symbols.get(name) == value, f"unexpected {name}")
    require(symbols.get("__euninit", RAM_END) <= expected["__fault_stack_bottom"], "static RAM overlaps fault stack")
    vector = elf.sections.get(".vector_table", {})
    require(vector.get("addr") == APP_START and vector.get("size", 0) >= 64, "bad vector table location")
    stack, reset, _, fault = struct.unpack_from("<IIII", image)
    require(stack == RAM_END and reset == elf.entry, "ELF/vector entry mismatch")
    require((fault & ~1) == (symbols.get("HardFault", 0) & ~1) and fault & 1, "HardFault vector mismatch")
    for offset in range(4, vector["size"], 4):
        handler = struct.unpack_from("<I", image, offset)[0]
        if handler:
            require(handler & 1 and APP_START <= (handler & ~1) < APP_START + len(image), f"invalid vector at offset {offset}")
    require(source_id.encode() in image, "stale firmware source ID; rebuild before reporting")
    for ptype, offset, vaddr, paddr, filesz, memsz, _, _ in elf.headers:
        if ptype != 1:
            continue
        if filesz:
            require(APP_START <= paddr and paddr + filesz <= APP_START + len(image), "load image outside application")
            require(elf.slice(offset, filesz) == image[paddr - APP_START:paddr - APP_START + filesz], "ELF/bin mismatch")
        if memsz and vaddr >= 0x20000000:
            require(0x24000000 <= vaddr and vaddr + memsz <= 0x2406FC00, "static RAM outside allocation")
    return expected | {"__euninit": symbols["__euninit"], "HardFault": fault, "Reset": reset}


def command(*args):
    return subprocess.check_output(args, cwd=ROOT, text=True).strip()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--release", type=Path, default=Path(os.environ.get("CARGO_TARGET_DIR", "target")) / "thumbv7em-none-eabihf/release")
    parser.add_argument("--fault-injection", action="store_true")
    parser.add_argument("--fault-dump", action="store_true")
    args = parser.parse_args()
    if args.fault_injection and args.fault_dump:
        parser.error("--fault-injection and --fault-dump are mutually exclusive")
    features = [
        name for name, enabled in (
            ("fault-injection", args.fault_injection),
            ("fault-dump", args.fault_dump),
        ) if enabled
    ]
    release = args.release.resolve()
    paths = {suffix: release / f"v6x-rust-led{suffix}" for suffix in ("", ".bin", ".px4", ".map")}
    try:
        identity, sources = source_identity(ROOT, features)
        elf = Elf32(paths[""].read_bytes())
        image = paths[".bin"].read_bytes()
        package = json.loads(paths[".px4"].read_text())
        symbols = validate(elf, image, package, identity)
        require(paths[".map"].stat().st_size > 0, "missing linker map")
        version = tomllib.loads((ROOT / "Cargo.toml").read_text())["package"]["version"]
        report = {
            "schema": 1, "version": version, "source_id": identity,
            "features": features,
            "git_head": command("git", "rev-parse", "HEAD"),
            "git_status": command("git", "status", "--short"),
            "rustc": command("rustc", "-Vv"), "cargo": command("cargo", "-V"),
            "environment": {name: os.environ.get(name) for name in ("CARGO_TARGET_DIR", "RUSTFLAGS", "CARGO_ENCODED_RUSTFLAGS", "SOURCE_DATE_EPOCH")},
            "sources_sha256": sources,
            "tools_sha256": {p.relative_to(ROOT).as_posix(): sha256(p) for p in sorted([
                *(ROOT / "tools").glob("*.py"), ROOT / "tools/firmware.sh",
                ROOT / "tools/requirements.txt", ROOT / ".github/workflows/p1.yml",
            ])},
            "artifacts": {p.name: {"bytes": p.stat().st_size, "sha256": sha256(p)} for p in paths.values()},
            "sections": {n: s for n, s in elf.sections.items() if s["flags"] & 2},
            "symbols": symbols,
            "resources": {
                "flash_raw_bytes": len(image), "flash_padded_bytes": package["image_size"],
                "flash_partition_bytes": APP_SIZE,
                "static_ram_bytes": symbols["__euninit"] - 0x24000000,
                "static_ram_free_bytes": 0x2406FC00 - symbols["__euninit"],
                "shared_stack_reserved_bytes": 65536, "fault_stack_reserved_bytes": 1024,
            },
            "hardware_validation": "pending; software report is not HIL acceptance",
        }
        output = release / "v6x-rust-led.build.json"
        output.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n")
        manifest = release / "SHA256SUMS"
        manifest.write_text("".join(f"{sha256(p)}  {p.name}\n" for p in [*paths.values(), output]))
        print(json.dumps(report["resources"], indent=2))
        print(f"source={identity}; report={output}")
    except (OSError, ValueError, KeyError, struct.error) as error:
        parser.error(str(error))


if __name__ == "__main__":
    main()
