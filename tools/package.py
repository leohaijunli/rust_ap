#!/usr/bin/env python3
"""Validate a standalone Rust image and package it for the FMUv6X USB loader."""

import argparse
import base64
import json
import os
import struct
import zlib
from pathlib import Path


APP_START = 0x08020000
APP_SIZE = 1920 * 1024
RAM_START = 0x24000000
RAM_END = RAM_START + 512 * 1024
BOARD_ID = 53
FLASH_WORD_SIZE = 32


def make_package(image: bytes) -> dict:
    if len(image) < 8:
        raise ValueError("image is too short to contain a Cortex-M vector table")
    if len(image) > APP_SIZE:
        raise ValueError("image exceeds the FMUv6X application flash partition")

    stack, entry = struct.unpack_from("<II", image)
    if not RAM_START < stack <= RAM_END or stack % 8:
        raise ValueError("initial stack pointer must be 8-byte aligned in AXI SRAM")
    if not entry & 1:
        raise ValueError("reset vector must have its Thumb bit set")
    if not APP_START <= (entry & ~1) < APP_START + len(image):
        raise ValueError("reset vector is outside the image linked at 0x08020000")

    # STM32H7 flash programming uses 256-bit words. Erased padding is 0xff.
    payload = image + b"\xff" * (-len(image) % FLASH_WORD_SIZE)
    return {
        "magic": "PX4FWv1",
        "board_id": BOARD_ID,
        "board_revision": 0,
        "description": "Rust/Embassy FMUv6X platform and timing diagnostics",
        "git_identity": "v6x-rust-led",
        "image_size": len(payload),
        "image_maxsize": APP_SIZE,
        "image": base64.b64encode(zlib.compress(payload, 9)).decode("ascii"),
    }


def main() -> None:
    root = Path(__file__).resolve().parents[1]
    target = Path(os.environ.get("CARGO_TARGET_DIR", str(root / "target")))
    release = target / "thumbv7em-none-eabihf" / "release"
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", type=Path, default=release / "v6x-rust-led.bin")
    parser.add_argument("--output", type=Path, default=release / "v6x-rust-led.px4")
    args = parser.parse_args()

    try:
        image = args.binary.read_bytes()
        package = make_package(image)
        args.output.write_text(json.dumps(package, indent=2) + "\n", encoding="utf-8")
    except (OSError, ValueError) as error:
        parser.error(str(error))

    print(f"Packaged {len(image)} raw bytes as {package['image_size']} flash bytes")
    print(f"Board ID: {BOARD_ID}; application address: 0x{APP_START:08x}")
    print(f"USB upload image: {args.output}")


if __name__ == "__main__":
    main()
