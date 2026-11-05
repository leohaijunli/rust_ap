#!/usr/bin/env python3
"""Reject oversized ARM Thumb entry frames before upload; not a call-chain/WCET proof."""

import argparse
import hashlib
import json
import re
import subprocess
from pathlib import Path

FUNCTION = re.compile(r"^([0-9a-f]+) <(.+)>:$")
RESERVE = re.compile(r"\bsub(?:\.w)?\s+sp,\s*(?:sp,\s*)?#(0x[0-9a-f]+|\d+)")
PUSH = re.compile(r"\b(v?push)(?:\.w)?\s+\{([^}]+)\}")


def entry_frames(text):
    frames = []
    current = None
    for line in text.splitlines():
        match = FUNCTION.match(line)
        if match:
            current = {"address": match[1], "function": match[2], "bytes": 0, "instructions": []}
            frames.append(current)
            continue
        if current is None:
            continue
        # Only the entry basic block is measured. A later adjustment or an
        # indirect callee can consume more: the limitation stays in the report.
        if re.search(r"\b(?:blx?|b(?:\.w)?|cbz|cbnz|tbh|tbb|bx|pop|vpop)\s", line):
            current = None
            continue
        reserve = RESERVE.search(line)
        push = PUSH.search(line)
        if reserve:
            current["bytes"] += int(reserve[1], 0)
        elif push:
            registers = []
            for item in push[2].split(","):
                item = item.strip()
                bounds = re.fullmatch(r"([rd])(\d+)-[rd](\d+)", item)
                registers.extend([item] * (int(bounds[3]) - int(bounds[2]) + 1) if bounds else [item])
            current["bytes"] += sum(8 if register.startswith("d") else 4 for register in registers)
        if reserve or push:
            current["instructions"].append(line.strip())
    return frames


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--elf", type=Path, required=True)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--limit", type=int, default=32768)
    args = parser.parse_args()
    sysroot = Path(subprocess.check_output(["rustc", "--print", "sysroot"], text=True).strip())
    host = next(line[6:] for line in subprocess.check_output(["rustc", "-vV"], text=True).splitlines() if line.startswith("host: "))
    tools = sysroot / "lib/rustlib" / host / "bin"
    objdump = tools / "llvm-objdump"
    if not objdump.exists(): objdump = objdump.with_suffix(".exe")
    text = subprocess.check_output([str(objdump), "-d", "--demangle", str(args.elf)], text=True)
    frames = sorted(entry_frames(text), key=lambda entry: entry["bytes"], reverse=True)
    if not frames:
        raise SystemExit("no ARM entry frames decoded")
    oversized = [frame for frame in frames if frame["bytes"] > args.limit]
    report = {"schema": 1, "elf_sha256": hashlib.sha256(args.elf.read_bytes()).hexdigest(),
              "entry_frame_limit_bytes": args.limit, "functions_decoded": len(frames),
              "largest_frames": frames[:20], "oversized": len(oversized),
              "limitations": "entry immediate reserves and push registers only; excludes later dynamic changes, call-chain nesting, ISR stacking and unused reserve gaps; board watermarks also required"}
    destination = args.output or args.elf.parent / "stack-frames.json"
    destination.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(f"ARM entry frames: largest={frames[0]['bytes']} B; oversized={len(oversized)}; report={destination}")
    if oversized:
        raise SystemExit("oversized ARM frame: do not upload; reduce temporaries before board validation")


if __name__ == "__main__":
    main()
