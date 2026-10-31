#!/usr/bin/env python3
"""Decode the read-only USB report from the P1 fault-dump image.

This host tool never opens a bootloader port, uploads firmware, reboots the
board or writes the retained record. With --port it only reads the dump CDC
serial stream produced by the diagnostic image.
"""

import argparse
import json
import math
import struct
import sys
import time
import zlib
from pathlib import Path

from backup_domain import decode_backup

ROOT = Path(__file__).resolve().parents[1]
MAGIC = 0x5031_4631
KINDS = {
    1: "PANIC",
    2: "HARD_FAULT",
    3: "UNEXPECTED_EXCEPTION",
    4: "STACK_GUARD",
}
PLL_SOURCES = {0: "HSI", 1: "CSI", 2: "HSE", 3: "DISABLE"}
USB_SELECT = {0: "DISABLE", 1: "PLL1_Q", 2: "PLL3_Q", 3: "HSI48"}
SYSCLK = {0: "HSI", 1: "CSI", 2: "HSE", 3: "PLL1_P"}

RSR_FLAGS = {
    17: "CPURSTF",
    19: "D1RSTF",
    20: "D2RSTF",
    21: "BORRSTF",
    22: "PINRSTF",
    23: "PORRSTF",
    24: "SFTRSTF",
    26: "IWDG1RSTF",
    28: "WWDG1RSTF",
    30: "LPWRRSTF",
}
CFSR_FLAGS = {
    0: "IACCVIOL",
    1: "DACCVIOL",
    3: "MUNSTKERR",
    4: "MSTKERR",
    5: "MLSPERR",
    7: "MMARVALID",
    8: "IBUSERR",
    9: "PRECISERR",
    10: "IMPRECISERR",
    11: "UNSTKERR",
    12: "STKERR",
    13: "LSPERR",
    15: "BFARVALID",
    16: "UNDEFINSTR",
    17: "INVSTATE",
    18: "INVPC",
    19: "NOCP",
    24: "UNALIGNED",
    25: "DIVBYZERO",
}
HFSR_FLAGS = {1: "VECTTBL", 30: "FORCED", 31: "DEBUGEVT"}


def parse_reports(text):
    """Return complete or partial reports, each as a list of tag fields."""
    reports = []
    current = None
    for raw in text.splitlines():
        line = raw.strip()
        if not line.startswith("V6X_DUMP"):
            continue
        tokens = line.split()
        tag = tokens[0]
        if tag == "V6X_DUMP":
            if current is not None:
                reports.append(current)
            current = []
        if current is None:
            continue
        fields = {"tag": tag}
        key = None
        for token in tokens[1:]:
            if "=" in token:
                key, value = token.split("=", 1)
                fields[key] = value
            elif key:
                fields[key] += " " + token
        current.append(fields)
        if tag == "V6X_DUMP_END":
            reports.append(current)
            current = None
    if current is not None:
        reports.append(current)
    return reports


def find(lines, tag, stage=None):
    for line in lines:
        if line["tag"] != tag:
            continue
        if stage is None or line.get("stage") == stage:
            return line
    return None


def int_field(line, name, default=0):
    try:
        return int(line.get(name, default), 16)
    except (TypeError, ValueError):
        return default


def flags(value, names):
    return [name for bit, name in names.items() if value & (1 << bit)]


def crc_ok(words):
    return (zlib.crc32(struct.pack("<15I", *words[:15])) & 0xFFFFFFFF) == words[15]


def fnv1a32(text):
    """Match src/fault_record.rs::file_hash."""
    value = 0x811C9DC5
    for byte in text.encode():
        value = ((value ^ byte) * 0x01000193) & 0xFFFFFFFF
    return value


def match_source(word, root=ROOT):
    """Return source paths whose panic Location hash equals the stored word."""
    root = Path(root)
    matches = []
    for path in sorted((root / "src").rglob("*.rs")):
        relative = path.relative_to(root).as_posix()
        if fnv1a32(relative) == word:
            matches.append(relative)
    return matches


def decode_fault(words, root=ROOT):
    words_ok = len(words) == 16 and all(0 <= word <= 0xFFFFFFFF for word in words)
    valid = (
        words_ok
        and words[0] == MAGIC
        and words[1] in KINDS
        and crc_ok(words)
    )
    return {
        "valid": valid,
        "magic": words[0] if words else 0,
        "kind": words[1] if len(words) > 1 else 0,
        "kind_name": KINDS.get(words[1], "UNKNOWN") if len(words) > 1 else "UNKNOWN",
        "source": (words[3] << 32) | words[2] if len(words) > 3 else 0,
        "detail": words[4] if len(words) > 4 else 0,
        "file_hash": words[5] if len(words) > 5 else 0,
        "file": match_source(words[5], root) if len(words) > 5 else [],
        "pc": words[6] if len(words) > 6 else 0,
        "lr": words[7] if len(words) > 7 else 0,
        "sp": words[8] if len(words) > 8 else 0,
        "xpsr": words[9] if len(words) > 9 else 0,
        "cfsr": words[10] if len(words) > 10 else 0,
        "hfsr": words[11] if len(words) > 11 else 0,
        "mmfar": words[12] if len(words) > 12 else 0,
        "bfar": words[13] if len(words) > 13 else 0,
        "exc_return": words[14] if len(words) > 14 else 0,
        "stored_crc": words[15] if len(words) > 15 else 0,
        "computed_crc": zlib.crc32(struct.pack("<15I", *words[:15])) & 0xFFFFFFFF if words_ok else 0,
    }


def decode_pll_clock(rcc):
    pllckselr = rcc.get("pllckselr", 0)
    pllcfgr = rcc.get("pllcfgr", 0)
    pll1 = rcc.get("pll1", 0)
    pll3 = rcc.get("pll3", 0)
    return {
        "source": PLL_SOURCES.get(pllckselr & 3, "UNKNOWN"),
        "divm1": (pllckselr >> 4) & 0x3F,
        "divm2": (pllckselr >> 12) & 0x3F,
        "divm3": (pllckselr >> 20) & 0x3F,
        "pll1_n": (pll1 & 0x1FF) + 1,
        "pll1_p": ((pll1 >> 9) & 0x7F) + 1,
        "pll1_q": ((pll1 >> 16) & 0x7F) + 1,
        "pll1_divp_en": bool(pllcfgr & (1 << 16)),
        "pll1_divq_en": bool(pllcfgr & (1 << 17)),
        "pll3_n": (pll3 & 0x1FF) + 1,
        "pll3_p": ((pll3 >> 9) & 0x7F) + 1,
        "pll3_q": ((pll3 >> 16) & 0x7F) + 1,
        "pll3_divp_en": bool(pllcfgr & (1 << 22)),
        "pll3_divq_en": bool(pllcfgr & (1 << 23)),
        # The wire field called usbsel contains the whole RCC_D2CCIP2R.
        "usbsel": USB_SELECT.get((rcc.get("usbsel", 0) >> 20) & 3, "UNKNOWN"),
        "requested_sysclk": SYSCLK.get(rcc.get("cfgr", 0) & 7, "UNKNOWN"),
        "sysclk": SYSCLK.get((rcc.get("cfgr", 0) >> 3) & 7, "UNKNOWN"),
    }


def decode_report(lines):
    complete = find(lines, "V6X_DUMP_END") is not None
    header = find(lines, "V6X_DUMP") or {}
    state = find(lines, "V6X_DUMP_STATE", "entry") or {}
    bkp0_line = find(lines, "V6X_DUMP_BKP16", "entry") or {}
    bkp0_value = int_field(find(lines, "V6X_DUMP_BKP0", "entry") or {}, "value")
    try:
        words = [int(word, 16) for word in bkp0_line.get("values", "").split()]
    except ValueError:
        words = []
    rcc_line = find(lines, "V6X_DUMP_RCC", "entry") or {}
    rcc = {
        "cr": int_field(rcc_line, "cr"),
        "cfgr": int_field(rcc_line, "cfgr"),
        "d1": int_field(rcc_line, "d1"),
        "d2": int_field(rcc_line, "d2"),
        "d3": int_field(rcc_line, "d3"),
        "pllckselr": int_field(rcc_line, "pllckselr"),
        "pllcfgr": int_field(rcc_line, "pllcfgr"),
        "pll1": int_field(rcc_line, "pll1"),
        "pll2": int_field(rcc_line, "pll2"),
        "pll3": int_field(rcc_line, "pll3"),
        "usbsel": int_field(rcc_line, "usbsel"),
    }
    post_line = find(lines, "V6X_DUMP_RCC", "p0") or {}
    post_rcc = {
        "cfgr": int_field(post_line, "cfgr"),
        "pllckselr": int_field(post_line, "pllckselr"),
        "pllcfgr": int_field(post_line, "pllcfgr"),
        "pll1": int_field(post_line, "pll1"),
        "pll3": int_field(post_line, "pll3"),
        "usbsel": int_field(post_line, "usbsel"),
    }
    power_line = find(lines, "V6X_DUMP_POWER", "entry") or {}
    rsr = int_field(state, "rsr")
    return {
        "schema": int_field(header, "schema", 1),
        "version": header.get("version", ""),
        "source": header.get("source", ""),
        "git": header.get("git", ""),
        "complete": complete,
        "rsr": rsr,
        "reset_flags": flags(rsr, RSR_FLAGS),
        "bkp0": bkp0_value,
        "bootloader_request": bkp0_value == 0xB007_B007,
        "backup_domain": decode_backup(find(lines, "V6X_DUMP_BACKUP")),
        "fault": decode_fault(words),
        "entry_clock": decode_pll_clock(rcc),
        "p0_clock": decode_pll_clock(post_rcc),
        "entry_rcc": {name: value for name, value in rcc.items()},
        "entry_power": {
            "cr3": int_field(power_line, "cr3"),
            "d3cr": int_field(power_line, "d3cr"),
            "csr1": int_field(power_line, "csr1"),
            "pwrcr": int_field(power_line, "pwrcr"),
            "flash_acr": int_field(power_line, "flash_acr"),
        },
    }


def decode_text(text):
    """Prefer the first complete snapshot when a capture starts/ends mid-report."""
    reports = parse_reports(text)
    if not reports:
        raise ValueError("No V6X_DUMP record found")
    complete = [report for report in reports if find(report, "V6X_DUMP_END") is not None]
    return decode_report((complete or reports)[0])


def print_report(result):
    print(f"V6X fault dump: version={result['version']} source={result['source']} "
          f"git={result['git']} complete={'yes' if result['complete'] else 'no'}")
    reset = result["rsr"]
    print(f"reset cause: 0x{reset:08x} {' '.join(result['reset_flags']) or '(none)'}")
    print(f"BKP0: 0x{result['bkp0']:08x} "
          f"{'(bootloader requested)' if result['bootloader_request'] else ''}")
    backup = result.get("backup_domain")
    if backup is None:
        print("backup-domain preservation: unknown (no schema-2 evidence)")
    else:
        print(f"backup-domain preservation: {'YES' if backup['preserved'] else 'NO'} "
              f"BDCR=0x{backup['bdcr_before']:08x}->0x{backup['bdcr_after']:08x} "
              f"RTC={backup['rtc_before']}->{backup['rtc_after']} "
              f"BKP={'unchanged' if backup['bkp_preserved'] else 'changed'}")
    fault = result["fault"]
    print(f"retained fault: valid={'YES' if fault['valid'] else 'NO'} "
          f"kind={fault['kind']} ({fault['kind_name']})")
    if fault["valid"]:
        source_file = fault["file"][0] if len(fault["file"]) == 1 else ""
        print(f"  source=0x{fault['source']:016x} detail={fault['detail']} "
              f"file_hash=0x{fault['file_hash']:08x}"
              + (f" file={source_file}" if source_file else ""))
        if len(fault["file"]) > 1:
            print(f"  file_hash is ambiguous: {fault['file']}")
        if fault["kind_name"] == "PANIC" and not source_file:
            print("  no local src/*.rs matches file_hash; use the source archive "
                  "for this source id, not the current tree")
        print(f"  frame pc=0x{fault['pc']:08x} lr=0x{fault['lr']:08x} "
              f"sp=0x{fault['sp']:08x} xpsr=0x{fault['xpsr']:08x} "
              f"exc_return=0x{fault['exc_return']:08x}")
        print(f"  cfsr=0x{fault['cfsr']:08x} {' '.join(flags(fault['cfsr'], CFSR_FLAGS)) or '(none)'}")
        print(f"  hfsr=0x{fault['hfsr']:08x} {' '.join(flags(fault['hfsr'], HFSR_FLAGS)) or '(none)'}")
        print(f"  mmfar=0x{fault['mmfar']:08x} bfar=0x{fault['bfar']:08x}")
    else:
        print(f"  magic=0x{fault['magic']:08x} stored_crc=0x{fault['stored_crc']:08x} "
              f"computed_crc=0x{fault['computed_crc']:08x}")
    entry = result["entry_clock"]
    print(f"entry clock: source={entry['source']} divm1={entry['divm1']} "
          f"divm2={entry['divm2']} divm3={entry['divm3']} "
          f"pll1={entry['pll1_n']}/{entry['pll1_p']}/{entry['pll1_q']} "
          f"pll3={entry['pll3_n']}/{entry['pll3_p']}/{entry['pll3_q']} "
          f"usbsel={entry['usbsel']}")
    p0 = result["p0_clock"]
    print(f"P0 clock:    source={p0['source']} divm1={p0['divm1']} "
          f"divm2={p0['divm2']} divm3={p0['divm3']} "
          f"pll1={p0['pll1_n']}/{p0['pll1_p']}/{p0['pll1_q']} "
          f"pll3={p0['pll3_n']}/{p0['pll3_p']}/{p0['pll3_q']} "
          f"usbsel={p0['usbsel']}")
    print(f"entry CFGR=0x{result['entry_rcc']['cfgr']:08x} "
          f"requested_sysclk={entry['requested_sysclk']} sysclk={entry['sysclk']}")
    power = result["entry_power"]
    print(f"entry power: cr3=0x{power['cr3']:08x} d3cr=0x{power['d3cr']:08x} "
          f"pwrcr=0x{power['pwrcr']:08x} flash_acr=0x{power['flash_acr']:08x}")


def read_port(port, seconds, raw=None):
    import serial

    if not math.isfinite(seconds) or seconds <= 0:
        raise ValueError("seconds must be positive and finite")
    with serial.Serial(port, 115200, timeout=0.2) as device:
        deadline = time.monotonic() + seconds
        chunks = []
        while time.monotonic() < deadline:
            data = device.read(4096)
            chunks.append(data)
            if raw is not None:
                raw.write(data)
                raw.flush()
        return b"".join(chunks).decode("utf-8", errors="replace")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", nargs="?", type=Path,
                        help="saved dump text (default: stdin)")
    parser.add_argument("--port", help="read the dump CDC port for --seconds")
    parser.add_argument("--seconds", type=float, default=8.0)
    parser.add_argument("--json", action="store_true")
    parser.add_argument("--raw", type=Path, help="save the captured text")
    args = parser.parse_args()

    if args.port and args.input:
        parser.error("use either an input file or --port, not both")
    if args.port:
        text = read_port(args.port, args.seconds)
    elif args.input:
        text = args.input.read_text(encoding="utf-8", errors="replace")
    else:
        text = sys.stdin.read()
    if args.raw:
        args.raw.write_text(text, encoding="utf-8")

    try:
        result = decode_text(text)
    except ValueError as error:
        parser.error(str(error))
    if args.json:
        print(json.dumps(result, indent=2))
    else:
        print_report(result)


if __name__ == "__main__":
    main()
