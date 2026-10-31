"""Decode P1 schema-2 backup-domain evidence, without accessing hardware."""

import re

# H753 RCC_BDCR: LSEON/BYP/DRV/CSSON, RTCSEL/EN, BDRST. See hardware ledger.
CONFIG_MASK = 0x0001833D
RTC_SOURCES = ("DISABLE", "LSE", "LSI", "HSE")


def decode_backup(values):
    if not values:
        return None  # Legacy reports contain no evidence of preservation.
    result = {}
    for name in ("bdcr_before", "bdcr_after", "csr_before", "csr_after"):
        raw = values.get(name, "")
        if not re.fullmatch(r"[0-9a-fA-F]{8}", raw):
            raise ValueError(f"missing/invalid backup-domain {name}")
        result[name] = int(raw, 16)
    for name in ("config_preserved", "bkp_preserved"):
        if values.get(name) not in ("true", "false"):
            raise ValueError(f"missing/invalid backup-domain {name}")
        result[name] = values[name] == "true"
    before, after = result["bdcr_before"], result["bdcr_after"]
    result["config_matches"] = (before ^ after) & CONFIG_MASK == 0
    result["rtc_before"] = RTC_SOURCES[(before >> 8) & 3]
    result["rtc_after"] = RTC_SOURCES[(after >> 8) & 3]
    result["preserved"] = (
        result["config_matches"] and result["config_preserved"]
        and result["bkp_preserved"] and not ((before | after) & (1 << 16))
    )
    return result
