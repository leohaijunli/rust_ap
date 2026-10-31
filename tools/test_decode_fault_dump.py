import contextlib
import io
import struct
import unittest
import zlib

from decode_fault_dump import (
    MAGIC,
    decode_report,
    decode_pll_clock,
    decode_text,
    fnv1a32,
    match_source,
    parse_reports,
    print_report,
)


def seal(words):
    words = list(words)
    words[15] = zlib.crc32(struct.pack("<15I", *words[:15])) & 0xFFFFFFFF
    return words


def valid_words(kind=2):
    words = [0] * 16
    words[0] = MAGIC
    words[1] = kind
    words[2] = 0x2345_6789
    words[3] = 0x0123_4567
    words[4] = 1
    words[5] = 0xDEAD_BEEF
    words[6] = 0x0802_AA42
    words[7] = 0x0802_AA43
    words[8] = 0x2407_FF80
    words[9] = 0x2100_0000
    words[10] = (1 << 9) | (1 << 15)
    words[11] = 1 << 30
    words[12] = 0
    words[13] = 0x2000_0000
    words[14] = 0xFFFF_FFF9
    return seal(words)


def report(words, complete=True, rsr=1 << 24):
    pllckselr = 2 | (1 << 4) | (4 << 20)
    pllcfgr = (1 << 16) | (1 << 17) | (1 << 22) | (1 << 23)
    # Raw STM32H753 registers, not already-decoded multipliers/mux values.
    # PAC 19.0.0: MUL60=0x3b, MUL48=0x2f; USBSEL[21:20], SWS[5:3].
    pll1 = 0x0000023B
    pll3 = 0x0003022F
    rcc = (f"V6X_DUMP_RCC stage={{stage}} cr=03000000 cfgr=0000001b d1=00000000 "
           f"d2=00000000 d3=00000000 pllckselr={pllckselr:08x} pllcfgr={pllcfgr:08x} "
           f"pll1={pll1:08x} pll2=00000000 pll3={pll3:08x} usbsel=00200000")
    lines = [
        "body text that is not part of the dump",
        "V6X_DUMP schema=1 version=0.2.0 source=0123456789abcdef git=abc feature=fault-dump",
        f"V6X_DUMP_STATE stage=entry rsr={rsr:08x} idc=20036450 backup_restored=true",
        rcc.format(stage="entry"),
        "V6X_DUMP_POWER stage=entry cr3=00000006 d3cr=00004000 csr1=00000000 pwrcr=00000001 flash_acr=00000024",
        f"V6X_DUMP_BKP0 stage=entry value={0xB007B007:08x} bootloader_request=true",
        "V6X_DUMP_BKP16 stage=entry values=" + " ".join(f"{word:08x}" for word in words),
        "V6X_DUMP_FRAME pc=0802aa42 lr=0802aa43 sp=2407ff80 xpsr=21000000 exc_return=fffffff9",
        "V6X_DUMP_FAULTREG cfsr=00008200 hfsr=40000000 mmfar=00000000 bfar=20000000 stored_crc=00000000",
        rcc.format(stage="p0"),
        "V6X_DUMP_POWER stage=p0 cr3=00000006 d3cr=00004000 csr1=00000000 pwrcr=00000001 flash_acr=00000024",
        f"V6X_DUMP_BKP0 stage=p0 value={0xB007B007:08x} bootloader_request=true",
        "V6X_DUMP_BKP16 stage=p0 values=" + " ".join(f"{word:08x}" for word in words),
    ]
    if complete:
        lines.append("V6X_DUMP_END")
    return "\r\n".join(lines) + "\r\n"


class DecodeFaultDumpTests(unittest.TestCase):
    def test_decodes_valid_hard_fault_and_names(self):
        reports = parse_reports(report(valid_words()))
        self.assertEqual(len(reports), 1)
        result = decode_report(reports[0])
        self.assertTrue(result["complete"])
        self.assertTrue(result["fault"]["valid"])
        self.assertEqual(result["fault"]["kind_name"], "HARD_FAULT")
        self.assertEqual(result["fault"]["source"], 0x01234567_23456789)
        self.assertEqual(result["fault"]["pc"], 0x0802AA42)
        self.assertEqual(result["bkp0"], 0xB007B007)
        self.assertTrue(result["bootloader_request"])
        self.assertIn("SFTRSTF", result["reset_flags"])

    def test_rejects_damaged_crc_without_crashing(self):
        words = valid_words()
        words[6] ^= 1
        result = decode_report(parse_reports(report(words))[0])
        self.assertFalse(result["fault"]["valid"])
        self.assertNotEqual(result["fault"]["computed_crc"], result["fault"]["stored_crc"])

    def test_decodes_pll_clock_fields(self):
        words = valid_words()
        result = decode_report(parse_reports(report(words))[0])
        for clock in (result["entry_clock"], result["p0_clock"]):
            self.assertEqual(clock["source"], "HSE")
            self.assertEqual((clock["divm1"], clock["divm2"], clock["divm3"]), (1, 0, 4))
            self.assertEqual((clock["pll1_n"], clock["pll1_p"], clock["pll1_q"]), (60, 2, 1))
            self.assertEqual((clock["pll3_n"], clock["pll3_p"], clock["pll3_q"]), (48, 2, 4))
            self.assertEqual(clock["usbsel"], "PLL3_Q")
            self.assertTrue(clock["pll1_divp_en"])
            self.assertTrue(clock["pll3_divq_en"])

    def test_partial_report_is_marked_incomplete(self):
        result = decode_report(parse_reports(report(valid_words(), complete=False))[0])
        self.assertFalse(result["complete"])
        self.assertTrue(result["fault"]["valid"])

    def test_first_complete_report_wins_when_stream_repeats(self):
        first = report(valid_words(kind=1))
        second = report(valid_words(kind=2))
        partial = report(valid_words(kind=3), complete=False)
        result = decode_text(partial + first + second)
        self.assertEqual(result["fault"]["kind_name"], "PANIC")
        self.assertTrue(result["complete"])

    def test_sysclk_uses_status_and_usbsel_ignores_other_muxes(self):
        # Requested PLL1 while hardware still reports HSI; USART mux low bits
        # must not be mistaken for USBSEL.
        text = report(valid_words()).replace("cfgr=0000001b", "cfgr=00000003")
        text = text.replace("usbsel=00200000", "usbsel=00200001")
        result = decode_text(text)
        for clock in (result["entry_clock"], result["p0_clock"]):
            self.assertEqual(clock["sysclk"], "HSI")
            self.assertEqual(clock["requested_sysclk"], "PLL1_P")
            self.assertEqual(clock["usbsel"], "PLL3_Q")
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            print_report(result)
        self.assertIn("requested_sysclk=PLL1_P sysclk=HSI", output.getvalue())

    def test_multiplier_encoding_at_valid_limits(self):
        clock = decode_pll_clock({"pll1": 0x003, "pll3": 0x1FF})
        self.assertEqual((clock["pll1_n"], clock["pll3_n"]), (4, 512))

    def test_non_u32_backup_words_do_not_crash_decoder(self):
        for bad in (-1, 0x1_0000_0000):
            with self.subTest(bad=bad):
                words = valid_words()
                words[6] = bad
                result = decode_text(report(words))
                self.assertFalse(result["fault"]["valid"])

    def test_text_without_dump_reports_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "No V6X_DUMP record found"):
            decode_text("unrelated serial data\n")

    def test_pll_field_helpers_do_not_guess_unknown_sources(self):
        clock = decode_pll_clock({"pllckselr": 3, "pll1": 0, "pll3": 0, "usbsel": 0})
        self.assertEqual(clock["source"], "DISABLE")
        self.assertEqual(clock["usbsel"], "DISABLE")

    def test_source_hash_matches_the_rust_fixture(self):
        # src/fault_record.rs pins the same value for "src/platform.rs".
        self.assertEqual(fnv1a32("src/platform.rs"), 0x60652E12)
        self.assertEqual(match_source(0x60652E12), ["src/platform.rs"])

    def test_unknown_source_hash_is_reported_without_a_guess(self):
        self.assertEqual(match_source(0xDEADBEEF), [])
        result = decode_report(parse_reports(report(valid_words()))[0])
        self.assertEqual(result["fault"]["file"], [])

    def test_legacy_dump_does_not_claim_backup_preservation(self):
        result = decode_text(report(valid_words()))
        self.assertIsNone(result["backup_domain"])
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            print_report(result)
        self.assertIn("backup-domain preservation: unknown", output.getvalue())

    def test_schema_2_backup_registers_are_checked_independently(self):
        text = report(valid_words()).replace("schema=1", "schema=2").replace(
            "V6X_DUMP_END",
            "V6X_DUMP_BACKUP bdcr_before=0000811b bdcr_after=0000811b "
            "csr_before=00000000 csr_after=00000000 config_preserved=true bkp_preserved=true\r\nV6X_DUMP_END",
        )
        result = decode_text(text)
        self.assertTrue(result["backup_domain"]["preserved"])
        self.assertEqual(result["backup_domain"]["rtc_before"], "LSE")
        # Readiness is live status; RTC selection, drive, enable and CSS are not.
        for word, expected in (("00008119", True), ("0000821b", False),
                               ("0000810b", False), ("0000011b", False),
                               ("0001811b", False), ("0000813b", False)):
            with self.subTest(bdcr_after=word):
                changed = text.replace("bdcr_after=0000811b", f"bdcr_after={word}")
                self.assertEqual(decode_text(changed)["backup_domain"]["preserved"], expected)
        changed = text.replace("bkp_preserved=true", "bkp_preserved=false")
        self.assertFalse(decode_text(changed)["backup_domain"]["preserved"])

    def test_malformed_backup_evidence_is_not_decoded_as_zero(self):
        base = report(valid_words()).replace("V6X_DUMP_END", "V6X_DUMP_BACKUP {values}\r\nV6X_DUMP_END")
        for values in ("bdcr_before=garbage", "unavailable=true"):
            with self.subTest(values=values), self.assertRaisesRegex(ValueError, "backup-domain"):
                decode_text(base.replace("{values}", values))


if __name__ == "__main__":
    unittest.main()
