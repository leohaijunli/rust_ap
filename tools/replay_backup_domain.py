#!/usr/bin/env python3
"""Execute the compiled startup caller, RTC wrapper and HAL with Unicorn 2.1.4.

Synthetic retained states and immediate-ready MMIO model; not board/RTC timing
evidence. Loads ELF data/BSS like the runtime, starts at main, and stops after
backup_domain::init returns (or before the panic handler can write a fault).
No Rust ABI layout guesses, skipped HAL calls, serial access or image changes.
"""

import argparse
import itertools
import json
import struct
import zlib
from pathlib import Path

from backup_domain import CONFIG_MASK, RTC_SOURCES
from build_report import Elf32, require, sha256

# Fixed stm32-metapac 19.0.0, cross-checked with PX4/NuttX STM32H7 headers.
# Exact sources and model limits: docs/acceptance/p1/20261031-backup-domain.md.
APP_START = 0x08020000
RAM_START, RAM_SIZE = 0x24000000, 0x80000
RETURN_PC = 0x20000000
RCC_CR, CFGR = 0x58024400, 0x58024410
BDCR, CSR, APB4ENR = 0x58024470, 0x58024474, 0x580244F4
PWR_CR1, PWR_CSR1, PWR_CR3, PWR_D3CR = 0x58024800, 0x58024804, 0x5802480C, 0x58024818
RTC, BKP = 0x58004000, 0x58004050
RTC_END = BKP + 32 * 4
BDRST, RTCSEL, LSEON, LSERDY = 1 << 16, 3 << 8, 1, 2


def symbol(elf, *parts):
    matches = [s['value'] for name, s in elf.symbols.items()
               if all(part in name for part in parts) and s['size']]
    require(len(matches) == 1, f'Expected one symbol matching {parts}, found {len(matches)}')
    return matches[0] & ~1


def seed_rtc():
    # Distinct synthetic sentinels, including PX4 magic and a CRC-valid fault.
    calendar = [0x12340000 + i for i in range((BKP - RTC) // 4)]
    words = [0xA5A50000 + i for i in range(32)]
    words[0:2] = [0xB007B007, 0xFACEFEED]
    words[16:20] = [0x50314631, 1, 0x89ABCDEF, 0x01234567]
    words[31] = zlib.crc32(struct.pack('<15I', *words[16:31])) & 0xFFFFFFFF
    return struct.pack('<52I', *calendar, *words)


def clocks_ready(value):
    # HSIRDY/CSIRDY/HSI48RDY/HSERDY/PLL1..3RDY follow their enable bits.
    for enable, ready in ((0, 2), (7, 8), (12, 13), (16, 17), (24, 25), (26, 27), (28, 29)):
        value = (value & ~(1 << ready)) | (((value >> enable) & 1) << ready)
    return value | (1 << 5)  # HSIDIVF: divider update complete


class Replay:
    def __init__(self, elf, image, bdcr, csr, disturbance=None):
        import unicorn as u
        from unicorn import arm_const as arm

        self.arm = arm
        self.cpu = cpu = u.Uc(u.UC_ARCH_ARM, u.UC_MODE_THUMB | u.UC_MODE_MCLASS)
        cpu.ctl_set_cpu_model(arm.UC_CPU_ARM_CORTEX_M7)
        cpu.mem_map(APP_START, (len(image) + 4095) & ~4095)
        cpu.mem_write(APP_START, image)
        cpu.mem_protect(APP_START, (len(image) + 4095) & ~4095, u.UC_PROT_READ | u.UC_PROT_EXEC)
        cpu.mem_map(RAM_START, RAM_SIZE)
        cpu.mem_map(RETURN_PC, 4096)
        for page in (0x40000000, 0x52002000, 0x58000000, 0x58004000,
                     0x58020000, 0x58021000, 0x58022000, 0x58024000,
                     0x5C001000, 0xE0001000, 0xE000E000, 0xE000F000):
            cpu.mem_map(page, 4096)
        # RAM starts zeroed; initialize .data at its run address, not flash LMA.
        for ptype, offset, vaddr, _, filesz, memsz, _, _ in elf.headers:
            if ptype == 1 and memsz and RAM_START <= vaddr < RAM_START + RAM_SIZE:
                require(vaddr + memsz <= RAM_START + RAM_SIZE, 'ELF RAM segment outside AXI SRAM')
                cpu.mem_write(vaddr, elf.slice(offset, filesz))
        self.registers = {
            RCC_CR: clocks_ready(1), CFGR: 0, BDCR: bdcr, CSR: csr,
            PWR_CSR1: 0xE000, PWR_CR3: 0x05000042, PWR_D3CR: 0xE000,
        }
        for address, value in self.registers.items():
            self.write_word(address, value)
        self.write_word(0x5800042C, 0x81)  # SYSCFG.PWRCR, ODEN
        self.write_word(0x52002000, 0x22)  # FLASH.ACR, initial P0 value
        self.write_word(0x5C001000, 0x20030450)  # DBGMCU.IDC, revision V
        self.seed = seed_rtc()
        cpu.mem_write(RTC, self.seed)
        self.before = bdcr
        self.writes = []
        self.hal_entered = False
        self.panicked = False
        self.disturbance = disturbance
        self.disturbed = False
        self.init = symbol(elf, 'backup_domain', '4init')
        self.predicate = symbol(elf, 'backup_domain', '9preserved')
        # LTO inlines Dump's WFI-only handler into core::panicking functions.
        # Stop at panic entry points, before either image can handle a fault.
        panic_entries = {s['value'] & ~1 for name, s in elf.symbols.items()
                         if s['size'] and ('9panicking' in name or 'rust_begin_unwind' in name)}
        require(panic_entries, 'ELF has no identifiable panic entry points')
        self.main = elf.symbols['main']['value']
        cpu.hook_add(u.UC_HOOK_MEM_READ, self.on_read, begin=0x40000000, end=0x5FFFFFFF)
        cpu.hook_add(u.UC_HOOK_MEM_WRITE, self.on_write, begin=0x40000000, end=0x5FFFFFFF)
        cpu.hook_add(u.UC_HOOK_MEM_INVALID, self.on_invalid)
        for entry in sorted(panic_entries):
            cpu.hook_add(u.UC_HOOK_CODE, self.on_panic, begin=entry, end=entry)
        cpu.reg_write(arm.UC_ARM_REG_SP, RAM_START + RAM_SIZE)
        cpu.reg_write(arm.UC_ARM_REG_LR, RETURN_PC | 1)

    def write_word(self, address, value):
        self.cpu.mem_write(address, struct.pack('<I', value))

    def read_word(self, address):
        return struct.unpack('<I', self.cpu.mem_read(address, 4))[0]

    def on_read(self, cpu, access, address, size, value, user):
        if RTC <= address < RTC_END:
            require(self.read_word(APB4ENR) & (1 << 16), 'RTC read with APB clock disabled')
        if address in self.registers:
            require(size == 4, f'Unexpected MMIO read width at {address:#x}: {size}')
            self.write_word(address, self.registers[address])

    def on_write(self, cpu, access, address, size, value, user):
        if address in (BDCR, CSR, PWR_CR3) or RTC <= address < RTC + 0x400:
            self.writes.append({'pc': cpu.reg_read(self.arm.UC_ARM_REG_PC),
                                'address': address, 'size': size, 'value': value})
        if address == PWR_CR3:
            self.hal_entered = True
            # Model inherited POR/write-once low byte; not supply validation.
            self.registers[address] = (value & ~0xFF) | 0x42
            if self.disturbance == 'hal-mismatch' and not self.disturbed:
                self.registers[BDCR] ^= 1 << 3
                self.disturbed = True
        elif address == RCC_CR:
            self.registers[address] = clocks_ready(value)
        elif address == CFGR:
            self.registers[address] = (value & ~(7 << 3)) | ((value & 7) << 3)
        elif address == PWR_D3CR:
            self.registers[address] = value | (1 << 13)
        elif address == CSR:
            self.registers[address] = (value & ~2) | ((value & 1) << 1)
        elif address == BDCR:
            require(size == 4 and self.read_word(PWR_CR1) & (1 << 8), 'BDCR write without DBP')
            previous = self.registers[BDCR]
            if value & BDRST:
                self.registers[BDCR] = BDRST
                cpu.mem_write(RTC, bytes(len(self.seed)))
            else:
                # RO status bits survive RMW; RTCSEL is locked once nonzero.
                current = (value & CONFIG_MASK) | (previous & 0x42)
                if previous & RTCSEL:
                    current = (current & ~RTCSEL) | (previous & RTCSEL)
                if not current & LSEON:
                    current &= ~LSERDY
                elif not previous & LSEON:
                    current |= LSERDY
                self.registers[BDCR] = current
            # The successful init writes BDCR only at the generated mux. Inject
            # disturbances here to verify the *compiled* post-HAL comparison.
            if not self.disturbed and self.disturbance not in (None, 'hal-mismatch'):
                if self.disturbance.startswith('bkp-'):
                    target = BKP + int(self.disturbance[4:]) * 4
                    self.write_word(target, self.read_word(target) ^ 1)
                else:
                    require(self.disturbance.startswith('bdcr-'), 'Unknown disturbance')
                    self.registers[BDCR] ^= int(self.disturbance[5:], 16)
                self.disturbed = True

    def on_panic(self, cpu, address, size, user):
        self.panicked = True
        cpu.emu_stop()

    def on_invalid(self, cpu, access, address, size, value, user):
        pc = cpu.reg_read(self.arm.UC_ARM_REG_PC)
        raise RuntimeError(f'Unmapped/protected access {access} at {address:#x}, size={size}, pc={pc:#x}')

    def run_until(self, entry, end, limit=100000):
        from unicorn import UcError

        try:
            self.cpu.emu_start(entry | 1, end, count=limit)
        except UcError as error:
            pc = self.cpu.reg_read(self.arm.UC_ARM_REG_PC)
            raise RuntimeError(f'Emulation failed at {pc:#x}: {error}') from error
        pc = self.cpu.reg_read(self.arm.UC_ARM_REG_PC)
        require(self.panicked or pc == end, f'Execution did not reach {end:#x}; stopped at {pc:#x}')

    def run(self):
        self.run_until(self.main, self.init)
        require(not self.panicked, 'Panic before backup-domain wrapper')
        return_pc = self.cpu.reg_read(self.arm.UC_ARM_REG_LR) & ~1
        self.run_until(self.init, return_pc)
        preserved = None
        if not self.panicked:
            self.cpu.reg_write(self.arm.UC_ARM_REG_LR, RETURN_PC | 1)
            self.run_until(self.predicate, RETURN_PC, 1000)
            value = self.cpu.reg_read(self.arm.UC_ARM_REG_R0)
            require(not self.panicked and value in (0, 1), 'Invalid preservation predicate return')
            preserved = bool(value)
        retained = bytes(self.cpu.mem_read(RTC, len(self.seed)))
        bdcr_writes = [w for w in self.writes if w['address'] == BDCR]
        return {
            'bdcr_before': self.before, 'bdcr_after': self.registers[BDCR],
            'csr_after': self.registers[CSR], 'hal_entered': self.hal_entered,
            'panicked': self.panicked, 'preserved': preserved,
            'bkp_matches': retained[BKP - RTC:] == self.seed[BKP - RTC:],
            'calendar_matches': retained[:BKP - RTC] == self.seed[:BKP - RTC],
            'bdrst_writes': sum(bool(w['value'] & BDRST) for w in bdcr_writes),
            'config_change_writes': sum(bool((w['value'] ^ self.before) & CONFIG_MASK) for w in bdcr_writes),
            'rtc_writes': sum(RTC <= w['address'] < RTC + 0x400 for w in self.writes),
            'disturbed': self.disturbed, 'writes': self.writes,
        }


def cases():
    # RTC off/LSE/LSI, all four raw drive settings, stopped/oscillator/bypass,
    # and an unrelated running LSI. HAL must keep the entire retained state.
    for rtc, mode, drive, csr in itertools.product(range(3), (0, 1, 5), range(4), (0, 3)):
        if rtc == 1 and mode == 0:
            continue
        bdcr = rtc << 8 | bool(rtc) << 15 | mode | drive << 3 | (2 if mode else 0)
        yield f'{RTC_SOURCES[rtc]}-mode{mode}-drive{drive}-csr{csr}', bdcr, csr, 'preserve', None
    for drive, status in itertools.product(range(4), (0, 2, 0x40)):
        yield f'LSE-css-drive{drive}-status{status:02x}', 0x8121 | drive << 3 | status, 0, 'preserve', None
    for rtc, enabled in itertools.product(range(4), range(2)):
        if rtc == 3 or bool(rtc) != bool(enabled):
            yield f'reject-rtc{rtc}-enabled{enabled}', rtc << 8 | enabled << 15 | 3, 0, 'reject', None
    for rtc in range(3):
        bdcr = rtc << 8 | bool(rtc) << 15
        yield f'reject-stopped-bypass-rtc{rtc}', bdcr | 4, 0, 'reject', None
        yield f'reject-reset-rtc{rtc}', bdcr | BDRST, 0, 'reject', None
    yield 'reject-stopped-LSE', 0x8100, 0, 'reject', None
    for index in range(32):
        yield f'detect-bkp{index}', 0x811B, 0, 'detect', f'bkp-{index}'
    for bit in (0, 2, 3, 4, 5, 8, 9, 15, 16):
        yield f'detect-bdcr-bit{bit}', 0x811B, 0, 'detect', f'bdcr-{1 << bit:x}'
    for bit in (1, 6):
        yield f'live-bdcr-bit{bit}', 0x811B, 0, 'live', f'bdcr-{1 << bit:x}'
    yield 'detect-HAL-reset', 0x811B, 0, 'reset', 'hal-mismatch'


def check(result, expected, bdcr, csr):
    require(result['rtc_writes'] == 0, 'Initialization wrote RTC/BKP registers')
    if expected == 'reject':
        require(result['panicked'] and not result['hal_entered'], 'Unsupported state reached HAL')
        require(not result['writes'], 'Unsupported state changed backup domain or entered RCC init')
    else:
        require(result['hal_entered'] and not result['panicked'], 'Supported init did not return')
        require(result['preserved'] == (expected in ('preserve', 'live')), 'Wrong preservation result')
        require(result['disturbed'] == (expected != 'preserve'), 'Disturbance hook was not exercised')
    if expected == 'reset':
        require(result['bdrst_writes'] > 0 and not result['bkp_matches']
                and not result['calendar_matches'], 'HAL reset control did not erase modeled RTC/BKP')
    else:
        require(result['bdrst_writes'] == result['config_change_writes'] == 0,
                'Initialization attempted to reset/reconfigure the backup domain')
    if expected in ('preserve', 'reject', 'live'):
        require((result['bdcr_after'] ^ bdcr) & CONFIG_MASK == 0, 'Retained configuration changed')
        require(result['bkp_matches'] and result['calendar_matches'], 'Retained RTC/BKP changed')
    if expected == 'preserve':
        require(result['csr_after'] == (3 if (bdcr & RTCSEL) == 0x200 else csr), 'Unexpected LSI change')
        require(sum(w['address'] == BDCR for w in result['writes']) == 1, 'Generated RTC mux write missing')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('release', type=Path, help='Directory with ELF, bin and build.json')
    parser.add_argument('--json', type=Path, help='Save detailed per-case MMIO evidence')
    args = parser.parse_args()
    try:
        import unicorn
    except ImportError:
        parser.error('Install the optional host dependency unicorn==2.1.4')
    require(unicorn.__version__ == '2.1.4', 'Replay requires the pinned Unicorn 2.1.4')
    report = json.loads((args.release / 'v6x-rust-ap.build.json').read_text())
    for name in ('v6x-rust-ap', 'v6x-rust-ap.bin', 'v6x-rust-ap.px4'):
        require(sha256(args.release / name) == report['artifacts'][name]['sha256'], f'Artifact hash mismatch: {name}')
    elf = Elf32((args.release / 'v6x-rust-ap').read_bytes())
    image = (args.release / 'v6x-rust-ap.bin').read_bytes()
    results = []
    print(f"Unicorn {unicorn.__version__} Cortex-M7; source={report['source_id']} features={report['features']}")
    print('Actual compiled caller + full HAL init; synthetic MMIO, not hardware/timing evidence')
    for name, bdcr, csr, expected, disturbance in cases():
        result = Replay(elf, image, bdcr, csr, disturbance).run()
        results.append({'case': name, 'expected': expected, 'csr_before': csr, **result})
        print(f"{name}: BDCR={bdcr:08x}->{result['bdcr_after']:08x} "
              f"panic={result['panicked']} preserved={result['preserved']} BDRST={result['bdrst_writes']}")
        check(result, expected, bdcr, csr)
    if args.json:
        args.json.write_text(json.dumps({
            'schema': 1, 'unicorn': unicorn.__version__, 'source_id': report['source_id'],
            'features': report['features'], 'artifacts': report['artifacts'],
            'tool_sha256': sha256(Path(__file__)), 'cases': results,
            'scope': 'Compiled startup/HAL with synthetic MMIO; no hardware, timing or RTC calendar progression validation',
        }, indent=2) + '\n')
    print(f'{len(results)} compiled backup-domain cases: PASS')


if __name__ == '__main__':
    main()
