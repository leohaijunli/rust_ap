#!/usr/bin/env python3
"""Replay the compiled P1 clock predicate on a host with Unicorn 2.1.4.

Only CR3 comes from the physical Dump; other registers model the intended P1
configuration. This executes the real ELF function, not HAL or board hardware.
See docs/acceptance/p1/20261031-startup-assertion.md for the evidence and setup.
"""

import argparse
import json
import struct
from pathlib import Path

from build_report import Elf32, sha256
from decode_fault_dump import decode_text


APP_START = 0x08020000
CR3 = 0x5802480C
CSR1 = 0x58024804
D3CR = 0x58024818
PWRCR = 0x5800042C
RCC_CR = 0x58024400
CFGR = 0x58024410
USBSEL = 0x58024454
FLASH_ACR = 0x52002000
TIM2_PSC = 0x40000028
TIM2_ARR = 0x4000002C


def p1_registers(cr3):
    # STM32H753 register addresses/encodings: fixed PAC 19.0.0, cross-checked
    # with PX4's STM32H7 memory map/RCC/PWR/FLASH, stm32_tim.h and board.h.
    return {
        RCC_CR: 0x3303C025,       # HSE, PLL1, PLL3 ready; HSE crystal
        CFGR: 0x0000001B,         # SW/SWS = PLL1_P
        0x58024418: 0x00000048,   # CPU /1, AHB /2, APB3 /2
        0x5802441C: 0x00000440,   # APB1 /2, APB2 /2
        0x58024420: 0x00000040,   # APB4 /2
        0x58024428: 0x00400012,   # HSE, DIVM1=1, DIVM3=4
        0x5802442C: 0x01C10A0C,   # PLL1P and PLL3P/Q/R, integer mode
        0x58024430: 0x0101023B,   # PLL1 N=60, P=2
        0x58024440: 0x0103022F,   # PLL3 N=48, P=2, Q=4, R=2
        USBSEL: 0x00200000,       # PLL3Q
        CR3: cr3,
        D3CR: 0x0000E000,         # VOS1, VOSRDY (Scale0 with ODEN)
        CSR1: 0x0000E000,         # ACTVOSRDY
        PWRCR: 0x00000081,        # ODEN
        FLASH_ACR: 0x00000024,    # LATENCY=4, WRHIGHFREQ=2
        TIM2_PSC: 239,            # 240 MHz /240 = 1 MHz
        TIM2_ARR: 65535,
    }


def replay(image, entry, registers):
    import unicorn as u
    from unicorn import arm_const as arm

    cpu = u.Uc(u.UC_ARCH_ARM, u.UC_MODE_THUMB | u.UC_MODE_MCLASS)
    cpu.ctl_set_cpu_model(arm.UC_CPU_ARM_CORTEX_M7)
    cpu.mem_map(APP_START, (len(image) + 4095) & ~4095)
    cpu.mem_write(APP_START, image)
    cpu.mem_map(0x24070000, 65536)
    return_pc = 0x20000000
    cpu.mem_map(return_pc, 4096)
    pages = {address & ~4095 for address in registers}
    for page in pages:
        cpu.mem_map(page, 4096)
    for address, value in registers.items():
        cpu.mem_write(address, struct.pack('<I', value))
    for page in pages:
        cpu.mem_protect(page, 4096, u.UC_PROT_READ)
    cpu.reg_write(arm.UC_ARM_REG_SP, 0x24080000)
    cpu.reg_write(arm.UC_ARM_REG_LR, return_pc | 1)
    cpu.emu_start(entry | 1, return_pc, count=10000)
    if cpu.reg_read(arm.UC_ARM_REG_PC) != return_pc:
        raise RuntimeError('clock predicate did not return within 10000 instructions')
    result = cpu.reg_read(arm.UC_ARM_REG_R0)
    if result not in (0, 1):
        raise RuntimeError(f'Unexpected bool return value: {result}')
    return bool(result)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('release', type=Path, help='Directory with ELF, bin and build.json')
    parser.add_argument('--dump', type=Path, required=True, help='Saved physical Dump text')
    parser.add_argument('--legacy-scuen', action='store_true', help='Expect the old SCUEN false rejection')
    args = parser.parse_args()
    try:
        import unicorn
    except ImportError:
        parser.error('Install optional host dependency unicorn==2.1.4; see the evidence document')
    report = json.loads((args.release / 'v6x-rust-led.build.json').read_text())
    for name in ('v6x-rust-led', 'v6x-rust-led.bin'):
        if sha256(args.release / name) != report['artifacts'][name]['sha256']:
            raise ValueError(f'Artifact hash mismatch: {name}')
    elf = Elf32((args.release / 'v6x-rust-led').read_bytes())
    matches = [s['value'] for name, s in elf.symbols.items()
               if 'platform' in name and 'clock_registers_ok' in name and s['size']]
    if len(matches) != 1:
        raise ValueError(f'Expected one compiled clock predicate, found {len(matches)}')
    image = (args.release / 'v6x-rust-led.bin').read_bytes()
    dump = decode_text(args.dump.read_text())
    if not dump['complete'] or not dump['fault']['valid']:
        raise ValueError('Expected a complete physical Dump with a valid fault record')
    cr3 = dump['entry_power']['cr3']
    if cr3 != 0x05000042:
        raise ValueError(f'Expected the incident CR3=0x05000042, got 0x{cr3:08x}')
    # Invalid-state cases start with SCUEN=1 so the legacy predicate reaches
    # every tested check instead of rejecting everything at its SCUEN check.
    base = p1_registers(cr3 | 4)
    cases = [
        ('physical_cr3', p1_registers(cr3), not args.legacy_scuen),
        ('scuen_set', base, True),
        ('ldo_disabled', base | {CR3: (cr3 | 4) & ~2}, False),
        ('bypass_enabled', base | {CR3: cr3 | 5}, False),
        ('voltage_not_ready', base | {D3CR: base[D3CR] & ~(1 << 13)}, False),
        ('active_voltage_not_ready', base | {CSR1: base[CSR1] & ~(1 << 13)}, False),
        ('overdrive_disabled', base | {PWRCR: base[PWRCR] & ~1}, False),
        ('wrong_sysclk', base | {CFGR: 0}, False),
        ('pll1_unlocked', base | {RCC_CR: base[RCC_CR] & ~(1 << 25)}, False),
        ('pll3_unlocked', base | {RCC_CR: base[RCC_CR] & ~(1 << 29)}, False),
        ('wrong_usb_clock', base | {USBSEL: 1 << 20}, False),
        ('insufficient_flash_latency', base | {FLASH_ACR: 0x22}, False),
        ('wrong_tick_prescaler', base | {TIM2_PSC: 63}, False),
        ('wrong_timer_period', base | {TIM2_ARR: 0xFFFFFFFF}, False),
    ]
    print(f"Unicorn {unicorn.__version__} Cortex-M7; source={report['source_id']} entry=0x{matches[0] & ~1:08x}")
    print('Modeled P1 clocks + physical Dump CR3; no HAL/board execution')
    for name, registers, expected in cases:
        actual = replay(image, matches[0], registers)
        print(f'{name}: actual={actual} expected={expected}')
        if actual != expected:
            raise AssertionError(f'{name}: clock predicate returned {actual}, expected {expected}')
    print(f'{len(cases)} compiled-predicate cases: PASS')


if __name__ == '__main__':
    main()
