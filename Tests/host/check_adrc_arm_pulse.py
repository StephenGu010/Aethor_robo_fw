"""Offline execution of an ARMCC simple-trial pulse tick; never accesses hardware.

The native packed-enum ARMCC layout below is checked against the trial size
and offsets visible in the reviewed target disassembly. This is a focused
code-generation reproduction, not a physical plant or full MCU simulation.
"""
import argparse
import json
import struct
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'output/adrc/offline_arm_deps'))
from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS
from unicorn.arm_const import (UC_CPU_ARM_CORTEX_M7, UC_ARM_REG_R0,
    UC_ARM_REG_R1, UC_ARM_REG_R2, UC_ARM_REG_R3, UC_ARM_REG_SP,
    UC_ARM_REG_LR, UC_ARM_REG_PC, UC_ARM_REG_S0, UC_ARM_REG_C1_C0_2, UC_ARM_REG_FPEXC)


def check_image(path, phase, age_us, previous_torque, profile):
    """Execute one pulse tick using healthy feedback and a specified phase age."""
    emulator = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
    emulator.ctl_set_cpu_model(UC_CPU_ARM_CORTEX_M7)
    for address, size in ((0x08000000, 0x100000), (0x20000000, 0x100000),
                          (0x24000000, 0x100000), (0x30000000, 0x100000),
                          (0x38000000, 0x10000)):
        emulator.mem_map(address, size)
    with path.open('rb') as stream:
        elf = ELFFile(stream)
        symbols = {symbol.name: symbol['st_value'] for symbol in elf.get_section_by_name('.symtab').iter_symbols()}
        for segment in elf.iter_segments():
            if segment['p_type'] == 'PT_LOAD' and segment['p_filesz']:
                emulator.mem_write(segment['p_vaddr'], segment.data())
    emulator.reg_write(UC_ARM_REG_C1_C0_2, 0xF << 20)
    emulator.reg_write(UC_ARM_REG_FPEXC, 0x40000000)
    trial, feedback, output, sentinel = 0x20080000, 0x20090000, 0x20090100, 0x200A0000

    def call(name, arguments, quantum=None):
        """Call a bounded target function using its hard-float AAPCS entry."""
        emulator.reg_write(UC_ARM_REG_SP, 0x200F0000)
        emulator.reg_write(UC_ARM_REG_LR, sentinel | 1)
        for register, value in zip((UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_R2, UC_ARM_REG_R3), arguments):
            emulator.reg_write(register, value)
        if quantum is not None:
            emulator.reg_write(UC_ARM_REG_S0, struct.unpack('<I', struct.pack('<f', quantum))[0])
        emulator.emu_start(symbols[name] | 1, sentinel, count=200000)
        assert emulator.reg_read(UC_ARM_REG_PC) == sentinel, 'Target function did not return within the instruction budget'
        return emulator.reg_read(UC_ARM_REG_R0)

    assert call('adrc_simple_init', (trial, sentinel | 1, 0), 25.0 / 65535.0) == 1
    assert call('adrc_simple_set_profile', (trial, 9)) == 0
    if profile:
        assert call('adrc_simple_set_profile', (trial, profile)) == 1
    current_time = 1000000
    input_bytes = bytearray(64)
    struct.pack_into('<QQ', input_bytes, 0, current_time, current_time)
    struct.pack_into('<ff', input_bytes, 0x20, 25.0, 25.0)
    input_bytes[0x2C:0x32] = bytes((6, 1, 0, 0, 0, 0))
    emulator.mem_write(feedback, bytes(input_bytes))
    assert call('adrc_simple_start', (trial, feedback, output)) == 1
    assert call('adrc_simple_set_profile', (trial, (profile + 1) % 9)) == 0
    emulator.mem_write(trial, bytes((phase,)))
    emulator.mem_write(trial + 0x70A0, struct.pack('<Q', current_time - age_us))
    emulator.mem_write(trial + 0x70A8, struct.pack('<Q', current_time - 4000))
    emulator.mem_write(trial + 0x70B0, struct.pack('<Q', current_time - 4000))
    emulator.mem_write(trial + 0x71D7, bytes((2,)))
    input_bytes[0x2E] = 1
    input_bytes[0x30] = 1
    struct.pack_into('<f', input_bytes, 0x28, previous_torque)
    input_bytes[0x3C:0x3E] = bytes((1, 1))
    emulator.mem_write(feedback, bytes(input_bytes))
    call('adrc_simple_tick', (trial, feedback, output))
    torque = struct.unpack('<f', emulator.mem_read(output, 4))[0]
    integral_us = struct.unpack('<Q', emulator.mem_read(trial + 0x71A0, 8))[0]
    impulse = struct.unpack('<f', emulator.mem_read(trial + 0x71C4, 4))[0]
    torque_limit = (0.03, 0.05, 0.08, 0.10, 0.14, 0.20, 0.25, 0.35, 1.75)[profile]
    step_nm = 0.016 if profile >= 7 else (0.012 if profile >= 5 else 0.008)
    expected_torque = max(-torque_limit, min(torque_limit, previous_torque + (step_nm if phase == 3 else -step_nm)))
    duration = (80000, 160000, 200000, 240000, 360000, 360000, 360000, 360000, 640000)[profile]
    if 4 <= profile <= 7:
        duration = 312000 if phase == 3 else 360000
    lower, upper = (32000, 56000) if profile == 0 else (duration - (160000 if profile >= 4 else 96000), duration - 32000)
    expected_integral = 4000 if profile != 8 and lower < age_us <= upper else 0
    result = {'image': str(path), 'phase': emulator.mem_read(trial, 1)[0], 'age_us': age_us,
              'torque_nm': torque, 'expected_torque_nm': expected_torque, 'slew_nm_s': step_nm / 0.004,
              'send_torque': emulator.mem_read(output + 5, 1)[0],
              'disable_requested': emulator.mem_read(output + 7, 1)[0],
              'pulse_integral_us': integral_us, 'pulse_impulse': impulse,
              'previous_torque': previous_torque, 'profile': profile, 'fault': emulator.mem_read(trial + 2, 1)[0]}
    if profile == 8 and phase == 5:
        # The forbidden negative phase must disable immediately, with controller fault and no integration.
        result['expected_torque_nm'] = 0.0
        print(json.dumps(result))
        return (torque == 0.0 and result['send_torque'] == 0 and result['disable_requested'] == 1 and
                result['phase'] == 8 and result['fault'] == 9 and integral_us == 0 and impulse == 0.0)
    print(json.dumps(result))
    return (abs(torque - expected_torque) < 0.000001 and result['send_torque'] == 1 and
            integral_us == expected_integral and result['phase'] == phase and result['fault'] == 0 and
            abs(impulse - previous_torque * expected_integral * 1.0e-6) < 1.0e-8)


def main():
    """Report legacy pulse fitting and fixed probe positive/forbidden-negative behavior for the supplied ELF."""
    parser = argparse.ArgumentParser()
    parser.add_argument('image', type=Path)
    arguments = parser.parse_args()
    results = []
    for profile, duration in enumerate((80000, 160000, 200000, 240000, 360000, 360000, 360000, 360000, 640000)):
        for phase in (3, 5):
            phase_duration = (312000 if phase == 3 else 360000) if 4 <= profile <= 7 else duration
            lower, upper = (32000, 56000) if profile == 0 else (phase_duration - (160000 if profile >= 4 else 96000), phase_duration - 32000)
            for age in (lower - 8000, lower, lower + 4000, lower + 8000, upper, upper + 4000):
                sign = 1.0 if phase == 3 else -1.0
                limit = (0.03, 0.05, 0.08, 0.10, 0.14, 0.20, 0.25, 0.35, 1.75)[profile]
                for previous in (0.0, sign * 0.01, sign * (limit - 0.001)):
                    results.append(check_image(arguments.image, phase, age, previous, profile))
    print(json.dumps({'cases': len(results), 'passed': sum(results), 'legacy_cases': 288, 'startup_probe_cases': 36}))
    raise SystemExit(0 if all(results) else 1)


if __name__ == '__main__':
    main()
