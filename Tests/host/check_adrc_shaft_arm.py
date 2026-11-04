"""Run all frozen synthetic core groups against each actual Keil target core object offline.

No serial, HAL, MCU reset or physical plant is involved. Linked target ARM instructions
are executed in Unicorn; captured SP is fixture execution evidence, not an RTOS WCET bound.
"""
from __future__ import annotations
import argparse
import hashlib
import json
import re
import html
import struct
import subprocess
import sys
from pathlib import Path
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'output/adrc/offline_arm_deps'))
from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS, UC_HOOK_CODE, UC_HOOK_BLOCK, UC_HOOK_MEM_INVALID
from unicorn.arm_const import (UC_CPU_ARM_MAX, UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_R2, UC_ARM_REG_SP,
    UC_ARM_REG_LR, UC_ARM_REG_PC, UC_ARM_REG_C1_C0_2, UC_ARM_REG_FPEXC)


def checked(arguments: list[str]) -> str:
    """Execute only local compilation/link commands with bounded timeout and exact diagnostics."""
    result = subprocess.run(arguments, cwd=ROOT, text=True, capture_output=True, timeout=120, check=False)
    if result.returncode:
        raise RuntimeError(json.dumps({'command': arguments, 'exit_code': result.returncode,
                                      'stdout': result.stdout, 'stderr': result.stderr}))
    return result.stdout + result.stderr


def execute(image: Path) -> dict:
    """Execute compiled fixture entry, rejecting any assertion, instruction exhaustion or incomplete group count."""
    emulator = Uc(UC_ARCH_ARM, UC_MODE_THUMB)
    emulator.ctl_set_cpu_model(UC_CPU_ARM_MAX)
    emulator.mem_map(0x08000000, 0x100000)
    emulator.mem_map(0x20000000, 0x200000)
    with image.open('rb') as stream:
        elf = ELFFile(stream)
        symbols = {symbol.name: {'address': symbol['st_value'], 'size': symbol['st_size']}
                   for symbol in elf.get_section_by_name('.symtab').iter_symbols()}
        for segment in elf.iter_segments():
            if segment['p_type'] == 'PT_LOAD' and segment['p_filesz']:
                emulator.mem_write(segment['p_vaddr'], segment.data())
        # Load initialized sections at their execution VMA, mirroring scatterload's data copy.
        # PT_LOAD may describe flash load addresses while pointer tables execute from SRAM.
        for section in elf.iter_sections():
            if section['sh_flags'] & 2 and section['sh_type'] != 'SHT_NOBITS' and section['sh_size']:
                emulator.mem_write(section['sh_addr'], section.data())
    emulator.reg_write(UC_ARM_REG_C1_C0_2, 0xF << 20)
    emulator.reg_write(UC_ARM_REG_FPEXC, 0x40000000)
    sentinel, stack_top = 0x201F0000, 0x201E0000
    minimum_sp = stack_top

    def track_stack(instance, address, size, data):
        """Capture actual minimum stack pointer at target basic-block boundaries."""
        nonlocal minimum_sp
        minimum_sp = min(minimum_sp, instance.reg_read(UC_ARM_REG_SP))

    def assertion(instance, address, size, data):
        """Reject real full/small assertion entries with original registers, caller and completed groups."""
        caller = instance.reg_read(UC_ARM_REG_LR) & ~1
        candidates = sorted(((value['address'] & ~1, name) for name, value in symbols.items()
                             if value['address'] and (value['address'] & ~1) <= caller), reverse=True)
        registers = {name: instance.reg_read(register) for name, register in
                     (('r0', UC_ARM_REG_R0), ('r1', UC_ARM_REG_R1), ('r2', UC_ARM_REG_R2))}
        groups = {name: struct.unpack('<I', instance.mem_read(symbols[name]['address'], 4))[0]
                  for name in ('completed', 'default_completed', 'low_completed', 'visible_completed',
                               'observe_completed', 'platform_completed', 'wide_completed')}
        entry = next((name for name in ('__aeabi_assert', '__assert', '__assert2', 'abort')
                      if name in symbols and (symbols[name]['address'] & ~1) == address), 'assertion')
        raise AssertionError({'entry': entry, 'pc': hex(address), 'caller_lr': hex(caller),
                              'nearest_caller': candidates[:1], 'registers': registers, 'groups': groups})

    emulator.hook_add(UC_HOOK_BLOCK, track_stack)
    for entry in ('__aeabi_assert', '__assert', '__assert2', 'abort'):
        if entry in symbols:
            location = symbols[entry]['address'] & ~1
            emulator.hook_add(UC_HOOK_CODE, assertion, begin=location, end=location)
    emulator.reg_write(UC_ARM_REG_SP, stack_top)
    emulator.reg_write(UC_ARM_REG_LR, sentinel | 1)
    try:
        # Low, observe and all three platform scheduling groups exhaustively search the 4096-code grid.
        # This offline-only instruction budget does not alter any real-time MCU deadline.
        emulator.emu_start(symbols['shaft_core_arm_main']['address'] | 1, sentinel, count=1_600_000_000)
    except Exception as error:
        pc = emulator.reg_read(UC_ARM_REG_PC)
        nearest = sorted(((data['address'] & ~1, name) for name, data in symbols.items() if data['address'] <= pc), reverse=True)[:3]
        raise RuntimeError({'pc': hex(pc), 'bytes': bytes(emulator.mem_read(pc, 8)).hex(), 'nearest': nearest}) from error
    finished_groups = struct.unpack('<I', emulator.mem_read(symbols['completed']['address'], 4))[0]
    assert emulator.reg_read(UC_ARM_REG_PC) == sentinel, f'ARM instruction budget exhausted after {finished_groups} groups'
    assert emulator.reg_read(UC_ARM_REG_R0) == 0, 'ARM fixture returned nonzero'
    completed = struct.unpack('<I', emulator.mem_read(symbols['completed']['address'], 4))[0]
    default_completed = struct.unpack('<I', emulator.mem_read(symbols['default_completed']['address'], 4))[0]
    assert default_completed == 23, f'Incomplete default ARM fixture groups: {default_completed}'
    low_completed = struct.unpack('<I', emulator.mem_read(symbols['low_completed']['address'], 4))[0]
    assert low_completed == 37, f'Incomplete default/low ARM fixture groups: {low_completed}'
    visible_completed = struct.unpack('<I', emulator.mem_read(symbols['visible_completed']['address'], 4))[0]
    assert visible_completed == 45, f'Incomplete default/low/visible ARM fixture groups: {visible_completed}'
    observe_completed = struct.unpack('<I', emulator.mem_read(symbols['observe_completed']['address'], 4))[0]
    assert observe_completed == 57, f'Incomplete legacy/observe ARM fixture groups: {observe_completed}'
    platform_completed = struct.unpack('<I', emulator.mem_read(symbols['platform_completed']['address'], 4))[0]
    assert platform_completed == 69, f'Incomplete unchanged legacy/observe/platform ARM fixture groups: {platform_completed}'
    wide_completed = struct.unpack('<I', emulator.mem_read(symbols['wide_completed']['address'], 4))[0]
    assert wide_completed == 83, f'Incomplete unchanged legacy/platform_wide ARM fixture groups: {wide_completed}'
    rated70_completed = struct.unpack('<I', emulator.mem_read(symbols['rated70_completed']['address'], 4))[0]
    assert rated70_completed == 101, f'Incomplete preserved platform70 ARM fixture groups: {rated70_completed}'
    assert completed == 119, f'Incomplete platform100 ARM fixture groups: {completed}'
    state_size = struct.unpack('<I', emulator.mem_read(symbols['shaft_arm_state_size']['address'], 4))[0]
    trace_size = struct.unpack('<I', emulator.mem_read(symbols['shaft_arm_trace_size']['address'], 4))[0]
    return {'groups': completed, 'legacy_groups': wide_completed,
            'wide_groups': wide_completed - platform_completed, 'platform70_groups': rated70_completed - wide_completed,
            'platform100_groups': completed - rated70_completed,
            'instruction_budget': 1_600_000_000,
            'state_bytes': state_size, 'trace_bytes': trace_size,
            'fixture_minimum_stack_bytes': stack_top - minimum_sp,
            'physical_send_proof': False, 'qualification': False,
            'instruction_engine': 'ARM MAX Thumb VFP-DP/FPv5; actual Cortex-M7-linked core object; no MCU/timing emulation'}


def formatter_execute(image: Path, target: str) -> dict:
    """Measure actual bridge/parser/CLIB formatter stack on finite ABI extremes; include task prefix separately."""
    emulator = Uc(UC_ARCH_ARM, UC_MODE_THUMB)
    emulator.ctl_set_cpu_model(UC_CPU_ARM_MAX)
    emulator.mem_map(0x08000000, 0x100000)
    emulator.mem_map(0x20000000, 0x200000)
    with image.open('rb') as stream:
        elf = ELFFile(stream)
        symbols = {symbol.name: symbol['st_value'] for symbol in elf.get_section_by_name('.symtab').iter_symbols()}
        for segment in elf.iter_segments():
            if segment['p_type'] == 'PT_LOAD' and segment['p_filesz']:
                emulator.mem_write(segment['p_vaddr'], segment.data())
        # Load initialized sections at their execution VMA, mirroring scatterload's data copy.
        # PT_LOAD may describe flash load addresses while pointer tables execute from SRAM.
        for section in elf.iter_sections():
            if section['sh_flags'] & 2 and section['sh_type'] != 'SHT_NOBITS' and section['sh_size']:
                emulator.mem_write(section['sh_addr'], section.data())
    emulator.reg_write(UC_ARM_REG_C1_C0_2, 0xF << 20)
    emulator.reg_write(UC_ARM_REG_FPEXC, 0x40000000)
    sentinel, stack_top = 0x201F0000, 0x201E0000
    minimum_sp = stack_top

    def track(instance, address, size, data):
        """Track SP on the actual production instructions including library indirect float formatting."""
        nonlocal minimum_sp
        minimum_sp = min(minimum_sp, instance.reg_read(UC_ARM_REG_SP))

    def call(name: str, argument: int = 0) -> int:
        """Call only fixture initialization or read-only formatter entry and reject non-returning code."""
        nonlocal minimum_sp
        minimum_sp = stack_top
        emulator.reg_write(UC_ARM_REG_SP, stack_top)
        emulator.reg_write(UC_ARM_REG_LR, sentinel | 1)
        emulator.reg_write(UC_ARM_REG_R0, argument)
        try:
            emulator.emu_start(symbols[name] | 1, sentinel, count=20_000_000)
        except Exception as error:
            pc = emulator.reg_read(UC_ARM_REG_PC)
            nearest = sorted(((location & ~1, symbol) for symbol, location in symbols.items() if location <= pc), reverse=True)[:3]
            raise RuntimeError({'case':argument,'pc':hex(pc),'bytes':bytes(emulator.mem_read(pc,8)).hex(),'r0':hex(emulator.reg_read(UC_ARM_REG_R0)),'nearest':nearest}) from error
        assert emulator.reg_read(UC_ARM_REG_PC) == sentinel, 'Formatter instruction budget exhausted'
        return emulator.reg_read(UC_ARM_REG_R0)

    def invalid(instance, access, address, size, value, data):
        """Report the actual unmapped read without inventing peripheral or heap state."""
        pc = instance.reg_read(UC_ARM_REG_PC)
        nearest = sorted(((location & ~1, name) for name, location in symbols.items() if location <= pc), reverse=True)[:3]
        raise RuntimeError({'formatter_pc': hex(pc), 'address': hex(address), 'size': size, 'nearest': nearest})

    emulator.hook_add(UC_HOOK_BLOCK, track)
    emulator.hook_add(UC_HOOK_MEM_INVALID, invalid)
    # Initialize only the real CLIB locale tables required by snprintf floating formatting.
    # No semihost I/O, heap setup, HAL or firmware main is executed.
    if '__rt_lib_init' in symbols:
        call('__rt_lib_init')
    call('shaft_formatter_arm_init')
    graph = (ROOT / 'MDK-ARM' / target / (target + '.htm')).read_text(errors='replace')
    prefix_names = ['StartProtocolTask', 'aethor_app_process_protocol_line']
    if target == 'LCD-MIT-ADRC':
        prefix_names.append('aethor_app_integrated_process_line')
    frames = {}
    for name in prefix_names:
        found = re.search(r'<P><STRONG><a name="[^" ]+"></a>' + name + r'</STRONG>.*?Stack size (\d+) bytes', graph, re.S)
        assert found, f'Missing actual target caller frame: {name}'
        frames[name] = int(found.group(1))
    prefix = sum(frames.values())
    cases = []
    for index in range(15):
        assert call('shaft_formatter_arm_case', index) == 0, f'ARM formatter case {index} failed'
        length = struct.unpack('<I', emulator.mem_read(symbols['formatter_length'], 4))[0]
        measured = stack_top - minimum_sp
        assert length < 520
        assert prefix + measured < 5120, 'New formatter path exceeds configured ProtocolTask stack before interrupts/context'
        cases.append({'index': index, 'packet_bytes': length, 'measured_fixture_and_production_stack': measured,
                      'task_prefix_plus_measured': prefix + measured, 'remaining_before_context_and_interrupts': 5120 - prefix - measured})
    return {'cases': cases, 'actual_caller_frames': frames, 'configured_task_bytes': 5120,
            'scope': 'Actual target bridge/adapter/CLIB instructions plus actual task caller frame prefix; includes extra fixture wrapper conservatively',
            'rtos_context_and_interrupt_stack_measured': False, 'physical_send_proof': False,
            'image_sha256': hashlib.sha256(image.read_bytes()).hexdigest()}


def main() -> int:
    """Link reviewed pure fixtures to actual per-target core objects, then retain hashes and ARM results."""
    parser = argparse.ArgumentParser()
    parser.add_argument('target', choices=('ADRC-Bench', 'LCD-MIT-ADRC'))
    parser.add_argument('--compiler', default='D:/application/keil/ARM/ARMCC/bin/armcc.exe')
    args = parser.parse_args()
    compiler = Path(args.compiler)
    directory = ROOT / 'output/adrc/host' / ('shaft_arm_' + args.target)
    directory.mkdir(parents=True, exist_ok=True)
    support = directory / 'shaft_arm_support.c'
    support.write_text('''/** @file shaft_arm_support.c: Offline fixture I/O stubs and native sizeof witnesses. */
#include "adrc_shaft_pilot.h"
#include <stdio.h>
const unsigned shaft_arm_state_size = sizeof(ShaftPilot);
const unsigned shaft_arm_trace_size = sizeof(ShaftPilotTrace);
/** @brief Offline no-output printf; no semihost or physical I/O. */
int printf(const char *format, ...) { (void)format; return 0; }
/** @brief Offline no-output puts; no semihost or physical I/O. */
int puts(const char *text) { (void)text; return 0; }
''', encoding='ascii')
    fixture_object = directory / 'fixture.o'
    support_object = directory / 'support.o'
    common = [str(compiler), '--c99', '--cpu=Cortex-M7', '--fpu=FPv5_D16', '-O3', '--diag_error=warning',
              '-I' + str(ROOT / 'App/Adrc')]
    compile_log = checked(common + ['-Dmain=shaft_core_arm_main', '-c',
        str(ROOT / 'Tests/host/adrc_shaft_pilot_core_test_main.c'), '-o', str(fixture_object)])
    compile_log += checked(common + ['-c', str(support), '-o', str(support_object)])
    target_object = ROOT / 'MDK-ARM' / args.target / 'adrc_shaft_pilot.o'
    assert target_object.is_file(), 'Actual target core object is absent'
    image = directory / 'shaft_core_fixture.axf'
    compile_log += checked([str(compiler.parent / 'armlink.exe'), '--cpu=Cortex-M7.fp.dp',
        '--entry=shaft_core_arm_main', '--datacompressor=off', '--ro_base=0x08000000', '--rw_base=0x20000000', '--map', '--symbols',
        '--keep=shaft_arm_state_size', '--keep=shaft_arm_trace_size', '--list=' + str(directory / 'fixture.map'),
        str(fixture_object), str(support_object), str(target_object), '-o', str(image)])
    result = execute(image)
    formatter_object = directory / 'formatter.o'
    formatter_source = ROOT / 'Tests/host/adrc_shaft_formatter_arm_fixture.c'
    formatter_includes = ['-I' + str(ROOT / 'App' / name) for name in ('Adrc', 'Arm', 'Config', 'Motor', 'Motion', 'Protocol', 'Telemetry')]
    compile_log += checked(common + formatter_includes + ['-c', str(formatter_source), '-o', str(formatter_object)])
    link_arguments = (ROOT / 'MDK-ARM' / args.target / (args.target + '.lnp')).read_text()
    target_objects = [str((ROOT / 'MDK-ARM' / name).resolve())
                      for name in re.findall(r'^"([^"\r\n]+\.o)"', link_arguments, re.M)]
    assert len(target_objects) > 20, 'Actual target linker object inventory is missing'
    formatter_image = directory / 'shaft_formatter_fixture.axf'
    compile_log += checked([str(compiler.parent / 'armlink.exe'), '--cpu=Cortex-M7.fp.dp',
        '--entry=shaft_formatter_arm_init', '--datacompressor=off', '--keep=shaft_formatter_arm_case', '--ro_base=0x08000000', '--rw_base=0x20000000',
        '--map', '--symbols', '--list=' + str(directory / 'formatter.map'), str(formatter_object), str(support_object),
        *target_objects, '-o', str(formatter_image)])
    result['formatter'] = formatter_execute(formatter_image, args.target)
    result['adapter_object_sha256'] = hashlib.sha256((ROOT / 'MDK-ARM' / args.target / 'adrc_shaft_pilot_adapter.o').read_bytes()).hexdigest()
    result.update({'target': args.target, 'core_object_sha256': hashlib.sha256(target_object.read_bytes()).hexdigest(),
                   'fixture_image_sha256': hashlib.sha256(image.read_bytes()).hexdigest(), 'compile_log': compile_log})
    (directory / 'evidence.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
    print(json.dumps(result))
    return 0

if __name__ == '__main__':
    raise SystemExit(main())
