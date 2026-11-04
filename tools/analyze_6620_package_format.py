"""Offline DM3520 package inspection; never loads vendor code or accesses hardware.

Requires capstone and pyelftools in the existing analysis_dependencies directory.
The transport model is transcribed from DMTool ARM64 static disassembly, not
claimed to be a motor bootloader decoder or a hardware-qualified updater.
"""

import hashlib
import json
import math
from collections import Counter
from pathlib import Path
import struct
import sys

ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / "output/driver_6620_20261023"
sys.path.insert(0, str(OUTPUT / "analysis_dependencies"))
from capstone import Cs, CS_ARCH_ARM64, CS_MODE_ARM
from elftools.elf.elffile import ELFFile

TOOL = Path("D:/download/TCG/S3519/DM-Tools/motor-debugging-tool-master/Linux/arm64/DMTool-v2.1.8.4-arm64/DMTool_arm64/bin/serial-port-assistant")
FUNCTIONS = {
    "_ZN10MainWindow21motorFirmwareFileOpenE7QString",
    "_ZN10MainWindow25on_firmwareUpgradeClickedEv",
    "_ZN9CustomCDC15firmwareUpgradeEv",
    "_ZN9CustomCDC13sendNextFrameEv",
    "_ZN9CustomCDC7packageE10QByteArrayh",
    "_Z5CRC_810QByteArrayi",
}


def crc8(payload):
    """Reproduce CRC_8 at VA 0x133c18: init 0, right shift, xor 0x8C."""
    result = 0
    for value in payload:
        result ^= value
        for _ in range(8):
            result = (result >> 1) ^ (0x8C if result & 1 else 0)
    return result


def frame_payload(payload, remaining):
    """Model package() at VA 0x138c88, without any transport capability."""
    if not 0 < len(payload) <= 8192 or not 0 <= remaining <= 255:
        raise ValueError("invalid offline chunk")
    return bytes((0x23, remaining, 0x23)) + struct.pack("<H", len(payload)) + payload + bytes((crc8(payload),))


def extract_frame(frame):
    """Validate the reconstructed envelope and return its unchanged payload."""
    if len(frame) < 7 or frame[0] != 0x23 or frame[2] != 0x23:
        raise ValueError("invalid frame header")
    length = struct.unpack_from("<H", frame, 3)[0]
    if not 0 < length <= 8192 or len(frame) != length + 6:
        raise ValueError("invalid frame length")
    payload = frame[5:-1]
    if crc8(payload) != frame[-1]:
        raise ValueError("invalid payload CRC")
    return payload


def read_virtual(elf, stream, address, length):
    """Resolve an ELF virtual address through file-backed sections."""
    for section in elf.iter_sections():
        start = section["sh_addr"]
        if section["sh_type"] != "SHT_NOBITS" and start <= address and address + length <= start + section["sh_size"]:
            stream.seek(section["sh_offset"] + address - start)
            return stream.read(length)
    raise ValueError(f"unmapped address: {address:#x}")


def disassemble_tool():
    """Save selected ARM64 functions and annotated calls without executing ELF."""
    records = []
    with TOOL.open("rb") as stream:
        elf = ELFFile(stream)
        if elf["e_machine"] != "EM_AARCH64":
            raise ValueError("unexpected tool architecture")
        symbols = list(elf.get_section_by_name(".symtab").iter_symbols())
        names = {symbol["st_value"]: symbol.name for symbol in symbols if symbol["st_value"]}
        relocations = elf.get_section_by_name(".rela.plt")
        dynamic = elf.get_section(relocations["sh_link"])
        plt = elf.get_section_by_name(".plt")
        for index, relocation in enumerate(relocations.iter_relocations()):
            names[plt["sh_addr"] + 32 + index * 16] = dynamic.get_symbol(relocation["r_info_sym"]).name
        for symbol in symbols:
            if symbol.name not in FUNCTIONS:
                continue
            address, length = symbol["st_value"], symbol["st_size"]
            raw = read_virtual(elf, stream, address, length)
            lines = []
            for instruction in Cs(CS_ARCH_ARM64, CS_MODE_ARM).disasm(raw, address):
                target = int(instruction.op_str[1:], 16) if instruction.mnemonic in ("bl", "b") and instruction.op_str.startswith("#0x") else 0
                lines.append(f"{instruction.address:x}: {instruction.mnemonic} {instruction.op_str} {names.get(target, '')}")
            path = OUTPUT / f"format_{address:x}.asm"
            path.write_text("\n".join(lines) + "\n", encoding="utf-8")
            records.append({"symbol": symbol.name, "address": hex(address), "size": length, "code_sha256": hashlib.sha256(raw).hexdigest(), "disassembly": path.name})
    if len(records) != len(FUNCTIONS):
        raise ValueError("required functions missing")
    return records


def inspect_package(path):
    """Record byte-level evidence, transport roundtrip, and qualified vector scan."""
    content = path.read_bytes()
    count = math.ceil(len(content) / 8192)
    chunks = [content[offset:offset + 8192] for offset in range(0, len(content), 8192)]
    frames = [frame_payload(chunk, count - index - 1) for index, chunk in enumerate(chunks)]
    reconstructed = b"".join(extract_frame(frame) for frame in frames)
    if reconstructed != content:
        raise AssertionError("transport model changed package bytes")
    # Conservative conventional STM32 SRAM/flash test, not proof of all MCU layouts.
    candidates = []
    for offset in range(0, len(content) - 64, 4):
        words = struct.unpack_from("<16I", content, offset)
        stack, reset = words[:2]
        handlers = sum(value & 1 and 0x08000000 <= value < 0x08200000 for value in words[1:])
        if 0x20000000 <= stack < 0x20100000 and stack % 8 == 0 and reset & 1 and 0x08000000 <= reset < 0x08200000 and handlers >= 5:
            candidates.append(offset)
    frequencies = Counter(content)
    entropy = -sum((value / len(content)) * math.log2(value / len(content)) for value in frequencies.values())
    return {"file": str(path.relative_to(ROOT)), "sha256": hashlib.sha256(content).hexdigest(), "size": len(content), "first32": content[:32].hex(), "last16": content[-16:].hex(), "entropy_bits_per_byte": entropy, "conventional_vector_candidates": candidates, "vector_scan_scope": "4-byte aligned; SRAM 0x20000000..0x200FFFFF; flash 0x08000000..0x081FFFFF; >=5 Thumb handlers", "transport_roundtrip": True, "frames": [{"payload_length": len(chunk), "header": frame[:5].hex(), "crc8": frame[-1], "frame_sha256": hashlib.sha256(frame).hexdigest()} for chunk, frame in zip(chunks, frames)]}


def main():
    """Write reproducible findings and refuse to infer a control-loop root cause."""
    if crc8(b"123456789") != 0xA1 or crc8(b"") != 0:
        raise AssertionError("CRC check vectors failed")
    packages = []
    for name in ("APP_DM3520_6620_01.bin", "APP_DM2325_6620_01.bin"):
        matches = list((OUTPUT / "vendor-firmware").rglob(name))
        if len(matches) != 1:
            raise ValueError(f"ambiguous package {name}")
        packages.append(inspect_package(matches[0]))
    report = {"tool": str(TOOL), "tool_sha256": hashlib.sha256(TOOL.read_bytes()).hexdigest(), "functions": disassemble_tool(), "packages": packages, "decoder_found": False, "FC_handler_located": False, "MIT_handler_located": False, "target_reset": "unknown", "position_reference_reset": "unknown", "current_loop_reset": "unknown", "root_cause_proven": False, "hardware_access": False, "limitations": "Static transport reconstruction only. No vendor execution, no bootloader, no decoded executable image. Empty vector scan is restricted to stated ranges and alignment."}
    (OUTPUT / "package_format.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"packages": [(item["size"], len(item["frames"]), item["transport_roundtrip"], item["conventional_vector_candidates"]) for item in packages], "functions": len(report["functions"]), "root_cause_proven": False}))


if __name__ == "__main__":
    main()
