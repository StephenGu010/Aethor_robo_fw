"""Check actual Keil linked memory maps, keeping hardware timing out of the result.

The maintained scatter file alone is not evidence that DMA landed in AXI RAM.
This checks real symbols, zero-init sections, capacities and clean build logs.
"""
from pathlib import Path
import argparse
import hashlib
import json
import re

TARGETS = ("CtrBoard-H7_FDCAN", "LCD-ReadOnly", "LCD-POS", "LCD-MIT")


def verify_known_stack_depth(contents: str, callgraph: str, with_ui: bool) -> list[dict]:
    """Reject even known call chains exceeding reserved task stacks plus a 512B margin.

    ARMCC marks indirect calls/assembly as unknown. These are lower bounds, not
    proof of complete maximum usage; board stack high-water marks remain required.
    """
    tasks = {"StartArmControlTask": "armControlTaskBuffer", "StartCanRxTask": "canRxTaskBuffer",
             "StartProtocolTask": "protocolTaskBuffer", "StartUsbTxTask": "usbTxTaskBuffer",
             "StartTelemetryTask": "telemetryTaskBuffer", "StartDiagnosticsTask": "diagnosticsTaskBuffer"}
    if with_ui:
        tasks["StartDebugUiTask"] = "debugUiTaskBuffer"
    records = []
    for task, buffer in tasks.items():
        section = re.search(r'<P><STRONG><a name="[^"]+"></a>' + task +
                            r'</STRONG>.*?(?=<P><STRONG>|$)', callgraph, re.S)
        assert section, task
        depth = re.search(r'Max Depth =\s*(\d+)', section[0])
        assert depth, task
        allocation = read_symbol(contents, buffer)[1]
        known = int(depth[1])
        assert known + 512 <= allocation, (task, known, allocation, "known chain plus 512B margin")
        records.append(dict(task=task, reserved_bytes=allocation, known_depth_bytes=known,
                            known_only=True, actual_stack_high_water_pending=True))
    return records


def read_symbol(contents: str, name: str) -> tuple[int, int]:
    """Return one linked Data symbol's address and byte size from ARM linker output."""
    match = re.search(r"^\s*" + re.escape(name) + r"\s+(0x[0-9a-fA-F]+)\s+Data\s+(\d+)\s", contents, re.M)
    assert match is not None, f"Missing linked symbol {name}"
    return int(match[1], 16), int(match[2])


def verify_target(root: Path, target: str) -> dict:
    """Validate final addresses and budgets, retaining the exact firmware hash."""
    output = root / "MDK-ARM" / target
    map_path = output / f"{target}.map"
    hex_path = output / f"{target}.hex"
    contents = map_path.read_text(encoding="utf-8", errors="replace")
    log = (root / "Tests/host/build/keil" / f"{target}.log").read_text(errors="replace")
    assert "0 Error(s), 0 Warning(s)" in log, target
    size = re.search(r"Program Size: Code=(\d+) RO-data=(\d+) RW-data=(\d+) ZI-data=(\d+)", log)
    assert size, target
    code, ro, rw, zi = map(int, size.groups())
    assert code + ro + rw <= 1024 * 1024, (target, "Flash budget")
    assert rw + zi <= (128 + 320) * 1024, (target, "DTCM plus AXI budget")
    report = dict(target=target, code_bytes=code, ro_bytes=ro, rw_bytes=rw, zi_bytes=zi,
                  ram_bytes=rw + zi, hex_sha256=hashlib.sha256(hex_path.read_bytes()).hexdigest(),
                  map=str(map_path), hardware_timing_verified=False)
    report["task_stack_lower_bounds"] = verify_known_stack_depth(
        contents, (output / f"{target}.htm").read_text(errors="replace"), target != TARGETS[0])
    if target == TARGETS[0]:
        assert not re.search(r"^\s*(?:draw_buffer_first|draw_buffer_second|debug_ui_lvgl_pool|debugUiTaskBuffer)\s+0x", contents, re.M)
        assert not re.search(r"^\s*StartDebugUiTask\s+0x.*Thumb Code", contents, re.M)
        report["ui_symbols_absent"] = True
    else:
        buffers = [read_symbol(contents, name) for name in ("draw_buffer_first", "draw_buffer_second")]
        assert all(address % 32 == 0 and size == 13440 and
                   0x24000000 <= address and address + size <= 0x24008000
                   for address, size in buffers), buffers
        ordered = sorted(buffers)
        assert ordered[0][0] + ordered[0][1] <= ordered[1][0]
        assert read_symbol(contents, "debug_ui_lvgl_pool") == (0x24008000, 65536)
        assert read_symbol(contents, "debugUiTaskBuffer")[1] == 8192
        for section in (".lcd_dma", ".lvgl_pool"):
            assert re.search(r"^\s*0x[0-9a-fA-F]+\s+0x[0-9a-fA-F]+\s+Zero\s+RW\s+\d+\s+" + re.escape(section), contents, re.M), section
        report.update(dma_buffers=[{"address": hex(address), "bytes": size} for address, size in buffers],
                      lvgl_pool_address="0x24008000", lvgl_pool_bytes=65536,
                      ui_stack_bytes=8192, dedicated_regions_zero_initialized=True)
    print(f"MAP_PASS {target}: code={code} ro={ro} rw={rw} zi={zi}")
    return report


def main() -> None:
    """Check selected targets, defaulting to the complete four-target build matrix."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--target", choices=TARGETS, action="append")
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[2]
    reports = [verify_target(root, target) for target in (args.target or TARGETS)]
    output = root / "Tests/host/build/verification/memory-layout.json"
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps({"software_only": True, "targets": reports}, indent=2) + "\n", encoding="utf-8")
    print(f"DEBUG_UI_MAP_VERIFIED targets={len(reports)} report={output}")


if __name__ == "__main__":
    main()
