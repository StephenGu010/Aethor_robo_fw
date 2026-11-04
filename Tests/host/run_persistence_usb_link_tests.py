"""Build actual CDC/PCD callback and USB suspend/resume bodies against offline stubs."""
from __future__ import annotations

import argparse
from pathlib import Path
import re
import subprocess


def main() -> int:
    """Extract maintained production bodies verbatim and propagate native test failures."""
    repository = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=repository / "USB_DEVICE/App/usbd_cdc_if.c")
    parser.add_argument("--target-source", type=Path, default=repository / "USB_DEVICE/Target/usbd_conf.c")
    parser.add_argument("--output", type=Path, default=repository / "output/repair_20261023/persistence_usb_link/v2_suspend/build")
    parser.add_argument("--compiler", default="D:/application/WinLibs/mingw64/bin/gcc.exe")
    arguments = parser.parse_args()
    source = arguments.source.read_text(encoding="utf-8")
    callbacks: list[str] = []
    for name in ("CDC_Init_HS", "CDC_DeInit_HS"):
        match = re.search(r"static int8_t " + name + r"\(void\)\s*\{.*?\n\}", source, re.S)
        if not match:
            raise ValueError(f"Maintained callback missing: {name}")
        callbacks.append(match.group())
    state = re.search(r"/\* BEGIN_PERSISTENCE_USB_LINK \*/(.*?)/\* END_PERSISTENCE_USB_LINK \*/", source, re.S)
    # Legacy-only RED adapter exposes the old absence of any event binding as zero.
    # The actual old callbacks, rather than a copied callback implementation, still run.
    state_source = state.group(1) if state else "static uint32_t CDC_PersistenceLinkState_HS(void) { return 0U; }\n"
    usb_core = (repository / "Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c").read_text(encoding="utf-8")
    target = arguments.target_source.read_text(encoding="utf-8")
    for event in ("Suspend", "Resume"):
        library_body = re.search(r"USBD_StatusTypeDef USBD_LL_" + event + r"\(USBD_HandleTypeDef \*pdev\)\s*\{.*?\n\}", usb_core, re.S)
        callback_body = re.search(r"#if \(USE_HAL_PCD_REGISTER_CALLBACKS == 1U\)\s*static void PCD_" + event + r"Callback\(.*?\n\}", target, re.S)
        if not library_body or not callback_body:
            raise ValueError(f"Maintained USB event bodies missing: {event}")
        callbacks.extend((library_body.group(), callback_body.group()))
    arguments.output.mkdir(parents=True, exist_ok=True)
    generated = arguments.output / "persistence_usb_callbacks.inc"
    generated.write_text("/* @file Extracted maintained CDC bodies; synthetic host fixture only. */\n" + state_source + "\n" + "\n".join(callbacks), encoding="utf-8")
    for callback_registration in (0, 1):
        executable = arguments.output / f"persistence_usb_link_test_{callback_registration}.exe"
        command = [arguments.compiler, "-std=c11", "-O2", "-Wall", "-Wextra", "-Werror", "-Wpedantic", f"-DUSE_HAL_PCD_REGISTER_CALLBACKS={callback_registration}", "-I", str(arguments.output), str(repository / "Tests/host/persistence_usb_link_test.c"), "-o", str(executable)]
        compilation = subprocess.run(command, cwd=repository, check=False)
        if compilation.returncode:
            return compilation.returncode
        execution = subprocess.run([str(executable)], cwd=repository, check=False)
        if execution.returncode:
            return execution.returncode
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
