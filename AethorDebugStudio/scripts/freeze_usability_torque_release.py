"""Freeze verified local usability/torque outputs; never opens hardware or flashes firmware."""
from __future__ import annotations

import hashlib
import json
import os
from datetime import datetime, timezone
from pathlib import Path
import shutil

HOST = Path(__file__).resolve().parents[1]
FIRMWARE = HOST.parent / "studio-usability-torque-candidate-20261029"
BASELINE = HOST.parent / "session-quota-recovery-candidate-20261029"
RELEASE = HOST / "output/usability-torque-release"


def digest(path: Path) -> str:
    """Return a streaming SHA-256 for a single explicit file."""
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def entry(path: Path, root: Path) -> dict:
    """Describe an existing regular file relative to its declared root."""
    assert path.is_file() and not path.is_symlink(), str(path)
    return {"path": path.relative_to(root).as_posix(), "sha256": digest(path), "bytes": path.stat().st_size}


def freeze_copy(source: Path, relative: str) -> dict:
    """Copy one explicit file under the release root and verify exact bytes."""
    destination = RELEASE / relative
    assert destination.resolve().is_relative_to(RELEASE.resolve()), str(destination)
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(source, destination)
    assert digest(source) == digest(destination), str(source)
    return entry(destination, RELEASE)


def source_files() -> list[Path]:
    """Enumerate source-only host inputs, excluding dependencies and generated outputs."""
    result: set[Path] = set()
    for relative in ("Frontend/src", "Frontend/tests", "Backend", "scripts", "docs"):
        for directory, children, files in os.walk(HOST / relative):
            children[:] = sorted(name for name in children if name not in {"bin", "obj", "node_modules", "__pycache__", ".git"})
            for name in files:
                path = Path(directory) / name
                if path.suffix.lower() in {".ts", ".tsx", ".css", ".cs", ".csproj", ".json", ".md", ".ps1", ".py", ".mjs", ".js", ".svg", ".html"}:
                    result.add(path)
    for directory in (HOST, HOST / "Frontend"):
        result.update(path for path in directory.iterdir() if path.is_file() and path.suffix.lower() in {".json", ".ts", ".html", ".ps1", ".md"})
    result.update(path for path in (HOST / "Frontend/public").rglob("*") if path.is_file())
    return sorted(result)


def main() -> None:
    """Check release gates and freeze source, product and evidence hashes without deployment."""
    baseline = json.loads((BASELINE / "output/session-quota-recovery/release-offline-20261029.json").read_text(encoding="utf-8-sig"))
    modified = json.loads((FIRMWARE / "output/adjustable-modified-files.json").read_text(encoding="utf-8-sig"))
    allowed_changes = {item["Path"] for item in modified}
    build = json.loads((FIRMWARE / "output/usability-torque/keil-final/build_manifest.json").read_text(encoding="utf-8-sig"))
    assert build["passed"] and build["exit_code"] == 0
    build_started = datetime.fromisoformat(build["started_at_utc"].replace("Z", "+00:00")).timestamp()
    firmware_files, changed = [], set()
    for original in baseline["files"]:
        relative = original["path"]
        assert digest(BASELINE / relative) == original["sha256"].lower(), f"Baseline changed: {relative}"
        current = entry(FIRMWARE / relative, FIRMWARE)
        assert (FIRMWARE / relative).stat().st_mtime <= build_started, f"Source newer than final build: {relative}"
        firmware_files.append(current)
        if current["sha256"] != original["sha256"].lower():
            changed.add(relative)
    assert len(firmware_files) == 934 and changed == allowed_changes, sorted(changed ^ allowed_changes)
    for item in modified:
        assert digest(FIRMWARE / item["Path"]) == item["Sha256"].lower(), item["Path"]

    browser_full = json.loads((HOST / "artifacts/usability-torque-browser-full-results-20261029.json").read_text(encoding="utf-8-sig"))
    browser_extra = json.loads((HOST / "artifacts/browser-results.json").read_text(encoding="utf-8-sig"))
    for result, expected in ((browser_full, 62), (browser_extra, 6)):
        assert result["stats"]["expected"] == expected and not result["errors"]
        assert all(result["stats"][name] == 0 for name in ("unexpected", "flaky", "skipped"))
    assert "# pass 93" in (HOST / "artifacts/usability-torque-frontend-tests-20261029.log").read_text(encoding="utf-8-sig")
    assert "GATEWAY_TESTS passed=39 failed=0" in (HOST / "artifacts/usability-torque-gateway-tests-final-20261029.log").read_text(encoding="utf-8-sig")
    config_log = (FIRMWARE / "output/adjustable-hello-link-configs.log").read_text(encoding="utf-8-sig")
    assert config_log.count("DEBUG_UI_CONFIG_PASS ") == 10 and "DEBUG_UI_INVALID_CONFIGS_REJECTED" in config_log

    products = []
    for kind, artifact in build["artifacts"].items():
        source = Path(artifact["source"])
        assert digest(source) == artifact["sha256"], kind
        products.append(freeze_copy(source, "firmware/" + source.name))
    products.append(freeze_copy(FIRMWARE / "output/usability-torque/keil-final/build_manifest.json", "firmware/build_manifest.json"))
    for path in sorted((HOST / "Frontend/dist").rglob("*")):
        if path.is_file():
            products.append(freeze_copy(path, "Frontend/dist/" + path.relative_to(HOST / "Frontend/dist").as_posix()))
    products.extend(entry(path, RELEASE) for path in sorted((RELEASE / "gateway").rglob("*")) if path.is_file())
    host_inputs = [entry(path, HOST) for path in source_files()]
    for item in host_inputs:
        freeze_copy(HOST / item["path"], "host-source/" + item["path"])

    evidence = []
    host_evidence = [
        "usability-torque-frontend-tests-20261029.log", "usability-torque-frontend-build-final-20261029.log",
        "usability-torque-gateway-build-final-20261029.log", "usability-torque-gateway-tests-final-20261029.log",
        "usability-torque-gateway-publish-20261029.log", "usability-torque-browser-final-20261029.log",
        "usability-torque-browser-races-20261029.log", "usability-torque-browser-full-results-20261029.json",
        "browser-results.json", "usability-focus-red-20261029.log", "manual-protection-page21-redesign-20261029.png",
    ]
    host_evidence += [f"usability-{page}-{width}.png" for page in ("manual", "static") for width in (320, 768, 1024, 1440, 1920)]
    for name in host_evidence:
        evidence.append(freeze_copy(HOST / "artifacts" / name, "evidence/host/" + name))
    firmware_evidence = [
        "adjustable-torque-red.log", "adjustable-torque-green.log", "adjustable-hello-link-red.log",
        "adjustable-hello-link-green.log", "adjustable-hello-link-configs.log", "adjustable-single-static.log",
        "adjustable-pos-load.log", "usability-torque/protocol-engine-final.log", "usability-torque/torque-release-final.log",
    ]
    for name in firmware_evidence:
        evidence.append(freeze_copy(FIRMWARE / "output" / name, "evidence/firmware/" + name))
    manifest = {
        "schema": "studio-usability-adjustable-torque-offline-v1", "created_at": datetime.now(timezone.utc).isoformat(),
        "target": "LCD-POS-Load-Test", "flashed": False, "hardware_qualified": False, "hardware_commands_sent": 0,
        "host_process_replaced": False, "firmware_source_root": str(FIRMWARE), "host_source_root": str(HOST),
        "baseline_verified_files": 934, "historical_host_snapshot_files": 85,
        "historical_host_diff_complete": False, "changed_firmware_inputs": sorted(changed),
        "policy": {"axis": "motor-output", "min_mnm": 3500, "max_mnm": 6240, "boot_authorized": False,
                   "max_continuous_excess_ms": 10000, "capacity_nm2_s": 175.616,
                   "temperature_exit_c": 55, "reset_temperature_max_c": 35,
                   "constant_6240_theoretical_seconds": 175.616 / (6.24**2 - 3.5**2)},
        "verification": {"frontend_unit": 93, "gateway_unit": 39, "browser_full": 62, "browser_supplemental": 6,
                         "browser_unique": 64, "firmware_app_configs": 10, "keil_errors": 0, "keil_warnings": 0},
        "firmware_files": firmware_files, "host_files": host_inputs, "products": products, "evidence": evidence,
    }
    destination = RELEASE / "release-manifest.json"
    destination.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    for item in products + evidence:
        assert digest(RELEASE / item["path"]) == item["sha256"], item["path"]
    print(json.dumps({"manifest": str(destination), "sha256": digest(destination), "firmware_inputs": len(firmware_files),
                      "changed_firmware_inputs": len(changed), "host_inputs": len(host_inputs), "products": len(products),
                      "evidence": len(evidence), "flashed": False}, ensure_ascii=False))


if __name__ == "__main__":
    main()
