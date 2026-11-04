"""Validate a pinned static recovery release offline, or deploy its immutable image with fresh explicit conditions.

The manifest binds the reviewed aggregate, complete maintained source inventory,
individual review/test evidence and a clean Keil HEX/AXF pair. It does not establish
driver EEPROM persistence, physical motor direction or load qualification.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
from pathlib import Path
import re
import xml.etree.ElementTree as ET

import deploy_401b_repair as context

shared = context.shared
ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / "output/repair_20261023"
INSTALLED_HEX = ROOT / "output/repair_20261026/persistence_same_tick_build_host/LCD-POS-Load-Test.hex"
INSTALLED_SHA256 = "339e9932e37a0490fca32e5cd8b6bf18bff4eae4d8ee56312f60d55398a7f6c6"
REQUIRED_CHECKS = frozenset(("health_recovery", "takeover_strict", "peak_ui", "persistence_pure",
    "persistence_slot", "persistence_core", "persistence_usb", "persistence_app_a", "persistence_app_b1",
    "persistence_b2_roundtrip", "persistence_host", "persistence_joint", "release_wrapper"))
EXPLICIT_SOURCES = frozenset(("App/Motor/motor_parameter_persistence.c", "App/Motor/motor_parameter_persistence.h",
    "App/Motor/motor_persistence_app.inc", "App/Motor/motor_persistence_wire.inc",
    "App/Motor/motor_persistence_app_port.h", "tools/static_recovery_release.py", "tools/pmax_persistence_live.py"))


def validate_target_project(payload: bytes) -> None:
    """Validate the actual selected Keil target, effective gate and included pure unit, ignoring XML comments."""
    try:
        project = ET.fromstring(payload)
    except ET.ParseError as error:
        raise RuntimeError("Invalid Keil project XML") from error
    targets = [target for target in project.findall("./Targets/Target")
               if target.findtext("TargetName") == "LCD-POS-Load-Test"]
    shared.require(len(targets) == 1, "Missing or duplicate actual target")
    target = targets[0]
    shared.require(target.findtext("./TargetOption/CommonProperty/IncludeInBuild") == "1", "Target is excluded from build")
    define = target.find("./TargetOption/TargetArmAds/Cads/VariousControls/Define")
    macro = "AETHOR_MOTOR_PERSISTENCE_TRANSPORT"
    shared.require(define is not None and [token.strip() for token in (define.text or "").split(",") if token.strip().split("=", 1)[0] == macro] == [macro + "=1"],
                   "Actual target persistence gate is not uniquely enabled")
    macro_pattern = re.compile(r"(?:^|[,\s])" + macro + r"(?:$|[,\s=])")
    for node in target.iter():
        if node.tag in ("Define", "Undefine") and node is not define:
            shared.require(macro_pattern.search(node.text or "") is None, "Conflicting group or file persistence gate")
    expected = (ROOT / "App/Motor/motor_parameter_persistence.c").resolve()
    selected = []
    for group in target.findall("./Groups/Group"):
        group_included = group.findtext("./GroupOption/CommonProperty/IncludeInBuild")
        for source in group.findall("./Files/File"):
            name = (source.findtext("FileName") or "").strip()
            if name.casefold() != "motor_parameter_persistence.c":
                continue
            path = (source.findtext("FilePath") or "").replace("\\", "/")
            actual = ((ROOT / "MDK-ARM") / path).resolve()
            shared.require(actual == expected and source.findtext("FileType") == "1" and
                           group_included in (None, "1", "2") and
                           source.findtext("./FileOption/CommonProperty/IncludeInBuild") in (None, "1", "2"),
                           "Persistence source path/type or inclusion is invalid")
            selected.append(source)
    shared.require(len(selected) == 1, "Missing or duplicate included persistence source unit")


def prepare_release(manifest_path: Path, manifest_sha256: str):
    """Reject drift and freeze reviewed bytes; importing this module never opens a serial port or probe."""
    shared.require(type(manifest_sha256) is str and re.fullmatch(r"[0-9a-f]{64}", manifest_sha256) is not None,
                   "Expected a lowercase pinned release SHA-256")
    frozen = {}

    def read(path: Path) -> bytes:
        """Bound each input to the checkout and retain exactly one immutable copy for deployment rechecks."""
        checked = shared.bounded_path(path, ROOT)
        payload = checked.read_bytes()
        prior = frozen.get(checked)
        shared.require(prior is None or prior.payload == payload, "Input changed while preparing: " + str(checked))
        frozen[checked] = shared.FrozenFile(checked, payload)
        return payload

    manifest_path = shared.bounded_path(manifest_path, ROOT)
    payload = read(manifest_path)
    shared.require(shared.digest(payload) == manifest_sha256, "Pinned release manifest changed")
    manifest = shared.read_json(payload)
    shared.require(manifest.get("schema") == "static-recovery-release-v1" and
                   manifest.get("target") == "LCD-POS-Load-Test" and manifest.get("software_only") is True and
                   manifest.get("hardware_qualified") is False and manifest.get("driver_persisted") is False,
                   "Wrong release identity or evidence tier")
    inventory = shared.maintained_sources(ROOT) | EXPLICIT_SOURCES
    entries = manifest.get("sources")
    shared.require(isinstance(entries, list) and len(entries) == len(inventory) and
                   {entry.get("path") for entry in entries if isinstance(entry, dict)} == inventory,
                   "Complete maintained source inventory changed")
    for entry in entries:
        shared.require(type(entry.get("path")) is str and not Path(entry["path"]).is_absolute() and
                       shared.digest(read(ROOT / entry["path"])) == entry.get("sha256"),
                       "Reviewed source changed: " + str(entry.get("path")))
    evidence = manifest.get("evidence")
    shared.require(isinstance(evidence, list) and evidence, "Missing frozen review and test evidence")
    evidence_paths = set()
    for entry in evidence:
        shared.require(isinstance(entry, dict) and type(entry.get("path")) is str and
                       not Path(entry["path"]).is_absolute() and entry["path"] not in evidence_paths,
                       "Duplicate or invalid evidence reference")
        shared.require(shared.digest(read(ROOT / entry["path"])) == entry.get("sha256"), "Review/test evidence changed")
        evidence_paths.add(entry["path"])
    checks = manifest.get("checks")
    shared.require(isinstance(checks, list) and len(checks) == len(REQUIRED_CHECKS) and
                   {entry.get("name") for entry in checks if isinstance(entry, dict)} == REQUIRED_CHECKS,
                   "Missing or duplicate release gate")
    for check in checks:
        shared.require(check.get("passed") is True and check.get("spec") in evidence_paths and
                       check.get("quality") in evidence_paths and check.get("verification") in evidence_paths,
                       "Review or verification gate incomplete: " + str(check.get("name")))
    shared.require(manifest.get("build_manifest") in evidence_paths and manifest.get("build_log") in evidence_paths,
                   "Build receipt and log are not bound to the pinned release")
    build = shared.read_json(read(ROOT / manifest["build_manifest"]))
    shared.require(build.get("target") == "LCD-POS-Load-Test" and build.get("passed") is True and
                   type(build.get("exit_code")) is int and build["exit_code"] == 0,
                   "Keil build did not pass")
    project = read(ROOT / "MDK-ARM/LCD-POS-Load-Test.uvprojx")
    shared.require(shared.digest(project) == build.get("project_sha256"), "Persistence project is not the built project")
    validate_target_project(project)
    log = read(ROOT / manifest["build_log"])
    shared.require(re.findall(rb"(\d+)\s+Error\(s\),\s*(\d+)\s+Warning\(s\)", log) == [(b"0", b"0")],
                   "Build log is not one clean zero-warning result")
    artifacts = {}
    for name in ("hex", "axf", "map", "htm"):
        entry = build["artifacts"][name]
        artifacts[name] = read(Path(entry["copy"]))
        shared.require(shared.digest(artifacts[name]) == entry["sha256"], "Linked product changed: " + name)
    shared.require(shared.digest(artifacts["hex"]) == manifest.get("hex_sha256") and
                   shared.digest(artifacts["axf"]) == manifest.get("axf_sha256") and
                   b"motor_parameter_persistence.o" in artifacts["map"], "Release image or linked persistence unit mismatch")
    previous = read(INSTALLED_HEX)
    shared.require(shared.digest(previous) == INSTALLED_SHA256, "Wrong installed image candidate")
    segments = shared.verify_image_pair(artifacts["hex"], artifacts["axf"])
    prepared = shared.Prepared(ROOT, manifest_path, payload, previous, artifacts["hex"], artifacts["axf"],
                               shared.parse_hex(previous), segments, tuple(frozen.values()))
    return prepared, manifest


def main(argv=None) -> int:
    """Validate offline by default; fresh current support, unloaded, LCD-released and powered flags gate deployment."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--manifest-sha256", required=True)
    parser.add_argument("--deploy", action="store_true")
    parser.add_argument("--supported-unloaded", action="store_true")
    parser.add_argument("--lcd-released", action="store_true")
    parser.add_argument("--motor-power-on", action="store_true")
    parser.add_argument("--expected-mode", choices=("1", "2"), default="2")
    args = parser.parse_args(argv)
    if args.deploy and not (args.supported_unloaded and args.lcd_released and args.motor_power_on):
        parser.error("--deploy requires all three current physical condition flags")
    prepared, _ = prepare_release(args.manifest, args.manifest_sha256)
    print("STATIC_RECOVERY_OFFLINE_FROZEN", shared.digest(prepared.image_payload), flush=True)
    if args.deploy:
        shared.TARGET_NAME = "LCD-POS-Load-Test"
        output = BASE / ("static_recovery_deploy_" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ"))
        report = shared.deploy(prepared, shared.load_hardware(), output, True, True, expected_mode=args.expected_mode)
        print("STATIC_RECOVERY_FLASH_READBACK_EQUAL", report, flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
