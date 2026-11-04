"""Verify explicit deployment gates and immutable release binding without importing any hardware driver."""
from __future__ import annotations

from contextlib import ExitStack, redirect_stdout
from io import StringIO
import json
from pathlib import Path
import sys
import unittest
from unittest.mock import patch
import uuid

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))
import static_recovery_release as RELEASE


class ReleaseFixture:
    """Retain one synthetic source/evidence tree; image-parser validation is separately mocked and explicitly outside these tests."""

    def __init__(self) -> None:
        """Create fresh bounded files without removing anything from the user's workspace."""
        self.root = ROOT / "output/repair_20261023/static_recovery_release_wrapper" / ("r_" + uuid.uuid4().hex[:10])
        self.root.mkdir(parents=True, exist_ok=False)
        self.sources = {"App/test.c", "App/Motor/motor_parameter_persistence.c", "MDK-ARM/LCD-POS-Load-Test.uvprojx"}
        self.write("App/test.c", b"/* synthetic source for validation only */\n")
        self.write("App/Motor/motor_parameter_persistence.c", b"/* synthetic persistence unit */\n")
        self.project_xml = ('<Project><Targets><Target><TargetName>LCD-POS-Load-Test</TargetName><TargetOption>'
                            '<TargetCommonOption/><CommonProperty><IncludeInBuild>1</IncludeInBuild></CommonProperty>'
                            '<TargetArmAds><Cads><VariousControls><Define>AETHOR_MOTOR_PERSISTENCE_TRANSPORT=1</Define>'
                            '</VariousControls></Cads></TargetArmAds></TargetOption><Groups><Group><Files><File>'
                            '<FileName>motor_parameter_persistence.c</FileName><FileType>1</FileType>'
                            '<FilePath>../App/Motor/motor_parameter_persistence.c</FilePath></File></Files></Group></Groups>'
                            '</Target></Targets></Project>')
        self.write("MDK-ARM/LCD-POS-Load-Test.uvprojx", self.project_xml.encode("utf-8"))
        self.write("spec.md", b"synthetic bound spec receipt\n")
        self.write("quality.md", b"synthetic bound quality receipt\n")
        self.write("verify.log", b"synthetic bound verification receipt\n")
        self.write("build.log", b"0 Error(s), 0 Warning(s).\n")
        self.write("old.hex", b"synthetic installed candidate")
        self.artifacts = {"hex": b"synthetic new hex", "axf": b"synthetic new axf",
                          "map": b"motor_parameter_persistence.o", "htm": b"synthetic link report"}
        build = dict(target="LCD-POS-Load-Test", passed=True, exit_code=0,
                     project_sha256=self.checksum("MDK-ARM/LCD-POS-Load-Test.uvprojx"), artifacts={})
        for name, payload in self.artifacts.items():
            self.write("img." + name, payload)
            build["artifacts"][name] = dict(copy=str(self.root / ("img." + name)), sha256=RELEASE.shared.digest(payload))
        self.write_json("build.json", build)
        self.manifest = dict(schema="static-recovery-release-v1", target="LCD-POS-Load-Test", software_only=True,
                             hardware_qualified=False, driver_persisted=False,
                             sources=[dict(path=path, sha256=self.checksum(path)) for path in sorted(self.sources)],
                             evidence=[dict(path=path, sha256=self.checksum(path)) for path in ("spec.md", "quality.md", "verify.log", "build.json", "build.log")],
                             checks=[dict(name=name, passed=True, spec="spec.md", quality="quality.md", verification="verify.log")
                                     for name in sorted(RELEASE.REQUIRED_CHECKS)],
                             build_manifest="build.json", build_log="build.log",
                             hex_sha256=RELEASE.shared.digest(self.artifacts["hex"]), axf_sha256=RELEASE.shared.digest(self.artifacts["axf"]))

    def write(self, relative: str, payload: bytes) -> None:
        """Write one explicit synthetic fixture path, preserving every previously created run directory."""
        path = self.root / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(payload)

    def write_json(self, relative: str, value: dict) -> None:
        """Serialize one deterministic synthetic manifest or build receipt."""
        self.write(relative, (json.dumps(value, sort_keys=True) + "\n").encode("utf-8"))

    def checksum(self, relative: str) -> str:
        """Hash the exact fixture bytes used by the real release input checker."""
        return RELEASE.shared.digest((self.root / relative).read_bytes())

    def replace_project(self, xml: str) -> None:
        """Rebind a deliberately changed synthetic project so semantic tests do not stop only on hash drift."""
        self.write("MDK-ARM/LCD-POS-Load-Test.uvprojx", xml.encode("utf-8"))
        for entry in self.manifest["sources"]:
            if entry["path"] == "MDK-ARM/LCD-POS-Load-Test.uvprojx": entry["sha256"] = self.checksum(entry["path"])
        build = json.loads((self.root / "build.json").read_text(encoding="utf-8"))
        build["project_sha256"] = self.checksum("MDK-ARM/LCD-POS-Load-Test.uvprojx")
        self.write_json("build.json", build)
        for entry in self.manifest["evidence"]:
            if entry["path"] == "build.json": entry["sha256"] = self.checksum("build.json")

    def prepare(self, *, checksum: str | None = None):
        """Run the actual manifest/source/evidence checks with only inventory and audited binary parsers isolated."""
        self.write_json("release.json", self.manifest)
        with ExitStack() as stack:
            stack.enter_context(patch.object(RELEASE, "ROOT", self.root))
            stack.enter_context(patch.object(RELEASE, "INSTALLED_HEX", self.root / "old.hex"))
            stack.enter_context(patch.object(RELEASE, "INSTALLED_SHA256", RELEASE.shared.digest(b"synthetic installed candidate")))
            stack.enter_context(patch.object(RELEASE, "EXPLICIT_SOURCES", frozenset()))
            stack.enter_context(patch.object(RELEASE.shared, "maintained_sources", return_value=self.sources))
            pair = stack.enter_context(patch.object(RELEASE.shared, "verify_image_pair", return_value=((0x08000000, b"new"),)))
            stack.enter_context(patch.object(RELEASE.shared, "parse_hex", return_value=((0x08000000, b"old"),)))
            stack.enter_context(patch.object(RELEASE.shared, "load_hardware", side_effect=AssertionError("hardware reached")))
            result = RELEASE.prepare_release(self.root / "release.json", checksum or self.checksum("release.json"))
            pair.assert_called_once_with(self.artifacts["hex"], self.artifacts["axf"])
            return result


class ReleaseGateTests(unittest.TestCase):
    """Ensure default validation remains offline and missing current conditions never reach hardware loading."""

    def test_default_is_offline(self) -> None:
        """The ordinary invocation only validates the pinned manifest and returns its frozen HEX identity."""
        prepared = type("PreparedFixture", (), {"image_payload": b"frozen"})()
        with patch.object(RELEASE, "prepare_release", return_value=(prepared, {})) as prepare, \
                patch.object(RELEASE.shared, "load_hardware", side_effect=AssertionError("hardware reached")), \
                redirect_stdout(StringIO()):
            self.assertEqual(RELEASE.main(["--manifest", "release.json", "--manifest-sha256", "1" * 64]), 0)
            prepare.assert_called_once()

    def test_deploy_requires_all_fresh_conditions(self) -> None:
        """No partial collection of current physical flags permits a deployment transaction."""
        flags = ("--supported-unloaded", "--lcd-released", "--motor-power-on")
        for missing in flags:
            with self.subTest(missing=missing), patch.object(RELEASE, "prepare_release") as prepare, \
                    patch.object(RELEASE.shared, "load_hardware", side_effect=AssertionError("hardware reached")):
                with self.assertRaises(SystemExit):
                    RELEASE.main(["--manifest", "release.json", "--manifest-sha256", "1" * 64,
                                  "--deploy", *(flag for flag in flags if flag != missing)])
                prepare.assert_not_called()

    def test_bad_manifest_hash_stops_before_inventory(self) -> None:
        """Malformed SHA input is refused before file inventory or build validation can run."""
        with patch.object(RELEASE.shared, "maintained_sources", side_effect=AssertionError("inventory reached")):
            with self.assertRaises(RuntimeError):
                RELEASE.prepare_release(ROOT / "does_not_exist.json", "not-a-sha")

    def test_exact_source_and_receipt_binding(self) -> None:
        """A complete synthetic aggregate freezes exact immutable bytes and invokes the separate HEX/AXF validator."""
        fixture = ReleaseFixture()
        prepared, manifest = fixture.prepare()
        self.assertEqual(prepared.image_payload, fixture.artifacts["hex"])
        self.assertEqual(manifest["target"], "LCD-POS-Load-Test")
        self.assertTrue(any(entry.path == fixture.root / "App/test.c" for entry in prepared.files))

    def test_manifest_hash_and_source_drift(self) -> None:
        """Neither a different pinned aggregate nor changed maintained source bytes can pass."""
        fixture = ReleaseFixture()
        with self.assertRaises(RuntimeError):
            fixture.prepare(checksum="1" * 64)
        fixture.write("App/test.c", b"changed source")
        with self.assertRaises(RuntimeError):
            fixture.prepare()

    def test_inventory_and_review_gate_completeness(self) -> None:
        """Missing source inventory, missing checks and unbound or failed review gates are rejected."""
        for variant in ("source", "check", "failed", "unbound"):
            with self.subTest(variant=variant):
                fixture = ReleaseFixture()
                if variant == "source": fixture.manifest["sources"].pop()
                if variant == "check": fixture.manifest["checks"].pop()
                if variant == "failed": fixture.manifest["checks"][0]["passed"] = False
                if variant == "unbound": fixture.manifest["checks"][0]["quality"] = "not-bound.md"
                with self.assertRaises(RuntimeError): fixture.prepare()

    def test_evidence_build_and_binary_drift(self) -> None:
        """A changed review, warning, unlinked persistence unit, changed artifact or wrong installed candidate fails offline."""
        for relative, changed in (("spec.md", b"changed receipt"), ("build.log", b"0 Error(s), 1 Warning(s)."),
                                  ("img.hex", b"changed hex"), ("img.map", b"unlinked"), ("old.hex", b"wrong installed candidate")):
            with self.subTest(relative=relative):
                fixture = ReleaseFixture()
                fixture.write(relative, changed)
                with self.assertRaises(RuntimeError): fixture.prepare()

    def test_nonsoftware_claim_and_escaped_reference(self) -> None:
        """A software aggregate cannot claim physical qualification or use a path outside its checkout."""
        for variant in ("qualified", "persisted", "escape"):
            with self.subTest(variant=variant):
                fixture = ReleaseFixture()
                if variant == "qualified": fixture.manifest["hardware_qualified"] = True
                if variant == "persisted": fixture.manifest["driver_persisted"] = True
                if variant == "escape": fixture.manifest["evidence"][0]["path"] = "../escaped.md"
                with self.assertRaises(RuntimeError): fixture.prepare()

    def test_unbound_build_receipts_are_rejected(self) -> None:
        """The passed build and zero-warning log must be covered by the pinned aggregate, not merely exist today."""
        fixture = ReleaseFixture()
        fixture.manifest["evidence"] = [entry for entry in fixture.manifest["evidence"] if entry["path"] not in ("build.json", "build.log")]
        with self.assertRaises(RuntimeError): fixture.prepare()

    def test_actual_project_target_gate_and_included_unit(self) -> None:
        """Comments, disabled targets, duplicate targets, per-file overrides and excluded source units cannot pass."""
        for variant in ("comment_gate", "comment_unit", "wrong_target", "duplicate_target", "excluded_unit", "override", "wrong_path"):
            with self.subTest(variant=variant):
                fixture = ReleaseFixture()
                xml = fixture.project_xml
                if variant == "comment_gate":
                    xml = xml.replace("TRANSPORT=1", "TRANSPORT=0").replace("<Project>", "<Project><!-- AETHOR_MOTOR_PERSISTENCE_TRANSPORT=1 -->")
                if variant == "comment_unit":
                    xml = xml[:xml.index("<Groups>")] + '<!-- motor_parameter_persistence.c --></Target></Targets></Project>'
                if variant == "wrong_target": xml = xml.replace("<TargetName>LCD-POS-Load-Test</TargetName>", "<TargetName>OTHER</TargetName>")
                if variant == "duplicate_target": xml = xml.replace("</Targets>", xml[xml.index("<Target>"):xml.index("</Target>") + len("</Target>")] + "</Targets>")
                if variant == "excluded_unit": xml = xml.replace("</File>", "<FileOption><CommonProperty><IncludeInBuild>0</IncludeInBuild></CommonProperty></FileOption></File>")
                if variant == "override": xml = xml.replace("</File>", "<FileOption><FileArmAds><Cads><VariousControls><Define>AETHOR_MOTOR_PERSISTENCE_TRANSPORT=0</Define></VariousControls></Cads></FileArmAds></FileOption></File>")
                if variant == "wrong_path": xml = xml.replace("../App/Motor/motor_parameter_persistence.c", "../App/test.c")
                fixture.replace_project(xml)
                with self.assertRaises(RuntimeError): fixture.prepare()

    def test_maintained_real_target_structure(self) -> None:
        """The enabled maintained target's actual Keil XML layout is accepted without a synthetic schema assumption."""
        RELEASE.validate_target_project((ROOT / "MDK-ARM/LCD-POS-Load-Test.uvprojx").read_bytes())


if __name__ == "__main__":
    unittest.main()
