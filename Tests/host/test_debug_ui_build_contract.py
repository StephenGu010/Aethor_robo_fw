"""Verify compile-time LCD motion gates with the real C preprocessor.

The checks need no board and never link or execute motor-control firmware.
"""
from pathlib import Path
import argparse
import shutil
import subprocess
import xml.etree.ElementTree as ET


def verify_keil_source_selection(root: Path) -> None:
    """Require explicit exclusions; uVision merges group inventories across targets."""
    project = ET.parse(root / "MDK-ARM/CtrBoard-H7_FDCAN.uvprojx").getroot()
    expected = {
        "CtrBoard-H7_FDCAN": (0, 0, 0), "LCD-ReadOnly": (1, 0, 0),
        "LCD-POS": (1, 1, 0), "LCD-MIT": (1, 1, 1),
    }
    targets = project.findall("./Targets/Target")
    assert len(targets) == len(expected)
    for target in targets:
        name = target.findtext("TargetName")
        flags = expected[name]
        controls = target.find("./TargetOption/TargetArmAds/Cads/VariousControls")
        definitions = set(controls.findtext("Define").split(","))
        for flag, value in zip(("AETHOR_DEBUG_UI_ENABLE", "AETHOR_DEBUG_UI_ALLOW_MOTION",
                                "AETHOR_DEBUG_UI_ALLOW_MIT"), flags):
            assert f"{flag}={value}" in definitions, (name, flag)
        assert ("AETHOR_DEBUG_UI_MOTOR7_POS_PROFILE=1" in definitions) == (name in ("LCD-POS", "LCD-MIT")), name
        assert ("AETHOR_DEBUG_UI_MOTOR7_MIT_PROFILE=1" in definitions) == (name == "LCD-MIT"), name
        assert ("AETHOR_S3519_SAME_MODEL_MASK=0x7F" in definitions) == name.startswith("LCD-"), name
        assert ("AETHOR_DEBUG_UI_S3519_PROFILE_MASK=0x7F" in definitions) == (name in ("LCD-POS", "LCD-MIT")), name
        assert "--no_multibyte_chars" in controls.findtext("MiscControls")
        groups = {group.findtext("GroupName"): group for group in target.findall("./Groups/Group")}
        for group_name in ("Ui", "LVGL 8.3.11"):
            assert groups[group_name].findtext("./GroupOption/CommonProperty/IncludeInBuild") == str(flags[0]), (name, group_name)
        paths = [node.findtext("FilePath").replace("\\", "/") for node in target.findall(".//File")]
        assert len(paths) == len(set(paths)), name
        assert all((root / "MDK-ARM" / path).is_file() for path in paths), name
        print(f"PASS Keil {name}: explicit UI/LVGL selection and source inventory")


def verify_home_animation_dirty_area(root: Path) -> None:
    """Require the idle home animation to invalidate only its fixed drawing area."""
    source = (root / "Ui/debug_ui_astra_layout.inc").read_text(encoding="utf-8")
    function_start = source.index("void debug_ui_view_animate")
    function_end = source.index("/** @brief Format one actual draft field", function_start)
    function_body = source[function_start:function_end]
    assert "static const lv_area_t home_animation_area = {12, 58, 268, 180};" in function_body
    assert "lv_obj_invalidate_area(page->root, &home_animation_area);" in function_body
    assert "lv_obj_invalidate(page->root);" not in function_body
    print("PASS home animation: fixed 257x123 dirty area")


def main() -> int:
    """Compile valid and forbidden feature/profile combinations independently."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--compiler", default=shutil.which("gcc") or
                        r"D:\application\WinLibs\mingw64\bin\gcc.exe")
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[2]
    verify_keil_source_selection(root)
    verify_home_animation_dirty_area(root)
    cases = [
        ("defaults locked", [], True),
        ("read only", ["AETHOR_DEBUG_UI_ENABLE=1"], True),
        ("POS capable", ["AETHOR_DEBUG_UI_ENABLE=1", "AETHOR_DEBUG_UI_ALLOW_MOTION=1"], True),
        ("MIT capable", ["AETHOR_DEBUG_UI_ENABLE=1", "AETHOR_DEBUG_UI_ALLOW_MOTION=1",
                         "AETHOR_DEBUG_UI_ALLOW_MIT=1"], True),
        ("production read only", ["AETHOR_ACTIVE_PROFILE=2", "AETHOR_DEBUG_UI_ENABLE=1"], True),
        ("motion without UI", ["AETHOR_DEBUG_UI_ALLOW_MOTION=1"], False),
        ("MIT without motion", ["AETHOR_DEBUG_UI_ENABLE=1", "AETHOR_DEBUG_UI_ALLOW_MIT=1"], False),
        ("production motion", ["AETHOR_ACTIVE_PROFILE=2", "AETHOR_DEBUG_UI_ENABLE=1",
                               "AETHOR_DEBUG_UI_ALLOW_MOTION=1"], False),
        ("nonboolean UI", ["AETHOR_DEBUG_UI_ENABLE=2"], False),
        ("negative MIT flag", ["AETHOR_DEBUG_UI_ALLOW_MIT=-1"], False),
    ]
    cases.extend([
        ("seven same-model readonly", ["AETHOR_DEBUG_UI_ENABLE=1", "AETHOR_S3519_SAME_MODEL_MASK=0x7F"], True),
        ("seven POS profiles", ["AETHOR_DEBUG_UI_ENABLE=1", "AETHOR_DEBUG_UI_ALLOW_MOTION=1",
            "AETHOR_DEBUG_UI_MOTOR7_POS_PROFILE=1", "AETHOR_S3519_SAME_MODEL_MASK=0x7F",
            "AETHOR_DEBUG_UI_S3519_PROFILE_MASK=0x7F"], True),
        ("seven profiles without matching compatibility", ["AETHOR_DEBUG_UI_ENABLE=1",
            "AETHOR_DEBUG_UI_ALLOW_MOTION=1", "AETHOR_DEBUG_UI_MOTOR7_POS_PROFILE=1",
            "AETHOR_DEBUG_UI_S3519_PROFILE_MASK=0x7F"], False),
        ("seven profiles readonly forbidden", ["AETHOR_DEBUG_UI_ENABLE=1",
            "AETHOR_S3519_SAME_MODEL_MASK=0x7F", "AETHOR_DEBUG_UI_S3519_PROFILE_MASK=0x7F"], False),
        ("invalid same-model mask", ["AETHOR_S3519_SAME_MODEL_MASK=0x80"], False),
        ("invalid profile mask", ["AETHOR_DEBUG_UI_S3519_PROFILE_MASK=0x80"], False),
        ("production preset forbidden", ["AETHOR_ACTIVE_PROFILE=2", "AETHOR_S3519_SAME_MODEL_MASK=0x7F"], False),
        ("bounded motor7 MIT", ["AETHOR_DEBUG_UI_ENABLE=1", "AETHOR_DEBUG_UI_ALLOW_MOTION=1",
            "AETHOR_DEBUG_UI_ALLOW_MIT=1", "AETHOR_DEBUG_UI_MOTOR7_POS_PROFILE=1",
            "AETHOR_DEBUG_UI_MOTOR7_MIT_PROFILE=1"], True),
        ("MIT trial without discovery profile", ["AETHOR_DEBUG_UI_MOTOR7_MIT_PROFILE=1",
            "AETHOR_DEBUG_UI_ENABLE=1", "AETHOR_DEBUG_UI_ALLOW_MOTION=1", "AETHOR_DEBUG_UI_ALLOW_MIT=1"], False),
        ("MIT trial nonboolean", ["AETHOR_DEBUG_UI_MOTOR7_MIT_PROFILE=2"], False),
        ("recorded motor7 POS", ["AETHOR_DEBUG_UI_ENABLE=1", "AETHOR_DEBUG_UI_ALLOW_MOTION=1",
                                "AETHOR_DEBUG_UI_MOTOR7_POS_PROFILE=1"], True),
        ("recorded profile readonly forbidden", ["AETHOR_DEBUG_UI_ENABLE=1",
                                                "AETHOR_DEBUG_UI_MOTOR7_POS_PROFILE=1"], False),
        ("recorded profile MIT forbidden", ["AETHOR_DEBUG_UI_ENABLE=1", "AETHOR_DEBUG_UI_ALLOW_MOTION=1",
            "AETHOR_DEBUG_UI_ALLOW_MIT=1", "AETHOR_DEBUG_UI_MOTOR7_POS_PROFILE=1"], False),
        ("recorded profile nonboolean", ["AETHOR_DEBUG_UI_MOTOR7_POS_PROFILE=2"], False),
    ])
    for name, definitions, expected_success in cases:
        result = subprocess.run(
            [args.compiler, "-std=c99", "-Wall", "-Wextra", "-Werror",
             "-IApp/Config", "-x", "c", "-fsyntax-only", "-", *["-D" + value for value in definitions]],
            cwd=root, input='#include "debug_ui_config.h"\n',
            text=True, capture_output=True, check=False)
        if (result.returncode == 0) != expected_success:
            print(f"FAIL {name}: exit={result.returncode}\n{result.stdout}{result.stderr}")
            return 1
        print(f"PASS {name}: {'allowed' if expected_success else 'rejected'}")
    print(f"DEBUG_UI_BUILD_GATES_PASSED ({len(cases)} combinations)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
