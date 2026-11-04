"""Verify compile-time LCD motion gates and task capacities with the real C preprocessor.

The checks need no board and never link or execute motor-control firmware.
"""
from pathlib import Path
import argparse
import copy
import re
import shutil
import subprocess
import xml.etree.ElementTree as ET


SINGLE_STATIC_TARGET = "LCD-SingleStatic-Test"
SINGLE_STATIC_FLAG = "AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD=1"


def verify_single_static_keil_target(root: Path) -> None:
    """Allow only target renaming and the single-axis opt-in in the derived image."""
    integrated_path = root / "MDK-ARM/LCD-MIT-ADRC.uvprojx"
    single_path = root / "MDK-ARM" / f"{SINGLE_STATIC_TARGET}.uvprojx"
    assert single_path.is_file(), f"Missing independent target: {single_path}"
    project_text = single_path.read_text(encoding="utf-8")
    assert "<!-- @file" in project_text, "Missing provenance comment"
    assert project_text.index("<Project ") < project_text.index("<!-- @file"), "Keil rejects a comment before the Project root"
    integrated = ET.parse(integrated_path).getroot()
    single = ET.parse(single_path).getroot()
    targets = single.findall("./Targets/Target")
    assert len(targets) == 1
    target = targets[0]
    for path, expected in (
        ("TargetName", SINGLE_STATIC_TARGET),
        ("./TargetOption/TargetCommonOption/OutputName", SINGLE_STATIC_TARGET),
        ("./TargetOption/TargetCommonOption/OutputDirectory", SINGLE_STATIC_TARGET + "\\"),
        ("./TargetOption/TargetCommonOption/ListingPath", SINGLE_STATIC_TARGET + "\\"),
    ):
        assert target.findtext(path) == expected, path
    definitions = target.find("./TargetOption/TargetArmAds/Cads/VariousControls/Define")
    assert definitions.text.split(",").count(SINGLE_STATIC_FLAG) == 1
    assert not any(value.startswith("AETHOR_DEBUG_UI_ALLOW_STATIC_LOAD=")
                   for value in definitions.text.split(",")), "Seven-axis gate must remain default-locked"
    paths = [node.findtext("FilePath").replace("\\", "/") for node in target.findall(".//File")]
    assert len(paths) == len(set(paths)), "Duplicate sources in single-axis target"
    assert all((root / "MDK-ARM" / path).is_file() for path in paths)
    normalized = copy.deepcopy(single)
    for node in normalized.iter():
        if node.text:
            node.text = node.text.replace(SINGLE_STATIC_TARGET, "LCD-MIT-ADRC")
        for attribute, value in node.attrib.items():
            node.attrib[attribute] = value.replace(SINGLE_STATIC_TARGET, "LCD-MIT-ADRC")
    normalized_definitions = normalized.find("./Targets/Target/TargetOption/TargetArmAds/Cads/VariousControls/Define")
    normalized_definitions.text = ",".join(
        value for value in normalized_definitions.text.split(",") if value != SINGLE_STATIC_FLAG)
    # XML parsers omit comments but retain their indentation; compare settings,
    # attributes and complete source inventory independently of that whitespace.
    for project in (normalized, integrated):
        for node in project.iter():
            if node.text is not None and not node.text.strip():
                node.text = None
            if node.tail is not None and not node.tail.strip():
                node.tail = None
    assert ET.tostring(normalized) == ET.tostring(integrated), "Derived project changed existing settings or source inventory"
    print(f"PASS Keil {SINGLE_STATIC_TARGET}: exact integrated clone plus single-axis gate")


def verify_static_load_compile_gates(root: Path, compiler: str) -> None:
    """Preprocess all seven targets and reject invalid or prerequisite-free static gates."""
    expected_targets = {
        "CtrBoard-H7_FDCAN.uvprojx": ("CtrBoard-H7_FDCAN", "LCD-ReadOnly", "LCD-POS", "LCD-MIT"),
        "ADRC-Bench.uvprojx": ("ADRC-Bench",),
        "LCD-MIT-ADRC.uvprojx": ("LCD-MIT-ADRC",),
        f"{SINGLE_STATIC_TARGET}.uvprojx": (SINGLE_STATIC_TARGET,),
    }
    for project_name, expected in expected_targets.items():
        targets = ET.parse(root / "MDK-ARM" / project_name).getroot().findall("./Targets/Target")
        assert tuple(target.findtext("TargetName") for target in targets) == expected
        for target in targets:
            name = target.findtext("TargetName")
            definitions = target.findtext("./TargetOption/TargetArmAds/Cads/VariousControls/Define").split(",")
            expected_single = int(name == SINGLE_STATIC_TARGET)
            source = ('#include "debug_ui_config.h"\n'
                      f'#if AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD != {expected_single}\n'
                      '#error "Unexpected single-axis gate"\n#endif\n'
                      '#if AETHOR_DEBUG_UI_ALLOW_STATIC_LOAD != 0\n'
                      '#error "Seven-axis gate must remain locked"\n#endif\n')
            result = subprocess.run(
                [compiler, "-std=c99", "-Wall", "-Wextra", "-Werror", "-IApp/Config",
                 "-x", "c", "-fsyntax-only", "-", *["-D" + value for value in definitions]],
                cwd=root, input=source, text=True, capture_output=True, check=False)
            assert result.returncode == 0, (name, result.stderr)
            print(f"PASS target static gates {name}: single={expected_single} seven=0")
    prerequisites = ("AETHOR_DEBUG_UI_ENABLE=1", "AETHOR_DEBUG_UI_ALLOW_MOTION=1",
                     "AETHOR_DEBUG_UI_ALLOW_MIT=1")
    checked = 0
    for gate in ("AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD", "AETHOR_DEBUG_UI_ALLOW_STATIC_LOAD"):
        for value in (-1, 2):
            result = subprocess.run(
                [compiler, "-IApp/Config", "-x", "c", "-fsyntax-only", "-",
                 *["-D" + item for item in prerequisites], f"-D{gate}={value}"],
                cwd=root, input='#include "debug_ui_config.h"\n', text=True, capture_output=True, check=False)
            assert result.returncode != 0, (gate, value)
            assert "flag must be zero or one" in result.stderr, result.stderr
            checked += 1
        for mask in range(8):
            definitions = [item for index, item in enumerate(prerequisites) if mask & (1 << index)]
            result = subprocess.run(
                [compiler, "-IApp/Config", "-x", "c", "-fsyntax-only", "-",
                 *["-D" + item for item in definitions], f"-D{gate}=1"],
                cwd=root, input='#include "debug_ui_config.h"\n', text=True, capture_output=True, check=False)
            assert (result.returncode == 0) == (mask == 7), (gate, mask, result.stderr)
            checked += 1
    print(f"PASS static compile gates: seven real target definitions and {checked} guard combinations")


def verify_task_stack_capacities(root: Path, compiler: str) -> None:
    """Compile real buffer/creation declarations for seven targets and independent static gates."""
    header = root / "Core/Inc/aethor_task_stacks.h"
    assert header.is_file(), "Missing gated task-stack configuration"
    freertos = (root / "Core/Src/freertos.c").read_text(encoding="utf-8")
    freertos = re.sub(r"/\*.*?\*/|//[^\r\n]*", "", freertos, flags=re.S)
    assert '#include "aethor_task_stacks.h"' in freertos
    buffers = ("armControlTaskBuffer", "canRxTaskBuffer", "protocolTaskBuffer",
               "usbTxTaskBuffer", "telemetryTaskBuffer", "diagnosticsTaskBuffer")
    tasks = ("ArmControlTask", "CanRxTask", "ProtocolTask", "UsbTxTask", "TelemetryTask", "DiagnosticsTask")
    declarations = []
    for buffer in buffers:
        matches = re.findall(r"(?m)^uint32_t\s+" + buffer + r"\s*\[[^;]+;", freertos)
        assert len(matches) == 1, buffer
        declarations.append(matches[0])
    creations = []
    for task in tasks:
        matches = re.findall(r"(?m)^\s*osThreadStaticDef\(\s*" + task + r",[^\r\n]+;", freertos)
        assert len(matches) == 1, task
        creations.append(matches[0])
    default_capacities = (1280, 384, 1280, 384, 1024, 384)
    static_capacities = (1280, 640, 2048, 384, 1024, 512)
    target_capacities = {
        "LCD-MIT-ADRC": (1280, 384, 2048, 384, 1024, 512),
        SINGLE_STATIC_TARGET: static_capacities,
    }
    configurations = [("default", [], default_capacities)]
    for project in ("CtrBoard-H7_FDCAN", "ADRC-Bench", "LCD-MIT-ADRC", SINGLE_STATIC_TARGET):
        for target in ET.parse(root / "MDK-ARM" / (project + ".uvprojx")).getroot().findall("./Targets/Target"):
            configurations.append((target.findtext("TargetName"),
                target.findtext("./TargetOption/TargetArmAds/Cads/VariousControls/Define").split(","),
                target_capacities.get(target.findtext("TargetName"), default_capacities)))
    static_prerequisites = ["AETHOR_DEBUG_UI_ENABLE=1", "AETHOR_DEBUG_UI_ALLOW_MOTION=1",
                            "AETHOR_DEBUG_UI_ALLOW_MIT=1"]
    for name, static_definitions in (
        ("single-static standalone", [SINGLE_STATIC_FLAG]),
        ("seven-axis static standalone", ["AETHOR_DEBUG_UI_ALLOW_STATIC_LOAD=1"]),
        ("combined static standalone", [SINGLE_STATIC_FLAG, "AETHOR_DEBUG_UI_ALLOW_STATIC_LOAD=1"]),
    ):
        configurations.append((name, static_prerequisites + static_definitions, static_capacities))
    failures = []
    for name, definitions, expected in configurations:
        source = ('#include <stdint.h>\n#include "aethor_task_stacks.h"\n' +
                  '\n'.join(declarations) + '\n' +
                  '#define osThreadStaticDef(name, entry, priority, instances, depth, buffer, control) '
                  'typedef char name##_creation_matches_buffer[((depth) == sizeof(buffer) / sizeof(buffer[0])) ? 1 : -1]\n' +
                  '\n'.join(creations) + '\n' +
                  '\n'.join(f'typedef char {buffer}_expected[(sizeof({buffer}) / sizeof(uint32_t) == {words}U) ? 1 : -1];'
                            for buffer, words in zip(buffers, expected)) + '\n')
        result = subprocess.run(
            [compiler, "-std=c99", "-Wall", "-Wextra", "-Werror", "-IApp/Config", "-ICore/Inc",
             "-x", "c", "-fsyntax-only", "-", *["-D" + value for value in definitions]],
            cwd=root, input=source, text=True, capture_output=True, check=False)
        if result.returncode != 0:
            failures.append((name, result.stderr))
            print(f"FAIL task stack compile contract {name}: {result.stderr}")
        else:
            print(f"PASS task stack compile contract {name}: can_rx={expected[1]} protocol={expected[2]} diagnostics={expected[5]} words")
    assert not failures, failures


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


def verify_integrated_keil_target(root: Path) -> None:
    """Require one additional firmware image with LCD-MIT and generated ADRC sources."""
    project = ET.parse(root / "MDK-ARM/LCD-MIT-ADRC.uvprojx").getroot()
    target = project.find("./Targets/Target")
    assert target is not None
    assert target.findtext("TargetName") == "LCD-MIT-ADRC"
    assert target.findtext("./TargetOption/TargetCommonOption/OutputName") == "LCD-MIT-ADRC"
    controls = target.find("./TargetOption/TargetArmAds/Cads/VariousControls")
    definitions = set(controls.findtext("Define").split(","))
    for required in (
        "AETHOR_DEBUG_UI_ENABLE=1", "AETHOR_DEBUG_UI_ALLOW_MOTION=1",
        "AETHOR_DEBUG_UI_ALLOW_MIT=1", "AETHOR_DEBUG_UI_MOTOR7_POS_PROFILE=1",
        "AETHOR_DEBUG_UI_MOTOR7_MIT_PROFILE=1", "AETHOR_S3519_SAME_MODEL_MASK=0x7F",
        "AETHOR_DEBUG_UI_S3519_PROFILE_MASK=0x7F", "AETHOR_ADRC_LCD_INTEGRATED=1",
    ):
        assert required in definitions, required
    assert "AETHOR_ADRC_BENCH=1" not in definitions
    paths = [node.findtext("FilePath").replace("\\", "/") for node in target.findall(".//File")]
    assert len(paths) == len(set(paths))
    for required in (
        "../App/Adrc/adrc_lcd_ownership.c", "../App/Adrc/adrc_experiment.c",
        "../App/Adrc/adrc_app_bridge.c", "../App/Adrc/adrc_generated_adapter.c",
        "../App/Platform/stm32_adrc_channel.c", "../App/Motor/motor_capability_collector.c",
        "../App/Adrc/Generated/916eb8008efc66d926c0ab2e6833afc6e61bf37106e47873072e59e45c9974f7/adrc_controller.c",
    ):
        assert required in paths, required
    assert all((root / "MDK-ARM" / path).is_file() for path in paths)
    print("PASS Keil LCD-MIT-ADRC: LCD motion gates and generated controller present")


def verify_adrc_integration_flags(root: Path, compiler: str) -> None:
    """Reject ambiguous and nonboolean ADRC target combinations at preprocessing time."""
    cases = (
        ("ADRC default", (), True),
        ("ADRC integrated", ("AETHOR_ADRC_LCD_INTEGRATED=1",), True),
        ("ADRC isolated", ("AETHOR_ADRC_BENCH=1",), True),
        ("ADRC both targets", ("AETHOR_ADRC_BENCH=1", "AETHOR_ADRC_LCD_INTEGRATED=1"), False),
        ("ADRC invalid integrated flag", ("AETHOR_ADRC_LCD_INTEGRATED=2",), False),
    )
    for name, definitions, expected_success in cases:
        result = subprocess.run(
            [compiler, "-std=c99", "-Wall", "-Wextra", "-Werror", "-IApp/Config",
             "-x", "c", "-fsyntax-only", "-", *["-D" + value for value in definitions]],
            cwd=root, input='#include "adrc_build_config.h"\n',
            text=True, capture_output=True, check=False)
        assert (result.returncode == 0) == expected_success, (name, result.stderr)
    print("PASS ADRC integration compile gates: five combinations")


def main() -> int:
    """Compile valid and forbidden feature/profile combinations independently."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--compiler", default=shutil.which("gcc") or
                        r"D:\application\WinLibs\mingw64\bin\gcc.exe")
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[2]
    verify_keil_source_selection(root)
    verify_home_animation_dirty_area(root)
    verify_integrated_keil_target(root)
    verify_single_static_keil_target(root)
    verify_static_load_compile_gates(root, args.compiler)
    verify_task_stack_capacities(root, args.compiler)
    verify_adrc_integration_flags(root, args.compiler)
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
