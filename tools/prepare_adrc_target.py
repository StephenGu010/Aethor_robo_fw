"""Freeze verified Simulink output and create an isolated, initially unqualified Keil target.

This tool never opens hardware or deletes files. It requires an actually successful
offline pipeline report, retains the original run, and records copied-file hashes.
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
import os
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET


def sha256(path: Path) -> str:
    """Return a content fingerprint for one exact maintained or generated file."""
    return hashlib.sha256(path.read_bytes()).hexdigest()


def verify_recorded_files(entries: list, allowed_root: Path) -> dict[Path, str]:
    """Verify recorded bytes and containment; never invent retrospective verification hashes."""
    if not isinstance(entries, list) or not entries:
        raise ValueError('Missing verification-time artifact hashes; rerun the real pipeline.')
    verified = {}
    for entry in entries:
        path = Path(entry['path']).resolve(strict=True)
        if not path.is_relative_to(allowed_root.resolve()) or not path.is_file():
            raise ValueError(f'Artifact is outside the permitted root: {path}')
        digest = entry['sha256'].lower()
        if path in verified or len(digest) != 64 or sha256(path) != digest:
            raise ValueError(f'Artifact changed since verification or is duplicated: {path}')
        verified[path] = digest
    return verified


def checked_generated_directory(project: Path, run_directory: Path) -> tuple[Path, dict]:
    """Require complete controller MIL, closed-loop MIL and actual generated-C replay evidence."""
    run_directory = run_directory.resolve(strict=True)
    if not run_directory.is_relative_to((project / 'output/adrc/models').resolve()):
        raise ValueError('Model output must be a run beneath output/adrc/models.')
    report = json.loads((run_directory / 'pipeline_report.json').read_text(encoding='utf-8'))
    if report.get('passed') is not True or report.get('stage') != 'completed':
        raise ValueError('The MATLAB pipeline has not passed; target preparation is blocked.')
    for key in ('controllerMil', 'closedLoopMil', 'cReplay'):
        if report.get(key, {}).get('passed') is not True:
            raise ValueError(f'Missing successful verification: {key}')
    generated = Path(report['generatedBuild']['BuildDirectory']).resolve(strict=True)
    if not generated.is_relative_to(run_directory):
        raise ValueError('Generated code path does not belong to the verified run.')
    for name in ('adrc_controller.c', 'adrc_controller.h', 'rtwtypes.h'):
        if not (generated / name).is_file():
            raise ValueError(f'Missing generated artifact: {name}')
    verified_artifacts = verify_recorded_files(report.get('verificationArtifacts'), run_directory)
    verified_sources = verify_recorded_files(report.get('verificationSources'), project / 'Models/Adrc')
    expected_sources = {(project / 'Models/Adrc' / name).resolve() for name in (
        'run_adrc_pipeline.m', 'build_adrc_models.m', 'adrc_simulation_parameters.m', 'adrc_generated_replay.c')}
    if set(verified_sources) != expected_sources:
        raise ValueError('Verification source manifest does not match maintained model inputs.')
    models = report.get('models', {})
    if not isinstance(models, dict) or len(models) != 3:
        raise ValueError('Expected three verified SLX models.')
    model_paths = {Path(path).resolve(strict=True) for path in models.values()}
    if {path.name for path in model_paths} != {'adrc_controller.slx', 'adrc_plant.slx', 'adrc_validation.slx'}:
        raise ValueError('Unexpected model artifact list.')
    generated_paths = {path.resolve() for path in generated.iterdir()
        if path.is_file() and path.suffix in ('.c', '.h')}
    if not (model_paths | generated_paths).issubset(verified_artifacts):
        raise ValueError('A model or generated file has no verification-time hash.')
    return generated, report


def prepare_target(project: Path, run_directory: Path) -> Path:
    """Copy the validated code snapshot and derive one separate Keil project from LCD-ReadOnly."""
    generated, report = checked_generated_directory(project, run_directory)
    run_directory = run_directory.resolve(strict=True)
    report_hash = sha256(run_directory / 'pipeline_report.json')
    model_hashes = {name: sha256(Path(path)) for name, path in report['models'].items()}
    generated_files = sorted(path for path in generated.iterdir()
        if path.is_file() and path.suffix in ('.c', '.h') and path.name != 'ert_main.c')
    copied_hashes = [{'name': path.name, 'sha256': sha256(path)} for path in generated_files]
    handwritten = ['adrc_experiment.c', 'adrc_protocol.c', 'adrc_bench.c',
        'adrc_app_bridge.c', 'adrc_generated_adapter.c']
    paths = [f'../App/Adrc/{name}' for name in handwritten]
    paths.append('../App/Motor/motor_adrc_command.c')
    paths.append('../App/Platform/stm32_adrc_channel.c')
    for relative_path in paths:
        if not (project / 'MDK-ARM' / relative_path).resolve().is_file():
            raise ValueError(f'Missing integration source: {relative_path}')
    # Immutable per-report snapshots keep the previously published target usable
    # even if copying a new run fails. The target is the final atomic publication.
    snapshot_relative = f'App/Adrc/Generated/{report_hash}'
    snapshot_directory = project / snapshot_relative
    source_project = project / 'MDK-ARM/CtrBoard-H7_FDCAN.uvprojx'
    source_hash = sha256(source_project)
    tree = ET.parse(source_project)
    project_xml = tree.getroot()
    targets = project_xml.find('Targets')
    assert targets is not None, 'Missing Keil target list.'
    candidates = [node for node in targets if node.findtext('TargetName') == 'LCD-ReadOnly']
    if len(candidates) != 1:
        raise ValueError('Expected exactly one LCD-ReadOnly baseline target.')
    target = copy.deepcopy(candidates[0])
    targets.clear()
    targets.append(target)
    target.find('TargetName').text = 'ADRC-Bench'
    common = target.find('TargetOption/TargetCommonOption')
    common.find('OutputDirectory').text = 'ADRC-Bench\\'
    common.find('OutputName').text = 'ADRC-Bench'
    listing = common.find('ListingPath')
    if listing is not None:
        listing.text = 'ADRC-Bench\\'
    controls = target.find('TargetOption/TargetArmAds/Cads/VariousControls')
    defines = controls.find('Define')
    defines.text = defines.text.replace('AETHOR_S3519_SAME_MODEL_MASK=0x7F',
        'AETHOR_S3519_SAME_MODEL_MASK=0x40') + ',AETHOR_ADRC_BENCH=1'
    controls.find('IncludePath').text += f';../App/Adrc;../{snapshot_relative}'
    for target_infos in project_xml.findall('.//targetInfos'):
        target_infos.clear()
        ET.SubElement(target_infos, 'targetInfo', {'name': 'ADRC-Bench'})
    paths.extend(f'../{snapshot_relative}/{path.name}' for path in generated_files if path.suffix == '.c')
    group = ET.SubElement(target.find('Groups'), 'Group')
    ET.SubElement(group, 'GroupName').text = 'ADRC experiment and generated controller'
    file_list = ET.SubElement(group, 'Files')
    for relative_path in paths:
        file_node = ET.SubElement(file_list, 'File')
        ET.SubElement(file_node, 'FileName').text = Path(relative_path).name
        ET.SubElement(file_node, 'FileType').text = '1'
        ET.SubElement(file_node, 'FilePath').text = relative_path
    target_path = project / 'MDK-ARM/ADRC-Bench.uvprojx'
    ET.indent(tree, space='  ')
    target_bytes = ET.tostring(project_xml, encoding='utf-8', xml_declaration=True)
    manifest = {'hardware_qualified': False, 'hardware_tested': False,
        'model_run': str(run_directory.resolve()), 'generated_files': copied_hashes,
        'pipeline_report_sha256': report_hash, 'model_hashes': model_hashes,
        'project_sha256': hashlib.sha256(target_bytes).hexdigest(), 'baseline_project_sha256': source_hash}
    snapshot_directory.mkdir(parents=True, exist_ok=True)
    for source, record in zip(generated_files, copied_hashes):
        destination = snapshot_directory / source.name
        if destination.exists():
            if sha256(destination) != record['sha256']:
                raise ValueError(f'Existing immutable snapshot differs: {destination}')
        else:
            shutil.copyfile(source, destination)
        if sha256(destination) != record['sha256']:
            raise ValueError(f'Copy mismatch: {source.name}')
    # Recheck evidence after copying, before publishing a new build target.
    checked_generated_directory(project, run_directory)
    if sha256(run_directory / 'pipeline_report.json') != report_hash or sha256(source_project) != source_hash:
        raise ValueError('Report or baseline project changed during target preparation.')
    manifest_path = snapshot_directory / 'generation-manifest.json'
    manifest_path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    pending_target = target_path.with_suffix('.uvprojx.pending')
    pending_target.write_bytes(target_bytes)
    os.replace(pending_target, target_path)
    return target_path


def main() -> None:
    """Parse the explicit verified run path; fail closed before generating a target on bad evidence."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model-output', required=True, type=Path)
    arguments = parser.parse_args()
    project = Path(__file__).resolve().parents[1]
    print(prepare_target(project, arguments.model_output))


if __name__ == '__main__':
    main()
