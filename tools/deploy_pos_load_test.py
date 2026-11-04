"""Freeze all-POS LCD acceptance and reuse the reviewed single-static flash transaction.

The default command verifies files only. Hardware access requires --deploy, both
physical confirmations and an explicit current mode. Trial mode is always POS 2;
preflight retains the installed mode and never writes a mode or moves a motor.
"""
import argparse
from datetime import datetime, timezone
import importlib.util
import os
from pathlib import Path
import re
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
TARGET_NAME = 'LCD-POS-Load-Test'
FEATURE_GATE = 'AETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST=1'
POS_RENDER_PREFIXES = tuple(str(index)+'_pos_load_' for index in range(210,220))


def load_shared_transaction():
    """Import an isolated legacy module; change only output/report target identity."""
    name = '_pos_load_shared_deployment_' + __name__
    specification = importlib.util.spec_from_file_location(name, ROOT/'tools/deploy_single_static_test.py')
    module = importlib.util.module_from_spec(specification)
    sys.modules[name] = module
    specification.loader.exec_module(module)
    module.TARGET_NAME = TARGET_NAME
    return module


shared = load_shared_transaction()
REQUIRED_TARGETS = ('LCD-POS', 'LCD-MIT', 'LCD-SingleStatic-Test', TARGET_NAME)
LCD_CHECKS = shared.CURRENT_LCD_CHECKS
REQUIRED_RECORDS = ('lcd_suite', 'pos_load_app_tests', 'pos_load_model_tests', 'pos_load_lcd_workflow', 'pos_load_platform_tests',
    'adrc_shaft', 'adrc_lcd_integration', 'single_static_platform', 'load_trials', 'load_trial_pages', 'startup_sync_tests',
    'startup_trace_export_tests', 'trace_map_tests', 'single_static_build_tests',
    'single_static_deploy_tests', 'pos_load_build_tests', 'pos_load_deploy_tests',
    *('keil_'+target for target in REQUIRED_TARGETS), 'map_pos_load')
REQUIRED_SOURCES = shared.REQUIRED_SOURCES | frozenset((
    'App/DebugUi/debug_ui_pos_load_app.inc', 'MDK-ARM/LCD-POS-Load-Test.uvprojx',
    'tools/build_pos_load_test.ps1', 'tools/verify_pos_load_test.ps1', 'tools/deploy_pos_load_test.py',
    'Tests/host/test_pos_load_build.py', 'Tests/hardware/test_pos_load_deploy.py',
    'Tests/host/run_pos_load_app_tests.ps1', 'Tests/host/run_pos_load_lcd_workflow_tests.ps1',
    'Tests/host/run_pos_load_platform_tests.ps1',
    'Tests/ui/run_pos_load_model_tests.ps1'))


def maintained_sources(workspace):
    """Enumerate git maintained inputs, including unignored new .inc implementations."""
    names = shared.maintained_sources(workspace)
    shared.require(REQUIRED_SOURCES <= names, 'Required POS maintained inputs are missing')
    return names


def validate_project(workspace):
    """Allow only independent target/output names and the added gate versus legacy XML."""
    baseline = shared.bounded_path('MDK-ARM/LCD-SingleStatic-Test.uvprojx', workspace).read_text(encoding='utf-8-sig')
    candidate = shared.bounded_path('MDK-ARM/'+TARGET_NAME+'.uvprojx', workspace).read_text(encoding='utf-8-sig')
    try:
        tree = ET.fromstring(candidate)
        shared.require(tree.findtext('./Targets/Target/TargetName') == TARGET_NAME, 'Wrong POS project identity')
        define = tree.find('./Targets/Target/TargetOption/TargetArmAds/Cads/VariousControls/Define')
        shared.require(define is not None and define.text.split(',').count(FEATURE_GATE) == 1, 'Missing/duplicate POS compile gate')
        restored = candidate.replace(TARGET_NAME,'LCD-SingleStatic-Test').replace(','+FEATURE_GATE,'')
        shared.require(ET.tostring(ET.fromstring(restored)) == ET.tostring(ET.fromstring(baseline)),
                       'Independent POS project changed preserved settings or inventory')
    except ET.ParseError as error:
        raise RuntimeError('Invalid maintained Keil project XML') from error


def validate_renders(run, evidence):
    """Require retained actual-LVGL setup/edit/review/run/results and default-locked pages."""
    render = shared.read_json(shared.evidence_file(str(run/'render/render_manifest.json'),run,evidence).payload)
    shared.require(render.get('renderer')=='actual LVGL 8.3.11 software raster + production double-buffer flush port',
                   'Missing actual production LVGL render identity')
    entries=render.get('images')
    shared.require(isinstance(entries,list) and entries, 'Missing LVGL images')
    names=[]
    for entry in entries:
        shared.require(isinstance(entry,dict) and isinstance(entry.get('image'),str), 'Invalid LVGL entry')
        name=entry['image']
        shared.require(Path(name).name==name and name.endswith('.png') and name not in names,
                       'Invalid/duplicate LVGL image identity')
        image=shared.evidence_file(str(run/'render'/name),run,evidence)
        shared.require(image.payload.startswith(b'\x89PNG\r\n\x1a\n') and
            type(entry.get('width')) is int and entry['width']==280 and
            type(entry.get('height')) is int and entry['height']==240 and
            re.fullmatch(r'[0-9a-f]{64}',str(entry.get('rgb_sha256',''))), 'Invalid LVGL image receipt')
        names.append(name)
    shared.require(all(sum(name.startswith(prefix) for name in names)==1 for prefix in POS_RENDER_PREFIXES),
                   'Missing POS LVGL acceptance page')


def prepare(manifest_path, manifest_sha256, workspace=ROOT):
    """Validate the fixed POS acceptance matrix, freezing complete source/evidence bytes."""
    workspace = shared.bounded_path(workspace, workspace, require_file=False)
    manifest_path = shared.bounded_path(manifest_path, workspace)
    manifest_payload = manifest_path.read_bytes()
    shared.require(isinstance(manifest_sha256,str) and re.fullmatch(r'[0-9a-fA-F]{64}',manifest_sha256)
        and shared.digest(manifest_payload)==manifest_sha256.lower(), 'Manifest anchor mismatch')
    manifest = shared.read_json(manifest_payload)
    shared.require(manifest.get('schema')=='pos-load-acceptance-v1' and manifest.get('target')==TARGET_NAME
        and manifest.get('feature_gate')==FEATURE_GATE and type(manifest.get('trial_mode')) is int
        and manifest['trial_mode']==2, 'Wrong POS schema/target/gate/mode')
    for field in ('passed','software_only','sources_unchanged'):
        shared.require(manifest.get(field) is True, 'Acceptance gate failed: '+field)
    shared.require(manifest.get('hardware_tested') is False and 'failure' in manifest and manifest['failure'] is None
        and type(manifest.get('lcd_checks')) is int and manifest['lcd_checks']==len(LCD_CHECKS),
        'Wrong software-only/count/failure flags')
    shared.require(manifest.get('previous_image_sha256')==shared.OLD_SHA256, 'Wrong previous image pin')
    shared.require(isinstance(manifest.get('run_id'),str) and re.fullmatch(r'\d{8}T\d{9,12}Z',manifest['run_id']),
        'Missing acceptance run identity')
    shared.require(shared.utc_time(manifest.get('started_at_utc'))<=shared.utc_time(manifest.get('completed_at_utc')),
        'Acceptance timestamps reversed')
    frozen=[shared.FrozenFile(manifest_path,manifest_payload)]
    sources=shared.collect_hashes(manifest.get('source_hashes'),workspace,True,frozen)
    names={item.path.relative_to(workspace).as_posix() for item in sources.values()}
    shared.require(names==maintained_sources(workspace), 'Acceptance omits or adds maintained input paths')
    validate_project(workspace)
    run=manifest_path.parent
    evidence=shared.collect_hashes(manifest.get('evidence_hashes'),run,False,frozen)
    shared.require(str(manifest_path).casefold() not in evidence,'Manifest must not hash itself')
    actual=set()
    for directory,directories,files in os.walk(run,followlinks=False):
        for name in directories:
            shared.bounded_path(Path(directory)/name,run,require_file=False)
        for name in files:
            path=shared.bounded_path(Path(directory)/name,run)
            if path!=manifest_path: actual.add(str(path).casefold())
    shared.require(set(evidence)==actual,'Unbound/missing evidence file')
    script=shared.bounded_path(manifest.get('verification_script_path',''),workspace)
    shared.require(Path(manifest.get('verification_script_path','')).is_absolute()
        and script==workspace/'tools/verify_pos_load_test.ps1'
        and shared.digest(script.read_bytes())==str(manifest.get('verification_script_sha256','')).lower(),
        'Wrong/changed verification script')
    for record in shared.unique_records(manifest.get('records'),REQUIRED_RECORDS,'name'):
        shared.require(type(record.get('exit_code')) is int and record['exit_code']==0,'Acceptance check failed')
        for field,suffix in (('log','.log'),('stderr','.stderr.log')):
            artifact=shared.evidence_file(record.get(field,''),run,evidence)
            shared.require(artifact.path==run/(record['name']+suffix), 'Wrong acceptance log identity')
    for build in shared.unique_records(manifest.get('builds'),REQUIRED_TARGETS,'target'):
        shared.require(type(build.get('errors')) is int and build['errors']==0 and
            type(build.get('warnings')) is int and build['warnings']==0,'Unclean build count')
        for field,extension in (('hex_path','.hex'),('elf_path','.axf'),('map_path','.map'),
                ('htm_path','.htm'),('log_path','.keil.log')):
            artifact=shared.evidence_file(build.get(field,''),run,evidence)
            shared.require(artifact.path==run/(build['target']+extension) and artifact.payload,
                'Wrong/empty build artifact identity')
        log=shared.evidence_file(build['log_path'],run,evidence).payload.decode('utf-8-sig')
        summaries=re.findall(r'(?<!\d)(\d+)\s+Error\(s\),\s*(\d+)\s+Warning\(s\)(?!\d)',log)
        shared.require(len(summaries)==1 and summaries[0]==('0','0'),'Unclean Keil log summary')
        shared.verify_image_pair(shared.evidence_file(build['hex_path'],run,evidence).payload,
                                 shared.evidence_file(build['elf_path'],run,evidence).payload)
    shared.validate_lcd(shared.evidence_file(str(run/'lcd_evidence/summary.json'),run,evidence),
                        run,evidence,LCD_CHECKS,False)
    validate_renders(run,evidence)
    image=shared.evidence_file(str(run/(TARGET_NAME+'.hex')),run,evidence).payload
    elf=shared.evidence_file(str(run/(TARGET_NAME+'.axf')),run,evidence).payload
    new_segments=shared.verify_image_pair(image,elf)
    old_path=shared.bounded_path(workspace/shared.OLD_RELATIVE,workspace)
    old=old_path.read_bytes()
    shared.require(shared.digest(old)==shared.OLD_SHA256,'Old candidate path/hash mismatch')
    frozen.append(shared.FrozenFile(old_path,old))
    prepared=shared.Prepared(workspace,manifest_path,manifest_payload,old,image,elf,
        shared.parse_hex(old),new_segments,tuple(frozen))
    shared.recheck_frozen(prepared)
    return prepared


def verify_post_reset_idle(report, hardware, output, expected_mode, port, clock, sleep):
    """Verify one new idle boot through read-only USB queries within one fixed three-second budget."""
    started=clock()
    deadline=started+3.0
    lines=[]
    serial_port=None
    report.update(post_reset_idle_verified=False, safe_startup_verified=False,
                  deployment_status='post_reset_verification_pending', new_boot=None)

    def save_post_reset():
        """Retain stage-specific proof without replacing the original preflight serial evidence."""
        report['post_reset_elapsed_ms']=(clock()-started)*1000
        (output/'report.json').write_text(shared.json.dumps(report,indent=2),encoding='utf-8')
        (output/'serial_post_reset.json').write_text(shared.json.dumps(lines,indent=2),encoding='utf-8')

    def bounded_clock():
        """Enforce the same deadline inside every shared protocol send/receive wait."""
        current=clock()
        shared.require(current<=deadline,'Post-reset USB/idle proof exceeded its three-second deadline')
        return current

    try:
        save_post_reset()
        previous_boot=report.get('disabled_feedback',{}).get('boot')
        shared.require(isinstance(previous_boot,str) and re.fullmatch(r'\d+',previous_boot),
                       'Missing pre-reset boot identity')
        # Poll device enumeration only. A complete but unsafe firmware snapshot is
        # rejected immediately; no control request or flash transaction is retried.
        for attempt in range(61):
            bounded_clock()
            ports=[item for item in hardware.ports() if item.vid==0x0483 and item.pid==0x5740
                   and item.serial_number==shared.USB_SERIAL]
            bounded_clock()
            shared.require(len(ports)<=1,'Duplicate exact USB identity after reset')
            if ports and (port is None or ports[0].device==port):
                serial_port=hardware.Serial(port=None,baudrate=115200,timeout=0.004,write_timeout=0.5)
                serial_port.dtr=False
                serial_port.rts=False
                serial_port.port=ports[0].device
                serial_port.open()
                report['post_reset_port']=ports[0].device
                bounded_clock()
                break
            remaining=deadline-clock()
            shared.require(remaining>0 and attempt<60,'Exact USB device did not return after reset')
            sleep(min(0.05,remaining))
        shared.require(serial_port is not None,'Exact USB device unavailable after reset')
        protocol=shared.Protocol(serial_port,lines,bounded_clock)
        new_boot=shared.idle_snapshot(protocol,expected_mode=expected_mode)
        shared.require(new_boot!=previous_boot,'Post-reset boot is unchanged; new firmware startup is unproven')
        # Bind a second complete snapshot to that new boot so queue/mode/idle
        # checks cannot straddle another reset during the first snapshot.
        shared.require(shared.idle_snapshot(protocol,expected_boot=new_boot,expected_mode=expected_mode)==new_boot,
                       'New boot changed while verifying idle startup')
        bounded_clock()
        report.update(new_boot=new_boot,post_reset_idle_verified=True,safe_startup_verified=True,
                      deployment_status='verified')
    except BaseException as error:
        report.update(post_reset_idle_verified=False,safe_startup_verified=False,deployment_status='failed',
                      error=type(error).__name__+': '+str(error))
        raise
    finally:
        try:
            if serial_port is not None: serial_port.close()
        except BaseException as error:
            report.update(post_reset_idle_verified=False,safe_startup_verified=False,deployment_status='failed',
                          error=type(error).__name__+': '+str(error))
            raise
        finally:
            save_post_reset()
    return report


def deploy(prepared, hardware, output, supported_unloaded, motor_power_on, port=None,
           clock=shared.time.perf_counter, sleep=shared.time.sleep, expected_mode=None):
    """Delegate unchanged safety mechanics while refusing an implicit current mode."""
    shared.require(type(expected_mode) is str and expected_mode in ('1','2'),
                   'Explicit expected current mode 1 or 2 is required')
    output=shared.bounded_path(output,prepared.workspace,require_file=False)
    report=shared.deploy(prepared,hardware,output,supported_unloaded,motor_power_on,port,
                         clock=clock,sleep=sleep,expected_mode=expected_mode)
    if report.get('error') is not None or report.get('flash_readback_equal') is not True or report.get('reset_completed') is not True:
        report.update(post_reset_idle_verified=False,safe_startup_verified=False,deployment_status='failed')
        (output/'report.json').write_text(shared.json.dumps(report,indent=2),encoding='utf-8')
        raise RuntimeError('Shared deployment did not complete successfully; post-reset verification is forbidden')
    return verify_post_reset_idle(report,hardware,output,expected_mode,port,clock,sleep)


def main(argv=None):
    """Verify offline by default and defer hardware imports until all explicit gates pass."""
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest',type=Path,required=True)
    parser.add_argument('--manifest-sha256',required=True)
    parser.add_argument('--deploy',action='store_true')
    parser.add_argument('--supported-unloaded-confirmed',action='store_true')
    parser.add_argument('--motor-power-on-confirmed',action='store_true')
    parser.add_argument('--expected-mode',choices=('1','2'))
    parser.add_argument('--port')
    arguments=parser.parse_args(argv)
    if arguments.deploy and not (arguments.supported_unloaded_confirmed and
            arguments.motor_power_on_confirmed and arguments.expected_mode):
        parser.error('--deploy requires both physical confirmations and explicit --expected-mode 1|2')
    prepared=prepare(arguments.manifest,arguments.manifest_sha256)
    print('POS_LOAD_ARTIFACTS_VERIFIED_NO_HARDWARE',shared.digest(prepared.image_payload),flush=True)
    if arguments.deploy:
        output=ROOT/'output/pos_load_auto_20261020'/('deploy_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ'))
        report=deploy(prepared,shared.load_hardware(),output,arguments.supported_unloaded_confirmed,
            arguments.motor_power_on_confirmed,arguments.port,expected_mode=arguments.expected_mode)
        print('POS_LOAD_DEPLOY_READBACK_EQUAL_NEWBOOT_IDLE',shared.json.dumps(report),flush=True)


if __name__=='__main__':
    main()
