"""Test POS acceptance and shared deployment gates without any hardware I/O."""
from pathlib import Path
import copy
import importlib.util
import json
import os
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]


def load_file(name, path):
    """Import one maintained module without running its command-line entry point."""
    specification = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(specification)
    sys.modules[name] = module
    specification.loader.exec_module(module)
    return module


class PosLoadAcceptanceTests(unittest.TestCase):
    """Reject omitted, replaced or downgraded offline receipts before hardware access."""

    def setUp(self):
        """Create retained isolated input trees and genuine matching HEX/ELF fixtures."""
        self.adapter = load_file('pos_load_adapter_test', ROOT / 'tools/deploy_pos_load_test.py')
        self.fake = load_file('pos_hardware_fixture', ROOT / 'Tests/hardware/test_single_static_deploy.py')
        fixture_root = ROOT / 'Tests/host/build/pos_load_deploy'
        fixture_root.mkdir(parents=True, exist_ok=True)
        self.workspace = Path(tempfile.mkdtemp(dir=fixture_root))
        subprocess.run(['git', 'init', '-q', str(self.workspace)], check=True, capture_output=True)
        for name in self.adapter.REQUIRED_SOURCES:
            path = self.workspace / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text('retained maintained fixture\n', encoding='utf-8')
        for name in ('LCD-SingleStatic-Test', self.adapter.TARGET_NAME):
            destination = self.workspace / 'MDK-ARM' / (name + '.uvprojx')
            destination.write_bytes((ROOT / 'MDK-ARM' / (name + '.uvprojx')).read_bytes())
        self.run = self.workspace / 'output' / 'acceptance'
        self.run.mkdir(parents=True)
        self.old, _ = self.fake.image_pair(0x08000000, b'\x00\x00\x02\x20\x09\x00\x00\x08old-image')
        old_path = self.workspace / self.adapter.shared.OLD_RELATIVE
        old_path.parent.mkdir(parents=True, exist_ok=True)
        old_path.write_bytes(self.old)
        self.pin = patch.object(self.adapter.shared, 'OLD_SHA256', self.adapter.shared.digest(self.old))
        self.pin.start()
        self.addCleanup(self.pin.stop)
        self.image, self.elf = self.fake.image_pair(0x08000000, b'\x00\x00\x02\x20\x09\x00\x00\x08new-pos-image')
        records = []
        for name in self.adapter.REQUIRED_RECORDS:
            records.append(dict(name=name, exit_code=0, log=self.write(name+'.log', b'pass\n'),
                                stderr=self.write(name+'.stderr.log', b'')))
        checks = []
        for name in self.adapter.LCD_CHECKS:
            checks.append(dict(name=name, exit_code=0, stdout=str(self.run / (name+'.log')),
                               stderr=str(self.run / (name+'.stderr.log'))))
            self.write('lcd_evidence/'+name+'.log', b'pass\n')
            self.write('lcd_evidence/'+name+'.stderr.log', b'')
        self.write('lcd_evidence/summary.json', json.dumps(dict(checks=checks, software_only=True,
            hardware_tested=False, keil_requested=False,
            first_party_sources_unchanged_during_verification=True)).encode())
        builds=[]
        for target in self.adapter.REQUIRED_TARGETS:
            builds.append(dict(target=target, errors=0, warnings=0,
                hex_path=self.write(target+'.hex', self.image), elf_path=self.write(target+'.axf', self.elf),
                map_path=self.write(target+'.map', b'map\n'), htm_path=self.write(target+'.htm', b'callgraph\n'),
                log_path=self.write(target+'.keil.log', b'0 Error(s), 0 Warning(s)\n')))
        images=[]
        for prefix in self.adapter.POS_RENDER_PREFIXES:
            name=prefix+'fixture.png'
            self.write('render/'+name,b'\x89PNG\r\n\x1a\n'+bytes(30))
            images.append(dict(image=name,width=280,height=240,rgb_sha256='0'*64))
        self.write('render/render_manifest.json',json.dumps(dict(
            renderer='actual LVGL 8.3.11 software raster + production double-buffer flush port',images=images)).encode())
        self.manifest = dict(schema='pos-load-acceptance-v1', target=self.adapter.TARGET_NAME,
            feature_gate='AETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST=1', trial_mode=2,
            passed=True, software_only=True, hardware_tested=False, sources_unchanged=True,
            failure=None, lcd_checks=len(checks), run_id='20261020T120000000Z',
            started_at_utc='2026-10-20T12:00:00Z', completed_at_utc='2026-10-20T12:01:00Z',
            previous_image_sha256=self.adapter.shared.OLD_SHA256,
            verification_script_path=str(self.workspace/'tools/verify_pos_load_test.ps1'),
            records=records, builds=builds)
        self.path=self.run/'manifest.json'
        self.refresh()

    def write(self, name, payload):
        """Retain one explicit evidence file and return its absolute location."""
        path=self.run/name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(payload)
        return str(path)

    def refresh(self):
        """Anchor a fixture manifest to all current source and evidence bytes."""
        shared=self.adapter.shared
        self.manifest['source_hashes']=[dict(path=name, sha256=shared.digest((self.workspace/name).read_bytes()))
            for name in sorted(self.adapter.maintained_sources(self.workspace))]
        self.manifest['verification_script_sha256']=shared.digest((self.workspace/'tools/verify_pos_load_test.ps1').read_bytes())
        self.manifest['evidence_hashes']=[dict(path=str(path),sha256=shared.digest(path.read_bytes()))
            for path in self.run.rglob('*') if path.is_file() and path != self.path]
        self.path.write_text(json.dumps(self.manifest),encoding='utf-8')
        self.anchor=shared.digest(self.path.read_bytes())

    def prepare(self):
        """Validate through the actual POS schema with the temporary workspace boundary."""
        return self.adapter.prepare(self.path,self.anchor,self.workspace)

    def test_complete_acceptance_freezes_exact_images_without_hardware(self):
        """Require the independent matrix and preserve complete immutable image bytes."""
        prepared=self.prepare()
        self.assertEqual(prepared.image_payload,self.image)
        self.assertEqual(prepared.elf_payload,self.elf)
        self.assertEqual(prepared.old_payload,self.old)
        self.assertIn('pos_load_app_tests',self.adapter.REQUIRED_RECORDS)
        self.assertIn('pos_load_model_tests',self.adapter.REQUIRED_RECORDS)
        self.assertIn('map_pos_load',self.adapter.REQUIRED_RECORDS)

    def test_rejects_schema_gate_mode_and_software_flag_downgrades(self):
        """Reject legacy manifests, missing gate, mismatched target/mode and fake successes."""
        for key,value in (('schema','single-static-startup-acceptance-v2'),('target','LCD-POS'),
                ('feature_gate',''),('trial_mode',1),('passed',False),('hardware_tested',True),
                ('sources_unchanged',False),('previous_image_sha256','0'*64)):
            with self.subTest(key=key):
                original=self.manifest[key]
                self.manifest[key]=value
                self.refresh()
                with self.assertRaises(RuntimeError): self.prepare()
                self.manifest[key]=original

    def test_rejects_missing_duplicate_extra_or_failed_records(self):
        """The acceptance matrix cannot be shortened or replaced with an arbitrary subset."""
        original=copy.deepcopy(self.manifest['records'])
        for mutation in ('missing','duplicate','extra','failure','bool_exit'):
            with self.subTest(mutation=mutation):
                records=copy.deepcopy(original)
                if mutation=='missing': records.pop()
                elif mutation=='duplicate': records[-1]=records[0]
                elif mutation=='extra': records.append(records[0])
                elif mutation=='failure': records[0]['exit_code']=1
                else: records[0]['exit_code']=False
                self.manifest['records']=records
                self.refresh()
                with self.assertRaises(RuntimeError): self.prepare()
        self.manifest['records']=original

    def test_each_pos_and_adrc_record_is_individually_mandatory(self):
        """A legacy LCD pass cannot substitute for any new POS or preserved ADRC receipt."""
        original=copy.deepcopy(self.manifest['records'])
        for name in ('pos_load_app_tests','pos_load_model_tests','pos_load_lcd_workflow','pos_load_platform_tests','adrc_shaft','adrc_lcd_integration'):
            with self.subTest(name=name):
                self.manifest['records']=[record for record in original if record['name']!=name]
                self.refresh()
                with self.assertRaises(RuntimeError): self.prepare()
        self.manifest['records']=original

    def test_required_check_cannot_borrow_another_checks_log(self):
        """Bind every named process to its own retained stdout and stderr identities."""
        self.manifest['records'][1]['log']=self.manifest['records'][0]['log']
        self.refresh()
        with self.assertRaises(RuntimeError): self.prepare()

    def test_missing_new_pos_render_is_rejected_even_with_old_lvgl_suite_pass(self):
        """All ten new real-LVGL parameter, lifecycle, report and gate pages are required."""
        path=self.run/'render/render_manifest.json'
        manifest=json.loads(path.read_text())
        manifest['images'].pop()
        path.write_text(json.dumps(manifest))
        self.refresh()
        with self.assertRaises(RuntimeError): self.prepare()

    def test_rejects_missing_source_and_unlisted_include_file(self):
        """Bind new .inc inputs rather than accepting a caller's incomplete source list."""
        self.manifest['source_hashes'].pop()
        self.path.write_text(json.dumps(self.manifest),encoding='utf-8')
        self.anchor=self.adapter.shared.digest(self.path.read_bytes())
        with self.assertRaises(RuntimeError): self.prepare()
        self.refresh()
        path=self.workspace/'App/DebugUi/new_required.inc'
        path.write_text('new maintained include\n')
        with self.assertRaises(RuntimeError): self.prepare()

    def test_rejects_unhashed_evidence_and_mutated_inputs(self):
        """Unknown evidence or post-freeze input replacement must invalidate preparation."""
        prepared=self.prepare()
        (self.workspace/'App/DebugUi/debug_ui_pos_load_app.inc').write_text('changed\n')
        with self.assertRaises(RuntimeError): self.adapter.shared.recheck_frozen(prepared)
        with self.assertRaises(RuntimeError): self.prepare()
        self.refresh()
        self.write('unlisted.log',b'unbound\n')
        with self.assertRaises(RuntimeError): self.prepare()

    def test_rejects_altered_hex_elf_or_unclean_build(self):
        """A rehashed image mismatch and warnings still fail the semantic image/build gate."""
        path=self.run/(self.adapter.TARGET_NAME+'.hex')
        path.write_bytes(self.fake.image_pair(0x08000000,b'other')[0])
        self.refresh()
        with self.assertRaises(RuntimeError): self.prepare()
        path.write_bytes(self.image)
        log=self.run/(self.adapter.TARGET_NAME+'.keil.log')
        log.write_text('0 Error(s), 1 Warning(s)\n')
        self.refresh()
        with self.assertRaises(RuntimeError): self.prepare()

    def test_new_project_preserves_legacy_inventory_and_all_other_defines(self):
        """The independent target changes only its identity and the POS feature gate."""
        self.adapter.validate_project(self.workspace)
        project=self.workspace/'MDK-ARM'/ (self.adapter.TARGET_NAME+'.uvprojx')
        project.write_text(project.read_text().replace('AETHOR_DEBUG_UI_ALLOW_MIT=1,',''))
        self.refresh()
        with self.assertRaises(RuntimeError): self.prepare()

    def make_hardware(self, prepared, scenario='success'):
        """Extend the legacy fake with deterministic post-reset USB and firmware replies."""
        hardware=self.fake.FakeHardware(self.adapter.shared,prepared,scenario)
        original_serial=hardware.Serial
        original_ports=hardware.ports
        hardware.post_reset_commands=[]
        hardware.post_reset_enumerations=0

        def ports():
            """Simulate exact-identity USB reenumeration only after the ordinary reset."""
            records=original_ports()
            if 'normal_reset' not in hardware.events: return records
            hardware.post_reset_enumerations+=1
            if scenario=='post_lost_usb': return []
            if scenario=='post_delayed_usb' and hardware.post_reset_enumerations<3: return []
            if scenario=='post_duplicate_usb': return records*2
            return records

        def serial(**options):
            """Keep all preflight replies intact and control only the reopened serial stream."""
            port=original_serial(**options)
            original_write=port.write

            def write(payload):
                """Return the new boot identity or one deliberate unsafe post-reset field."""
                result=original_write(payload)
                if 'normal_reset' not in hardware.events: return result
                command=payload.decode('ascii').strip().split(' ',1)[1]
                hardware.post_reset_commands.append(command)
                if scenario!='post_same_boot': port.reply=port.reply.replace(b'boot=7',b'boot=8')
                replacements={
                    'post_enabled':('show state',b'enabled=00',b'enabled=01'),
                    'post_moving':('show state',b'moving=0',b'moving=1'),
                    'post_active':('show state',b'active=0',b'active=2'),
                    'post_fault':('show state',b'fault=none',b'fault=driver'),
                    'post_owner':('adrc status',b'owner=lcd',b'owner=bench'),
                    'post_queue':('adrc gate motor=7',b'ui_results=0',b'ui_results=1'),
                    'post_can':('show diag can',b'error=0',b'error=1'),
                    'post_mode':('adrc gate motor=7',('mode='+hardware.current_mode).encode(),
                                 ('mode='+('2' if hardware.current_mode=='1' else '1')).encode()),
                    'post_missing_boot':('ping',b'boot=8',b'no_boot=8'),
                }
                if scenario in replacements:
                    expected,old,new=replacements[scenario]
                    if command==expected: port.reply=port.reply.replace(old,new)
                if scenario=='post_deadline': hardware.clock.now+=3.1
                return result

            port.write=write
            return port

        hardware.Serial=serial
        hardware.ports=ports
        return hardware

    def test_post_reset_rejects_every_missing_idle_or_new_boot_proof(self):
        """Readback alone must not pass oldboot, lost USB or any unsafe firmware snapshot."""
        prepared=self.prepare()
        for scenario in ('post_same_boot','post_lost_usb','post_duplicate_usb','post_enabled',
                'post_moving','post_active','post_fault','post_owner','post_queue','post_can',
                'post_mode','post_missing_boot','post_deadline'):
            with self.subTest(scenario=scenario):
                hardware=self.make_hardware(prepared,scenario)
                output=self.workspace/scenario
                with self.assertRaises(RuntimeError):
                    self.adapter.deploy(prepared,hardware,output,True,True,
                        clock=hardware.clock,sleep=hardware.clock.sleep,expected_mode='2')
                report=json.loads((output/'report.json').read_text())
                self.assertTrue(report['flash_readback_equal'])
                self.assertFalse(report['post_reset_idle_verified'])
                self.assertEqual(report['deployment_status'],'failed')
                self.assertIsNotNone(report['error'])
                self.assertTrue(set(hardware.post_reset_commands)<=set(('show state','adrc status',
                    'adrc gate motor=7','show diag can','ping')))
                self.assertEqual(hardware.events.count('bench disable 7'),1)

    def test_failed_shared_transaction_does_not_enter_post_reset_queries(self):
        """Preserve an earlier deployment error without attempting any startup readout."""
        prepared=self.prepare()
        output=self.workspace/'shared_report_error'
        output.mkdir()
        report=dict(error='earlier deployment failed',flash_readback_equal=False,reset_completed=False)
        with patch.object(self.adapter.shared,'deploy',return_value=report), patch.object(
                self.adapter,'verify_post_reset_idle',side_effect=AssertionError('post-reset entered')):
            with self.assertRaises(RuntimeError):
                self.adapter.deploy(prepared,object(),output,True,True,expected_mode='2')
        retained=json.loads((output/'report.json').read_text())
        self.assertEqual(retained['error'],'earlier deployment failed')
        self.assertEqual(retained['deployment_status'],'failed')
        self.assertFalse(retained['post_reset_idle_verified'])

    def test_post_reset_waits_boundedly_for_exact_usb_reenumeration(self):
        """A delayed exact-device return can pass once, without any extra control command."""
        prepared=self.prepare()
        hardware=self.make_hardware(prepared,'post_delayed_usb')
        report=self.adapter.deploy(prepared,hardware,self.workspace/'reenumerated',True,True,
            clock=hardware.clock,sleep=hardware.clock.sleep,expected_mode='2')
        self.assertTrue(report['post_reset_idle_verified'])
        self.assertEqual(report['new_boot'],'8')
        self.assertGreaterEqual(hardware.post_reset_enumerations,3)
        self.assertLessEqual(report['post_reset_elapsed_ms'],3000)

    def test_real_shared_transaction_preserves_explicit_current_mode(self):
        """Fake USB/SWD exercises unchanged 100ms, backup, image/readback and mode gates."""
        prepared=self.prepare()
        for mode in ('1','2'):
            hardware=self.make_hardware(prepared)
            hardware.current_mode=mode
            report=self.adapter.deploy(prepared,hardware,self.workspace/('deploy_mode_'+mode),True,True,
                clock=hardware.clock,sleep=hardware.clock.sleep,expected_mode=mode)
            self.assertEqual(report['target'],self.adapter.TARGET_NAME)
            self.assertEqual(report['expected_mode'],mode)
            self.assertTrue(report['post_reset_idle_verified'])
            self.assertEqual(report['new_boot'],'8')
            self.assertEqual(report['deployment_status'],'verified')
            self.assertTrue(report['flash_readback_equal'])
            self.assertEqual(report['motion_commands_sent'],0)
            self.assertTrue(report['disabled_feedback']['age_at_halt_completion_upper_ms']<=100)

    def test_shared_hardware_protections_reject_before_programming(self):
        """Freshness, old-image and mode changes retain the original no-program gates."""
        prepared=self.prepare()
        for scenario in ('stale','old_mismatch','mode_changed_after_disable','deadline'):
            with self.subTest(scenario=scenario):
                hardware=self.fake.FakeHardware(self.adapter.shared,prepared,scenario)
                hardware.current_mode='1'
                with self.assertRaises(RuntimeError):
                    self.adapter.deploy(prepared,hardware,self.workspace/('rejected_'+scenario),True,True,
                        clock=hardware.clock,sleep=hardware.clock.sleep,expected_mode='1')
                self.assertNotIn('program',hardware.events)
                self.assertNotIn('halt',hardware.events)

    def test_default_cli_does_not_import_or_enumerate_hardware(self):
        """The ordinary command is purely artifact verification even when physical flags exist."""
        prepared=self.prepare()
        with patch.object(self.adapter,'prepare',return_value=prepared), patch.object(
                self.adapter.shared,'load_hardware',side_effect=AssertionError('hardware opened')):
            self.adapter.main(['--manifest',str(self.path),'--manifest-sha256',self.anchor])

    def test_cli_requires_explicit_mode_before_any_hardware_import(self):
        """Deployment may not silently default the currently installed firmware mode."""
        with patch.object(self.adapter.shared,'load_hardware',side_effect=AssertionError('hardware opened')):
            with self.assertRaises(SystemExit):
                self.adapter.main(['--manifest',str(self.path),'--manifest-sha256',self.anchor,'--deploy',
                    '--supported-unloaded-confirmed','--motor-power-on-confirmed'])


if __name__=='__main__':
    unittest.main(verbosity=2)
