"""Run bounded offline gain regressions and retain logs/hashes; never access hardware."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]


def main():
    """Execute maintained synthetic suites once and fail on any failed subprocess."""
    output = ROOT / 'gain_verification_final'
    output.mkdir(exist_ok=False)
    suites = [
        'run_single_static_load_tests.ps1', 'run_single_static_lcd_config_tests.ps1',
        'run_static_lcd_workflow_tests.ps1', 'run_static_load_tests.ps1',
        'run_static_startup_sync_tests.ps1', 'run_single_static_platform_tests.ps1',
        'run_static_peak_reason_tests.ps1', 'run_pos_load_app_tests.ps1',
        'run_pos_load_lcd_workflow_tests.ps1']
    results = []
    for suite in suites:
        result = subprocess.run(['powershell.exe', '-NoProfile', '-ExecutionPolicy', 'Bypass',
            '-File', str(ROOT / 'Tests/host' / suite)], cwd=ROOT,
            capture_output=True, timeout=180)
        log = output / (suite + '.log')
        log.write_bytes(result.stdout + result.stderr)
        results.append(dict(suite=suite, exit_code=result.returncode,
            log_sha256=hashlib.sha256(log.read_bytes()).hexdigest()))
        (output / 'receipt.json').write_text(json.dumps(dict(hardware_access=False,
            passed=all(row['exit_code'] == 0 for row in results) and len(results) == len(suites),
            suites=results), indent=2), encoding='utf-8')
        print(suite, result.returncode, flush=True)
        if result.returncode:
            sys.exit(result.returncode)


if __name__ == '__main__':
    main()
