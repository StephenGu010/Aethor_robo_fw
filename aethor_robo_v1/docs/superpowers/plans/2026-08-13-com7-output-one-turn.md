# COM7 Output One-Turn Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Create, automatically verify, and run a guarded PowerShell script that turns CAN ID 1 once, CAN ID 3 once, then IDs 1 and 3 together once through COM7.

**Architecture:** Keep the firmware and its 3 deg per-command limit unchanged. A dedicated host script owns the three-stage plan, sends one 3 deg relative command at a time, waits for ACK/DONE while maintaining heartbeats, and performs STOP/DISABLE after every stage or failure.

**Tech Stack:** Windows PowerShell 5.1, .NET `System.IO.Ports.SerialPort`, `aethor-arm-ascii-v1`, CRC-16/CCITT-FALSE, existing STM32 USB CDC firmware.

---

## File map

- Create `Tests/hardware/run_com7_output_one_turn_1_3.ps1`: serial protocol client, immutable three-stage motion plan, self-test, guarded live execution, and best-effort shutdown.
- Create `Tests/host/run_com7_output_one_turn_script_tests.ps1`: host entry point that invokes the hardware script in self-test mode without opening COM7.
- Do not modify firmware, Keil project files, generated HEX files, or the existing small-angle debug script.

### Task 1: Add the host regression entry point

**Files:**
- Create: `Tests/host/run_com7_output_one_turn_script_tests.ps1`
- Test: `Tests/host/run_com7_output_one_turn_script_tests.ps1`

- [ ] **Step 1: Write the failing host test**

Create a PowerShell test with a file-level comment and a single script body that resolves `Tests/hardware/run_com7_output_one_turn_1_3.ps1`, invokes it with `-SelfTest`, requires exit code zero, and requires the marker below:

```powershell
$result = & powershell.exe -NoProfile -ExecutionPolicy Bypass `
    -File $hardwareScriptPath -SelfTest 2>&1
if ($LASTEXITCODE -ne 0) { throw "ONE_TURN_SELF_TEST_PROCESS_FAILED: $result" }
if (($result -join "`n") -notmatch 'ONE_TURN_SELF_TESTS_PASSED') {
    throw "ONE_TURN_SELF_TEST_MARKER_MISSING: $result"
}
Write-Output 'COM7_OUTPUT_ONE_TURN_SCRIPT_TESTS_PASSED'
```

- [ ] **Step 2: Run the test and verify RED**

Run:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\Tests\host\run_com7_output_one_turn_script_tests.ps1
```

Expected: FAIL because `Tests/hardware/run_com7_output_one_turn_1_3.ps1` does not exist.

- [ ] **Step 3: Keep the failing test uncommitted until GREEN**

Confirm only the new host test and pre-existing unrelated build artifacts are shown by:

```powershell
git status --short --untracked-files=all
```

### Task 2: Implement the guarded one-turn script

**Files:**
- Create: `Tests/hardware/run_com7_output_one_turn_1_3.ps1`
- Test: `Tests/host/run_com7_output_one_turn_script_tests.ps1`

- [ ] **Step 1: Define the safe public parameters and constants**

Use parameters that default to no physical motion:

```powershell
param(
    [ValidatePattern('^COM\d+$')][string]$PortName = 'COM7',
    [switch]$RunMotion,
    [switch]$SelfTest
)
$stepDegrees = 3.0
$speedDegreesPerSecond = 3.0
$stepsPerTurn = 120
```

Reject live execution unless `-RunMotion` is present. Add a file-level comment and a function-level comment to every function.

- [ ] **Step 2: Implement and self-test the immutable stage plan**

Implement `Get-OneTurnStages` returning exactly:

```powershell
@(
    [pscustomobject]@{ Name = 'ID1'; Motors = '1'; Delta = 3.0; Steps = 120 },
    [pscustomobject]@{ Name = 'ID3'; Motors = '3'; Delta = 3.0; Steps = 120 },
    [pscustomobject]@{ Name = 'ID1_ID3'; Motors = '1,3'; Delta = -3.0; Steps = 120 }
)
```

`Invoke-OneTurnSelfTest` must assert that every stage totals exactly `+360`, `+360`, and `-360` deg, no step exceeds 3 deg, the first two stages select one motor, the final stage selects two motors, all speed lists match motor counts, the CRC of ASCII `123456789` is `0x29B1`, and the default does not run motion. It prints:

```text
ONE_TURN_SELF_TESTS_PASSED stages=3 steps=360
```

- [ ] **Step 3: Implement the serial protocol boundary**

Implement focused functions equivalent to the already verified small-angle client:

```text
Get-Crc16CcittFalse
ConvertTo-AethorFrame
ConvertFrom-AethorFrame
Open-AethorSerialPort
New-AethorContext
Send-AethorRequest
Send-AethorHeartbeatIfDue
Read-AethorResponse
Wait-AethorRequest
Invoke-AethorQuery
Invoke-AethorAction
Initialize-AethorSession
```

Use ASCII, 115200 8N1, 50 ms read timeout, 500 ms write timeout, CRC validation, a 200 ms heartbeat interval, unique request IDs, and ACK-before-DONE enforcement. Treat `ERR`, failed DONE, bad CRC, and timeout as terminating errors.

- [ ] **Step 4: Implement one stage and global sequencing**

`Invoke-OneTurnStage` must execute:

```text
INIT_MOTORS -> CLEAR_FAULT -> ENABLE -> 120 x MOVE_REL -> STOP -> DISABLE
```

For each MOVE_REL, build `delta_deg` and `speed_deg_s` lists matching the stage motor count. Wait up to 10 seconds for each small step, print progress at steps 1, 10, 20, ..., 120, then query `GET_MOTORS` and `GET_DIAG`.

`Invoke-OneTurnSequence` must initialize one session, disable streaming, query initial state/diagnostics, and run the three stages in order. A stage failure prevents all later stages.

- [ ] **Step 5: Implement best-effort shutdown**

Keep the currently active motor list in the outer scope. In `finally`, if a live stage has not completed cleanup, send `STOP` then `DISABLE` for that exact list, each with a bounded three-second timeout. Always close and dispose COM7.

- [ ] **Step 6: Run the host test and verify GREEN**

Run:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\Tests\host\run_com7_output_one_turn_script_tests.ps1
```

Expected:

```text
COM7_OUTPUT_ONE_TURN_SCRIPT_TESTS_PASSED
```

- [ ] **Step 7: Commit the tested script**

Run:

```powershell
git add -- .\aethor_robo_v1\Tests\hardware\run_com7_output_one_turn_1_3.ps1 .\aethor_robo_v1\Tests\host\run_com7_output_one_turn_script_tests.ps1
git commit -m "test: add guarded COM7 output one-turn sequence"
```

Do not stage `MDK-ARM/CtrBoard-H7_FDCAN/CtrBoard-H7_FDCAN.hex` or `MDK-ARM/RTE/_CtrBoard-H7_FDCAN/RTE_Components.h`.

### Task 3: Verify the script and the unchanged software baseline

**Files:**
- Test: `Tests/hardware/run_com7_output_one_turn_1_3.ps1`
- Test: `Tests/host/run_com7_output_one_turn_script_tests.ps1`

- [ ] **Step 1: Run direct self-test**

Run:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\Tests\hardware\run_com7_output_one_turn_1_3.ps1 -SelfTest
```

Expected: `ONE_TURN_SELF_TESTS_PASSED stages=3 steps=360`.

- [ ] **Step 2: Run the existing small-angle script tests**

Run:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\Tests\host\run_com7_debug_script_tests.ps1
```

Expected: `COM7_DEBUG_SCRIPT_TESTS_PASSED`.

- [ ] **Step 3: Check the exact worktree diff**

Run `git status --short --branch` and `git diff --check`. Expected: the plan and script commits are clean; only the two pre-existing build artifact modifications remain unstaged.

### Task 4: Execute the three real-hardware stages through COM7

**Files:**
- Run: `Tests/hardware/run_com7_output_one_turn_1_3.ps1`

- [ ] **Step 1: Confirm COM7 is present**

Run:

```powershell
Get-CimInstance Win32_SerialPort | Where-Object DeviceID -eq 'COM7'
```

Expected: one `USB serial device (COM7)` with status `OK`.

- [ ] **Step 2: Execute the approved sequence**

Run:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\Tests\hardware\run_com7_output_one_turn_1_3.ps1 -PortName COM7 -RunMotion
```

Expected serial milestones:

```text
STAGE_COMPLETED name=ID1 motors=1 total_deg=360
STAGE_COMPLETED name=ID3 motors=3 total_deg=360
STAGE_COMPLETED name=ID1_ID3 motors=1,3 total_deg=-360
OUTPUT_ONE_TURN_SEQUENCE_PASSED port=COM7 stages=3
```

- [ ] **Step 3: Record hardware versus protocol evidence separately**

Report the exact last successful stage, request error if any, STOP/DISABLE result, final motor fault fields, CAN Bus-Off field, and elapsed time. State that serial completion is confirmed by logs; physical one-turn movement remains dependent on the user's direct observation.
