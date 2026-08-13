# COM7 Fixed Relative Target Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a reusable `MOVE_REL_TARGET` command that locks `command-start feedback + delta` once, supports per-motor angles up to 360 degrees, and drives the existing COM7 one-turn sequence with one command per stage.

**Architecture:** Preserve `MOVE_REL` as the bounded 3 degree commissioning command. Add an independent protocol command type for long fixed relative targets, share the existing application target snapshot and periodic resend path, and reject the entire selected set before sending CAN if any final target exceeds runtime PMAX.

**Tech Stack:** C11, STM32H723, FreeRTOS/CMSIS-RTOS, S3519 Classic CAN POS_VEL, `aethor-arm-ascii-v1`, PowerShell 5.1, GCC host tests, Keil ARMCC 5, OpenOCD CMSIS-DAP.

---

## File map

- Modify `App/Config/app_profile.h`: declare the separate 360 degree fixed-target limit.
- Modify `App/Protocol/protocol_engine.h`: add `PROTOCOL_COMMAND_MOVE_RELATIVE_TARGET`.
- Modify `App/Protocol/protocol_engine.c`: parse `MOVE_REL_TARGET` with its own validation while preserving `MOVE_REL`.
- Modify `App/aethor_app.c`: route both relative command types through one fixed-target snapshot path.
- Modify `Tests/host/protocol_test_main.c`: protocol RED/GREEN coverage for valid and invalid long targets.
- Modify `Tests/host/phase0_test_main.c`: application regression proving the absolute target frame does not change after feedback changes.
- Modify `Tests/hardware/run_com7_output_one_turn_1_3.ps1`: replace 120 small commands with one long command per stage and bounded STOP retry.
- Modify `Tests/host/run_com7_output_one_turn_script_tests.ps1`: continue exercising the no-hardware self-test path.
- Modify `docs/compatibility/aethor-arm-ascii-v1-schema.json`: publish the new bench-only command and separate limits.
- Do not stage generated HEX/RTE changes or unrelated user files.

### Task 1: Add protocol contract RED tests

**Files:**
- Modify: `Tests/host/protocol_test_main.c:709`
- Modify: `Tests/host/phase0_test_main.c:184`
- Test: `Tests/host/run_protocol_tests.ps1`

- [ ] **Step 1: Add the fixed-target protocol test**

Extend `test_protocol_engine_bench_subset_commands()` after the existing `MOVE_REL` checks with requests equivalent to:

```c
request_length = build_request_frame(
    "REQ 6 MOVE_REL_TARGET motors=1,3 delta_deg=360.0,-45.0 speed_deg_s=3.0,1.0",
    request_frame,
    sizeof(request_frame));
assert(protocol_engine_process_line(&engine,
                                    request_frame,
                                    request_length,
                                    6000U,
                                    &output_batch) == PROTOCOL_ENGINE_STATUS_OK);
assert(protocol_engine_pop_command(&engine, &command) == 1U);
assert(command.type == PROTOCOL_COMMAND_MOVE_RELATIVE_TARGET);
assert(command.motor_mask == 0x05U);
assert(command.values[0] == 360.0F);
assert(command.values[2] == -45.0F);
assert(command.speeds[0] == 3.0F);
assert(command.speeds[2] == 1.0F);
```

Add distinct requests asserting `BAD_VALUE field=delta_deg` for `0`, `360.1`, and `-360.1`; retain the existing assertion that `MOVE_REL 3.1` is rejected. Existing parser helpers already reject NaN/Inf, duplicate motors, and mismatched list widths, so include one fixed-target mismatched-list request to prove this command reaches that shared validation.

- [ ] **Step 2: Add the profile limit assertion**

Add beside the current Phase 0 profile assertions:

```c
assert(AETHOR_BENCH_MAX_TARGET_RELATIVE_DEGREES == 360.0F);
```

- [ ] **Step 3: Run the protocol test and verify RED**

Run:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\Tests\host\run_protocol_tests.ps1
```

Expected: compile failure because `PROTOCOL_COMMAND_MOVE_RELATIVE_TARGET` and `AETHOR_BENCH_MAX_TARGET_RELATIVE_DEGREES` do not exist.

### Task 2: Implement the independent protocol command

**Files:**
- Modify: `App/Config/app_profile.h:27`
- Modify: `App/Protocol/protocol_engine.h:91`
- Modify: `App/Protocol/protocol_engine.c:1750`
- Test: `Tests/host/protocol_test_main.c`

- [ ] **Step 1: Define the public constant and command type**

Add:

```c
#define AETHOR_BENCH_MAX_TARGET_RELATIVE_DEGREES (360.0F)
```

Add `PROTOCOL_COMMAND_MOVE_RELATIVE_TARGET` immediately after `PROTOCOL_COMMAND_MOVE_RELATIVE` so the new enum remains bench-motion adjacent.

- [ ] **Step 2: Parameterize bench relative validation**

In `protocol_engine_handle_bench_action()`, treat both relative command types as carrying angle and speed lists. Select the angle limit explicitly:

```c
float maximum_delta_degrees =
    (command_type == PROTOCOL_COMMAND_MOVE_RELATIVE_TARGET)
        ? AETHOR_BENCH_MAX_TARGET_RELATIVE_DEGREES
        : AETHOR_BENCH_MAX_RELATIVE_DEGREES;
```

For every selected motor, reject non-finite/zero/out-of-range deltas and non-finite/non-positive/over-limit speeds:

```c
if (!isfinite(command.values[joint_index]) ||
    (command.values[joint_index] == 0.0F) ||
    (command.values[joint_index] < -maximum_delta_degrees) ||
    (command.values[joint_index] > maximum_delta_degrees))
{
    /* Emit existing BAD_VALUE field=delta_deg response and return BAD_REQUEST. */
}
```

Use `AETHOR_BENCH_MAX_SPEED_DEGREES_S` instead of a literal `3.0F`. Keep the existing file-level and function-level comments; update the handler comment to mention both explicit-motor relative commands.

- [ ] **Step 3: Dispatch the new operation**

Immediately after the `MOVE_REL` branch in `protocol_engine_process_line()`, add:

```c
else if (ascii_protocol_request_operation_equals(&request,
                                                 "MOVE_REL_TARGET") != 0U)
{
    status = protocol_engine_handle_bench_action(
        engine,
        &request,
        PROTOCOL_COMMAND_MOVE_RELATIVE_TARGET,
        timestamp_us,
        output_batch);
}
```

- [ ] **Step 4: Run protocol and Phase 0 tests GREEN**

Run:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\Tests\host\run_protocol_tests.ps1
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\Tests\host\run_phase0_tests.ps1
```

Expected: `PROTOCOL_TESTS_PASSED` and `PHASE0_TESTS_PASSED`.

- [ ] **Step 5: Commit the protocol slice**

Stage only the four source/test files and commit:

```powershell
git commit -m "feat: add fixed relative target command"
```

### Task 3: Prove and implement the application fixed-target path

**Files:**
- Modify: `Tests/host/phase0_test_main.c:809`
- Modify: `App/aethor_app.c:870`
- Test: `Tests/host/run_phase0_tests.ps1`

- [ ] **Step 1: Add the application RED regression**

Create `test_aethor_app_locks_fixed_relative_target_from_command_start()` using the existing discovery/enable helpers. Submit:

```c
"REQ 4 MOVE_REL_TARGET motors=1 delta_deg=360.0 speed_deg_s=3.0"
```

Capture the first ID `0x101` target frame. Then feed a changed, still-not-at-target motor-1 position and service the application until the next ID `0x101` target frame is produced. Assert both CAN payloads are byte-identical:

```c
assert(repeated_target.identifier == first_target.identifier);
assert(repeated_target.length == first_target.length);
assert(memcmp(repeated_target.data,
              first_target.data,
              first_target.length) == 0);
```

Add a second case with feedback close enough to PMAX that `feedback + 2*pi` cannot be encoded. Assert the action returns `DONE ... FAILED` and no position-velocity target frame becomes available.

Call the new test from `main()` immediately after `test_aethor_app_repeats_unfinished_bench_target_batch()`.

- [ ] **Step 2: Run Phase 0 and verify RED**

Run `Tests/host/run_phase0_tests.ps1`.

Expected: the new protocol command is accepted by the protocol layer but the application returns a failure before emitting the expected fixed target because it does not route the new command type yet.

- [ ] **Step 3: Share the existing fixed-target application path**

Change the application branch condition to:

```c
else if ((command->type == PROTOCOL_COMMAND_MOVE_RELATIVE) ||
         (command->type == PROTOCOL_COMMAND_MOVE_RELATIVE_TARGET))
```

Do not change how `motor_position_rad` is computed, copied into `motion_plan.target_position_rad`, encoded into `application_action.frames`, or periodically resent. The existing `motor_runtime_build_position_velocity_subset()` remains the final PMAX/VMAX validation boundary. The deadline continues to use the maximum `abs(delta)/speed` plus `AETHOR_APP_ACTION_TIMEOUT_US`.

- [ ] **Step 4: Run Phase 0 GREEN**

Run `Tests/host/run_phase0_tests.ps1`.

Expected: `PHASE0_TESTS_PASSED`; the frame identity and PMAX atomic-rejection assertions pass.

- [ ] **Step 5: Commit the application slice**

Commit only `App/aethor_app.c` and `Tests/host/phase0_test_main.c`:

```powershell
git commit -m "feat: lock extended relative motion target"
```

### Task 4: Migrate and test the COM7 one-turn script

**Files:**
- Modify: `Tests/hardware/run_com7_output_one_turn_1_3.ps1:22-25, 319-383, 437-485`
- Modify: `Tests/host/run_com7_output_one_turn_script_tests.ps1`
- Test: `Tests/host/run_com7_output_one_turn_script_tests.ps1`

- [ ] **Step 1: Change the self-test expectations first**

Update self-test assertions to require three stages with one command each:

```powershell
@(
    [pscustomobject]@{ Name = 'ID1'; Motors = '1'; Delta = 360.0 },
    [pscustomobject]@{ Name = 'ID3'; Motors = '3'; Delta = 360.0 },
    [pscustomobject]@{ Name = 'ID1_ID3'; Motors = '1,3'; Delta = -360.0 }
)
```

Require operation name `MOVE_REL_TARGET`, total command count 3, per-stage planned duration 120 seconds at 3 degrees/s, and final marker:

```text
ONE_TURN_SELF_TESTS_PASSED stages=3 commands=3
```

- [ ] **Step 2: Run script test and verify RED**

Run `Tests/host/run_com7_output_one_turn_script_tests.ps1`.

Expected: FAIL because the script still declares 120 steps and prints the old marker.

- [ ] **Step 3: Replace the stage loop with one fixed-target action**

Remove the 120-iteration MOVE_REL loop. In `Invoke-OneTurnStage`, build angle/speed lists once and call:

```powershell
$plannedDurationMilliseconds = [int][math]::Ceiling(
    ([math]::Abs($Stage.Delta) / $speedDegreesPerSecond) * 1000.0)
$actionTimeoutMilliseconds = $plannedDurationMilliseconds + 5000
Invoke-AethorAction -Context $Context -Operation 'MOVE_REL_TARGET' `
    -Fields "motors=$($Stage.Motors) delta_deg=$deltaValues speed_deg_s=$speedValues" `
    -TimeoutMilliseconds $actionTimeoutMilliseconds | Out-Null
```

Print one `STAGE_PROGRESS` at command acceptance and retain `STAGE_COMPLETED` only after DONE, STOP, and DISABLE.

- [ ] **Step 4: Add one bounded normal STOP retry**

Create `Invoke-AethorStopWithRetry` with a function-level comment. It calls STOP with a 3000 ms timeout, catches only the first failure, logs one warning, and retries once. A second failure propagates and prevents the next stage. `finally` still calls the existing best-effort STOP/DISABLE path.

- [ ] **Step 5: Run script tests GREEN**

Run:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\Tests\host\run_com7_output_one_turn_script_tests.ps1
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\Tests\host\test_com7_dual_motor_debug_script.ps1
```

Expected: `COM7_OUTPUT_ONE_TURN_SCRIPT_TESTS_PASSED` and `COM7_DEBUG_SCRIPT_TESTS_PASSED` without opening COM7.

- [ ] **Step 6: Commit the script slice**

Commit the two script files:

```powershell
git commit -m "test: drive one-turn stages with fixed targets"
```

### Task 5: Update compatibility contract and run full software regression

**Files:**
- Modify: `docs/compatibility/aethor-arm-ascii-v1-schema.json:46`
- Test: all `Tests/host/run_*.ps1` suites and architecture guard

- [ ] **Step 1: Publish the new command and limits**

Add `MOVE_REL_TARGET` to `bench_only_commands`. Replace the ambiguous single delta field with:

```json
"maximum_move_rel_delta_deg": 3.0,
"maximum_move_rel_target_delta_deg": 360.0,
"maximum_speed_deg_s": 3.0
```

- [ ] **Step 2: Validate JSON and run all software tests**

Run JSON parsing plus these suites:

```powershell
Get-Content -Raw .\docs\compatibility\aethor-arm-ascii-v1-schema.json | ConvertFrom-Json | Out-Null
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\Tests\host\run_tests.ps1
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\Tests\host\run_phase0_tests.ps1
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\Tests\host\run_protocol_tests.ps1
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\Tests\host\run_motor_core_tests.ps1
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\Tests\host\run_platform_io_tests.ps1
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\Tests\host\run_motion_tests.ps1
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\Tests\host\run_simulator_tests.ps1
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\Tests\host\check_phase0_architecture.ps1
```

Expected: every suite prints its PASS marker; `git diff --check` reports no whitespace errors.

- [ ] **Step 3: Commit the compatibility contract**

Commit only the schema:

```powershell
git commit -m "docs: publish fixed relative target command"
```

### Task 6: Rebuild, verify, and flash firmware

**Files:**
- Rebuild: `MDK-ARM/CtrBoard-H7_FDCAN.uvprojx`
- Flash: `MDK-ARM/CtrBoard-H7_FDCAN/CtrBoard-H7_FDCAN.hex`

- [ ] **Step 1: Confirm there is no running UV4 process**

Run `Get-Process UV4 -ErrorAction SilentlyContinue`. If a process exists, wait for the existing build to finish; do not start a concurrent Keil build.

- [ ] **Step 2: Run one single-instance Keil Rebuild**

Run:

```powershell
& "E:\keil\UV4\UV4.exe" -r "MDK-ARM\CtrBoard-H7_FDCAN.uvprojx" -j0
```

Inspect `MDK-ARM/CtrBoard-H7_FDCAN/CtrBoard-H7_FDCAN.build_log.htm`. Expected: `0 Error(s), 0 Warning(s)` and `aethor_app.c` plus `protocol_engine.c` compiled into the target.

- [ ] **Step 3: Record generated artifact hashes without staging them**

Run `Get-FileHash -Algorithm SHA256` for the generated HEX and MAP. Record file sizes and hashes. Leave the already modified HEX/RTE files unstaged.

- [ ] **Step 4: Flash and verify through CMSIS-DAP**

Run:

```powershell
& "E:\oss-cad-suite\bin\openocd.exe" `
  -f interface/cmsis-dap.cfg `
  -f target/stm32h7x.cfg `
  -c "program E:/Desktop_E/TCG/Aethor_robo_fw/.worktrees/com7-bench-periodic-resend/aethor_robo_v1/MDK-ARM/CtrBoard-H7_FDCAN/CtrBoard-H7_FDCAN.hex verify reset exit"
```

Expected: programming finished, verified OK, target reset.

### Task 7: Perform staged COM7 verification

**Files:**
- Execute: `Tests/hardware/debug_com7_motors_1_3.ps1`
- Execute: `Tests/hardware/run_com7_output_one_turn_1_3.ps1`

- [ ] **Step 1: Run read-only discovery after reset**

Run the existing debug script without `-RunMotion` for motors `1,3`. Expected: both initialize, driver faults zero, CAN Bus-Off zero.

- [ ] **Step 2: Run the approved three-stage fixed-target sequence**

Run:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File .\Tests\hardware\run_com7_output_one_turn_1_3.ps1 `
  -PortName COM7 -RunMotion
```

The script must complete ID1 `+360 deg`, then ID3 `+360 deg`, then IDs 1 and 3 together at `-360 deg`. Each stage waits for its single fixed-target DONE and completes STOP/DISABLE before the next stage. Stop immediately on unexpected direction, abnormal sound, driver fault, TX error, or Bus-Off.

- [ ] **Step 3: Record software and physical evidence separately**

Report exact ACK/DONE results, elapsed times, final feedback validity, driver faults, CAN overflow/TX error/Bus-Off counters, and the user's marked-shaft observations. Do not claim full one-turn accuracy from serial logs alone.
