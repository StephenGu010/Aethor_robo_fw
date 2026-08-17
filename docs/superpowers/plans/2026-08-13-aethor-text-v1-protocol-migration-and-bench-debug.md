# Aethor Text V1 Protocol Migration and Bench Debug Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the current CRC-heavy `aethor-arm-ascii-v1` wire format with the reviewed `aethor-text-v1` command interface, then build, flash, and verify bounded S3519 bench rotation through COM7.

**Architecture:** Keep the existing fixed-memory `ProtocolEngine`, Arm/Motion/Motor domains, USB CDC transport, command queue, and STOP priority slot. Replace only the text contract at the boundary: the ASCII codec parses optional request IDs, command paths, positional arguments, and `key=value` fields; the engine maps `show/stream/arm/bench` commands onto existing typed domain commands and emits raw LF-terminated `ok/done/error/event/data` lines. The bench profile remains the default and retains the existing 3 degree and 3 degree per second limits.

**Tech Stack:** C11 host tests, ARMCC5/Keil MDK, STM32H723, FreeRTOS, USB CDC, FDCAN1 Classic CAN at 1 Mbps, PowerShell 5.1 hardware tooling, Python simulator tests.

---

### Task 1: Freeze the migration contract and baseline

**Files:**
- Read: `E:/Desktop_E/TCG/固件开发prd/09_上位机串口接口重设计草案_aethor-text-v1.md`
- Read: `aethor_robo_v1/App/Protocol/*.c`
- Read: `aethor_robo_v1/Tests/host/protocol_test_main.c`
- Preserve: `aethor_robo_v1/MDK-ARM/CtrBoard-H7_FDCAN.uvprojx`

- [ ] **Step 1: Record the current repository state**

Run:

```powershell
git status --short
git diff -- aethor_robo_v1/MDK-ARM/CtrBoard-H7_FDCAN.uvprojx
```

Expected: only the existing `<LayerInfo>` addition is present before protocol work.

- [ ] **Step 2: Run the current protocol suite as the old-contract baseline**

Run:

```powershell
powershell.exe -ExecutionPolicy Bypass -File .\Tests\host\run_protocol_tests.ps1
```

Expected: `PROTOCOL_TESTS_PASSED` before new RED tests are added.

### Task 2: Specify and implement the aethor-text-v1 line codec

**Files:**
- Create: `aethor_robo_v1/Tests/host/text_protocol_test_main.c`
- Create: `aethor_robo_v1/Tests/host/run_text_protocol_tests.ps1`
- Modify: `aethor_robo_v1/App/Protocol/protocol_contract.h`
- Create: `aethor_robo_v1/App/Protocol/text_protocol.h`
- Create: `aethor_robo_v1/App/Protocol/text_protocol.c`

- [ ] **Step 1: Replace CRC framing tests with failing text-line tests**

Add a new focused test executable that asserts:

```c
static void test_text_protocol_parses_manual_query(void)
{
    static const char line[] = "show state\r\n";
    TextProtocolRequest request;

    assert(text_protocol_parse_request(line, sizeof(line) - 1U, &request) ==
           TEXT_PROTOCOL_STATUS_OK);
    assert(request.request_id == 0U);
    assert(request.has_request_id == 0U);
    assert(text_protocol_request_path_equals(&request, "show", "state") != 0U);
    assert(request.positional_count == 0U);
}

static void test_text_protocol_parses_numbered_arm_move(void)
{
    static const char line[] =
        "42 arm move 0,-15,30,0,20,0,5 speed=5\n";
    TextProtocolRequest request;
    TextProtocolSpan position_list;
    TextProtocolSpan speed;

    assert(text_protocol_parse_request(line, sizeof(line) - 1U, &request) ==
           TEXT_PROTOCOL_STATUS_OK);
    assert(request.request_id == 42U);
    assert(request.has_request_id != 0U);
    assert(text_protocol_request_path_equals(&request, "arm", "move") != 0U);
    assert(text_protocol_get_positional(&request, 0U, &position_list) != 0U);
    assert(text_protocol_find_field(&request, "speed", &speed) != 0U);
}
```

Also cover LF/CRLF, consecutive spaces, duplicate keys, 160-byte request limit, Tab rejection, scientific notation rejection at typed conversion, and too many positional arguments.

- [ ] **Step 2: Run the suite and verify RED**

Run:

```powershell
powershell.exe -ExecutionPolicy Bypass -File .\Tests\host\run_protocol_tests.ps1
```

Expected: compilation fails because `text_protocol.h` does not exist yet.

- [ ] **Step 3: Implement the minimal bounded parser**

Update the contract to:

```c
#define PROTOCOL_MAX_REQUEST_LINE_LENGTH (160U)
#define PROTOCOL_MAX_RESPONSE_LINE_LENGTH (192U)
#define PROTOCOL_VERSION_TEXT "aethor-text-v1"
```

Define `TextProtocolRequest` with fixed spans for two command words, up to two positional arguments, up to eight key/value fields, and `has_request_id`. Parse in place after copying the line into fixed storage. Do not allocate memory and do not provide CRC-16 framing APIs. Keep the old ASCII parser unchanged until the engine migration is GREEN.

- [ ] **Step 4: Run the protocol suite and verify GREEN**

Run `run_text_protocol_tests.ps1`. Expected: the new codec tests pass while the existing old protocol suite remains GREEN and unchanged.

- [ ] **Step 5: Commit the codec slice**

```powershell
git add aethor_robo_v1/App/Protocol/protocol_contract.h aethor_robo_v1/App/Protocol/text_protocol.h aethor_robo_v1/App/Protocol/text_protocol.c aethor_robo_v1/Tests/host/text_protocol_test_main.c aethor_robo_v1/Tests/host/run_text_protocol_tests.ps1
git commit -m "feat: parse aethor text v1 commands"
```

### Task 3: Migrate hello, ping, replay, and common errors

**Files:**
- Modify: `aethor_robo_v1/Tests/host/protocol_test_main.c`
- Modify: `aethor_robo_v1/App/Protocol/protocol_engine.h`
- Modify: `aethor_robo_v1/App/Protocol/protocol_engine.c`

- [ ] **Step 1: Write failing lifecycle tests**

Specify these exact behaviors:

```c
process("1 hello\n") -> "ok 1 hello protocol=aethor-text-v1 ... profile=bench ...\n"
process("ping\n") -> "ok 0 ping state=disabled enabled=00 boot=...\n"
repeat("7 show state\n") -> byte-identical cached response
conflict("7 show info\n") -> "error 7 show info code=request_conflict\n"
process("nonsense\n") -> "error 0 parse code=unknown_command\n"
```

Verify `hello` clears connection-scoped stream and replay state before storing its own response, and request ID 0 is never cached.

- [ ] **Step 2: Run and verify RED**

Expected: old session/CRC response assertions fail.

- [ ] **Step 3: Implement raw LF response formatting and new dispatch**

`protocol_engine_append_format` writes the supplied body plus `\n` directly, rejects bodies exceeding 192 bytes, and no longer calls the CRC formatter. Replace HELLO/HEARTBEAT dispatch with `hello` and `ping`; remove the session-token gate while preserving `boot_id`, recent-result conflict detection for nonzero IDs, and the 1,000 ms watchdog timer.

- [ ] **Step 4: Run and verify GREEN**

Expected: lifecycle tests pass with lowercase wire tokens.

- [ ] **Step 5: Commit the lifecycle slice**

```powershell
git add aethor_robo_v1/App/Protocol/protocol_engine.h aethor_robo_v1/App/Protocol/protocol_engine.c aethor_robo_v1/Tests/host/protocol_test_main.c
git commit -m "feat: add aethor text lifecycle responses"
```

### Task 4: Migrate read-only show commands and fixed streams

**Files:**
- Modify: `aethor_robo_v1/Tests/host/protocol_test_main.c`
- Modify: `aethor_robo_v1/App/Protocol/protocol_engine.c`
- Modify: `aethor_robo_v1/App/Protocol/protocol_engine.h`

- [ ] **Step 1: Write failing query tests**

Cover `show info`, `show state`, `show joints`, `show motors`, `show motor 1`, `show config`, `show config 1`, and `show diag can`. Assert bitmaps use two lowercase hexadecimal digits and unknown physical values render as `?` rather than numeric zero.

- [ ] **Step 2: Run and verify RED**

Expected: command paths are unknown or responses retain old field arrays.

- [ ] **Step 3: Implement query formatting against existing snapshots**

Use existing `ProtocolQueryContext`; add only derived bitmap helpers and single-joint formatting. `show joints` and all streams read the cached snapshot and must not enqueue CAN work.

- [ ] **Step 4: Specify and implement fixed streams**

Write RED tests for:

```text
3 stream joints rate=20
4 stream motors rate=5
stream off
data 1 joints ...
data 2 motors ...
```

Then implement fixed field sets, 1..50 Hz joint rate, 1..10 Hz motor rate, default off, and drop-old data priority.

- [ ] **Step 5: Run and verify GREEN, then commit**

```powershell
powershell.exe -ExecutionPolicy Bypass -File .\Tests\host\run_protocol_tests.ps1
git add aethor_robo_v1/App/Protocol/protocol_engine.h aethor_robo_v1/App/Protocol/protocol_engine.c aethor_robo_v1/Tests/host/protocol_test_main.c
git commit -m "feat: add compact show and stream commands"
```

### Task 5: Migrate bench commands without changing motor behavior

**Files:**
- Modify: `aethor_robo_v1/Tests/host/protocol_test_main.c`
- Modify: `aethor_robo_v1/App/Protocol/protocol_engine.c`

- [ ] **Step 1: Write failing bench command tests**

Specify:

```text
20 bench init 1,3
21 bench enable 1,3
22 bench jog 1,3 delta=0.2 speed=1
23 bench stop 1,3
24 bench disable 1,3
25 bench clear 1,3
```

`delta` and `speed` are scalar values applied to every selected motor, matching the public interface document. Reject unsorted or duplicate motor lists and values outside the fixed 3 degree and 3 degree-per-second bench limits.

- [ ] **Step 2: Run and verify RED**

Expected: `bench` paths are unknown.

- [ ] **Step 3: Map text requests to existing typed bench commands**

Reuse `PROTOCOL_COMMAND_INIT_MOTORS`, `PROTOCOL_COMMAND_ENABLE`, `PROTOCOL_COMMAND_MOVE_RELATIVE`, `PROTOCOL_COMMAND_STOP`, `PROTOCOL_COMMAND_DISABLE`, and `PROTOCOL_COMMAND_CLEAR_FAULT`. Do not change `MotorRuntime`, CAN frame encoding, periodic target resend, arrival thresholds, or fault policy.

- [ ] **Step 4: Run and verify GREEN, then commit**

```powershell
powershell.exe -ExecutionPolicy Bypass -File .\Tests\host\run_protocol_tests.ps1
git add aethor_robo_v1/App/Protocol/protocol_engine.c aethor_robo_v1/Tests/host/protocol_test_main.c
git commit -m "feat: expose bounded bench text commands"
```

### Task 6: Migrate formal arm commands and fix public mode to POS_VEL

**Files:**
- Modify: `aethor_robo_v1/Tests/host/protocol_test_main.c`
- Modify: `aethor_robo_v1/App/Protocol/protocol_engine.c`

- [ ] **Step 1: Write failing formal command tests**

Cover `arm align`, `arm enable`, `arm move`, `arm stop`, `arm disable`, and `arm clear`. Assert `arm move` has exactly seven degrees values, a positive `speed` in degrees per second, no mode field, and emits a typed motion command fixed to `JOINT_MOTION_MODE_POSITION_VELOCITY`.

- [ ] **Step 2: Run and verify RED**

Expected: old uppercase operations and 0..1 speed ratio remain.

- [ ] **Step 3: Implement the POS_VEL-only public mapping**

Convert degrees to radians at the protocol/domain boundary. Use the requested degrees-per-second value to compute the existing common-duration POS_VEL plan and reject any joint exceeding its verified velocity/acceleration/soft limits. Remove public SET_MODE dispatch but retain internal motor mode verification and switching during initialization.

- [ ] **Step 4: Run and verify GREEN, then commit**

```powershell
powershell.exe -ExecutionPolicy Bypass -File .\Tests\host\run_protocol_tests.ps1
powershell.exe -ExecutionPolicy Bypass -File .\Tests\host\run_motion_tests.ps1
git add aethor_robo_v1/App/Protocol/protocol_engine.c aethor_robo_v1/Tests/host/protocol_test_main.c
git commit -m "feat: expose pos vel arm text commands"
```

### Task 7: Migrate transport fixtures, simulator, and shared vectors

**Files:**
- Modify: `aethor_robo_v1/Tests/host/platform_io_test_main.c`
- Modify: `aethor_robo_v1/Tests/protocol/aethor-arm-ascii-v1-golden.json`
- Modify: `aethor_robo_v1/Tests/protocol/aethor-arm-ascii-v1-compatibility-vectors.json`
- Modify: `aethor_robo_v1/Tests/protocol/generate_golden_header.ps1`
- Modify: `aethor_robo_v1/Tests/protocol/README.md`
- Modify: `aethor_robo_v1/Tools/aethor_host_simulator.py`
- Modify: `aethor_robo_v1/Tools/aethor_reference_client.py`
- Modify: `aethor_robo_v1/Tests/host/test_host_simulator.py`
- Modify: `aethor_robo_v1/Tests/host/test_soak_simulator.py`

- [ ] **Step 1: Update tests first and verify RED**

Replace CRC fixtures with raw lines such as `1 hello\n`, concatenated `2 show state\n3 show joints\n`, and overlength/Tab cases. Simulator expectations use lowercase `ok/done/error/event/data`.

- [ ] **Step 2: Run the platform and simulator suites**

Expected: failures from old CRC encoder and old response prefixes.

- [ ] **Step 3: Implement the simulator/client migration**

Remove CRC encode/decode, parse optional request IDs and command paths, retain byte fragmentation, replay, command lifecycle, watchdog, and logical soak behavior.

- [ ] **Step 4: Run all affected suites and commit**

```powershell
powershell.exe -ExecutionPolicy Bypass -File .\Tests\host\run_platform_io_tests.ps1
powershell.exe -ExecutionPolicy Bypass -File .\Tests\host\run_simulator_tests.ps1
git add aethor_robo_v1/Tests aethor_robo_v1/Tools
git commit -m "test: migrate shared assets to aethor text v1"
```

### Task 8: Migrate COM7 tooling and compatibility documents

**Files:**
- Modify: `aethor_robo_v1/Tests/host/test_com7_dual_motor_debug_script.ps1`
- Modify: `aethor_robo_v1/Tests/hardware/debug_com7_motors_1_3.ps1`
- Modify: `aethor_robo_v1/docs/compatibility/RobotGatewayV1.md`
- Modify: `aethor_robo_v1/docs/compatibility/state-and-command-lifecycle.md`
- Modify: `aethor_robo_v1/docs/compatibility/aethor-arm-ascii-v1-schema.json`
- Modify: `aethor_robo_v1/docs/compatibility/compatibility-result-template.md`

- [ ] **Step 1: Write failing script self-tests**

Assert the script sends raw lines, parses `ok/done/error`, uses `hello`, `ping`, `show`, and `bench`, has no CRC function, defaults to read-only, and retains best-effort `bench stop` then `bench disable`.

- [ ] **Step 2: Run and verify RED**

```powershell
powershell.exe -ExecutionPolicy Bypass -File .\Tests\host\test_com7_dual_motor_debug_script.ps1
```

- [ ] **Step 3: Implement the PowerShell protocol client migration**

Use `"$requestId $command`r`n"`, parse the first four whitespace tokens, wait for `ok ... accepted=1` followed by matching `done`, and send `ping` every 200 ms during enabled stages. Do not close UartAssist or another process automatically.

- [ ] **Step 4: Update compatibility documents to the approved contract**

Rename content and schema fields to `aethor-text-v1`, record no application CRC, the five output types, fixed POS_VEL public mode, and bench/arm Profile split.

- [ ] **Step 5: Run script self-tests and commit**

```powershell
powershell.exe -ExecutionPolicy Bypass -File .\Tests\host\test_com7_dual_motor_debug_script.ps1
git add aethor_robo_v1/Tests/hardware/debug_com7_motors_1_3.ps1 aethor_robo_v1/Tests/host/test_com7_dual_motor_debug_script.ps1 aethor_robo_v1/docs/compatibility
git commit -m "feat: migrate com7 tooling to aethor text v1"
```

### Task 9: Full software verification and Keil build

**Files:**
- Verify: all source and test files
- Preserve: `aethor_robo_v1/MDK-ARM/CtrBoard-H7_FDCAN.uvprojx` user `<LayerInfo>` change

- [ ] **Step 1: Run every host suite in separate PowerShell processes**

```powershell
powershell.exe -ExecutionPolicy Bypass -File .\Tests\host\run_tests.ps1
powershell.exe -ExecutionPolicy Bypass -File .\Tests\host\run_phase0_tests.ps1
powershell.exe -ExecutionPolicy Bypass -File .\Tests\host\run_protocol_tests.ps1
powershell.exe -ExecutionPolicy Bypass -File .\Tests\host\run_motor_core_tests.ps1
powershell.exe -ExecutionPolicy Bypass -File .\Tests\host\run_platform_io_tests.ps1
powershell.exe -ExecutionPolicy Bypass -File .\Tests\host\run_motion_tests.ps1
powershell.exe -ExecutionPolicy Bypass -File .\Tests\host\run_simulator_tests.ps1
powershell.exe -ExecutionPolicy Bypass -File .\Tests\host\check_phase0_architecture.ps1
```

Expected: every named pass marker and no compiler warnings.

- [ ] **Step 2: Rebuild the Keil target**

Run the installed UV4 command against `MDK-ARM/CtrBoard-H7_FDCAN.uvprojx`, wait for the spawned build process, and inspect the generated log.

Expected: `0 Error(s), 0 Warning(s)` and a new HEX containing `aethor-text-v1`.

- [ ] **Step 3: Hash and record the firmware**

```powershell
Get-FileHash -Algorithm SHA256 .\MDK-ARM\CtrBoard-H7_FDCAN\CtrBoard-H7_FDCAN.hex
```

### Task 10: Flash and perform staged COM7 motor rotation

**Files:**
- Use: `aethor_robo_v1/MDK-ARM/CtrBoard-H7_FDCAN/CtrBoard-H7_FDCAN.hex`
- Use: `aethor_robo_v1/Tests/hardware/debug_com7_motors_1_3.ps1`
- Record: `aethor_robo_v1/docs/handoffs/aethor-text-v1-bench/verification.txt`

- [ ] **Step 1: Identify the connected STM32 programmer and COM port**

Run read-only ST-LINK enumeration and verify COM7 is present. Do not flash if multiple ambiguous STM32 targets are connected.

- [ ] **Step 2: Flash and verify the exact HEX**

Use STM32CubeProgrammer CLI with reset-after-program and verification. Expected: programming and verification succeed for the selected STM32H723.

- [ ] **Step 3: Run COM7 read-only protocol and motor discovery**

```powershell
powershell.exe -ExecutionPolicy Bypass -File .\Tests\hardware\debug_com7_motors_1_3.ps1 -PortName COM7 -MotorList 1,3
```

Expected: `READ_ONLY_DEBUG_PASSED`, S3519 identities/ranges/modes valid, no motor enable.

- [ ] **Step 4: Rotate one motor at the minimum bounded command**

```powershell
powershell.exe -ExecutionPolicy Bypass -File .\Tests\hardware\debug_com7_motors_1_3.ps1 -PortName COM7 -MotorList 1 -RunMotion -DeltaDegrees 0.2 -SpeedDegreesPerSecond 1
```

Expected: `MOTION_DEBUG_PASSED`, forward and reverse `done ... result=completed`, final enabled mask zero, motor fault zero, bus-off zero.

- [ ] **Step 5: Rotate the second motor, then the pair**

Repeat with `-MotorList 3`, then `-MotorList 1,3`. Stop immediately on any fault, unexpected direction, abnormal noise, timeout, or communication loss.

- [ ] **Step 6: Record hardware evidence and final repository status**

Record timestamp, COM port, motor IDs, exact commands, transcript, firmware hash, final state, and the boundary that this proves only the selected unloaded bench motors—not seven-axis production motion.
