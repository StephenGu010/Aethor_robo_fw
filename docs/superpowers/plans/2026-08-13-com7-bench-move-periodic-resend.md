# COM7 Bench Move Periodic Resend Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make USB CDC `MOVE_REL` periodically resend the selected S3519 absolute POS_VEL targets until arrival, timeout, or safe stop.

**Architecture:** Keep the existing one-time target calculation and selected-motor frame batch. After a complete batch has been transmitted, `BENCH_MOVE_WAIT` checks the current motor snapshot; if every selected motor is not yet within the existing `0.5 deg` window, reset only the batch read index so the unchanged target batch is emitted again on subsequent 4 ms control cycles.

**Tech Stack:** C11, STM32H723, FreeRTOS/CMSIS-RTOS, Classic CAN, S3519 POS_VEL, PowerShell 5.1, GCC host tests, Keil ARMCC 5, OpenOCD CMSIS-DAP.

---

## File map

- Modify `Tests/host/phase0_test_main.c`: add public-facade regression coverage proving a selected POS_VEL batch repeats while feedback remains away from target.
- Modify `App/aethor_app.c`: reset the consumed selected-motor batch only after the existing arrival check fails.
- Do not modify `MDK-ARM/CtrBoard-H7_FDCAN.uvprojx`; it contains an unrelated user change.

### Task 1: Add the failing bench resend regression

**Files:**
- Modify: `Tests/host/phase0_test_main.c`
- Test: `Tests/host/run_phase0_tests.ps1`

- [ ] **Step 1: Add deterministic frame/request helpers**

Add `#include <math.h>`, `#include "can_frame.h"`, and `#include "s3519_codec.h"`, then add the following file-local helpers before the application-facade tests:

```c
/**
 * @brief Reinterprets one float as a raw little-endian register value.
 * @param value Floating-point register value.
 * @return Bit-identical unsigned register payload.
 */
static uint32_t phase0_float_to_raw_register(float value)
{
    uint32_t raw_value;

    memcpy(&raw_value, &value, sizeof(raw_value));
    return raw_value;
}

/**
 * @brief Supplies a valid discovery value for the requested selected motor.
 * @param register_address Vendor register requested by the application.
 * @param esc_id One-based selected motor identifier.
 * @return Raw response value matching the production mapping.
 */
static uint32_t phase0_discovery_raw_value(uint8_t register_address,
                                           uint8_t esc_id)
{
    switch ((S3519Register)register_address)
    {
        case S3519_REGISTER_ACCELERATION:
            return phase0_float_to_raw_register(30.0F);
        case S3519_REGISTER_DECELERATION:
            return phase0_float_to_raw_register(-25.0F);
        case S3519_REGISTER_MAXIMUM_SPEED:
            return phase0_float_to_raw_register(20.0F);
        case S3519_REGISTER_MASTER_ID:
            return (uint32_t)(esc_id + 0x10U);
        case S3519_REGISTER_ESC_ID:
            return esc_id;
        case S3519_REGISTER_CONTROL_MODE:
            return 2U;
        case S3519_REGISTER_HARDWARE_VERSION:
            return 0x00010002U;
        case S3519_REGISTER_SOFTWARE_VERSION:
            return 0x00030004U;
        case S3519_REGISTER_SUB_VERSION:
            return 0x00000005U;
        case S3519_REGISTER_POSITION_RANGE:
            return phase0_float_to_raw_register(12.5F);
        case S3519_REGISTER_VELOCITY_RANGE:
            return phase0_float_to_raw_register(45.0F);
        case S3519_REGISTER_TORQUE_RANGE:
            return phase0_float_to_raw_register(18.0F);
        default:
            assert(0);
            return 0U;
    }
}

/**
 * @brief Formats and submits one CRC-protected application request.
 * @param body Request body without CRC suffix.
 * @param timestamp_us Monotonic request time.
 */
static void phase0_submit_request(const char *body, uint64_t timestamp_us)
{
    char frame[256];
    size_t frame_length = 0U;
    ProtocolOutputBatch output_batch;

    assert(ascii_protocol_format_frame(body,
                                       strlen(body),
                                       frame,
                                       sizeof(frame),
                                       &frame_length) == ASCII_PROTOCOL_STATUS_OK);
    assert(aethor_app_process_protocol_line(frame,
                                            frame_length,
                                            timestamp_us,
                                            &output_batch) ==
           PROTOCOL_ENGINE_STATUS_OK);
}

/**
 * @brief Feeds one centered S3519 feedback frame with an explicit driver state.
 * @param esc_id One-based motor identifier.
 * @param driver_state S3519 enabled or disabled state nibble.
 * @param timestamp_us Monotonic receive time.
 */
static void phase0_feed_feedback(uint8_t esc_id,
                                 uint8_t driver_state,
                                 uint64_t timestamp_us)
{
    uint8_t payload[8] = {
        (uint8_t)((driver_state << 4U) | esc_id),
        0x80U, 0x00U, 0x80U, 0x08U, 0x00U, 35U, 27U
    };
    CanFrame frame;

    assert(can_frame_init(&frame,
                          (uint16_t)(esc_id + 0x10U),
                          payload,
                          sizeof(payload)) == CAN_FRAME_STATUS_OK);
    assert(aethor_app_receive_can_frame(&frame, timestamp_us) ==
           MOTOR_RUNTIME_STATUS_OK);
}
```

- [ ] **Step 2: Add selected discovery and mode-switch setup**

Add a helper that starts `INIT_MOTORS motors=1,3`, answers every emitted parameter read, answers POS_VEL mode readbacks, and services the action until its terminal result is available:

```c
/**
 * @brief Completes selected motor discovery and POS_VEL mode readback.
 * @param timestamp_us Mutable monotonic timestamp used by the setup.
 */
static void phase0_initialize_selected_motors(uint64_t *timestamp_us)
{
    ProtocolOutputBatch output_batch;
    uint16_t response_index;

    phase0_submit_request("REQ 2 INIT_MOTORS motors=1,3", *timestamp_us);
    ++(*timestamp_us);
    (void)aethor_app_service(*timestamp_us);
    for (response_index = 0U;
         response_index < (uint16_t)(2U * MOTOR_DISCOVERY_REGISTER_COUNT);
         ++response_index)
    {
        CanFrame request;
        CanFrame response;
        CanTxPriority priority;
        uint8_t payload[8] = {0U};
        uint8_t esc_id;
        uint32_t raw_value;

        ++(*timestamp_us);
        assert(aethor_app_next_can_frame(*timestamp_us, &request, &priority) ==
               MOTOR_RUNTIME_STATUS_FRAME_READY);
        assert(priority == CAN_TX_PRIORITY_PARAMETER);
        esc_id = request.data[0];
        raw_value = phase0_discovery_raw_value(request.data[3], esc_id);
        payload[0] = esc_id;
        payload[2] = 0x33U;
        payload[3] = request.data[3];
        payload[4] = (uint8_t)(raw_value & 0xFFU);
        payload[5] = (uint8_t)((raw_value >> 8U) & 0xFFU);
        payload[6] = (uint8_t)((raw_value >> 16U) & 0xFFU);
        payload[7] = (uint8_t)((raw_value >> 24U) & 0xFFU);
        assert(can_frame_init(&response,
                              (uint16_t)(esc_id + 0x10U),
                              payload,
                              sizeof(payload)) == CAN_FRAME_STATUS_OK);
        assert(aethor_app_receive_can_frame(&response, *timestamp_us) ==
               MOTOR_RUNTIME_STATUS_OK);
        (void)aethor_app_service(*timestamp_us);
    }
    for (response_index = 0U; response_index < 4U; ++response_index)
    {
        CanFrame request;
        CanTxPriority priority;

        ++(*timestamp_us);
        assert(aethor_app_next_can_frame(*timestamp_us, &request, &priority) ==
               MOTOR_RUNTIME_STATUS_FRAME_READY);
        if (request.data[2] == 0x33U)
        {
            uint8_t payload[8] = {request.data[0], 0U, 0x33U,
                                  S3519_REGISTER_CONTROL_MODE,
                                  2U, 0U, 0U, 0U};
            CanFrame response;

            assert(can_frame_init(&response,
                                  (uint16_t)(request.data[0] + 0x10U),
                                  payload,
                                  sizeof(payload)) == CAN_FRAME_STATUS_OK);
            MotorRuntimeStatus response_status =
                aethor_app_receive_can_frame(&response, *timestamp_us);

            assert((response_status == MOTOR_RUNTIME_STATUS_OK) ||
                   (response_status == MOTOR_RUNTIME_STATUS_ACTION_COMPLETE));
        }
        (void)aethor_app_service(*timestamp_us);
    }
    assert(aethor_app_pop_protocol_result_output(&output_batch) == 1U);
}
```

- [ ] **Step 3: Add the failing public-facade regression**

```c
/**
 * @brief Verifies an unfinished bench move repeats its exact selected target batch.
 */
static void test_aethor_app_repeats_unfinished_bench_target_batch(void)
{
    ProtocolOutputBatch output_batch;
    CanFrame first_target;
    CanFrame second_target;
    CanFrame repeated_target;
    CanTxPriority priority;
    uint64_t timestamp_us = 1000U;

    aethor_app_init(timestamp_us, 9999U);
    (void)aethor_app_service(++timestamp_us);
    (void)aethor_app_service(++timestamp_us);
    phase0_submit_request("REQ 1 HELLO client=bench-resend protocol=1",
                          ++timestamp_us);
    phase0_initialize_selected_motors(&timestamp_us);

    phase0_submit_request("REQ 3 ENABLE motors=1,3", ++timestamp_us);
    (void)aethor_app_service(++timestamp_us);
    assert(aethor_app_next_can_frame(++timestamp_us, &first_target, &priority) ==
           MOTOR_RUNTIME_STATUS_FRAME_READY);
    phase0_feed_feedback(1U, S3519_DRIVER_STATE_ENABLED, ++timestamp_us);
    (void)aethor_app_service(++timestamp_us);
    assert(aethor_app_next_can_frame(++timestamp_us, &second_target, &priority) ==
           MOTOR_RUNTIME_STATUS_FRAME_READY);
    phase0_feed_feedback(3U, S3519_DRIVER_STATE_ENABLED, ++timestamp_us);
    assert(aethor_app_service(++timestamp_us) == 1U);
    assert(aethor_app_pop_protocol_result_output(&output_batch) == 1U);

    phase0_submit_request(
        "REQ 4 MOVE_REL motors=1,3 delta_deg=3.0,-3.0 speed_deg_s=1.0,1.0",
        ++timestamp_us);
    (void)aethor_app_service(++timestamp_us);
    assert(aethor_app_next_can_frame(++timestamp_us, &first_target, &priority) ==
           MOTOR_RUNTIME_STATUS_FRAME_READY);
    (void)aethor_app_service(++timestamp_us);
    assert(aethor_app_next_can_frame(++timestamp_us, &second_target, &priority) ==
           MOTOR_RUNTIME_STATUS_FRAME_READY);
    (void)aethor_app_service(++timestamp_us);

    assert(aethor_app_next_can_frame(++timestamp_us,
                                     &repeated_target,
                                     &priority) ==
           MOTOR_RUNTIME_STATUS_FRAME_READY);
    assert(repeated_target.identifier == first_target.identifier);
    assert(repeated_target.length == first_target.length);
    assert(memcmp(repeated_target.data,
                  first_target.data,
                  first_target.length) == 0);
}
```

Call `test_aethor_app_repeats_unfinished_bench_target_batch();` from `main()` before the link-timeout test.

- [ ] **Step 4: Run the regression and verify RED**

Run:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "Tests\host\run_phase0_tests.ps1"
```

Expected: the new assertion requesting `repeated_target` fails because the current implementation has consumed the two selected target frames and returns no frame ready.

### Task 2: Implement the minimal periodic resend

**Files:**
- Modify: `App/aethor_app.c:1218`
- Test: `Tests/host/run_phase0_tests.ps1`

- [ ] **Step 1: Reset only the completed bench target batch**

After the existing `all_selected_at_target` completion block and before leaving the `BENCH_MOVE_WAIT` branch, add:

```c
        application_action.frame_read_index = 0U;
```

This preserves the fixed target payload, selection, timeout, and arrival logic. No new function or state field is required.

- [ ] **Step 2: Run the regression and verify GREEN**

Run:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "Tests\host\run_phase0_tests.ps1"
```

Expected:

```text
PHASE0_TESTS_PASSED
PRODUCTION_PROFILE_TEST_PASSED
```

- [ ] **Step 3: Inspect the focused diff and commit the slice**

Run:

```powershell
git diff --check -- "aethor_robo_v1/Tests/host/phase0_test_main.c" "aethor_robo_v1/App/aethor_app.c"
git add -- "aethor_robo_v1/Tests/host/phase0_test_main.c" "aethor_robo_v1/App/aethor_app.c"
git commit -m "fix: repeat unfinished COM7 bench targets"
```

Expected: only the regression test and one production behavior line are committed; the user-modified Keil project file remains unstaged.

### Task 3: Run complete software verification

**Files:**
- Verify only; no source change.

- [ ] **Step 1: Run every host suite in isolated PowerShell processes**

Run `run_tests.ps1`, `run_phase0_tests.ps1`, `run_protocol_tests.ps1`, `run_motor_core_tests.ps1`, `run_platform_io_tests.ps1`, `run_motion_tests.ps1`, `run_simulator_tests.ps1`, `check_phase0_architecture.ps1`, and `test_com7_dual_motor_debug_script.ps1` from `Tests/host`.

Expected: all named pass markers, nine simulator tests with `OK`, and `COM7_DEBUG_SCRIPT_TESTS_PASSED`.

- [ ] **Step 2: Verify repository hygiene**

Run:

```powershell
git diff --check
git status --short --branch
```

Expected: no whitespace errors; only the pre-existing `MDK-ARM/CtrBoard-H7_FDCAN.uvprojx` modification remains unstaged.

### Task 4: Rebuild, verify, and flash firmware

**Files:**
- Rebuild: `MDK-ARM/CtrBoard-H7_FDCAN.uvprojx`
- Flash: `MDK-ARM/CtrBoard-H7_FDCAN/CtrBoard-H7_FDCAN.hex`

- [ ] **Step 1: Run a Keil full rebuild**

Run:

```powershell
& "E:\keil\UV4\UV4.exe" -r "MDK-ARM\CtrBoard-H7_FDCAN.uvprojx" -j0
```

Inspect `MDK-ARM/first-arm-final-rebuild.log`. Expected: `0 Error(s), 0 Warning(s)` and a newly written HEX.

- [ ] **Step 2: Record artifact hashes**

Run:

```powershell
Get-FileHash -Algorithm SHA256 "MDK-ARM\CtrBoard-H7_FDCAN\CtrBoard-H7_FDCAN.hex"
Get-FileHash -Algorithm SHA256 "MDK-ARM\CtrBoard-H7_FDCAN\CtrBoard-H7_FDCAN.map"
```

Expected: both files exist and have non-empty SHA-256 values.

- [ ] **Step 3: Flash with CMSIS-DAP and verify**

Run:

```powershell
& "E:\oss-cad-suite\bin\openocd.exe" `
  -f interface/cmsis-dap.cfg `
  -f target/stm32h7x.cfg `
  -c "program E:/Desktop_E/TCG/Aethor_robo_fw/aethor_robo_v1/MDK-ARM/CtrBoard-H7_FDCAN/CtrBoard-H7_FDCAN.hex verify reset exit"
```

Expected: OpenOCD reports programming and verification success, then resets the target.

### Task 5: Perform staged COM7 hardware verification

**Files:**
- Execute: `Tests/hardware/debug_com7_motors_1_3.ps1`

- [ ] **Step 1: Confirm read-only discovery after reset**

Run without `-RunMotion` for `MotorList "1,3"`. Expected: `READ_ONLY_DEBUG_PASSED`, Master IDs `0x11/0x13` remain discoverable, motors are disabled, and driver faults are zero.

- [ ] **Step 2: Test CAN ID 1 only**

Run with `-MotorList "1" -RunMotion -DeltaDegrees 3 -SpeedDegreesPerSecond 1`.

Expected: visible out/back movement and `MOTION_DEBUG_PASSED port=COM7 motors=1`; final STOP/DISABLE, driver fault zero, CAN Bus-Off zero.

- [ ] **Step 3: Test CAN ID 3 only**

Run with `-MotorList "3" -RunMotion -DeltaDegrees 3 -SpeedDegreesPerSecond 1`.

Expected: visible out/back movement and `MOTION_DEBUG_PASSED port=COM7 motors=3`; final STOP/DISABLE, driver fault zero, CAN Bus-Off zero.

- [ ] **Step 4: Test CAN ID 1 and 3 together**

Run with `-MotorList "1,3" -RunMotion -DeltaDegrees 3 -SpeedDegreesPerSecond 1`.

Expected: ID 1 moves `+3/-3 deg`, ID 3 moves `-3/+3 deg`, both return to their starts, and the script reports `MOTION_DEBUG_PASSED port=COM7 motors=1,3` before final disable.

- [ ] **Step 5: Stop on any discrepancy**

If any stage reports failure, unexpected direction, abnormal sound, stalling, driver fault, or Bus-Off, do not run the next stage. Preserve the serial transcript and return to root-cause investigation. Do not raise the angle limit in this implementation slice.
