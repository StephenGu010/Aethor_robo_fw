# Seven-DOF Joint Control Firmware Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Deliver STM32H723 firmware for a seven-degree-of-freedom arm with explicit DH and drivetrain parameters, joint-angle control, a stable USB CDC command set, and no RGB or inverse-kinematics commands.

**Architecture:** Keep HAL/FDCAN and USB CDC at the hardware boundary, use an allocation-free command parser as the host contract, keep robot geometry and drivetrain data in a read-only configuration module, and run a hardware-independent seven-joint controller from the 5 ms FreeRTOS task. Physical parameters that are not supplied remain explicitly uncommissioned, while host tests use a complete test profile to prove the full seven-axis path.

**Tech Stack:** STM32H723VGT6, STM32CubeMX 6.11.1, STM32CubeH7 V1.11.2, HAL FDCAN classic CAN, CMSIS-RTOS V1/FreeRTOS, USB CDC, C99 host tests, Keil ARMCC 5.

**Execution status (2026-08-12):** Firmware, host tests, Keil build, CMSIS-DAP download, COM7 probes, and the two-motor disabled startup check are complete. The local machine does not contain STM32CubeMX, so code regeneration was not rerun; `.ioc` and generated FDCAN/GPIO files were synchronized manually and compiled. Motor-motion and seven-axis hardware tests remain intentionally unexecuted until physical parameters are commissioned.

---

### Task 1: Establish the Git and configuration baseline

**Files:**
- Create: `User/robot_config.h`
- Create: `User/robot_config.c`
- Test: `Tests/host/test_main.c`
- Modify: `Tests/host/run_tests.ps1`

- [ ] **Step 1: Create a feature branch without resetting the dirty workspace**

Run: `git switch -c feature/7dof-joint-control`

Expected: the current changes remain present and `git branch --show-current` prints `feature/7dof-joint-control`.

- [ ] **Step 2: Write failing configuration and conversion tests**

Add tests for seven entries, unique motor/Master IDs, finite DH fields, rejected invalid configuration, and this conversion contract:

```c
motor_output_deg = direction * (joint_deg - joint_zero_deg) * external_reduction_ratio;
joint_deg = direction * motor_output_deg / external_reduction_ratio + joint_zero_deg;
```

- [ ] **Step 3: Run the host suite and verify the missing configuration API fails compilation**

Run: `powershell.exe -ExecutionPolicy Bypass -File Tests\host\run_tests.ps1`

Expected: FAIL because `robot_config.h` or its declared functions do not exist.

- [ ] **Step 4: Implement the static robot configuration module**

Define these public types and functions:

```c
#define ROBOT_JOINT_COUNT 7U

typedef struct {
    float theta_offset_degrees;
    float d_millimeters;
    float a_millimeters;
    float alpha_degrees;
} RobotDhParameter;

typedef struct {
    uint8_t motor_id;
    uint16_t master_id;
    int8_t direction;
    float joint_zero_degrees;
    float external_reduction_ratio;
    float minimum_degrees;
    float maximum_degrees;
    float maximum_velocity_degrees_s;
    float maximum_acceleration_degrees_s2;
    uint8_t commissioned;
} RobotJointParameter;

typedef struct {
    RobotDhParameter dh[ROBOT_JOINT_COUNT];
    RobotJointParameter joint[ROBOT_JOINT_COUNT];
    uint8_t active_joint_mask;
    uint8_t dh_parameters_valid;
} RobotConfiguration;

const RobotConfiguration *robot_config_get(void);
uint8_t robot_config_validate(const RobotConfiguration *configuration);
int robot_config_joint_to_motor_position_rad(const RobotConfiguration *configuration,
                                             uint8_t joint_index,
                                             float joint_degrees,
                                             float *motor_position_rad);
int robot_config_motor_to_joint_position_degrees(const RobotConfiguration *configuration,
                                                 uint8_t joint_index,
                                                 float motor_position_rad,
                                                 float *joint_degrees);
int robot_config_joint_to_motor_velocity_rad_s(const RobotConfiguration *configuration,
                                               uint8_t joint_index,
                                               float joint_velocity_degrees_s,
                                               float *motor_velocity_rad_s);
```

Production starts with `active_joint_mask=0x03` for the two connected unloaded motors. Geometry and J3-J7 physical data remain marked uncommissioned until measured values are supplied; no guessed mechanical values unlock full enable.

- [ ] **Step 5: Run the host suite**

Expected: `HOST_TESTS_PASSED`.

### Task 2: Complete the seven-axis USB command contract

**Files:**
- Modify: `User/usb_command.h`
- Modify: `User/usb_command.c`
- Test: `Tests/host/test_main.c`

- [ ] **Step 1: Write failing parser tests**

Cover `!START`, `!STOP`, `!DISABLE`, `!HOME`, `#GETJPOS`, `#GETMPOS`, `#GETSTATE`, `#GETENABLE`, `#GETCAPS`, `#GETDH`, `#GETCONFIG`, `#SELECT 1..7`, and both `>` and `&` seven-joint moves with optional speed. Assert that `@...`, RGB commands, NaN/Inf, extra fields, and out-of-range speed are rejected.

- [ ] **Step 2: Run the host suite and verify failures**

Expected: FAIL on the new command types and optional-speed syntax.

- [ ] **Step 3: Implement the allocation-free parser**

Use fixed-size `UsbCommand`, bounded `strtof`/`strtoul`, exact command matching, default speed `100.0F`, and an explicit unsupported-command result for RGB and `@` inverse-kinematics input.

- [ ] **Step 4: Run the host suite**

Expected: `HOST_TESTS_PASSED`.

### Task 3: Refactor the controller around robot configuration

**Files:**
- Modify: `User/joint_controller.h`
- Modify: `User/joint_controller.c`
- Test: `Tests/host/test_main.c`

- [ ] **Step 1: Write failing controller tests with a fully commissioned test configuration**

Test seven unique Master IDs, CTRL_MODE position-speed setup/readback, PMAX/VMAX/TMAX discovery, full enable, joint-to-motor reduction conversion, direction and zero offset, mechanical-limit rejection, synchronized arrival, hold, stop, disable, feedback timeout, motor fault, TX failure, and Bus-Off.

- [ ] **Step 2: Run the host suite and verify failures**

Expected: FAIL because the current controller owns hard-coded placeholder configuration and accepts only Master ID zero.

- [ ] **Step 3: Inject the read-only configuration**

Change initialization to:

```c
void joint_controller_init(JointControllerSendFunction send_function,
                           const RobotConfiguration *configuration,
                           uint32_t time_ms);
```

Keep all runtime storage static. Store only the const configuration pointer; do not allocate or free memory.

- [ ] **Step 4: Apply the conversion boundary and per-joint Master IDs**

Keep trajectories and public commands in joint degrees. Convert only at the motor protocol boundary, and convert feedback back to joint degrees before publishing state.

- [ ] **Step 5: Implement safe startup and command semantics**

Set/read CTRL_MODE=2, discover PMAX/VMAX/TMAX, require matching Master ID/motor ID/RID, allow full enable only for a validated commissioned profile, preserve single-axis commissioning, and implement HOME as a synchronized move to the configured DH theta offsets only after enable.

- [ ] **Step 6: Run the host suite**

Expected: `HOST_TESTS_PASSED`.

### Task 4: Restore the seven-axis application path

**Files:**
- Modify: `User/aethor_application.c`
- Modify: `User/aethor_application.h`
- Modify: `User/bsp_fdcan.h`
- Modify: `User/bsp_fdcan.c`
- Create: `User/firmware_probe.h`
- Create: `User/firmware_probe.c`
- Modify: `User/usb_cdc_transport.h`
- Modify: `User/usb_cdc_transport.c`
- Create: `Tests/hardware/monitor_com7.ps1`
- Test: `Tests/host/test_main.c`

- [ ] **Step 1: Add application policy tests for accepted and rejected commands**

Assert that joint commands reach `joint_controller_submit`, RGB and inverse-kinematics commands return explicit unsupported errors, and diagnostics never mutate targets or enable state.

- [ ] **Step 2: Make FDCAN filtering configuration-driven**

Use seven exact standard-ID filters from `RobotJointParameter.master_id`; reject nonmatching standard frames, extended frames, and remote frames. Keep `FDCAN_CLASSIC_CAN`, `FDCAN_BRS_OFF`, FDCAN1, PD0/PD1, and Bus-Off latching.

- [ ] **Step 3: Switch the 5 ms application service and ISR forwarding to `joint_controller`**

Initialize with `robot_config_get()`, execute parsed commands in task context, keep ISR work limited to frame extraction and forwarding, and keep the previous dual-motor module in the tree but inactive.

- [ ] **Step 4: Add task-context firmware probes**

Expose a bounded USB CDC line-queue API and emit only state changes in this stable format:

```text
probe seq=<n> t=<ms> level=<INFO|WARN|ERROR> event=<name> key=value ...
```

Cover boot/config validation, parameter discovery, ready, enable, moving, holding, command accept/reject, feedback timeout, motor fault, TX failure, and Bus-Off. The FDCAN ISR only updates controller/driver state; `aethor_application_service()` detects and writes transitions.

- [ ] **Step 5: Add a dependency-free COM7 monitor**

Create `Tests/hardware/monitor_com7.ps1` with `-PortName COM7` as the default, optional `-QueryIntervalMs`, automatic `#GETCAPS`/`#GETSTATE` queries, timestamped receive lines, and reliable serial-port disposal.

- [ ] **Step 6: Run the host suite**

Expected: `HOST_TESTS_PASSED`.

### Task 5: Synchronize CubeMX and Keil

**Files:**
- Modify: `CtrBoard-H7_FDCAN.ioc`
- Modify: `Core/Src/fdcan.c`
- Modify: `MDK-ARM/CtrBoard-H7_FDCAN.uvprojx`

- [ ] **Step 1: Set FDCAN1 standard-filter capacity to seven**

Keep CubeMX 6.11.1, FW_H7 V1.11.2, CMSIS-V1, TIM23, USB CDC, PA15, clock tree, and 1 Mbps timing unchanged.

- [ ] **Step 2: Add `robot_config.c` and `firmware_probe.c` to Keil**

Preserve all existing source groups and add one source entry under the existing user-code group.

- [ ] **Step 3: Regenerate using CubeMX 6.11.1 and inspect user-code preservation**

Run the installed CubeMX against `CtrBoard-H7_FDCAN.ioc`, then confirm the seven filter slots, USB middleware, FreeRTOS task, TIM23 timebase, and user source references remain intact.

- [ ] **Step 4: Run a Keil full Rebuild**

Expected: `0 Error(s), 0 Warning(s)` and a newly generated HEX.

### Task 6: Document, audit, and commit

**Files:**
- Create: `CHANGELOG.md`
- Create: `HANDOFF.md`
- Create: `README.md`
- Modify: `.gitignore`

- [ ] **Step 1: Document the protocol and architecture**

List exact commands, units, response forms, unsupported RGB/IK commands, static-memory limits, layer ownership, and the configuration-validity safety gate.

- [ ] **Step 2: Record the hardware handoff boundary**

List the unresolved DH dimensions, J3-J7 Master IDs, direction, zero offsets, external reduction ratios, joint limits, and real commissioning sequence. State that host/Keil success does not prove physical movement.

- [ ] **Step 3: Run the completion audit**

Run host tests, Keil Rebuild, XML file-reference checks, `git diff --check`, and a requirement-to-evidence scan for every named command and deliverable.

- [ ] **Step 4: Create focused Git commits**

Use non-interactive commits on `feature/7dof-joint-control`, for example:

```text
feat: add configurable seven-axis robot model
feat: add seven-axis joint command firmware
docs: add firmware changelog and handoff
```

Do not push, merge, or delete worktrees without an explicit user request.
