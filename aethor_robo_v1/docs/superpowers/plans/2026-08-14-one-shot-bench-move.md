# Aethor One-Shot Bench Move Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在 `USB_BENCH_RELATIVE` 配置中实现一条 `bench move` 串口命令，使固件基于本次启动发现的 S3519 实际范围，独立控制所选电机的绝对位置和速度，并在无周期 `ping` 的情况下完成发现、使能、运动、停止和失能。

**Architecture:** 保持现有固定内存、单动作对象和 `Protocol -> AethorApp -> MotorRuntime -> S3519Codec` 分层。`ProtocolEngine` 只解析严格的一一对应列表并格式化稳定结果；`MotorRuntime` 成为 `PMAX/VMAX/MAX_SPD` 的唯一 POS_VEL 准入点；`AethorApp` 拥有自包含复合状态机、内部目标重发、失败清理和局部看门狗豁免。旧 `bench jog` 继续存在，但与新命令共用动态范围规则；生产配置和全局安全故障路径不改变。

**Tech Stack:** C11 host tests, ARMCC5/Keil MDK, STM32H723, FreeRTOS/CMSIS-RTOS, USB CDC, FDCAN1 Classic CAN, S3519 POS_VEL, PowerShell 5.1, Python protocol simulators.

---

## 执行约束

- 所有命令均从 `E:\Desktop_E\TCG\Aethor_robo_fw\aethor_robo_v1` 执行，除非步骤明确说明从仓库根目录执行。
- 不修改或暂存 `MDK-ARM/CtrBoard-H7_FDCAN.uvprojx`；其中现存 `<LayerInfo>` 是用户本地变更。
- 不硬编码 S3519 SDK 默认范围，也不使用额定转速替代本次启动发现的寄存器值。
- 所有新增 C 文件内容、结构体、枚举和函数均添加文件级、类型级和函数级注释；沿用项目现有 Doxygen 风格。
- 每个任务先写会失败的测试并记录 RED，再实现最小代码并记录 GREEN；不得把多个任务堆到一个提交后再补测试。
- 不以实机运动到 `PMAX` 或 `MAX_SPD` 证明边界；最大边界由主机测试和运行时寄存器值证明。

### Task 1: 固定基线和工作树边界

**Files:**
- Read: `App/Config/app_profile.h`
- Read: `App/Motor/motor_discovery.h`
- Read: `App/Motor/motor_runtime.h`
- Read: `App/Protocol/protocol_engine.h`
- Read: `App/aethor_app.c`
- Preserve: `MDK-ARM/CtrBoard-H7_FDCAN.uvprojx`

- [ ] **Step 1: 记录分支和现有用户改动**

Run from repository root:

```powershell
git status --short --branch
git diff -- aethor_robo_v1/MDK-ARM/CtrBoard-H7_FDCAN.uvprojx
git log -3 --oneline --decorate
```

Expected: 当前分支为 `feature/one-shot-bench-move`；只有设计/计划文档提交和未暂存的 `<LayerInfo>` 用户改动，没有未知代码差异。

- [ ] **Step 2: 运行当前软件基线**

```powershell
powershell.exe -ExecutionPolicy Bypass -File .\Tests\host\run_motor_core_tests.ps1
powershell.exe -ExecutionPolicy Bypass -File .\Tests\host\run_text_protocol_engine_tests.ps1
powershell.exe -ExecutionPolicy Bypass -File .\Tests\host\run_phase0_tests.ps1
```

Expected: 三组当前测试通过；这只证明改动前基线，不证明新命令存在。

### Task 2: 在 MotorRuntime 建立唯一的动态 POS_VEL 范围合同

**Files:**
- Modify: `Tests/host/motor_core_test_main.c`
- Modify: `App/Motor/motor_runtime.h`
- Modify: `App/Motor/motor_runtime.c`

- [ ] **Step 1: 写动态能力和全有或全无校验的失败测试**

在测试中用已发现的两台电机构造不同的 `PMAX/VMAX/MAX_SPD`，覆盖：未发现、`90°` 在范围内、恰好等于边界、位置越界、速度越界、非有限值、零/负速度，以及第二台电机失败时第一台也不生成帧。

核心断言：

```c
MotorPositionVelocityLimits limits;
uint8_t failed_joint_index = UINT8_MAX;

assert(motor_runtime_get_position_velocity_limits(&runtime, 0U, &limits) ==
       MOTOR_RUNTIME_STATUS_OK);
assert(limits.position_max_rad == 12.5F);
assert(limits.velocity_mapping_max_rad_s == 45.0F);
assert(limits.maximum_speed_rad_s == 20.0F);
assert(limits.move_speed_limit_rad_s == 20.0F);

positions[0] = 1.57079632679F;
speeds[0] = 0.52359877559F;
assert(motor_runtime_validate_position_velocity_move_subset(
           &runtime,
           &feedback_snapshot,
           0x01U,
           positions,
           speeds,
           &failed_joint_index) ==
       MOTOR_RUNTIME_STATUS_OK);
```

另加 `positions[0] = 12.5F` 和 `speeds[0] = 20.0F` 的闭区间成功断言；将任一值增加一个明确 epsilon 后分别得到 `POSITION_OUT_OF_RANGE` 和 `SPEED_OUT_OF_RANGE`。

- [ ] **Step 2: 运行并确认 RED**

```powershell
powershell.exe -ExecutionPolicy Bypass -File .\Tests\host\run_motor_core_tests.ps1
```

Expected: 因新类型/函数未定义而编译失败，或新增边界断言失败；不得通过放宽测试绕过 RED。

- [ ] **Step 3: 定义范围值和明确状态**

在 `motor_runtime.h` 中加入：

```c
/** @brief Reports the discovered POS_VEL limits for one motor. */
typedef struct
{
    float position_max_rad;
    float velocity_mapping_max_rad_s;
    float maximum_speed_rad_s;
    float move_speed_limit_rad_s;
} MotorPositionVelocityLimits;
```

向 `MotorRuntimeStatus` 追加 `MOTOR_RUNTIME_STATUS_POSITION_OUT_OF_RANGE`、`MOTOR_RUNTIME_STATUS_SPEED_OUT_OF_RANGE` 和 `MOTOR_RUNTIME_STATUS_FAULT_PRESENT`，并声明：

```c
MotorRuntimeStatus motor_runtime_get_position_velocity_limits(
    const MotorRuntime *runtime,
    uint8_t joint_index,
    MotorPositionVelocityLimits *limits);

MotorRuntimeStatus motor_runtime_validate_position_velocity_move_subset(
    const MotorRuntime *runtime,
    const MotorFeedbackSnapshot *feedback_snapshot,
    uint8_t motor_mask,
    const float motor_position_rad[ARM_JOINT_COUNT],
    const float motor_speed_rad_s[ARM_JOINT_COUNT],
    uint8_t *failed_joint_index);
```

- [ ] **Step 4: 实现单一范围来源和原子批次准入**

`motor_runtime_get_position_velocity_limits()` 必须要求目标电机本次启动的发现字段完整，检查 `PMAX`、`VMAX`、`MAX_SPD` 均为有限正数，并计算：

```c
limits->move_speed_limit_rad_s =
    fminf(result->ranges.velocity_max_rad_s,
          result->maximum_speed_rad_s);
```

`motor_runtime_validate_position_velocity_move_subset()` 按电机编号顺序检查：

```c
fabsf(position_rad) <= position_max_rad
speed_rad_s > 0.0F
speed_rad_s <= move_speed_limit_rad_s
```

同时要求 `feedback_snapshot->valid_joint_mask` 包含全部所选电机，且每台反馈的 `fault_flags == 0U`；过期反馈由调用方生成快照时的现有 freshness 窗口排除。任一失败时设置第一个失败的零基 joint index 并返回稳定状态。

为避免 HOLD 的零速度与运动命令的正速度规则混淆，`motor_runtime_build_position_velocity_subset()` 继续作为通用 POS_VEL 编码器，允许零速度；但它必须复用同一个私有“范围与有限值”helper，在局部临时批次中完成全部编码后才整体复制。接受新 move/jog 之前必须显式调用公共 move validator；HOLD 只调用通用编码器。任何失败都清零输出批次，不产生部分帧。

- [ ] **Step 5: 运行 GREEN 并提交**

```powershell
powershell.exe -ExecutionPolicy Bypass -File .\Tests\host\run_motor_core_tests.ps1
git diff --check
git add App/Motor/motor_runtime.h App/Motor/motor_runtime.c Tests/host/motor_core_test_main.c
git commit -m "feat: validate discovered motor motion limits"
```

Expected: `MOTOR_CORE_TESTS_PASSED`，且提交不包含 Keil 工程文件。

### Task 3: 在 show motor 中公开本次启动的动态能力

**Files:**
- Modify: `Tests/host/text_protocol_engine_test_main.c`
- Modify: `App/Protocol/protocol_engine.h`
- Modify: `App/Protocol/protocol_engine.c`
- Modify: `App/aethor_app.c`

- [ ] **Step 1: 写查询输出的失败测试**

扩展 `make_query_context()`，分别构造已发现与未发现电机。断言已发现输出含：

```text
pmax_deg=716.197 vmax_deg_s=2578.31 max_speed_deg_s=1145.92 move_speed_limit_deg_s=1145.92
```

未发现时四个字段均为 `?`。测试使用包含关系而不是绑定其它既有反馈字段的完整字符串。

- [ ] **Step 2: 运行并确认 RED**

```powershell
powershell.exe -ExecutionPolicy Bypass -File .\Tests\host\run_text_protocol_engine_tests.ps1
```

Expected: 新字段缺失导致断言失败。

- [ ] **Step 3: 扩展固定查询快照，不引入跨层指针**

向 `ProtocolQueryContext` 加入固定数组：

```c
float motor_position_max_rad[ARM_JOINT_COUNT];
float motor_velocity_max_rad_s[ARM_JOINT_COUNT];
float motor_maximum_speed_rad_s[ARM_JOINT_COUNT];
uint8_t motor_motion_limits_valid_mask;
```

`aethor_app_update_protocol_context()` 对每个 joint 调用 `motor_runtime_get_position_velocity_limits()`；成功才复制数值并设置 valid bit，失败保留零且不设置 bit。`protocol_engine_handle_text_show_motor()` 根据 valid bit 输出数值或 `?`，并把弧度转换为度。

- [ ] **Step 4: 运行 GREEN 并提交**

```powershell
powershell.exe -ExecutionPolicy Bypass -File .\Tests\host\run_text_protocol_engine_tests.ps1
powershell.exe -ExecutionPolicy Bypass -File .\Tests\host\run_phase0_tests.ps1
git diff --check
git add App/Protocol/protocol_engine.h App/Protocol/protocol_engine.c App/aethor_app.c Tests/host/text_protocol_engine_test_main.c
git commit -m "feat: report discovered motor move limits"
```

Expected: 查询测试和 Phase 0 回归均通过。

### Task 4: 解析严格的一一对应 bench move 列表

**Files:**
- Modify: `Tests/host/text_protocol_engine_test_main.c`
- Modify: `Tests/host/phase0_test_main.c`
- Modify: `App/Protocol/protocol_engine.h`
- Modify: `App/Protocol/protocol_engine.c`
- Modify: `App/Config/app_profile.h`

- [ ] **Step 1: 写新命令和兼容性失败测试**

覆盖以下输入：

```text
50 bench move 1 position=90 speed=30
51 bench move 1,3 position=90,-45 speed=30,20
58 bench move 3,1 position=-45,90 speed=20,30
52 bench move 1,3 position=90 speed=30,20
53 bench move 1,3 position=90,-45 speed=30
54 bench move 1,1 position=10,20 speed=1,1
55 bench move 1 position=nan speed=1
56 bench move 1 position=1 speed=0
57 bench jog 1 delta=90 speed=30
```

成功命令断言 `values[]` 和 `speeds[]` 按用户给出的唯一电机顺序写入对应 joint；`3,1` 必须保持 `position[0] -> motor 3`、`position[1] -> motor 1` 的对应关系。数量不匹配返回 `count_mismatch`；重复电机、NaN、空 token、零/负速度返回明确 `error`。将原 `3.1°` 的协议层拒绝测试改为“解析接受，运行时再基于动态范围决定”。另测相同非零 request ID 与相同正文只重放响应、不重复入队；相同 ID 不同正文返回 `request_conflict`。

- [ ] **Step 2: 运行并确认 RED**

```powershell
powershell.exe -ExecutionPolicy Bypass -File .\Tests\host\run_text_protocol_engine_tests.ps1
```

Expected: `bench move` 为未知命令，且旧 `bench jog` 仍拒绝大于 3 的值。

- [ ] **Step 3: 实现列表解析和新业务命令类型**

在 `ProtocolCommandType` 增加：

```c
PROTOCOL_COMMAND_MOVE_ABSOLUTE_SELF_CONTAINED,
```

新增 text-protocol 专用 helper，复用现有有限 float 转换，不在 `TextProtocol` 低层加入电机知识。新命令不能复用只返回 mask 的升序 parser，因为位置/速度列表必须跟随用户输入顺序：

```c
static uint8_t protocol_engine_parse_text_motor_list(
    const TextProtocolSpan *span,
    uint8_t *motor_mask,
    uint8_t joint_order[ARM_JOINT_COUNT],
    uint8_t *motor_count);

static uint8_t protocol_engine_parse_text_selected_motor_values(
    const TextProtocolSpan *span,
    const uint8_t joint_order[ARM_JOINT_COUNT],
    uint8_t motor_count,
    float values[ARM_JOINT_COUNT]);
```

motor list helper 接受 `1..7` 的任意顺序、拒绝重复并同时保留 order 与 mask；value helper 必须恰好消费 `motor_count` 个元素，按 order 写入 joint-indexed 数组，不广播单值。现有生命周期命令和 `bench jog` 的 motor 语法保持不变。`protocol_engine_text_bench_command_type()` 映射 `bench move`，并生成：

```text
ok <id> bench move accepted=1
```

解析阶段只检查结构、有限值和 `speed > 0`，不读取或猜测硬件范围。

- [ ] **Step 4: 删除固定 3° 解析限制但保留动态安全边界**

删除 `AETHOR_BENCH_MAX_RELATIVE_DEGREES` 和 `AETHOR_BENCH_MAX_SPEED_DEGREES_S` 的协议依赖及已无用途的宏。`bench jog` 仍把 scalar 广播给所选电机，但只做有限值和正速度检查；其绝对目标在应用层生成后必须走 Task 2 的动态校验。

- [ ] **Step 5: 运行 GREEN 并提交**

```powershell
powershell.exe -ExecutionPolicy Bypass -File .\Tests\host\run_text_protocol_engine_tests.ps1
powershell.exe -ExecutionPolicy Bypass -File .\Tests\host\run_phase0_tests.ps1
git diff --check
git add App/Config/app_profile.h App/Protocol/protocol_engine.h App/Protocol/protocol_engine.c Tests/host/text_protocol_engine_test_main.c Tests/host/phase0_test_main.c
git commit -m "feat: parse strict one shot bench moves"
```

Expected: 协议测试通过；旧命令路径仍存在；没有旧 ASCII 协议的新别名。

### Task 5: 定义可稳定格式化的动作阶段和失败码

**Files:**
- Modify: `Tests/host/text_protocol_engine_test_main.c`
- Modify: `App/Protocol/protocol_engine.h`
- Modify: `App/Protocol/protocol_engine.c`

- [ ] **Step 1: 写终态输出失败测试**

为成功、校验失败、发现失败、运动超时和失能反馈超时构造 `ProtocolCommandResult`，断言：

```text
done 50 bench move result=completed elapsed_ms=3200 motors=01
done 50 bench move result=failed stage=validate code=position_out_of_range motor=1
done 50 bench move result=failed stage=discovery code=not_ready motor=1
done 50 bench move result=failed stage=motion code=timeout motor=1
done 50 bench move result=failed stage=disable code=feedback_timeout motor=1
```

- [ ] **Step 2: 运行并确认 RED**

```powershell
powershell.exe -ExecutionPolicy Bypass -File .\Tests\host\run_text_protocol_engine_tests.ps1
```

Expected: 新类型尚不能格式化为上述稳定字段。

- [ ] **Step 3: 加入固定枚举，不用自由文本穿过任务边界**

在 `protocol_engine.h` 定义类型级注释完整的 `ProtocolCommandStage` 和 `ProtocolCommandError`，至少包含 `VALIDATE/DISCOVERY/MODE/CLEAR/ENABLE/MOTION/HOLD/DISABLE` 以及 `NONE/NOT_READY/POSITION_OUT_OF_RANGE/SPEED_OUT_OF_RANGE/FAULT_PRESENT/STALE_FEEDBACK/TIMEOUT/FEEDBACK_TIMEOUT/ACTION_FAILED`。向 `ProtocolCommandResult` 加入：

```c
ProtocolCommandStage stage;
ProtocolCommandError error;
uint8_t failed_motor_number;
```

格式化函数将枚举映射为稳定小写 token；成功结果不输出失败字段，失败结果不得用易变的内部 `detail` 替代公共错误码。

- [ ] **Step 4: 运行 GREEN 并提交**

```powershell
powershell.exe -ExecutionPolicy Bypass -File .\Tests\host\run_text_protocol_engine_tests.ps1
git diff --check
git add App/Protocol/protocol_engine.h App/Protocol/protocol_engine.c Tests/host/text_protocol_engine_test_main.c
git commit -m "feat: report one shot move stages"
```

### Task 6: 建立发现、范围、模式、清故障和使能的复合前半程

**Files:**
- Modify: `Tests/host/phase0_test_main.c`
- Modify: `App/aethor_app.c`

- [ ] **Step 1: 写冷启动复合动作的失败测试**

新增 helper 逐帧响应现有发现和 POS_VEL 模式读回。测试从未初始化的应用状态提交：

```text
50 bench move 1 position=90 speed=30
```

断言顺序为：先出现只读参数请求；完成发现前无 enable/POS_VEL 目标；随后为模式切换、清故障、使能。再构造电机 2 已发现而电机 1 未发现的场景，断言只发现缺失子集且最终仍只操作请求 mask。

- [ ] **Step 2: 运行并确认 RED**

```powershell
powershell.exe -ExecutionPolicy Bypass -File .\Tests\host\run_phase0_tests.ps1
```

Expected: 新命令无法启动所需状态序列。

- [ ] **Step 3: 扩展固定动作对象和显式状态**

在 `AethorAppActionState` 加入自包含阶段：

```c
AETHOR_APP_ACTION_ONE_SHOT_DISCOVERY,
AETHOR_APP_ACTION_ONE_SHOT_MODE_SWITCH,
AETHOR_APP_ACTION_ONE_SHOT_CLEAR_WAIT,
AETHOR_APP_ACTION_ONE_SHOT_ENABLE_WAIT,
AETHOR_APP_ACTION_ONE_SHOT_MOVE_WAIT,
AETHOR_APP_ACTION_ONE_SHOT_HOLD_WAIT,
AETHOR_APP_ACTION_ONE_SHOT_DISABLE_WAIT,
AETHOR_APP_ACTION_ONE_SHOT_CLEANUP_DISABLE_WAIT
```

在 `AethorAppAction` 加入固定目标、失败元数据和阶段所需 mask，不分配堆内存：

```c
float target_position_rad[ARM_JOINT_COUNT];
float target_speed_rad_s[ARM_JOINT_COUNT];
ProtocolCommandStage failed_stage;
ProtocolCommandError failure_error;
uint8_t failed_motor_number;
uint8_t enabled_by_action_mask;
```

- [ ] **Step 4: 实现前半程状态转换**

增加职责单一的静态函数，例如：

```c
static uint8_t aethor_app_start_one_shot_move(...);
static uint8_t aethor_app_validate_one_shot_targets(...);
static uint8_t aethor_app_advance_one_shot_setup(...);
static uint8_t aethor_app_begin_one_shot_cleanup(...);
```

流程必须是 `DISCOVER_IF_NEEDED -> VALIDATE_LIMITS -> ENSURE_POS_VEL_MODE -> CLEAR_FAULT -> ENABLE`。范围和当前反馈的全有或全无校验必须调用 `motor_runtime_validate_position_velocity_move_subset()`，并发生在任何 enable 帧之前；任一电机失败时记录第一个失败电机并进入失能清理。已发现但模式未确认时仍执行易失模式写入/读回。现有 `PROTOCOL_COMMAND_MOVE_RELATIVE` 分支在由当前反馈算出绝对目标后，也必须调用同一 validator；因此旧 jog 去掉 3° parser 限制后仍不能越过运行时 `PMAX/VMAX/MAX_SPD`。

- [ ] **Step 5: 运行 GREEN 并提交**

```powershell
powershell.exe -ExecutionPolicy Bypass -File .\Tests\host\run_phase0_tests.ps1
powershell.exe -ExecutionPolicy Bypass -File .\Tests\host\run_motor_core_tests.ps1
git diff --check
git add App/aethor_app.c Tests/host/phase0_test_main.c
git commit -m "feat: prepare one shot motor actions"
```

### Task 7: 完成绝对运动、内部重发、保持和自动失能

**Files:**
- Modify: `Tests/host/phase0_test_main.c`
- Modify: `App/aethor_app.c`

- [ ] **Step 1: 写完整动作和超过 1000 ms 无串口流量的失败测试**

测试必须：提交一次命令；完成 setup；读取并保存两台电机各自的第一帧绝对目标；将时间推进到超过 `PROTOCOL_ENGINE_WATCHDOG_TIMEOUT_US` 且不再提交任何串口请求；确认目标帧按原字节重发且没有 `link_timeout`；输入到位反馈；确认依次输出零速保持和 disable；只在 disabled 反馈后得到成功 `done`。

关键断言：

```c
assert(memcmp(first_target.data,
              repeated_target.data,
              first_target.length) == 0);
assert(aethor_app_pop_protocol_result_output(&output_batch) == 0U);
/* Feed disabled feedback for every selected motor. */
assert(strstr(output_batch.messages[0].data,
              "done 50 bench move result=completed") != NULL);
```

同时断言电机 1/3 的位置和速度字节不同，未选电机没有控制帧。

- [ ] **Step 2: 运行并确认 RED**

```powershell
powershell.exe -ExecutionPolicy Bypass -File .\Tests\host\run_phase0_tests.ps1
```

Expected: 当前 1000 ms 看门狗取消动作，且不存在 HOLD→DISABLE 完整状态链。

- [ ] **Step 3: 实现固定目标批次和有界期限**

绝对目标仅在范围校验通过后转换一次并保存在 `application_action`；MOVE 阶段反复发送完全相同的 selected batch。动作期限按每台电机的 `abs(target-current)/speed` 取最大值，再加现有 settle 和 timeout margin；对非有限计算、乘法/加法溢出和无有效反馈失败关闭。

到位判据沿用现有可测试阈值，但不得以目标发送完成代替反馈到位。到位后构造“最终位置 + 0 速度”的 HOLD 批次，再构造 selected disable 批次；只有全部所选电机反馈 disabled 才提交 `COMPLETED`。

- [ ] **Step 4: 仅对自包含动作实行应用层看门狗豁免**

增加明确 helper：

```c
static uint8_t aethor_app_active_action_owns_link_lifecycle(void);
```

只有命令类型为 `PROTOCOL_COMMAND_MOVE_ABSOLUTE_SELF_CONTAINED` 且处于 discovery 到 disable/cleanup 的显式状态时返回 1。`aethor_app_service()` 在此时不调用通信超时取消路径；不得修改 `last_valid_request_at_us`、伪造 `ping` 或全局关闭 `protocol_engine_watchdog_expired()`。动作超时、反馈新鲜度、故障、Bus-Off、控制周期故障和 STOP 仍有效。

- [ ] **Step 5: 写并运行旧看门狗回归**

保留并增强测试：`bench enable` 完成后不发送 `ping`，在 1000 ms 时仍产生 `link_timeout` 和 emergency disable；旧 `bench jog` 仍属于需主机保活的非自包含动作。

```powershell
powershell.exe -ExecutionPolicy Bypass -File .\Tests\host\run_phase0_tests.ps1
powershell.exe -ExecutionPolicy Bypass -File .\Tests\host\run_text_protocol_engine_tests.ps1
```

Expected: 新动作无 ping 可完成；旧使能路径仍严格超时。

- [ ] **Step 6: 提交完整成功路径**

```powershell
git diff --check
git add App/aethor_app.c Tests/host/phase0_test_main.c
git commit -m "feat: complete one shot bench moves"
```

### Task 8: 完成失败清理、STOP 抢占和 busy 语义

**Files:**
- Modify: `Tests/host/phase0_test_main.c`
- Modify: `Tests/host/text_protocol_engine_test_main.c`
- Modify: `App/aethor_app.c`
- Modify: `App/Protocol/protocol_engine.c`

- [ ] **Step 1: 写逐阶段失败测试**

覆盖：范围不可用、过期反馈、非零 fault、发现超时、模式切换失败、使能反馈超时、运动超时、失能反馈超时、CAN Bus-Off、动作中显式 `bench stop`、动作中第二条普通命令。逐项断言：

- 范围失败前不出现 enable 帧。
- 使能后的失败先尝试 HOLD（反馈有效时）再 selected disable。
- Bus-Off/全局运行故障仍走现有全七轴 emergency disable。
- `bench stop` 抢占后原命令终态为 `cancelled`，STOP 自己有独立终态。
- 第二条普通命令立即得到 `busy`，不覆盖当前动作对象。
- 清理失败仍产生唯一终态，且阶段/错误码/电机号与最初失败一致。

- [ ] **Step 2: 运行并确认 RED**

```powershell
powershell.exe -ExecutionPolicy Bypass -File .\Tests\host\run_phase0_tests.ps1
powershell.exe -ExecutionPolicy Bypass -File .\Tests\host\run_text_protocol_engine_tests.ps1
```

Expected: 至少失败阶段、清理顺序或结果格式断言失败。

- [ ] **Step 3: 实现统一清理入口和结果映射**

所有 setup/move/hold/disable 错误都调用一个 `aethor_app_begin_one_shot_cleanup()`，只记录第一次失败，并根据 `enabled_by_action_mask` 和反馈有效性选择 HOLD+DISABLE 或直接 DISABLE。`aethor_app_complete_action()` 把固定阶段、错误枚举和一基电机号复制到 `ProtocolCommandResult`。STOP 与全局 fault 分支复用现有优先级和 emergency disable，不创建第二套发送队列。

- [ ] **Step 4: 运行 GREEN 并提交**

```powershell
powershell.exe -ExecutionPolicy Bypass -File .\Tests\host\run_phase0_tests.ps1
powershell.exe -ExecutionPolicy Bypass -File .\Tests\host\run_text_protocol_engine_tests.ps1
git diff --check
git add App/aethor_app.c App/Protocol/protocol_engine.c Tests/host/phase0_test_main.c Tests/host/text_protocol_engine_test_main.c
git commit -m "fix: fail closed during one shot moves"
```

### Task 9: 同步协议资产、参考客户端和使用文档

**Files:**
- Modify: `README.md`
- Modify: `CHANGELOG.md`
- Modify: `docs/compatibility/aethor-text-v1-migration.md`
- Modify: `docs/compatibility/aethor-text-v1-manifest.json`
- Modify: `docs/compatibility/state-and-command-lifecycle.md`
- Modify: `Tests/protocol/aethor-text-v1-vectors.json`
- Modify: `Tests/protocol/README.md`
- Modify: `Tools/aethor_reference_client.py`
- Modify: `Tests/host/test_text_host_simulator.py`

- [ ] **Step 1: 写参考客户端和向量的失败测试**

在模拟器测试中断言客户端生成：

```text
50 bench move 1 position=90 speed=30
51 bench move 1,3 position=90,-45 speed=30,20
```

并能解析新的成功/失败终态。向兼容性向量加入单电机、多电机、数量不匹配、重放和请求 ID 冲突用例。

- [ ] **Step 2: 运行并确认 RED**

```powershell
powershell.exe -ExecutionPolicy Bypass -File .\Tests\host\run_simulator_tests.ps1
```

Expected: 客户端尚无 `bench move` 生成/解析入口或向量不匹配。

- [ ] **Step 3: 实现最小参考客户端入口**

客户端接受等长的 motor/position/speed 数组，在本地先检查长度、有限值和正速度，然后只发送一次命令并等待 `done`；不得在该方法中启动 keepalive。旧 `bench enable/jog` helper 继续保留原 keepalive 指南。

- [ ] **Step 4: 同步人类可读合同**

文档必须明确：

- `bench move` 是绝对输出端角度，零点为本次上电零点。
- 每台电机位置/速度独立，一一对应且无广播。
- 最大值来自本次发现的 `PMAX/VMAX/MAX_SPD`，允许等于边界，拒绝而不截断越界值。
- 新动作只发一次且最终自动失能；旧 `bench enable/jog` 在带电期间仍需约 250 ms `ping`。
- `show motor` 可先查看运行时能力。
- 软件范围不是机械臂软限位，不代表负载、方向、减速比或七轴安全已经验证。

更新 manifest 的命令、字段和响应 schema；更新向量而不改变 `aethor-arm-ascii-v1` 回归文件。

- [ ] **Step 5: 运行 GREEN、校验 JSON 并提交**

```powershell
powershell.exe -ExecutionPolicy Bypass -File .\Tests\host\run_simulator_tests.ps1
Get-Content .\docs\compatibility\aethor-text-v1-manifest.json -Raw | ConvertFrom-Json | Out-Null
Get-Content .\Tests\protocol\aethor-text-v1-vectors.json -Raw | ConvertFrom-Json | Out-Null
git diff --check
git add README.md CHANGELOG.md docs/compatibility/aethor-text-v1-migration.md docs/compatibility/aethor-text-v1-manifest.json docs/compatibility/state-and-command-lifecycle.md Tests/protocol/aethor-text-v1-vectors.json Tests/protocol/README.md Tools/aethor_reference_client.py Tests/host/test_text_host_simulator.py
git commit -m "docs: publish one shot bench move contract"
```

### Task 10: 全量软件验收和 Keil 构建

**Files:**
- Modify after evidence exists: `docs/handoffs/aethor-text-v1-bench/verification.txt`
- Preserve: `MDK-ARM/CtrBoard-H7_FDCAN.uvprojx`

- [ ] **Step 1: 运行完整主机和架构测试矩阵**

```powershell
powershell.exe -ExecutionPolicy Bypass -File .\Tests\host\run_tests.ps1
powershell.exe -ExecutionPolicy Bypass -File .\Tests\host\run_motor_core_tests.ps1
powershell.exe -ExecutionPolicy Bypass -File .\Tests\host\run_motion_tests.ps1
powershell.exe -ExecutionPolicy Bypass -File .\Tests\host\run_platform_io_tests.ps1
powershell.exe -ExecutionPolicy Bypass -File .\Tests\host\run_protocol_tests.ps1
powershell.exe -ExecutionPolicy Bypass -File .\Tests\host\run_text_protocol_tests.ps1
powershell.exe -ExecutionPolicy Bypass -File .\Tests\host\run_text_protocol_engine_tests.ps1
powershell.exe -ExecutionPolicy Bypass -File .\Tests\host\run_text_protocol_arm_profile_tests.ps1
powershell.exe -ExecutionPolicy Bypass -File .\Tests\host\run_phase0_tests.ps1
powershell.exe -ExecutionPolicy Bypass -File .\Tests\host\run_simulator_tests.ps1
powershell.exe -ExecutionPolicy Bypass -File .\Tests\host\check_phase0_architecture.ps1
```

Expected: 所有 suite 输出其 PASS 标记；新命令不得破坏生产 profile 或旧协议回归。

- [ ] **Step 2: 执行静态审计**

```powershell
$markerPattern = ('TO' + 'DO') + '|' + ('TB' + 'D') + '|' + ('place' + 'holder')
rg -n $markerPattern App Tests Tools docs README.md CHANGELOG.md
rg -n "PROTOCOL_COMMAND_MOVE_ABSOLUTE_SELF_CONTAINED|motor_runtime_validate_position_velocity_move_subset|motor_motion_limits_valid_mask" App Tests
git diff --check
git status --short
```

Expected: 没有新增占位符或旧固定 3° 宏；新类型在解析、执行、测试和文档中一致；工作树只含本任务差异和已知 `<LayerInfo>` 用户改动。

- [ ] **Step 3: Keil 全量重建并核对镜像**

```powershell
& 'E:\keil\UV4\UV4.exe' -b '.\MDK-ARM\CtrBoard-H7_FDCAN.uvprojx' -j0 -o '.\MDK-ARM\one-shot-bench-move-build.log'
Select-String -Path '.\MDK-ARM\one-shot-bench-move-build.log' -Pattern '0 Error\(s\), 0 Warning\(s\)|Program Size'
Get-FileHash '.\MDK-ARM\CtrBoard-H7_FDCAN\CtrBoard-H7_FDCAN.hex' -Algorithm SHA256
```

如果实际 HEX 路径不同，先用 `Get-ChildItem .\MDK-ARM -Filter *.hex -File` 只读定位，再哈希确切文件。Expected: `0 Error(s), 0 Warning(s)`，HEX 时间戳更新并记录 SHA-256；构建不应改写 `<LayerInfo>`。

- [ ] **Step 4: 记录可复现的软件证据并提交**

仅把本轮实际命令、PASS 标记、Keil 结果和新 HEX 哈希追加到 verification 文档；不得复制旧 COM7 结果冒充本轮实机证据。

```powershell
git add docs/handoffs/aethor-text-v1-bench/verification.txt
git commit -m "test: record one shot move verification"
```

### Task 11: 卸载单电机串口验收

**Files:**
- Modify after evidence exists: `docs/handoffs/aethor-text-v1-bench/verification.txt`
- Optional modify if reusable automation is needed: `Tests/hardware/debug_com7_aethor_text_v1.ps1`

- [ ] **Step 1: 建立明确的实机前置和停止条件**

仅在确认以下条件时执行：一台 S3519 空载可靠固定；CAN 终端、电源和急停手段就绪；串口端口已只读确认；目标电机 ID 已确认。先执行 `hello`、`show motor <id>` 和发现流程，记录 `PMAX/VMAX/MAX_SPD`，但不命令电机到这些最大值。

停止条件：Bus-Off、非零 fault、身份/模式/范围字段不完整、反馈陈旧、异常噪声/温升/方向、无法随时切断 24 V，任一出现即不执行运动。

- [ ] **Step 2: 用保守目标只发送一次动作命令**

示例格式（角度和速度以现场安全值替换，不得使用最大值）：

```text
100 bench move 1 position=5 speed=2
```

发送后不再发 `ping`，等待超过 1000 ms。记录 `ok`、最终 `done`、执行时间和中间 `show`/反馈证据；不得重复发送相同动作正文来维持运行。

- [ ] **Step 3: 验收自动失能和故障恢复**

成功标准：得到 `done ... result=completed`，随后 `show motors` 显示目标电机未使能、无 fault、无 Bus-Off。若超时或失败，只执行明确的 `bench stop <id>` 和 `bench disable <id>`，不继续扩大角度/速度或增加电机数量。

- [ ] **Step 4: 记录硬件证据并提交**

把端口、固件 HEX SHA-256、电机 ID、发现范围、命令、完整回复、最终 disabled 状态和未验证边界写入 verification 文档。明确本次只证明空载单电机一条命令闭环，不证明七轴、负载、机械软限位或实际角速度标定。

```powershell
git add docs/handoffs/aethor-text-v1-bench/verification.txt
git commit -m "test: verify one shot move on unloaded motor"
```

### Task 12: 最终自审和交付边界检查

**Files:**
- Review: all files changed since `a46bc75`
- Preserve: `MDK-ARM/CtrBoard-H7_FDCAN.uvprojx`

- [ ] **Step 1: 对照设计规格逐项审查**

```powershell
git diff --stat a46bc75..HEAD
git diff --name-only a46bc75..HEAD
git log --oneline --decorate a46bc75..HEAD
```

逐项确认：严格等长列表、逐电机动态范围、边界允许/越界拒绝、无默认回退、一次提交、内部固定目标重发、超过 1000 ms 无 ping、HOLD、自动失能后才 done、STOP 抢占、失败清理、旧 watchdog 回归、未选电机不变。

- [ ] **Step 2: 检查类型和文档一致性**

确认所有公共枚举都被完整格式化；所有状态分支都有明确退出；`failed_motor_number` 始终是一基公共编号；C 与 Python 都拒绝 NaN/Inf、空列表和数量不匹配；manifest、向量、README 的示例与测试完全一致。

- [ ] **Step 3: 检查提交和未暂存内容**

```powershell
git diff --check
git status --short --branch
git diff --cached --name-only
git diff -- MDK-ARM/CtrBoard-H7_FDCAN.uvprojx
```

Expected: 没有暂存遗漏或无关文件；`MDK-ARM/CtrBoard-H7_FDCAN.uvprojx` 的 `<LayerInfo>` 仍未暂存且内容未被任务修改。只有在所有软件验收完成后才可声明代码完成；只有 Task 11 有本轮直接证据时才可声明实机一条命令完成。
