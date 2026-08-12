# PRD Phase 0 Firmware Architecture Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan.

**Goal:** 在不驱动任何电机的前提下，把现有 STM32H723 工程迁移为符合 PRD 的七轴固件 Phase 0 基线：建立静态内存的分层骨架、七轴配置、启动自检、结构化诊断、CubeMX/Keil 同步、测试证据与多人交接文档。

**Architecture:** 采用同一工程内的增量迁移。新增 `App` 分层，依赖方向固定为 `Protocol -> Arm -> Motion -> Motor -> Platform`，`Config` 和 `Telemetry` 提供只读配置与结构化反馈；Phase 0 的运行链只执行 `BOOT -> SELF_TEST -> FAULT(CONFIG_INCOMPLETE)`，不暴露使能、位置、速度或 CAN 下发入口。旧 `User` 源文件保留在磁盘，Keil 目标仅保留生成代码仍引用的 `usb_cdc_transport.c`，避免旧按键电机示例进入正式固件。

**Tech Stack:** STM32H723VGT6、STM32CubeMX 6.11.1 系列、STM32CubeH7 1.11.2、CMSIS-RTOS v1/FreeRTOS、Keil MDK 5.43/ARMCC 5.05、PowerShell 主机测试、Git。

---

## 执行边界

- 本计划只完成 PRD Phase 0，不实现 UART 正式协议、七轴运动、同步到达、MIT 控制、减速器换算、DH 正逆解、RGB 或上位机业务逻辑。
- 电机方向、软限位、速度/加速度、MIT 增益、真实量程、减速比等未知项必须以“未验证位”为证据，不填猜测值。
- 不批量删除任何文件或目录；旧文件原地保留，通过 Keil 工程分组控制是否参与固件构建。
- 不覆盖 `MDK-ARM/CtrBoard-H7_FDCAN.uvprojx` 中现有未提交的 `<LayerInfo>` 用户元数据；修改该文件时仅提交与本计划有关的 XML 块。
- 新增 C/H 文件必须有文件级注释，所有函数声明与定义必须有函数级注释；结构体需有类型用途注释。
- 每个任务先写失败测试，再写最小实现，再运行测试并提交。任何提交前都运行 `git diff --check`。

## Task 1: 固化 Phase 0 基线证据

**Files:**

- Create: `docs/handoffs/phase-00/config-snapshot/baseline.md`
- Create: `docs/handoffs/phase-00/config-snapshot/toolchain.md`
- Create: `docs/handoffs/phase-00/config-snapshot/backup-manifest.md`
- Create: `docs/handoffs/phase-00/config-snapshot/wiring.md`
- Create: `docs/handoffs/phase-00/config-snapshot/map-baseline.txt`

**Step 1: 记录当前工程事实**

在 `baseline.md` 中逐项记录并附来源路径：MCU、时钟、FDCAN1 经典 CAN 1 Mbps、标准过滤器数量、USB CDC 现状、FreeRTOS 默认任务当前分配方式、当前固件入口、现有七轴但未启用的模块、工作树中 `<LayerInfo>` 的归属边界。只记录命令可复核的事实。

验证命令：

```powershell
Select-String -Path '.\CtrBoard-H7_FDCAN.ioc' -Pattern 'Mcu.Name|FDCAN1.Nominal|FREERTOS.Tasks01|USB_OTG_HS'
Select-String -Path '.\Core\Src\freertos.c' -Pattern 'osThreadDef|osThreadStaticDef|osThreadCreate'
Select-String -Path '.\User\aethor_application.c' -Pattern 'HAL_GPIO_ReadPin|motor|key' -CaseSensitive:$false
git status --short
```

Expected: 每一项均能指向当前文件；工作树仅保留开始执行前已有的 `uvprojx` 用户元数据变更。

**Step 2: 建立可恢复的工程备份清单**

不复制或覆盖用户工作树文件。`backup-manifest.md` 记录执行前 `HEAD`、当前分支、`.ioc` 和 `uvprojx` 的 `HEAD` 对象哈希、工作树 SHA-256，以及恢复到独立文件的命令。这样既满足可恢复性，也不把未提交的 `<LayerInfo>` 混入 Phase 0 提交。

```powershell
$baselineCommit = git rev-parse HEAD
$baselineCommit
git hash-object '.\CtrBoard-H7_FDCAN.ioc'
git hash-object '.\MDK-ARM\CtrBoard-H7_FDCAN.uvprojx'
Get-FileHash '.\CtrBoard-H7_FDCAN.ioc' -Algorithm SHA256
Get-FileHash '.\MDK-ARM\CtrBoard-H7_FDCAN.uvprojx' -Algorithm SHA256
```

清单中的恢复命令必须输出到新文件名，不覆盖当前工程：

```powershell
git show "${baselineCommit}:aethor_robo_v1/CtrBoard-H7_FDCAN.ioc" | Set-Content -Encoding utf8 '.\CtrBoard-H7_FDCAN.phase00-baseline.ioc'
git show "${baselineCommit}:aethor_robo_v1/MDK-ARM/CtrBoard-H7_FDCAN.uvprojx" | Set-Content -Encoding utf8 '.\MDK-ARM\CtrBoard-H7_FDCAN.phase00-baseline.uvprojx'
```

**Step 3: 记录工具链**

在 `toolchain.md` 中记录可执行文件绝对路径、文件版本、Cube 包版本和当前日期。执行：

```powershell
(Get-Item 'E:\keil\UV4\UV4.exe').VersionInfo | Select-Object FileVersion,ProductVersion
(Get-Item 'E:\keil\ARM\ARMCC\bin\armcc.exe').VersionInfo | Select-Object FileVersion,ProductVersion
(Get-Item 'E:\STM32CubeMX\STM32CubeMX.exe').VersionInfo | Select-Object FileVersion,ProductVersion
Select-String -Path '.\CtrBoard-H7_FDCAN.ioc' -Pattern 'MxCube.Version|Mcu.Family|ProjectManager.FirmwarePackage'
```

**Step 4: 记录已证实的接线与 Map 基线**

`wiring.md` 只记录板卡手册与本地官方示例共同证实的信息：FDCAN1 当前工程引脚与收发器连接、USART1 PA9/PA10 候选链路、921600 8N1，以及未接线/未收发验证状态。未知项明确写“未验证”，不绘制推测连接。

从当前提交已有的 Keil map 文件提取 Code/RO/RW/ZI、FreeRTOS heap 和主要自有模块摘要写入 `map-baseline.txt`。先查明实际 map 路径：

```powershell
Get-ChildItem '.\MDK-ARM' -Recurse -File -Filter '*.map' | Select-Object FullName,LastWriteTime,Length
Select-String -Path '.\MDK-ARM\**\*.map' -Pattern 'Total RO  Size|Total RW  Size|Heap|Program Size'
```

若 glob 不能被 `Select-String` 解析，使用上一条命令得到的明确单个 map 路径；不递归删除或整理构建目录。

**Step 5: 校验并提交**

```powershell
git diff --check
git add docs/handoffs/phase-00/config-snapshot/baseline.md docs/handoffs/phase-00/config-snapshot/toolchain.md docs/handoffs/phase-00/config-snapshot/backup-manifest.md docs/handoffs/phase-00/config-snapshot/wiring.md docs/handoffs/phase-00/config-snapshot/map-baseline.txt
git commit -m "docs: capture phase 0 firmware baseline"
```

## Task 2: 用主机测试定义七轴配置模型

**Files:**

- Create: `App/Config/arm_config.h`
- Create: `App/Config/arm_config.c`
- Create: `App/Config/build_info.h`
- Create: `App/Config/build_info.c`
- Create: `App/Config/board_config.h`
- Create: `Tests/host/phase0_test_main.c`
- Create: `Tests/host/run_phase0_tests.ps1`

**Step 1: 写失败的配置测试**

`phase0_test_main.c` 先覆盖以下行为：

```c
/* 文件说明：Phase 0 分层架构的主机侧单元测试入口。 */
#include <assert.h>
#include <math.h>
#include "arm_config.h"

/** 验证生产配置固定包含七个且索引连续的关节。 */
static void test_production_config_has_seven_ordered_joints(void)
{
    const ArmConfig *config = arm_config_get_production();
    assert(config != NULL);
    assert(config->joint_count == ARM_JOINT_COUNT);
    for (uint8_t index = 0U; index < ARM_JOINT_COUNT; ++index) {
        assert(config->joints[index].joint_index == index);
        assert(config->joints[index].esc_id == (uint16_t)(index + 1U));
        assert(config->joints[index].master_id == (uint16_t)(index + 11U));
    }
}

/** 验证所有待实机确认的字段都不会被误判为可使能。 */
static void test_production_config_is_not_enable_ready(void)
{
    ArmConfigValidation validation;
    const ArmConfig *config = arm_config_get_production();
    assert(arm_config_validate_schema(config, &validation));
    assert(!arm_config_is_enable_ready(config, &validation));
    assert(validation.missing_verified_fields != 0U);
}

/** 验证重复 CAN 标识会触发配置结构错误。 */
static void test_duplicate_can_id_is_rejected(void)
{
    ArmConfig mutable_config = *arm_config_get_production();
    ArmConfigValidation validation;
    mutable_config.joints[1].esc_id = mutable_config.joints[0].esc_id;
    assert(!arm_config_validate_schema(&mutable_config, &validation));
    assert((validation.schema_errors & ARM_CONFIG_ERROR_DUPLICATE_ESC_ID) != 0U);
}

/** 验证缺失关节会触发配置结构错误。 */
static void test_missing_joint_is_rejected(void)
{
    ArmConfig mutable_config = *arm_config_get_production();
    ArmConfigValidation validation;
    mutable_config.joint_count = ARM_JOINT_COUNT - 1U;
    assert(!arm_config_validate_schema(&mutable_config, &validation));
    assert((validation.schema_errors & ARM_CONFIG_ERROR_JOINT_COUNT) != 0U);
}

/** 验证只有被标记为已验证的数值才进入语义检查。 */
static void test_verified_invalid_direction_is_rejected(void)
{
    ArmConfig mutable_config = *arm_config_get_production();
    ArmConfigValidation validation;
    mutable_config.joints[0].direction = 0;
    mutable_config.joints[0].verified_fields |= ARM_JOINT_VERIFIED_DIRECTION;
    assert(!arm_config_validate_schema(&mutable_config, &validation));
    assert((validation.schema_errors & ARM_CONFIG_ERROR_DIRECTION) != 0U);
}

/** 验证已确认但次序错误的软限位会被拒绝。 */
static void test_verified_invalid_limits_are_rejected(void)
{
    ArmConfig mutable_config = *arm_config_get_production();
    ArmConfigValidation validation;
    mutable_config.joints[0].soft_limit_min_rad = 1.0F;
    mutable_config.joints[0].soft_limit_max_rad = -1.0F;
    mutable_config.joints[0].verified_fields |= ARM_JOINT_VERIFIED_LIMITS;
    assert(!arm_config_validate_schema(&mutable_config, &validation));
    assert((validation.schema_errors & ARM_CONFIG_ERROR_LIMITS) != 0U);
}

/** 运行 Phase 0 配置测试集。 */
int main(void)
{
    test_production_config_has_seven_ordered_joints();
    test_production_config_is_not_enable_ready();
    test_duplicate_can_id_is_rejected();
    test_missing_joint_is_rejected();
    test_verified_invalid_direction_is_rejected();
    test_verified_invalid_limits_are_rejected();
    return 0;
}
```

`run_phase0_tests.ps1` 使用本机已存在的 C 编译器；先检测 `gcc`，不可用时检测 `clang`，两者都不存在则明确失败并输出原因。编译参数固定包含 `-std=c11 -Wall -Wextra -Werror`，产物放入 `Tests/host/build/phase0_tests.exe`。

**Step 2: 运行并确认失败**

```powershell
& '.\Tests\host\run_phase0_tests.ps1'
```

Expected: FAIL，原因是 `arm_config.h` 或配置符号尚不存在。

**Step 3: 实现类型与验证接口**

`arm_config.h` 的核心公共模型：

```c
/* 文件说明：七轴机械臂的静态配置模型与验证接口。 */
#ifndef APP_CONFIG_ARM_CONFIG_H
#define APP_CONFIG_ARM_CONFIG_H

#include <stdbool.h>
#include <stdint.h>

#define ARM_JOINT_COUNT (7U)

typedef enum {
    ARM_JOINT_VERIFIED_DIRECTION = (1UL << 0),
    ARM_JOINT_VERIFIED_LIMITS = (1UL << 1),
    ARM_JOINT_VERIFIED_MAX_VELOCITY = (1UL << 2),
    ARM_JOINT_VERIFIED_MAX_ACCELERATION = (1UL << 3),
    ARM_JOINT_VERIFIED_MIT_GAINS = (1UL << 4),
    ARM_JOINT_VERIFIED_MOTOR_RANGES = (1UL << 5),
    ARM_JOINT_VERIFIED_GEAR_RATIO = (1UL << 6)
} ArmJointVerifiedField;

#define ARM_JOINT_REQUIRED_ENABLE_FIELDS \
    (ARM_JOINT_VERIFIED_DIRECTION | ARM_JOINT_VERIFIED_LIMITS | \
     ARM_JOINT_VERIFIED_MAX_VELOCITY | ARM_JOINT_VERIFIED_MAX_ACCELERATION | \
     ARM_JOINT_VERIFIED_MIT_GAINS | ARM_JOINT_VERIFIED_MOTOR_RANGES | \
     ARM_JOINT_VERIFIED_GEAR_RATIO)

typedef enum {
    ARM_CONFIG_ERROR_NONE = 0U,
    ARM_CONFIG_ERROR_NULL = (1UL << 0),
    ARM_CONFIG_ERROR_JOINT_COUNT = (1UL << 1),
    ARM_CONFIG_ERROR_JOINT_INDEX = (1UL << 2),
    ARM_CONFIG_ERROR_DUPLICATE_ESC_ID = (1UL << 3),
    ARM_CONFIG_ERROR_DUPLICATE_MASTER_ID = (1UL << 4),
    ARM_CONFIG_ERROR_DIRECTION = (1UL << 5),
    ARM_CONFIG_ERROR_LIMITS = (1UL << 6),
    ARM_CONFIG_ERROR_VELOCITY = (1UL << 7),
    ARM_CONFIG_ERROR_ACCELERATION = (1UL << 8),
    ARM_CONFIG_ERROR_MIT_GAINS = (1UL << 9),
    ARM_CONFIG_ERROR_MOTOR_RANGES = (1UL << 10),
    ARM_CONFIG_ERROR_GEAR_RATIO = (1UL << 11)
} ArmConfigError;

/** 单关节静态配置；未验证字段保留零值且对应 verified_fields 位为零。 */
typedef struct {
    uint8_t joint_index;
    uint16_t esc_id;
    uint16_t master_id;
    int8_t direction;
    float soft_limit_min_rad;
    float soft_limit_max_rad;
    float max_velocity_rad_s;
    float max_acceleration_rad_s2;
    float mit_kp;
    float mit_kd;
    float motor_pmax_rad;
    float motor_vmax_rad_s;
    float motor_tmax_nm;
    float gear_ratio;
    uint32_t verified_fields;
} JointConfig;

/** 整机静态配置。 */
typedef struct {
    uint8_t joint_count;
    JointConfig joints[ARM_JOINT_COUNT];
} ArmConfig;

/** 配置验证结果，区分结构错误与尚未验证的使能字段。 */
typedef struct {
    uint32_t schema_errors;
    uint32_t missing_verified_fields;
    uint8_t first_error_joint;
} ArmConfigValidation;

/** 返回只读生产配置。 */
const ArmConfig *arm_config_get_production(void);

/** 验证配置结构以及所有已验证字段的数值语义。 */
bool arm_config_validate_schema(const ArmConfig *config, ArmConfigValidation *validation);

/** 判断配置是否满足未来电机使能前置条件。 */
bool arm_config_is_enable_ready(const ArmConfig *config, ArmConfigValidation *validation);

#endif
```

`arm_config.c` 使用 `static const ArmConfig`：J1-J7 的 `esc_id` 固定为 1-7，`master_id` 固定为 11-17；所有未知物理参数为 `0.0F`，所有 `verified_fields` 为 `0U`。验证函数必须：检查空指针、关节数、连续索引、非零且不重复的两类 CAN ID；仅当对应已验证位存在时检查方向为 `-1/+1`、上下限有序、速度/加速度/量程/减速比有限且大于零、MIT 增益有限且不小于零；任何输入都先完整初始化 `ArmConfigValidation`。

`build_info.h/.c` 提供只读的产品名、协议代号、固件语义版本和 Git 描述注入入口，不使用动态字符串。`board_config.h` 固化 `controller_id = "aethor-controller-01"`、`arm_id = "arm-01"`、MCU、关节数、FDCAN 标称速率、未来正式 UART 波特率常量，并在编译期断言关节数为 7。身份字符串使用定长只读数组，协议层只能读取，不能在运行时改写。

**Step 4: 运行测试并确认通过**

```powershell
& '.\Tests\host\run_phase0_tests.ps1'
```

Expected: PASS，进程退出码为 0。

**Step 5: 提交**

```powershell
git diff --check
git add App/Config Tests/host/phase0_test_main.c Tests/host/run_phase0_tests.ps1
git commit -m "feat: add verified seven-axis configuration model"
```

## Task 3: 建立固定容量诊断与机械臂启动状态机

**Files:**

- Create: `App/Telemetry/diagnostics.h`
- Create: `App/Telemetry/diagnostics.c`
- Create: `App/Arm/arm_controller.h`
- Create: `App/Arm/arm_controller.c`
- Modify: `Tests/host/phase0_test_main.c`
- Modify: `Tests/host/run_phase0_tests.ps1`

**Step 1: 写失败的诊断和状态机测试**

新增测试：固定环形缓冲区填满后保留最新事件并增加丢弃计数；初始化后的第一轮服务从 `BOOT` 进入 `SELF_TEST`；生产配置在自检后进入 `FAULT` 且故障码为 `ARM_FAULT_CONFIG_INCOMPLETE`；重复调用不会离开锁存故障；快照的关节数为 7；全过程没有电机帧发送依赖。

核心断言：

```c
/** 验证未完成标定的生产配置会被安全锁定。 */
static void test_arm_controller_latches_incomplete_config_fault(void)
{
    Diagnostics diagnostics;
    ArmController controller;
    ArmSnapshot snapshot;
    diagnostics_init(&diagnostics);
    arm_controller_init(&controller, arm_config_get_production(), &diagnostics, 1000ULL);
    arm_controller_step(&controller, 2000ULL);
    arm_controller_step(&controller, 3000ULL);
    arm_controller_get_snapshot(&controller, &snapshot);
    assert(snapshot.state == ARM_STATE_FAULT);
    assert(snapshot.fault == ARM_FAULT_CONFIG_INCOMPLETE);
    assert(snapshot.joint_count == ARM_JOINT_COUNT);
}
```

**Step 2: 运行并确认失败**

```powershell
& '.\Tests\host\run_phase0_tests.ps1'
```

Expected: FAIL，缺少 `diagnostics` 和 `arm_controller` 符号。

**Step 3: 实现固定环形缓冲区**

`diagnostics.h` 固定容量，不返回内部可写指针：

```c
/* 文件说明：Phase 0 结构化诊断事件与固定容量缓冲区。 */
#ifndef APP_TELEMETRY_DIAGNOSTICS_H
#define APP_TELEMETRY_DIAGNOSTICS_H

#include <stdbool.h>
#include <stdint.h>

#define DIAGNOSTICS_CAPACITY (64U)

typedef enum {
    DIAGNOSTIC_SEVERITY_INFO = 0,
    DIAGNOSTIC_SEVERITY_WARNING,
    DIAGNOSTIC_SEVERITY_ERROR
} DiagnosticSeverity;

typedef enum {
    DIAGNOSTIC_CODE_BOOT = 1,
    DIAGNOSTIC_CODE_SELF_TEST_STARTED,
    DIAGNOSTIC_CODE_CONFIG_INVALID,
    DIAGNOSTIC_CODE_CONFIG_INCOMPLETE
} DiagnosticCode;

/** 单条结构化诊断事件。 */
typedef struct {
    uint64_t timestamp_us;
    uint32_t sequence;
    DiagnosticCode code;
    DiagnosticSeverity severity;
    uint32_t detail;
} DiagnosticEvent;

/** 固定容量诊断环形缓冲区。 */
typedef struct {
    DiagnosticEvent events[DIAGNOSTICS_CAPACITY];
    uint32_t next_sequence;
    uint32_t dropped_count;
    uint16_t head;
    uint16_t count;
} Diagnostics;

/** Phase 0 诊断计数器快照。 */
typedef struct {
    uint32_t service_cycles;
    uint32_t config_validation_failures;
    uint32_t can_rx_frames;
    uint32_t can_tx_frames;
    uint32_t uart_rx_bytes;
    uint32_t uart_tx_bytes;
    uint32_t queue_high_watermark;
    uint32_t minimum_stack_words;
    uint32_t minimum_heap_bytes;
} DiagnosticCounters;

/** 初始化诊断缓冲区。 */
void diagnostics_init(Diagnostics *diagnostics);

/** 写入一条诊断事件，满载时覆盖最旧记录并累计覆盖计数。 */
bool diagnostics_push(Diagnostics *diagnostics, uint64_t timestamp_us,
                      DiagnosticCode code, DiagnosticSeverity severity,
                      uint32_t detail);

/** 按从旧到新的逻辑索引复制一条事件。 */
bool diagnostics_get(const Diagnostics *diagnostics, uint16_t logical_index,
                     DiagnosticEvent *event);

/** 复制当前诊断计数器，避免暴露内部可写状态。 */
bool diagnostics_get_counters(const Diagnostics *diagnostics,
                              DiagnosticCounters *counters);

#endif
```

`Diagnostics` 内含 `DiagnosticCounters counters`；实现使用值拷贝、边界检查和断言，不调用 `malloc/calloc/realloc/free`。Phase 0 未启用 CAN/UART 数据面时，对应计数器必须保持零；每次应用服务递增 `service_cycles`，配置失败递增 `config_validation_failures`。栈/堆水位尚未接入 RTOS 采样时使用显式的 `DIAGNOSTIC_WATERMARK_NOT_SAMPLED` 哨兵值，不能伪造为零水位。

**Step 4: 实现最小安全状态机**

`arm_controller.h` 定义 `BOOT`、`SELF_TEST`、`FAULT`，故障码定义 `NONE`、`CONFIG_INVALID`、`CONFIG_INCOMPLETE`。`ArmController` 只持有只读配置指针、诊断指针、状态、故障和状态进入时间；`ArmSnapshot` 是独立值对象。`arm_controller_step()` 逻辑必须严格为：

```c
switch (controller->state) {
case ARM_STATE_BOOT:
    arm_controller_transition(controller, ARM_STATE_SELF_TEST, timestamp_us);
    break;
case ARM_STATE_SELF_TEST:
    if (!arm_config_validate_schema(controller->config, &validation)) {
        arm_controller_latch_fault(controller, ARM_FAULT_CONFIG_INVALID,
                                   validation.schema_errors, timestamp_us);
    } else if (!arm_config_is_enable_ready(controller->config, &validation)) {
        arm_controller_latch_fault(controller, ARM_FAULT_CONFIG_INCOMPLETE,
                                   validation.missing_verified_fields, timestamp_us);
    }
    break;
case ARM_STATE_FAULT:
default:
    break;
}
```

此模块不得包含 HAL、FreeRTOS、FDCAN、USB 或 UART 头文件，也不得依赖 `Motor` 层。

**Step 5: 运行测试并提交**

```powershell
& '.\Tests\host\run_phase0_tests.ps1'
git diff --check
git add App/Telemetry App/Arm Tests/host/phase0_test_main.c Tests/host/run_phase0_tests.ps1
git commit -m "feat: add phase 0 diagnostics and boot self-test"
```

## Task 4: 建立协议、运动、电机、平台契约与安全应用入口

**Files:**

- Create: `App/Protocol/protocol_contract.h`
- Create: `App/Motion/motion_types.h`
- Create: `App/Motor/motor_types.h`
- Create: `App/Platform/platform_contract.h`
- Create: `App/aethor_app.h`
- Create: `App/aethor_app.c`
- Modify: `Tests/host/phase0_test_main.c`
- Modify: `Tests/host/run_phase0_tests.ps1`

**Step 1: 写失败的应用入口测试**

测试 `aethor_app_init()` 后快照为 `BOOT`，连续两次 `aethor_app_service()` 后锁存 `CONFIG_INCOMPLETE`；重复初始化可恢复到确定的 `BOOT`；空输出指针返回 `false`；接口中不存在 enable、move、set-position、set-speed 或 send-frame 函数。

**Step 2: 运行并确认失败**

```powershell
& '.\Tests\host\run_phase0_tests.ps1'
```

Expected: FAIL，缺少 `aethor_app` 接口。

**Step 3: 新增只定义边界的契约头文件**

- `protocol_contract.h`: 固定最大帧长 512、近期结果容量 32、业务命令容量 8、协议版本字符串 `aethor-arm-ascii-v1`、Phase 0 状态 `PROTOCOL_STATUS_NOT_AVAILABLE`；用枚举冻结高优先级响应与可丢旧遥测两个通道，不实现解析器。
- `motion_types.h`: 七轴目标/反馈值对象，仅定义 `position_rad`、`velocity_rad_s`、时间戳和有效掩码；不定义规划入口。
- `motor_types.h`: 电机总线状态、七轴反馈快照和错误枚举；不定义发送入口。
- `platform_contract.h`: 控制周期 4000 us、FDCAN1、未来 USART1 921600 8N1 的板级常量；明确 `PLATFORM_FORMAL_LINK_VALIDATED` 为 0。

四个头文件都必须能由独立 C11 翻译单元包含，不依赖 HAL 头文件。

**Step 4: 实现安全应用门面**

`aethor_app.h` 只公开：

```c
/* 文件说明：Phase 0 固件应用生命周期与只读快照接口。 */
#ifndef APP_AETHOR_APP_H
#define APP_AETHOR_APP_H

#include <stdbool.h>
#include <stdint.h>
#include "arm_controller.h"
#include "diagnostics.h"

/** 初始化静态应用上下文。 */
void aethor_app_init(uint64_t timestamp_us);

/** 执行一次非阻塞的 Phase 0 应用服务。 */
void aethor_app_service(uint64_t timestamp_us);

/** 复制当前机械臂状态快照。 */
bool aethor_app_get_snapshot(ArmSnapshot *snapshot);

/** 按逻辑索引复制诊断事件。 */
bool aethor_app_get_diagnostic(uint16_t logical_index, DiagnosticEvent *event);

#endif
```

`aethor_app.c` 只使用静态文件作用域的 `Diagnostics` 和 `ArmController`，维护显式初始化标志；所有查询在未初始化时返回 `false`。不存在堆分配和外设写操作。

**Step 5: 运行测试并提交**

```powershell
& '.\Tests\host\run_phase0_tests.ps1'
git diff --check
git add App Tests/host/phase0_test_main.c Tests/host/run_phase0_tests.ps1
git commit -m "feat: add phase 0 layered application contracts"
```

## Task 5: 同步 CubeMX 静态任务与正式固件入口

**Files:**

- Modify: `CtrBoard-H7_FDCAN.ioc`
- Modify: `Core/Src/freertos.c` (CubeMX generated user blocks only)
- Modify: `Core/Src/main.c` (CubeMX generated user blocks only)

**Step 1: 修改 `.ioc` 的默认任务分配方式**

把 `FREERTOS.Tasks01` 中 defaultTask 的分配方式从动态改为静态，并指定 `defaultTaskBuffer` 与 `defaultTaskControlBlock`。保持任务栈 512 words，优先级 Normal。不得顺带新增 UART 或改变时钟/FDCAN/USB 配置。

验证：

```powershell
Select-String -Path '.\CtrBoard-H7_FDCAN.ioc' -Pattern 'FREERTOS.Tasks01'
```

Expected: 行内包含 `Static,defaultTaskBuffer,defaultTaskControlBlock`。

**Step 2: 使用当前 CubeMX 重新生成代码**

```powershell
Start-Process -FilePath 'E:\STM32CubeMX\STM32CubeMX.exe' -ArgumentList 'E:\Desktop_E\TCG\Aethor_robo_fw\aethor_robo_v1\CtrBoard-H7_FDCAN.ioc'
```

在 CubeMX 中仅执行 Generate Code。生成后检查 diff，确认用户代码区没有丢失，确认 `freertos.c` 使用 `osThreadStaticDef`。若 CubeMX 产生与任务无关的大范围改写，停止并定位版本或工程元数据差异，不提交该改写。

**Step 3: 接入应用初始化与 4 ms 服务周期**

在 `main.c` 的 `USER CODE BEGIN 2` 中调用：

```c
aethor_app_init((uint64_t)HAL_GetTick() * 1000ULL);
```

在 `freertos.c` 默认任务循环中调用：

```c
for (;;) {
    aethor_app_service((uint64_t)HAL_GetTick() * 1000ULL);
    osDelay(4U);
}
```

只在 CubeMX 用户代码块中添加 `aethor_app.h` 和 `stdint.h` 引用。不得调用旧 `aethor_application_init/process`。

**Step 4: 验证生成边界并提交**

```powershell
git diff --check
Select-String -Path '.\Core\Src\freertos.c' -Pattern 'osThreadStaticDef|aethor_app_service|osDelay\(4U\)'
Select-String -Path '.\Core\Src\main.c' -Pattern 'aethor_app_init'
git add CtrBoard-H7_FDCAN.ioc Core/Src/freertos.c Core/Src/main.c
git commit -m "build: switch phase 0 application task to static allocation"
```

## Task 6: 同步 Keil 工程并隔离旧电机示例

**Files:**

- Modify: `MDK-ARM/CtrBoard-H7_FDCAN.uvprojx`

**Step 1: 编辑 Keil 分组和包含路径**

新增 `App`、`App-Config`、`App-Arm`、`App-Telemetry` 分组，加入以下 C 文件：

```text
..\App\aethor_app.c
..\App\Config\arm_config.c
..\App\Config\build_info.c
..\App\Arm\arm_controller.c
..\App\Telemetry\diagnostics.c
```

添加包含路径：

```text
..\App;..\App\Config;..\App\Protocol;..\App\Arm;..\App\Motion;..\App\Motor;..\App\Telemetry;..\App\Platform
```

旧 `User` 文件均保留在磁盘；从当前 Keil 目标编译列表中移除旧应用/电机/七轴控制源，仅保留生成的 USB CDC 回调仍直接依赖的 `usb_cdc_transport.c`。不得改动用户已有 `<LayerInfo>` 内容。

**Step 2: 检查 XML 与目标源列表**

```powershell
[xml](Get-Content '.\MDK-ARM\CtrBoard-H7_FDCAN.uvprojx' -Raw) | Out-Null
Select-String -Path '.\MDK-ARM\CtrBoard-H7_FDCAN.uvprojx' -Pattern 'aethor_app.c|arm_config.c|arm_controller.c|diagnostics.c|usb_cdc_transport.c'
Select-String -Path '.\MDK-ARM\CtrBoard-H7_FDCAN.uvprojx' -Pattern 'User\\aethor_application.c|User\\seven_axis_controller.c|User\\s3519_motor.c'
```

Expected: XML 可解析；新增源和 `usb_cdc_transport.c` 存在；三个旧运行源无匹配。

**Step 3: 仅暂存计划内 XML 块并提交**

使用 `git diff -- MDK-ARM/CtrBoard-H7_FDCAN.uvprojx` 明确区分计划改动和 `<LayerInfo>` 用户改动，再使用交互式暂存只加入计划内块：

```powershell
git add -p MDK-ARM/CtrBoard-H7_FDCAN.uvprojx
git diff --cached --check
git diff --cached -- MDK-ARM/CtrBoard-H7_FDCAN.uvprojx
git commit -m "build: compile the phase 0 layered application"
```

Expected: 提交后 `<LayerInfo>` 仍以未提交用户变更留在工作树。

## Task 7: 增加架构守卫并完成自动化验证

**Files:**

- Create: `Tests/host/check_phase0_architecture.ps1`
- Create: `docs/handoffs/phase-00/test-results.txt`
- Create: `docs/handoffs/phase-00/build.txt`
- Create: `docs/handoffs/phase-00/map-summary.txt`

**Step 1: 写架构守卫脚本**

脚本必须失败于以下任一条件：

- `App` 下出现 `malloc`、`calloc`、`realloc`、`free`、`pvPortMalloc`、`vPortFree`。
- `App/Config`、`Protocol`、`Arm`、`Motion`、`Motor`、`Telemetry` 中包含 `stm32h7xx_hal`、`cmsis_os` 或 `FreeRTOS`。
- `App` 中出现可执行电机控制 API 名称 `enable_motor`、`set_position`、`set_velocity`、`send_can_frame`。
- `.ioc` 默认任务不是静态分配。
- Keil 目标缺少五个新增 C 文件或仍编译旧按键电机入口。
- `aethor_app.c` 直接包含 FDCAN/USB/UART 头文件。

脚本对每项输出 `[PASS]` 或 `[FAIL]`，任一失败时退出码非零。

**Step 2: 运行新旧主机测试**

```powershell
& '.\Tests\host\run_phase0_tests.ps1' *>&1 | Tee-Object '.\docs\handoffs\phase-00\test-results.txt'
& '.\Tests\host\run_tests.ps1' *>&1 | Tee-Object -Append '.\docs\handoffs\phase-00\test-results.txt'
& '.\Tests\host\check_phase0_architecture.ps1' *>&1 | Tee-Object -Append '.\docs\handoffs\phase-00\test-results.txt'
```

Expected: Phase 0 测试、旧主机回归测试、架构守卫全部退出码为 0。若旧脚本真实文件名不同，先用 `Get-ChildItem Tests/host -File` 查明现有入口，再在测试证据中记录实际命令。

**Step 3: 构建 Keil 固件**

```powershell
& 'E:\keil\UV4\UV4.exe' -b '.\MDK-ARM\CtrBoard-H7_FDCAN.uvprojx' -j0 -o '.\MDK-ARM\phase00-build.log'
Get-Content '.\MDK-ARM\phase00-build.log' | Tee-Object '.\docs\handoffs\phase-00\build.txt'
Select-String -Path '.\MDK-ARM\phase00-build.log' -Pattern '0 Error\(s\), 0 Warning\(s\)|Program Size'
```

Expected: `0 Error(s), 0 Warning(s)`；`Program Size` 写入证据。解析新 map 的 Code/RO/RW/ZI 和 FreeRTOS heap，写入 `map-summary.txt`，与 `config-snapshot/map-baseline.txt` 对比并解释差值；若出现不能由新增静态对象解释的堆增长，则本任务失败。构建成功只证明软件构建，不声明 UART、CAN 或电机硬件验证完成。

**Step 4: 提交测试与证据**

```powershell
git diff --check
git add Tests/host/check_phase0_architecture.ps1 docs/handoffs/phase-00/test-results.txt docs/handoffs/phase-00/build.txt docs/handoffs/phase-00/map-summary.txt
git commit -m "test: verify phase 0 architecture and firmware build"
```

## Task 8: 完成版本说明与多人交接

**Files:**

- Modify: `README.md`
- Modify: `CHANGELOG.md`
- Create: `docs/handoffs/phase-00/HANDOFF.md`
- Create: `docs/handoffs/phase-00/known-issues.md`

**Step 1: 更新项目入口说明**

`README.md` 说明当前正式入口是 Phase 0 静态应用，旧按键示例仅作为保留源码且不进入目标；列出分层目录、主机构建命令、Keil 构建命令、CubeMX 文件、硬件未验证边界和后续 Phase 1 入口。

**Step 2: 更新变更日志**

`CHANGELOG.md` 在 `Unreleased` 下记录：新增七轴验证配置、启动自检、结构化诊断、静态 RTOS 任务、Keil/CubeMX 同步、架构守卫；明确没有实现电机控制和正式 UART 数据链路。

**Step 3: 编写 Phase 0 交接**

`HANDOFF.md` 必须包含：

- 分支名与提交列表。
- PRD Phase 0 验收项逐条结果。
- 当前状态机实际路径。
- 内存策略与禁止动态分配的检查证据。
- 七轴 CAN ID 已确认项和所有未验证物理参数。
- CubeMX/Keil 版本与生成步骤。
- 测试命令、构建结果、产物路径。
- 未进行实机验证的明确声明。
- Phase 1 首个任务：参数确认机制与配置来源，不越级开发运动功能。

`known-issues.md` 记录：USART1 仅为正式链路候选且未写入 `.ioc`；真实关节参数未验证；FDCAN/电机未上电验收；旧 `User` 源码仍保留但不参与目标；`uvprojx` 用户 `<LayerInfo>` 工作树变更不属于本阶段提交。

**Step 4: 全量复核**

```powershell
& '.\Tests\host\run_phase0_tests.ps1'
& '.\Tests\host\run_tests.ps1'
& '.\Tests\host\check_phase0_architecture.ps1'
git diff --check
git status --short
git log --oneline --decorate -10
```

Expected: 三类测试全部通过；除执行前已有的 `<LayerInfo>` 用户元数据外，没有意外未提交变更。

**Step 5: 提交交接资料**

```powershell
git add README.md CHANGELOG.md docs/handoffs/phase-00/HANDOFF.md docs/handoffs/phase-00/known-issues.md
git commit -m "docs: hand off PRD phase 0 firmware baseline"
```

## 最终验收清单

- `App` 七层边界存在，核心逻辑可在主机脱离 HAL 编译测试。
- `JointConfig[7]` 的 CAN 标识固定且可验证，未知物理参数不能通过使能就绪检查。
- 正式固件启动后只进入配置未完成故障，不产生任何电机控制帧。
- 默认 RTOS 任务静态创建，`App` 无动态分配调用。
- `.ioc`、CubeMX 生成文件与 Keil 源列表同步。
- 新旧主机测试、架构守卫、Keil 编译均有可追溯结果。
- README、CHANGELOG、HANDOFF、known-issues 完整记录提交、工具链、限制与下一阶段入口。
- 用户已有 `<LayerInfo>` 变更未被覆盖、删除或混入计划提交。
