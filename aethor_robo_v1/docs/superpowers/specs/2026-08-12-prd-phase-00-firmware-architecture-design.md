# PRD Phase 0 固件框架设计

**状态：** 已批准方案的设计规格
**日期：** 2026-08-12
**需求基线：** `E:\Desktop_E\TCG\固件开发prd` 0.1.0-draft
**目标工程：** STM32H723VGT6、STM32CubeH7 1.11.2、STM32CubeMX 6.11.1、Keil MDK 5.43 / ARMCC 5.05 update 1、FreeRTOS CMSIS-RTOS V1

## 1. 目标与边界

本阶段只实现 PRD Phase 0：在现有 `aethor_robo_v1` 中建立可继续演进的业务分层、七轴配置契约、安全启动基线、静态资源规则、Host 测试和可交接的 Git 证据。Phase 0 不使能电机、不发送运动控制帧，也不把已有双电机实机验证结果解释为七轴能力。

本阶段不实现正式 UART 线协议、持续遥测、参考位对齐、电机动作、MIT 插补、七轴同步、上位机功能或第二组机械臂。上述能力分别属于 PRD Phase 1–8，只有前一阶段退出条件满足后才能接入。

## 2. 采用的重构策略

采用原工程内的增量迁移，不复制第二套 CubeMX、HAL、FreeRTOS 或 Keil 工程：

1. `Core/`、`Drivers/`、`Middlewares/`、`USB_DEVICE/` 和 `.ioc` 保持 CubeMX 所有权。
2. 新建 `App/` 作为全部自研业务代码的唯一长期位置。
3. 现有 `User/` 文件先保留，按模块通过测试后再替换工程引用；Phase 0 不批量删除旧代码。
4. Phase 0 的生产入口进入安全诊断状态，不调用现有 PA15 双电机运动控制器或七轴运动控制器。
5. 每个迁移单元独立提交，使目录迁移、行为变更、CubeMX 生成结果和文档证据可以分别审查。

不采用新建 `aethor_robo_v2`，以免产生两套供应商代码和外设配置；不采用一次性全量移动，以免丢失现有 CAN/USB/电机协议回归基线。

## 3. 目录与依赖

```text
aethor_robo_v1/
├─ CtrBoard-H7_FDCAN.ioc
├─ MDK-ARM/
├─ Core/
├─ Drivers/
├─ Middlewares/
├─ USB_DEVICE/
├─ App/
│  ├─ Config/       # 身份、版本、JointConfig[7]、配置校验
│  ├─ Protocol/     # Phase 0 只定义协议边界；正式实现属于 Phase 3
│  ├─ Arm/          # 控制器状态所有权和安全启动状态
│  ├─ Motion/       # 运动接口边界；Phase 0 不生成设定值
│  ├─ Motor/        # DM3520 编解码/传输边界；Phase 1 开始迁移
│  ├─ Telemetry/    # 诊断快照、计数器和结构化事件
│  └─ Platform/     # FDCAN、UART、时钟、RTOS 和看门狗适配
├─ Tests/
│  ├─ host/
│  └─ bench/
└─ docs/
   └─ handoffs/phase-00/
```

依赖方向固定为：

```text
Protocol → Arm → Motion → Motor → Platform
                ↘ Telemetry ← Platform
Config 为只读叶子依赖，可被各业务层读取但不依赖任何业务层。
```

`Platform` 不能调用 `Protocol` 或直接格式化上位机文本；ISR 不能改变 Arm 状态。状态转换只能由 Arm 层的唯一所有者完成。

## 4. Phase 0 模块职责

### 4.1 Config

`App/Config` 提供只读构建信息、部署身份和七轴配置：

- `product = aethor-robo`
- `controller_id = aethor-arm-controller-01`
- `arm_id = arm-01`
- `dof = 7`
- `protocol = aethor-arm-ascii-v1`
- J1–J7 `motor_can_id = 1..7`
- J1–J7 `master_id = 11..17`

方向、机械参考位、软限位、速度、加速度、MIT Kp/Kd 和实际 PMAX/VMAX/TMAX 均没有七轴实测证据。它们不使用看似合理的猜测值解锁功能，而由逐字段有效位表示“未验证”。配置校验分为两级：

1. `arm_config_validate_schema()` 校验七个索引、ID 唯一性、枚举、字段结构和已声明有效数值的有限性/范围。
2. `arm_config_get_readiness()` 汇总部署就绪位；任何运动必需参数未验证时返回 `ARM_CONFIG_INCOMPLETE`。

这样 Phase 0 可以拥有结构合法的配置，同时确保未验证参数必然阻止使能。

### 4.2 Arm

`App/Arm` 是控制状态的唯一所有者。Phase 0 只启用：

```text
BOOT → SELF_TEST → FAULT(CONFIG_INCOMPLETE)
```

如果身份、ID 或内部结构校验失败，故障原因使用更具体的 `CONFIG_INVALID`。由于七轴机械参数尚未验证，Phase 0 不进入 `UNALIGNED/DISABLED`，更不会进入 `ENABLING/READY/MOVING`。

Arm 对外只发布不可变状态快照；Platform、Protocol 和测试代码不得直接写内部状态。

### 4.3 Protocol

Phase 0 只冻结 `aethor-arm-ascii-v1` 的端口和类型边界，不实现正式命令执行。协议模块预留固定上限：512 字节输入行、32 条近期结果、8 条业务命令、高优先级响应与可丢旧遥测分离。

旧 Dummy 命令和当前 USB CDC 命令不作为新协议兼容层。正式 CRC、请求幂等、ACK/RSP/ERR/EVT/TEL/DONE 在 Phase 3 按 PRD 测试向量实现。

### 4.4 Motion 与 Motor

Phase 0 仅定义不依赖 HAL 的接口和数据类型，不产生任何 CAN 控制帧：

- Motion 接收七轴关节角、速度比例和模式，未来拥有统一时间标度与完成判定。
- Motor 未来拥有 DM3520 MIT/POS_VEL 编解码、参数读取和反馈解码。
- Platform 是唯一允许接触 `HAL_FDCAN_*` 的层。

现有 `dm_motor_protocol`、`joint_controller` 和 `sync_trajectory` 作为迁移参考保留；它们在新 Phase 0 安全入口中不被调用。

### 4.5 Telemetry

Phase 0 建立无文本格式化的诊断模型：启动阶段、配置状态、故障原因、控制周期、队列高水位、CAN/UART 计数和静态内存水位。日志项使用固定大小结构体和固定容量环形缓冲，不在实时路径调用 `printf/snprintf`。

## 5. 平台与 CubeMX 设计

当前 `.ioc` 已确认：STM32H723VGT6、480 MHz CPU、TIM23 HAL 时基、FreeRTOS CMSIS-V1、FDCAN1 1 Mbps、7 个标准过滤器以及 USB CDC。当前没有启用 UART。

Phase 0 的处理原则：

- 保留已经工作的 FDCAN1、USB CDC、时钟和引脚配置，不删除无关外设。
- 将默认 FreeRTOS 任务改为静态创建，并使业务层在调度器启动后不调用动态分配。
- 在 `Platform` 中定义 UART 端口接口；正式 UART DMA + IDLE 数据面在 Phase 3 实现。
- 将 USART1（PA9 TX、PA10 RX、921600 8N1）记录为首选上位机 UART，因为板卡手册和官方 `CtrBoard-H7_UART` 示例均确认该引脚与速率。Phase 0 不把它标记为已完成链路；接线、DMA RX 和 PC 收发必须在后续阶段验证。
- USB CDC 仅作为现有工程能力保留，不承载 `aethor-arm-ascii-v1` 的正式控制权。

CubeMX 6.11.1 重新生成后，必须检查用户代码块、FDCAN1 参数、USB、TIM23、FreeRTOS 静态任务设置和 Keil 文件引用。生成差异与手写业务差异分开提交。

## 6. 静态内存与并发规则

Phase 0 起执行以下规则：

- `App/` 禁止 `malloc/free/calloc/realloc/new/delete`。
- FreeRTOS 任务、队列、事件和流缓冲使用静态创建接口或 CubeMX 静态配置。
- 所有缓冲区、队列、快照和结果缓存容量为编译期常量，并使用 `_Static_assert` 校验关键尺寸和七轴数量。
- 环形缓冲使用显式读写索引和溢出计数；不允许静默覆盖高优先级命令或响应。
- Host 构建启用断言；目标构建对外部输入返回错误，对内部不变量锁存故障。
- 任务栈水位、FreeRTOS 堆余量和队列高水位纳入诊断。保留 `heap_4.c` 只用于兼容现有中间件不等于允许 App 运行期分配；Phase 0 构建报告必须说明实际堆使用来源。

## 7. 迁移映射

| 现有模块 | 长期位置 | Phase 0 处理 |
|---|---|---|
| `User/robot_config.*` | `App/Config/arm_config.*` | 重新按 PRD 配置模型实现，不直接搬运旧减速器假设 |
| `User/aethor_application.*` | `App/aethor_app.*` | 替换为安全启动与服务入口 |
| `User/firmware_probe.*` | `App/Telemetry/` | 提取固定结构诊断，移除实时路径文本格式化 |
| `User/bsp_fdcan.*` | `App/Platform/` | Phase 1/2 迁移；Phase 0 只定义接口边界 |
| `User/dm_motor_protocol.*` | `App/Motor/` | Phase 1 依据供应商测试向量重构 |
| `User/joint_controller.*` | `App/Arm/` + `App/Motion/` | 后续阶段拆分，不作为 Phase 0 运行入口 |
| `User/sync_trajectory.*` | `App/Motion/` | Phase 6 由 PRD 五次时间标度替代或重构 |
| `User/usb_command.*`、`usb_cdc_transport.*` | 旧调试链路 | 保留但不定义新正式协议 |
| `User/dual_motor_controller.*` | 旧验证模块 | 保留源码但从 Phase 0 生产入口隔离 |

## 8. 错误处理与安全启动

Phase 0 的安全不变量：

1. 复位后不会因为按键、USB 数据、CAN 反馈或配置缺省值发送使能/运动帧。
2. ID 重复、索引缺失、方向非法、参数非有限、范围非法或参数未验证都会阻止运动权限。
3. 未实现的协议入口返回明确不可用状态，不复用裸 `ok`。
4. FDCAN bus-off、RX overflow、控制周期严重超期和内部断言失败均可进入锁存故障。
5. 故障不能由普通查询清除；未来 `CLEAR_FAULT` 也必须满足去使能、无运动和故障来源消失。

## 9. 测试与验证

Phase 0 Host 测试至少覆盖：

- 七个关节索引完整且唯一。
- CAN ID 固定为 1–7，Master ID 固定为 11–17，并能发现重复/越界。
- 未验证参数使部署就绪检查失败。
- 已声明有效的方向只能为 `+1/-1`。
- 已声明有效的限位、速度、加速度、增益和映射范围必须有限且合法。
- Arm 启动从 `BOOT` 经 `SELF_TEST` 进入 `FAULT(CONFIG_INCOMPLETE)`，不产生电机发送调用。
- 固定容量、结构尺寸和禁止动态分配的源码扫描。

目标验证包括：

- CubeMX 6.11.1 重新生成成功，业务目录未被覆盖。
- Keil 全量 Rebuild 为 0 error；warning 必须逐项解释，目标为 0 warning。
- `git diff --check` 通过，Keil 工程引用无缺失文件。
- Map 文件记录代码、RO/RW/ZI 和堆栈基线，并确认没有无法解释的增长。
- 本阶段不烧录并驱动电机；若只做上电静态检查，也必须保持电机物理失能并单独记录证据边界。

## 10. Git 与交接

工作分支为 `refactor/prd-phase-00`。提交按以下责任拆分：

1. `docs: define PRD phase 0 firmware architecture`
2. `phase(0): add layered firmware configuration baseline`
3. `phase(0): synchronize CubeMX and Keil project`
4. `docs: record phase 0 verification and handoff`

现有未提交的 Keil `LayerInfo` 元数据保持隔离，先确认是否由 uVision 自动生成，再决定归入 CubeMX/Keil 同步提交或保留为用户改动。不会推送、合并或删除旧文件。

Phase 0 交付文档位于 `docs/handoffs/phase-00/`，只记录实际完成和验证结果；未执行的硬件测试必须明确标记为未验证。

## 11. Phase 0 完成条件

- PRD 规定的 App 分层和单向依赖已经建立。
- 七轴身份及 ID 映射已冻结，未知机械参数被显式标记并阻止使能。
- Phase 0 安全入口不调用任何运动控制器。
- App 和 RTOS 对象满足静态分配规则，资源基线可观测。
- CubeMX 重新生成、Host 测试、Keil Rebuild、文件引用和 Map 基线均有证据。
- `CHANGELOG`、Phase 00 `HANDOFF`、构建结果、测试结果、配置快照和已知问题已更新。
- 没有把编译、Host 测试或静态上电检查描述为电机运动或七轴实机通过。
