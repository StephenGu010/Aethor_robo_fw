# 变更记录

## Unreleased — PRD Phase 0

### 2026-08-14 一次提交的绝对台架运动合同

- 新增 `bench move <motors> position=<deg-list> speed=<deg/s-list>`：电机、位置和速度列表严格等长并按顺序一一对应，电机编号唯一且限于 1–7，数值必须有限且速度大于 0，不提供广播。
- 位置采用相对本次上电零点的 S3519 输出端绝对角度；运行时只使用本次发现的 `PMAX/VMAX/MAX_SPD` 校验，边界值允许，越界值拒绝且不截断，发现值缺失时不回退默认范围。
- `show motor` 新增 `pmax_deg/vmax_deg_s/max_speed_deg_s/move_speed_limit_deg_s`；缺失值显示 `?`，数值沿用三位小数的紧凑格式。
- 新命令由主机只发送一次，不启动 `ping` 保活；固件内部重发固定 CAN 目标，完成发现、模式、清错、使能、运动、HOLD、所选电机失能，并在新鲜 disabled 反馈后报告完成。
- 保留旧 `bench enable/jog` 的约 250 ms `ping` 要求和 1000 ms 通信看门狗；新命令只在其自包含生命周期实际持有期间局部豁免通信超时，驱动故障、反馈新鲜度、控制周期和 Bus-Off 安全路径保持有效。
- 新增稳定的 `bench move` 完成/失败/取消/停止终态、阶段和错误码合同；`bench stop` 可抢占，普通并发命令返回 `busy`，重放与 `request_conflict` 语义保持不变。
- 同步兼容性 Manifest、共享向量、参考客户端和使用文档。参考客户端会先本地验证列表和值，只写入一次动作正文并等待匹配的 `ok/done`，不会发送保活。
- 本轮软件与主机测试不证明机械软限位、方向、减速比、带载性能、七轴联动或真实硬件验收。

### 2026-08-13 aethor-text-v1 与 COM7 台架验证

- 固件正式 USB CDC 入口迁移为 `aethor-text-v1`：使用可打印 ASCII 和 LF/CRLF 行边界，不再使用应用层 CRC 和 `!/#/>/@/&/$` 前缀。
- 新增 `hello/ping/help`、`show/stream`、台架 `bench` 与正式机械臂 `arm` 命令路径；输出统一为 `ok/done/error/event/data`。
- 请求编号改为可选十进制 `uint32`；非零编号支持 60 秒近期结果重放，相同编号不同正文会被拒绝，编号 `0` 用于不重放的手工调试。
- `bench jog` 只需由上位机提交一次。动作未完成时，固件在内部周期重发固定 CAN 目标批次；上位机只需在电机使能或运动期间每 250 ms 或更快发送有效 `ping`。
- 通信看门狗保持 1000 ms：仅在存在已使能电机或活动运动时触发，超时执行停止和失能；全部电机失能且空闲时不触发。
- 新增文本协议解析器、协议引擎、正式机械臂 Profile、确定性模拟器、参考客户端和 COM7 调试脚本测试。
- ARMCC 5 全量构建为 `0 Error(s), 0 Warning(s)`；镜像已通过 CMSIS-DAP 下载与校验。
- COM7 已验证空载 S3519 CAN ID 1、3 的发现、使能、正反向 `1°` 点动、停止和失能，最终 `enabled=00`、`fault=00`、`can_error=0`。
- 实机结果不证明实际输出角度/速度比例、带载性能、关节方向/零位/限位或七轴整机运动；生产配置 `verified_fields=00` 继续禁止使能。

### 2026-08-13 首组七轴固件软件闭环（历史 aethor-arm-ascii-v1 切片）

> 本节记录迁移前的软件闭环。其 CRC 协议和大写命令已由上面的 `aethor-text-v1` 正式入口取代，只保留为回归测试资产。

- 正式传输切换为 Type-C USB CDC，完成 `aethor-arm-ascii-v1` 的 CRC、分包/粘包、会话、重放缓存、心跳看门狗、查询、动作 `ACK/DONE` 和优先级发送。
- 完成六个静态 FreeRTOS 任务、FDCAN1 适配、七电机 S3519 发现/反馈/模式回读、整组原子 CAN 调度和安全失能。
- 完成启动状态机、RAM 参考位、七轴一致性快照、POS_VEL 共同到达、MIT 五次时间标度、受控停止和台架显式电机子集控制。
- 完成 50 Hz `JOINT_STATE`、10 Hz/状态变化 `MOTOR_STATE`、运行诊断、Bus-Off/overflow/驱动/反馈/实时性/断线故障处理。
- 新增确定性七电机主机模拟器、脚本化参考客户端、Schema、Golden Frames、分包/坏帧向量、外部 Aethor Studio V2 验收模板，以及 8 小时/2 小时逻辑时钟长稳。
- 生产配置仍因方向、参考位、限位、速度、加速度、驱动量程、减速比和 MIT 增益未实测而禁止使能；USB/CAN/电机/机械臂硬件与 Aethor Studio V2 外部验收未计为完成。

### 新增

- 建立 `App/Config`、`Protocol`、`Arm`、`Motion`、`Motor`、`Telemetry`、`Platform` 分层契约和统一 `aethor_app` 入口。
- 新增固定七轴配置模型；ESC ID 1–7、Master ID 11–17 已固化，所有未实测物理参数用验证位阻止误使能。
- 新增 `BOOT → SELF_TEST → FAULT(CONFIG_INCOMPLETE)` 安全启动路径、固定 64 项结构化诊断环和饱和计数器。
- 新增 Phase 0 主机测试与架构守卫，检查动态分配、层间依赖、CubeMX 静态任务和 Keil 源列表。

### 变更

- CubeMX 默认任务改为静态创建，`aethor_app_service()` 以 4 ms 周期运行；USB 初始化保持在调度器启动后的默认任务中。
- Keil 目标改为编译 Phase 0 分层应用；旧 `User/` 源码保留在磁盘，但旧按键双电机和旧七轴控制链不参与当前固件构建。
- 同步 STM32CubeH7 1.11.2 的 USB HAL/中间件生成依赖与当前 `.ioc`。

### 验证边界

- Phase 0 主机测试、旧代码回归测试、架构守卫和 ARMCC 5 构建均通过；Keil 结果为 `0 Error(s), 0 Warning(s)`。
- 本阶段没有实现电机使能、位置/速度控制、同步轨迹、正式 UART 数据链路、DH 正逆解、RGB 或上位机业务协议。
- 未进行本固件镜像的 USB、CAN 或电机实机验证；软件测试与构建结果不得解释为七轴硬件验收完成。

## 2026-08-12

### PA15 双电机运行入口

- 当前 `aethor_application` 已切换为 `dual_motor_controller`，每 5 ms 读取低有效 PA15。
- 每次完成并释放按键后可再次触发；两台电机每次各转一圈，方向按 `+2π/-2π` 交替，速度上限为 `0.5 rad/s`。
- 首选方向超过任一电机 PMAX 时自动改选两台共同安全的反方向；只有两个方向都不可行时才锁存目标范围故障。
- FDCAN 接收和 Bus-Off 回调已切换到双电机状态机；七轴模块保留但当前不运行。
- COM7 保留阶段、掩码、按键、累计受理次数、下一方向、起点、目标、位置、速度、反馈年龄和故障探针。
- USB 仅允许查询和诊断；所有运动类命令返回 `err key-only-control`。
- 新增 CAN 初始化失败的双电机诊断故障路径。

### 新增

- 新增固定 7 轴 `RobotConfiguration`，集中保存 DH、CAN/Master ID、方向、零位、外部减速比、限位、速度/加速度和标定状态。
- 新增关节角与电机输出端弧度之间的双向换算，预留外部减速器参数化接口。
- 新增无动态内存的七轴同步轨迹和关节控制状态机。
- 新增 S3519 `CTRL_MODE=2` 临时写入/回读、PMAX/VMAX/TMAX 回读、Master ID 与 D0 电机 ID 双重校验。
- 新增 USB CDC 七轴命令、诊断查询、DH/配置查询，以及 `>` 顺序动作和 `&` 可打断动作。
- 新增结构化固件探针与依赖 .NET 的 COM7 监视脚本。
- 新增两电机启动、错误 Master ID、三次重试、模式错误、使能反馈、速度单位和故障路径的 Windows 主机测试。

### 变更

- 应用入口从按键双电机演示切换为七轴关节控制器；原双电机模块保留在工程中但不再由应用调用。
- FDCAN1 过滤槽数量由 2 改为 7；运行时仅为 `active_joint_mask` 中的 Master ID 配置精确过滤器。
- USB CDC 发送队列深度增至 8，单行容量增至 512 字节，用于同时容纳命令应答和诊断探针。
- READY 状态下每 100 ms 发送失能查询，使位置和反馈年龄保持新鲜且不使能电机。
- 运动超时调整为 30 秒，并且只在轨迹运行期间计时；终点保持不会因普通命令空闲而自动失能。
- 位置速度模式中的 `v_des` 始终作为非负速度上限发送。

### 安全边界

- 当前活动轴只有 J1/J2；两轴均保持 `commissioned=0`，完整多轴使能锁定。
- 单轴标定必须先 `#SELECT`，动作目标限制在该轴使能位置的 `±3°`。
- 反馈超过 50 ms、电机状态故障、CAN 发送失败和 Bus-Off 均锁存故障并尽力失能。
- DH 参数仍标记为无效；不实现或接受逆运动学与 RGB 命令。

### 验证

- Windows 主机测试：`HOST_TESTS_PASSED`。
- Keil ARMCC 5 全量构建：`0 Error(s), 0 Warning(s)`。
- CMSIS-DAP/OpenOCD：Flash 写入完成且 `Verified OK`。
- COM7 实机：两台电机完成 `CONFIG_VALIDATE → BOOT → MODE_SETUP → RANGE_DISCOVERY → READY`，`mode/ranges/feedback=0x03`，`enabled=0x00`，CAN 无 warning、passive 或 Bus-Off。
- 尚未执行任何电机运动验收；不能据此声明关节方向、零位、限位、外部减速比或运动精度已经验证。
