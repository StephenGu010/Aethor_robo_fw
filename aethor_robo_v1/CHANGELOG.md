# 变更记录

## 2026-08-12

### PA15 双电机运行入口

- 当前 `aethor_application` 已切换为 `dual_motor_controller`，每 5 ms 读取低有效 PA15。
- 每次完成并释放按键后可再次触发；两台电机每次各转一圈，方向按 `+2π/-2π` 交替，速度上限为 `0.5 rad/s`。
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
