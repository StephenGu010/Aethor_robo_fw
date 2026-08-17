# 首组 PRD 软件需求追踪审计

审计日期：2026-08-13。状态分为“软件完成”“外部待验收”“硬件待验证”。后两类不是用模拟值填充的软件缺口，也不能计为整体 PRD 完成。

| 目标章节 | 状态 | 主要证据与边界 |
|---|---|---|
| 统一 PRD、USB CDC、ID、双 Profile | 软件完成 | `App/Config`、`board_config.h`、生产 Profile 编译测试；DH 不进入 STM32 |
| 六静态 FreeRTOS 任务与固定容量 | 软件完成 | `Core/Src/freertos.c`、平台队列、架构守卫 |
| ISR 只搬运、任务解码、七帧原子入队 | 软件完成 | `stm32_platform.*`、`can_rx_inbox.*`、`can_tx_scheduler.*` |
| S3519 MIT/POS_VEL/模式命令/参数/反馈 | 软件完成 | `s3519_codec.*`、`motor_discovery.*`、`motor_runtime.*` 与固定帧测试 |
| `0x7FF/0xCC` 明确查询 | 软件完成，硬件待验证 | 编码路径和固定帧测试存在；`S3519_EXPLICIT_FEEDBACK_QUERY_VALIDATED=0` 防止误称实测 |
| 七电机独立状态和部分电机台架 | 软件完成 | 生命周期对象、显式 `motors=` 子集测试、未选轴不变 |
| 完整 `aethor-arm-ascii-v1` | 软件完成 | CRC、分包/粘包、帧型、会话、32 项/60 秒重放、命令队列、STOP 槽、查询与动作测试 |
| HELLO/心跳/超时重连语义 | 软件完成 | 250 ms 契约、1,000 ms STOP_DISABLE、boot/session ID、重连兼容测试 |
| 七轴状态机和 RAM 参考位 | 软件完成 | `arm_controller.*`、`joint_reference.*`、重启失效与转换测试 |
| POS_VEL 七轴整组控制 | 软件完成 | 原子目标校验、共同到达时长、完成稳定窗口和反馈门控测试 |
| MIT 五次时间标度 | 软件完成 | 4 ms 采样、速度/加速度边界、七帧编码和调度测试；增益仍待实机标定 |
| STOP 非 DISABLE、快速失能回退 | 软件完成 | 共同有界减速、反馈失效/调度失败回退和恢复测试 |
| 一致性快照 | 软件完成 | CAN 单写者、seqlock 发布、查询/遥测只读快照 |
| 50 Hz JOINT_STATE、10 Hz MOTOR_STATE | 软件完成 | 独立节拍、状态变化立即发布、替换式队列与模拟长稳 |
| 运行诊断 | 软件完成 | 周期、miss、CAN/USB、队列、栈/堆、逐轴年龄、启动四掩码、活动请求/预计/实际/最大误差 |
| 故障与恢复 | 软件完成 | 驱动、反馈、Bus-Off、overflow、原子组失败、deadline、USB 高优先级拥塞、链路超时 |
| 主机模拟器与参考客户端 | 软件完成 | 七电机、运动、遥测、故障、分片/粘包/坏帧、脚本化只读探测 |
| Schema/Golden Frames/RobotGatewayV1 | 软件完成 | `docs/compatibility` 与 `Tests/protocol` |
| 8h/2h 软件长稳 | 软件完成 | 加速逻辑时钟 8h 静态、2h MOVE/STOP/恢复；不代表墙钟/硬件长稳 |
| Windows 主机测试和 Keil Rebuild | 软件完成 | 所有套件通过；ARMCC 0 error/0 warning；68 个源引用、0 缺失；HEX/Map/哈希见 `verification.txt` |
| CubeMX 重新生成一致性 | 已有 Phase 0 证据 | Phase 0 已用 6.11.1 同步；本轮 App/测试改动未再次覆盖生成文件，架构守卫复核 `.ioc`、六静态任务和入口 |
| Aethor Studio V2 正式兼容 | 外部待验收 | 未取得可运行版本/commit；必须填写外部验收模板，不声明已完成 |
| USB/CAN/七电机/机械臂实机 | 硬件待验证 | 软件和构建不能证明枚举、收发、方向、精度、负载、温升或长稳 |
| 方向/零位/限位/量程/减速比/Kp/Kd | 硬件待验证 | 生产配置验证位继续锁定使能 |
| DH/URDF 与姿态 | 机械/上位机待定 | 机械结构确定后建立模型，不进入 STM32 关节空间控制 |
| 第二组机械臂 | 仅预留 | 首组完成；身份/会话扩展存在，不实现正式双会话 |

## 审计结论

在“排除外部 Aethor Studio V2、真实硬件、机械参数和 DH/URDF”的口径下，目标计划中的首组固件软件与兼容性测试包已完成。当前剩余项都有明确外部证据依赖，不能由继续编写固件代码真实关闭。
