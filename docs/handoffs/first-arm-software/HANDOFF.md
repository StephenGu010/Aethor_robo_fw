# 首组七轴固件软件交接（历史 aethor-arm-ascii-v1 切片）

> 状态：已被根目录 `HANDOFF.md`、`docs/compatibility/aethor-text-v1-migration.md` 和 `docs/handoffs/aethor-text-v1-bench/verification.txt` 取代。本文件保留迁移前 CRC 协议的软件交付边界和验证记录，不是当前固件接口说明。

## 结论

首组 `arm-1` 的固件软件、主机验证和上位机兼容性测试包已形成闭环。可以声明：

> PRD 首组固件软件与上位机兼容性测试包完成；Aethor Studio V2 外部交付待契约验收，USB/CAN/电机/机械臂实机与机械参数待验证。

不能声明“PRD 首组软件部分全部完成”，因为外部 Aethor Studio V2 的可运行版本/commit 尚未取得，真实上位机九项契约验收没有证据。

## 已实现

- STM32H723、FreeRTOS 六静态任务、USB CDC、FDCAN1 Classic CAN。
- 七个 S3519 对象，ESC ID `0x01–0x07`、Master ID `0x11–0x17`，身份交叉校验、参数发现和反馈快照。
- 完整 `aethor-arm-ascii-v1` 查询与动作命令、会话、重放、心跳、STOP 高优先级和有界资源。
- `BOOT/SELF_TEST/UNALIGNED/DISABLED/ENABLING/READY/MOVING/STOPPING/FAULT` 状态机。
- RAM 参考位对齐、关节/电机双向换算、软限位门控。
- 正式七轴 POS_VEL 与 MIT 同步运动、完成稳定窗口、受控停止、快速失能回退。
- 默认台架 Profile 的显式单个/多个电机子集初始化、使能、相对小角度运动、停止、失能和清错。
- 50 Hz 关节遥测、10 Hz/状态变化电机遥测、诊断计数和故障注入路径。
- 协议模拟器、参考客户端、共享测试向量、Schema、RobotGatewayV1 清单和逻辑长稳。

## 软件证据

最终证据由以下命令重现：

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

Keil 必须执行全量 Rebuild，并保留 `0 Error(s), 0 Warning(s)`、HEX、Map 和构建日志。逻辑时钟长稳不替代真实墙钟和硬件长稳。

## 明确未验证

- 七台电机的真实接线、ESC/Master ID、`0x7FF/0xCC` 失能反馈查询能力。
- PMAX、VMAX、TMAX、ACC、DEC、MAX_SPD 和驱动版本。
- J1–J7 方向、参考姿态、软限位、最大安全速度/加速度、外部减速比和 MIT Kp/Kd。
- CAN 平均/峰值负载、七帧真实偏斜、跟随误差、到位差、温升和故障响应。
- 8 小时静态、2 小时运动的实机长稳。
- DH/URDF 和模型姿态；它们属于上位机/机械模型，不进入 STM32 关节空间运算。
- Aethor Studio V2 的真实版本、模型跟随、重连和 UI 生命周期验收。

## 下一步硬件入口

1. 只连接并固定一台无负载电机，确认供电、急停和 CAN 收发。
2. 使用只读查询确认 ID、模式、量程和版本，再做 `±3°`、不超过 `3°/s` 的台架动作。
3. 一次增加一轴并记录方向、零位、限位、减速比和故障；七轴参数全部完成前不开放生产整组使能。
4. 七电机全部失能接入后验证轮询、反馈年龄、Bus-Off 和 30 分钟 CAN 负载。
5. 最后才进行生产 Profile 的 POS_VEL、MIT 和实机长稳。

## 上位机入口

取得 Aethor Studio V2 可运行版本与 commit 后，执行 `docs/compatibility/compatibility-result-template.md` 的全部项目。只有外部记录通过后，才可把“上位机软件完成”计入 PRD 总进度。
