# Aethor STM32H723 固件交接说明

## 当前运行入口

- 分支：`refactor/prd-phase-00`
- MCU：STM32H723VGT6
- 正式传输：板卡 Type-C USB CDC
- 正式协议：`aethor-text-v1`，可打印 ASCII，LF/CRLF 结尾，无应用层 CRC
- 默认 Profile：`USB_BENCH_RELATIVE`，对外命令路径为 `bench init/enable/jog/stop/disable/clear`
- 电机：S3519/DM3520，七轴 ESC ID `0x01–0x07`、Master ID `0x11–0x17`
- 正式机械臂 Profile：已实现七轴关节空间软件路径，但物理验证掩码未放行，不能真实整组使能

旧 PA15 按键双电机程序、旧 `#PING` 命令和 `aethor-arm-ascii-v1` CRC 协议均为历史/回归资产，不是当前 Keil 固件入口。

## 当前协议最小操作

请求可以直接在串口终端逐行输入：

```text
hello
stream off
show info
show config
show state
show motors
show diag
```

安全台架动作使用非零请求编号：

```text
10 bench init 1
11 bench clear 1
12 bench enable 1
13 bench jog 1 delta=0.2 speed=1
14 bench stop 1
15 bench disable 1
```

动作命令只提交一次。收到 `ok <id> ... accepted=1` 后等待同一编号的 `done`；运动未到位时，固件会在内部周期重发固定 CAN 目标。电机使能或运动期间，上位机每 250 ms 或更快发送独立 `ping`，连续 1000 ms 无有效请求时固件停止并失能。全部电机失能且无运动时，通信看门狗不触发停止/失能。

## 当前可复现证据

- 全部主机协议、应用、运动、电机、平台、模拟器和架构测试已通过。
- ARMCC 5 全量构建结果为 `0 Error(s), 0 Warning(s)`。
- 生成 HEX 已通过 CMSIS-DAP/OpenOCD 下载与校验。
- COM7 已验证空载 S3519 CAN ID 1、3 的身份/模式/量程/版本读取、使能、正反向 `1°` 点动、停止和失能。
- 最终状态为所选电机全部失能，驱动故障和 CAN 错误为零。

精确工具版本、构建尺寸、固件哈希、烧录记录和 COM7 结果见：

```text
docs/handoffs/aethor-text-v1-bench/verification.txt
```

该证据不证明实际输出角度或速度比例。实测完成时间明显短于名义 `1°/s` 预期，S3519 POS_VEL 速度比例和机械输出比例仍需测量。

## Windows 调试入口

只读探测，默认不会使能或转动电机：

```powershell
powershell.exe -ExecutionPolicy Bypass -File `
  .\Tests\hardware\debug_com7_aethor_text_v1.ps1 `
  -PortName COM7 -MotorList 1,3
```

显式小角度运动必须从一台空载电机开始：

```powershell
powershell.exe -ExecutionPolicy Bypass -File `
  .\Tests\hardware\debug_com7_aethor_text_v1.ps1 `
  -PortName COM7 -MotorList 1 -RunMotion `
  -DeltaDegrees 0.2 -SpeedDegreesPerSecond 1
```

只有上一阶段完整返回 `done`、最终失能且无故障，才允许测试下一台电机。不得直接执行七轴同时使能；任一方向异常、噪声、堵转、过热、反馈超时、驱动故障或 Bus-Off 都应立即停止并切断 24 V 电机电源。

## 软件验证命令

```powershell
powershell.exe -ExecutionPolicy Bypass -File .\Tests\host\run_tests.ps1
powershell.exe -ExecutionPolicy Bypass -File .\Tests\host\run_phase0_tests.ps1
powershell.exe -ExecutionPolicy Bypass -File .\Tests\host\run_protocol_tests.ps1
powershell.exe -ExecutionPolicy Bypass -File .\Tests\host\run_text_protocol_tests.ps1
powershell.exe -ExecutionPolicy Bypass -File .\Tests\host\run_text_protocol_engine_tests.ps1
powershell.exe -ExecutionPolicy Bypass -File .\Tests\host\run_text_protocol_arm_profile_tests.ps1
powershell.exe -ExecutionPolicy Bypass -File .\Tests\host\run_motor_core_tests.ps1
powershell.exe -ExecutionPolicy Bypass -File .\Tests\host\run_platform_io_tests.ps1
powershell.exe -ExecutionPolicy Bypass -File .\Tests\host\run_motion_tests.ps1
powershell.exe -ExecutionPolicy Bypass -File .\Tests\host\run_simulator_tests.ps1
powershell.exe -ExecutionPolicy Bypass -File .\Tests\host\test_com7_dual_motor_debug_script.ps1
powershell.exe -ExecutionPolicy Bypass -File .\Tests\host\check_phase0_architecture.ps1
```

旧 `run_protocol_tests.ps1` 继续验证迁移前 CRC 回归资产；三个 `run_text_protocol_*` 脚本才验证当前正式文本协议。

## 未完成边界

- J1–J7 的实际方向、机械零位、软限位、最大安全速度/加速度、外部减速比和 MIT Kp/Kd 尚未逐轴标定。
- 七电机带载、同步误差、温升、CAN 负载和实机长稳尚未完成。
- DH/URDF、逆运动学、笛卡尔控制、动力学和碰撞规划属于上位机/机械模型范围。
- Aethor Studio V2 的真实版本、模型跟随、请求重放、断线恢复和 UI 生命周期仍需按 `docs/compatibility/compatibility-result-template.md` 外部验收。

在生产验证掩码全部有效之前，不得通过修改默认值绕过固件使能门控。
