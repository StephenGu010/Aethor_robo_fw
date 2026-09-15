# Aethor 七自由度机械臂固件

## 当前发布：Astra 参数弹窗对齐版 B2

2026-09-15速度弹窗修正：POS、MIT默认10°/s居中，增减1°/s时放大附近滑块位移，数值与实际速度范围不变。见 [修正说明](docs/debug-ui/speed-popup-centered-20260915.md) 与 [修正版固件](firmware/2026-09-15-astra-speed-preview/README.md)。下方B2原始固件继续保留。

最新参数弹窗已按确认的参考图对齐圆角、居中排版、Noto字号和细滑轨，见 [弹窗对齐说明](docs/debug-ui/astra-popup-alignment.md)。可烧录HEX、目标选择和源码校验记录见 [B2固件目录](firmware/2026-09-14-astra-popup-preview/README.md)。已完成软件验证，尚未烧录；最新实屏及硬件时序仍待上板确认。B1 版保留在 `archive/lvgl-astra-ui-b1`，此前固件归档不变。

`feature/lvgl-astra-ui` 基于已保留的方案 A 实施三图标首页、六项电机列表、字段弹窗和分类诊断。操作及软件验证见 [Astra 界面 B](docs/debug-ui/astra-ui-verification.md)。本版不烧录，方案 A 分支、HEX 和原预览继续保留。

## 已保留的图标界面 A

`feature/lvgl-icon-ui` 在下述已上传基线之后实现共享LVGL图标界面：两屏动作菜单、大数值编辑、800 ms确认与独立失能结果，LVGL池预留降至32 KiB。当前操作及软件验证见[图标界面说明](docs/debug-ui/icon-ui-verification.md)。本次没有烧录；固定基线HEX保留不变。

## 已保留的 2026-09-14 文字界面基线

此前文字界面基线为电机7的 **LCD-MIT 输出轴坐标与3.5 Nm反馈保护版本**，同时支持本地POS。固定HEX与可移植验证摘要见 [文字版固件目录](firmware/2026-09-14/README.md)。该目录保留原文字式LVGL界面；最新图标界面使用上方B2固件目录。

LCD角度以输出轴、本次上电零点为基准；POS使用固定终点及速度上限，MIT使用本地轨迹。正常动作后保留本地控制，每个动作仍需松开后长按800 ms确认，运行中中键立即请求STOP。LCD Kp=80、Kd=0.2；3.5 Nm为反馈保护阈值，不是瞬时硬限流。操作与证据边界见 [最新发布说明](docs/debug-ui/release-20260914.md)。

当前分支已经实现 PRD 首组七自由度机械臂的固件软件与上位机兼容性测试包：电脑通过板卡 Type-C USB CDC 发送可手工输入的 `aethor-text-v1` 文本指令，可按正式整组或台架显式子集控制 S3519 电机。默认台架 Profile 支持自包含 `bench move`、安全失能态 `bench mode` 和单电机 `bench mit`；位置、速度和转矩编码边界只来自本次上电发现的 S3519 `PMAX/VMAX/TMAX/MAX_SPD`。MIT 动作由 STM32 本地连续执行五次时间缩放轨迹或定时保持，最后自动失能。生产 Profile 仍因真实机械参数未逐轴验证而锁定使能。旧 `aethor-arm-ascii-v1` 及其 CRC 测试资产仅用于回归，不进入固件正式协议入口。

## 当前入口与范围

- 正式入口：`App/aethor_app.c`，默认由 6 个静态 FreeRTOS 任务分别承担 250 Hz 控制、CAN RX、协议、USB TX、遥测和诊断；可选 LCD 目标另加低优先级静态 UiTask。
- 关节模型：固定7轴；ESC ID为`0x01–0x07`，Master ID为`0x11–0x17`。
- 物理参数：方向、零位/限位、速度、加速度、MIT 增益、电机量程和外部减速比均保持未验证状态，因此配置不能通过使能就绪检查。
- 旧验证代码：`User/` 原样保留作为迁移参考，但旧按键双电机和旧七轴控制链不进入当前 Keil 目标。
- 正式传输：板卡Type-C USB CDC虚拟串口；USART不作为第二套正式协议入口。
- 当前默认配置：`USB_BENCH_RELATIVE`，允许使用 `bench init/mode/mit/enable/jog/move/stop/disable/clear`；MIT 动作当前只允许一个电机，其他台架命令按各自合同选择子集。
- `ARM_PRODUCTION` 已实现参考位、整组 POS_VEL、MIT 五次时间标度、受控停止、反馈确认、故障和通信看门狗，但需全部参数验证位有效才可真实使能。
- STM32 只执行关节空间控制。DH/URDF、逆运动学、笛卡尔控制、动力学和碰撞规划不在固件范围内。
- 首组 `arm-1` 完成；第二组只保留 `controller_id/arm_id` 扩展边界，尚未实现双会话。

## 分层目录

DM_TFT 的 LVGL 五向键界面、构建目标和软件验证入口见 [LCD 调试界面说明](docs/debug-ui/README.md)。默认目标关闭 LCD；首次硬件接入使用 `LCD-ReadOnly`。POS/MIT 本地入口仍要求有效配置和运行时授权。现已取得电机7的有限空载台架证据；模拟画面与主机测试不能代替整臂、带载及完整故障验收。

```text
App/
├─ Config/       七轴只读配置、构建身份、验证位
├─ Protocol/     可读文本解析、请求重放、命令生命周期和遥测
├─ Arm/          状态唯一所有者、参考位和安全门控
├─ Motion/       POS_VEL、MIT 五次时间标度和受控停止
├─ Motor/        S3519 编解码、七电机发现、反馈和 CAN 调度
├─ Telemetry/    256 项固定事件环与运行诊断
└─ Platform/     HAL/RTOS/FDCAN/USB CDC适配契约
```

核心业务层不包含 HAL、FreeRTOS 或 USB/FDCAN 头文件；`App/` 禁止动态分配。架构规则由脚本持续检查。

## 串口快速调试

Windows 串口工具使用 COM7（以实际枚举为准）、115200、8N1、串口校验位 `NONE` 和 ASCII 发送；USB CDC 不使用该波特率进行物理定时。协议不需要 CRC 或其他应用层校验，每条请求必须以 LF 或 CRLF 结束。常见串口助手应选择“校验算法：无”，并把自动附加的指令结束符设为 `0D 0A`。如果曾发送不带结束符的正文，应先复位板卡或发送一个单独换行清空残留半行，再开始 `hello` 探测。

请求编号是可选的十进制 `uint32`；编号 `0` 适合只读手工调试且不进入结果重放，`bench move` 必须使用未被其他正文占用的非零编号，以便匹配 `ok/done/error`：

```text
1 hello
2 stream off
3 show state
4 show motors
5 show motor 1
70 bench move 1 position=30 speed=5
71 bench move 1 position=0 speed=5
50 bench move 1 position=90 speed=30
51 bench move 1,3 position=90,-45 speed=30,20
60 bench mode 1 mode=mit
61 bench mit 1 action=hold kp=1 kd=1 torque_ff=0 duration_ms=1000
62 bench mit 1 action=move position=5 speed=2 kp=1 kd=1 torque_ff=0 duration_ms=1000
63 bench mode 1 mode=pos_vel
1 bench init 1
2 bench enable 1
3 bench jog 1 delta=0.2 speed=1
4 bench stop 1
5 bench disable 1
```

`bench move` 是绝对输出端角度命令，零点是本次上电零点。请求编号必须位于 `1..UINT32_MAX`；`motor/position/speed` 三个列表必须严格等长，按列表顺序一一对应，电机编号必须是唯一的 `1..7`，不支持广播。数值必须是正常 `float32`，位置另允许 `0`，速度必须大于 0。固件允许位置等于已发现 `PMAX`、速度等于 `min(VMAX,MAX_SPD)`，越界时拒绝而不截断；任一发现值缺失时不回退默认范围。`show motor <id>` 可查看 `pmax_deg/vmax_deg_s/max_speed_deg_s/move_speed_limit_deg_s`，不可用字段显示 `?`。

新 `bench move` 只发送一次。收到 `ok <id> bench move accepted=1` 后无需发送 `ping`，固件内部会重发固定 CAN 目标，并继续执行新鲜反馈、驱动故障、控制周期和 Bus-Off 安全检查；到位后依次发送最终位置加零速度的 HOLD、仅对所选电机失能，并在收到全部所选电机的新鲜 disabled 反馈后返回 `done`。旧 `bench enable/jog` 仍需在带电期间约每 250 ms 发送一次独立 `ping`，连续 1000 ms 无有效请求仍会停止和失能。不要周期重发任何动作正文来代替保活。

`bench mode` 只在所选电机已发现、反馈新鲜、无故障、已失能且输出速度接近零时执行易失性模式写入与读回，完成后保持失能。`bench mit` 使用命令内的 `kp/kd/torque_ff`，当前限定单电机；`kp=1 kd=1 torque_ff=0` 只是厂商示例起点，并未完成真实机械参数标定。逐项参数清单和实调步骤见 [S3519 MIT 参数标定与调试指南](docs/compatibility/S3519-MIT参数标定与调试指南.md)。

软件解析、主机测试和构建只证明接口与状态机合同，不证明机械臂软限位、输出方向、外部减速比、实际角速度、带载性能、七轴联动或真实硬件已经验收。

只读探测和小角度台架调试见 `Tests/hardware/debug_com7_aethor_text_v1.ps1`；现行接口契约见 `docs/compatibility/`。

## 构建与自动化验证

```powershell
powershell.exe -ExecutionPolicy Bypass -File .\Tests\host\run_phase0_tests.ps1
powershell.exe -ExecutionPolicy Bypass -File .\Tests\host\run_tests.ps1
powershell.exe -ExecutionPolicy Bypass -File .\Tests\host\run_protocol_tests.ps1
powershell.exe -ExecutionPolicy Bypass -File .\Tests\host\run_motor_core_tests.ps1
powershell.exe -ExecutionPolicy Bypass -File .\Tests\host\run_platform_io_tests.ps1
powershell.exe -ExecutionPolicy Bypass -File .\Tests\host\run_motion_tests.ps1
powershell.exe -ExecutionPolicy Bypass -File .\Tests\host\run_simulator_tests.ps1
powershell.exe -ExecutionPolicy Bypass -File .\Tests\host\run_text_protocol_tests.ps1
powershell.exe -ExecutionPolicy Bypass -File .\Tests\host\run_text_protocol_engine_tests.ps1
powershell.exe -ExecutionPolicy Bypass -File .\Tests\host\run_text_protocol_arm_profile_tests.ps1
powershell.exe -ExecutionPolicy Bypass -File .\Tests\host\test_com7_dual_motor_debug_script.ps1
powershell.exe -ExecutionPolicy Bypass -File .\Tests\host\check_phase0_architecture.ps1
```

Keil 工程：`MDK-ARM\CtrBoard-H7_FDCAN.uvprojx`

CubeMX 工程：`CtrBoard-H7_FDCAN.ioc`

最近一次 ARMCC 5 构建结果为 `0 Error(s), 0 Warning(s)`，生成镜像已通过 CMSIS-DAP 下载和校验。COM7 已验证空载 S3519 CAN ID 1、3 的一次性绝对运动：两轴分别完成 `35°`、`100°`、`360°` 和回零，随后共同完成同角度 `35°/100°/360°`、独立目标 `ID1=35°/ID3=100°` 以及共同回零。所有动作均只发送一条 `bench move`、无串口 `ping`，终态为 `completed`，最终 `present=05 enabled=00 moving=00 holding=00 fault=00`，CAN `drop/error/busoff=0`。详细实机记录见 `docs/handoffs/aethor-text-v1-bench/verification.txt`。

上述结果证明所选空载电机的位置到达、列表映射、内部目标重发、HOLD 和自动失能链路，不证明请求速度等于真实机械速度。实测完成时间明显短于按命令 `5°/s` 推算的时间，因此速度比例、机械软限位、带载性能和七轴机械臂联动仍未标定或验收。兼容性 Manifest、测试向量、模拟器、参考客户端和外部验收模板见 `docs/compatibility/`。

## 后续入口

下一入口是按 `Tests/hardware/` 的脚本继续执行安全隔离单轴和七轴硬件验收：先确认真实 CAN/Master ID、量程、方向、参考位、限位、速度比例和反馈查询，再逐轴解锁；不得直接进行七轴同时使能。Aethor Studio V2 仍需用共享兼容性包做外部契约验收。

<details>
<summary>历史 PA15 双电机验证资料（保留源码参考，不是当前固件入口）</summary>

# Aethor STM32H723 S3519 电机控制固件

该历史镜像以STM32H723VGT6、FreeRTOS、USB CDC和FDCAN1为基础，运行入口曾由PA15按键触发两台S3519各转一圈；它保留为迁移与回归证据，不是当前生产入口。

> 当前烧录目标为 PA15 双电机按键演示：第一次有效按下优先增加 `2π rad`；若任一电机的正向目标超过 PMAX，则自动改为两台同时减少 `2π rad`。后续按键优先与上次反向，并在必要时选择共同安全方向。速度上限为 `0.5 rad/s`，每圈到达后保持。

## 当前硬件配置

- 当前启用掩码为 `0x03`，只包含两台空载 S3519：
  - J1：CAN ID `0x1`，Master ID `0x11`
  - J2：CAN ID `0x2`，Master ID `0x12`
- FDCAN1 使用 PD0/PD1、1 Mbps、11 位标准经典 CAN、8 字节数据帧、BRS 关闭。
- USB CDC 在 Windows 中实测枚举为 COM7。
- KEY1 为 PA15、外部上拉、按下低电平；5 ms 采样并要求连续稳定 20 ms。
- 当前运动权限只属于 KEY1。USB 运动、使能、失能、回零和选择命令均返回 `err key-only-control`，不会改变目标。
- 启动未完成或运动中的按键不缓存；必须完成当前一圈并稳定释放按键，下一次按下才会受理。
- 七轴关节控制、同步轨迹和 DH 配置模块仍保留在工程中，但当前应用入口不调用，也不通过 USB 暴露运动控制。

## 软件分层

```text
USB CDC / FDCAN HAL
        ↓
usb_cdc_transport / bsp_fdcan
        ↓
usb_command / dm_motor_protocol
        ↓
dual_motor_controller
        ↓
PA15 key / robot_config
```

所有运行时对象、接收环形缓冲区、发送队列和轨迹状态都使用静态内存，不在运行期申请堆内存。CAN 中断只取帧并转发；命令解析、状态机、控制发送和探针输出均在 5 ms FreeRTOS 默认任务中执行。

## 启动阶段和 COM7 探针

固件只在状态变化或命令处理时输出探针，不会按 5 ms 周期刷屏：

```text
probe seq=<序号> t=<毫秒> level=<INFO|ERROR> event=<事件> key=value ...
```

正常启动顺序：

```text
BOOT → MODE_SETUP → RANGE_DISCOVERY → READY
```

- `BOOT`：上电等待 2 秒。
- `MODE_SETUP`：逐台临时写入并回读 `CTRL_MODE=2`。
- `RANGE_DISCOVERY`：逐台读取 PMAX、VMAX、TMAX，再发送失能帧取得状态反馈。
- `READY`：每个 5 ms 服务周期发送两台失能查询，以保持位置反馈新鲜且禁止误使能。
- `ENABLING / MOVING / HOLDING`：分别表示等待使能反馈、轨迹运行和终点锁位。
- `FAULT`：启动超时、模式不匹配、反馈超时、电机故障、发送错误或 Bus-Off 已锁存。

Windows 监视命令：

```powershell
powershell.exe -ExecutionPolicy Bypass -File .\Tests\hardware\monitor_com7.ps1
```

脚本默认打开 COM7，先发送 `#PING`，随后每秒发送 `#GETSTATE`。按 `Ctrl+C` 退出时会关闭串口。

按键测试必须先看到：

```text
stage=READY mode=0x03 ranges=0x03 feedback=0x03 enabled=0x00 fault=0
```

然后短按并松开 KEY1。每次受理后串口阶段应依次进入 `ENABLING → MOVING → HOLDING`。固件优先正反交替；首选方向越界时自动选择两台共同安全的反方向。两个方向都不能让两台同时完成一圈时才锁存目标范围故障。理论匀速一圈约 12.57 秒，驱动器自身加减速会使实际时间略长；30 秒未到达会锁存故障。

## USB 命令

每条命令以 CR、LF 或 CRLF 结束。

| 命令 | 作用 |
|---|---|
| `#PING` | 返回 `ok PONG` |
| `#ECHO text` | 回显最多 95 个字符 |
| `#GETSTATE` | 查询阶段、累计受理次数 `moves`、下一方向 `next_dir`、模式/参数/反馈、位置、反馈年龄、故障和 CAN/USB 状态 |
| 其他可解析命令 | 当前按键模式统一返回 `err key-only-control` |

保留的七轴命令、逆运动学命令以及 RGB/LED 命令在当前入口均不能改变目标或使能状态；除上述三个只读命令外，可解析命令统一返回 `err key-only-control`。

## 构建、下载和测试

主机测试：

```powershell
powershell.exe -ExecutionPolicy Bypass -File .\Tests\host\run_tests.ps1
```

Keil 工程：

```text
MDK-ARM\CtrBoard-H7_FDCAN.uvprojx
```

当前板上实际连接的是 CMSIS-DAP，不是工程用户配置中遗留的 J-Link。已验证的下载命令是：

```powershell
E:\oss-cad-suite\bin\openocd.exe `
  -f interface/cmsis-dap.cfg `
  -f target/stm32h7x.cfg `
  -c "adapter speed 2000" `
  -c "program E:/Desktop_E/TCG/Aethor_robo_fw/MDK-ARM/CtrBoard-H7_FDCAN/CtrBoard-H7_FDCAN.hex verify reset exit"
```

当前 PA15 镜像已完成主机测试和 Keil 构建，但尚未由本轮烧录，按键、CAN 反馈和双电机一圈运动仍属于待完成的实机验收。烧录前应可靠固定两台空载电机，并准备可立即切断 24 V 电机电源的措施。

</details>
