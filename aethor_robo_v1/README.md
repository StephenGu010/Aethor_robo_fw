# Aethor STM32H723 七轴关节控制固件

本工程以 STM32H723VGT6、FreeRTOS、USB CDC 和 FDCAN1 为基础。控制边界是“上位机给出 7 个关节角，固件完成轨迹、单位换算、CAN 控制和反馈监控”；固件不实现逆运动学，也不接受 RGB 指令。

## 当前硬件配置

- 当前启用掩码为 `0x03`，只包含两台空载 S3519：
  - J1：CAN ID `0x1`，Master ID `0x11`
  - J2：CAN ID `0x2`，Master ID `0x12`
- FDCAN1 使用 PD0/PD1、1 Mbps、11 位标准经典 CAN、8 字节数据帧、BRS 关闭。
- USB CDC 在 Windows 中实测枚举为 COM7。
- J1/J2 的方向、机械零位、关节软限位和外部减速比尚未完成机械标定，因此 `commissioned=0`，`lock=1`。固件不会执行双轴整体使能，只允许选择单轴后进行最多相对使能位置 `±3°` 的标定动作。
- 七轴 DH 数组已经具备固定接口，但当前数值未测量，`dh_parameters_valid=0`。纯关节角控制不依赖 DH；`#GETDH` 会明确返回有效标志。

## 软件分层

```text
USB CDC / FDCAN HAL
        ↓
usb_cdc_transport / bsp_fdcan
        ↓
usb_command / dm_motor_protocol
        ↓
joint_controller / sync_trajectory
        ↓
robot_config
```

所有运行时对象、接收环形缓冲区、发送队列和轨迹状态都使用静态内存，不在运行期申请堆内存。CAN 中断只取帧并转发；命令解析、状态机、控制发送和探针输出均在 5 ms FreeRTOS 默认任务中执行。

## 启动阶段和 COM7 探针

固件只在状态变化或命令处理时输出探针，不会按 5 ms 周期刷屏：

```text
probe seq=<序号> t=<毫秒> level=<INFO|ERROR> event=<事件> key=value ...
```

正常启动顺序：

```text
CONFIG_VALIDATE → BOOT → MODE_SETUP → RANGE_DISCOVERY → READY
```

- `BOOT`：上电等待 2 秒。
- `MODE_SETUP`：逐台临时写入并回读 `CTRL_MODE=2`。
- `RANGE_DISCOVERY`：逐台读取 PMAX、VMAX、TMAX，再发送失能帧取得状态反馈。
- `READY`：未使能时每 100 ms 发送失能查询，以保持位置和反馈年龄可观测。
- `ENABLING / MOVING / HOLDING`：分别表示等待使能反馈、轨迹运行和终点锁位。
- `FAULT`：启动超时、模式不匹配、反馈超时、电机故障、发送错误或 Bus-Off 已锁存。

Windows 监视命令：

```powershell
powershell.exe -ExecutionPolicy Bypass -File .\Tests\hardware\monitor_com7.ps1
```

脚本默认打开 COM7，先发送 `#GETCAPS`，随后每秒发送 `#GETSTATE`。按 `Ctrl+C` 退出时会关闭串口。

## USB 命令

每条命令以 CR、LF 或 CRLF 结束。

| 命令 | 作用 |
|---|---|
| `#PING` | 返回 `ok PONG` |
| `#ECHO text` | 回显最多 95 个字符 |
| `#GETCAPS` | 查询轴数、活动掩码、DH/IK/RGB/静态内存/探针能力 |
| `#GETSTATE` | 查询阶段、模式/参数/反馈/使能掩码、位置、反馈年龄、故障和 CAN/USB 状态 |
| `#GETJPOS` | 返回 7 个关节角，单位为度 |
| `#GETMPOS` | 返回 7 个电机输出端角度，单位为度 |
| `#GETENABLE` | 返回已由反馈确认的使能掩码 |
| `#GETDH`、`#GETDH 1` | 查询 DH 有效性或指定行 |
| `#GETCONFIG`、`#GETCONFIG 1` | 查询轴配置摘要或指定轴参数 |
| `#SELECT 1` | 选择单轴标定对象，只接受活动轴 |
| `!START` | 发送使能请求；只有状态值 1 的反馈才计为已使能 |
| `!STOP` | 停止轨迹并保持当前命令位置 |
| `!DISABLE` | 失能已使能或待确认的电机 |
| `!HOME` | 以 20% 速度前往配置的 DH `theta_offset` 关节姿态 |
| `>j1,...,j7[,speed]` | 顺序动作；已有轨迹运行时返回 busy |
| `&j1,...,j7[,speed]` | 可打断动作；从当前命令位置重建轨迹 |

`speed` 为 `1..100` 的百分比，省略时为 100。关节命令使用度；电机协议边界转换为小端 `float` 弧度。外部减速器按以下关系集中配置：

```text
motor_output_deg = direction × (joint_deg - joint_zero_deg) × external_reduction_ratio
```

`@...` 逆运动学命令以及 RGB/LED 命令会返回显式 unsupported 解析错误，不会改变目标或使能状态。

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
  -c "program E:/Desktop_E/TCG/Aethor_robo_fw/aethor_robo_v1/MDK-ARM/CtrBoard-H7_FDCAN/CtrBoard-H7_FDCAN.hex verify reset exit"
```

只有软件测试和实机启动/通信已经完成验证。任何单轴运动测试前，仍应固定空载电机、准备立即断电措施，并使用 `Tests\hardware\s3519_commissioning_test.py` 的相对 `±3°` 流程；七轴同步脚本必须等真实七轴参数全部标定并解除锁后才能使用。
