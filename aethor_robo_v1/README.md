# Aethor STM32H723 S3519 电机控制固件

本工程以 STM32H723VGT6、FreeRTOS、USB CDC 和 FDCAN1 为基础。当前运行入口由 PA15 按键触发两台 S3519 各自正向增加一圈；USB CDC 只承担查询和探针输出。原七轴关节控制模块仍保留，但当前不参与运行。

> 当前烧录目标已切换为 PA15 双电机按键演示：上电检查完成后，每次上电第一次有效按下 KEY1，使电机 `0x1/0x2` 分别从当前位置正向增加 `2π rad`，速度上限 `0.5 rad/s`，到达后保持。七轴关节控制模块仍保留，但当前应用入口不调用它。

## 当前硬件配置

- 当前启用掩码为 `0x03`，只包含两台空载 S3519：
  - J1：CAN ID `0x1`，Master ID `0x11`
  - J2：CAN ID `0x2`，Master ID `0x12`
- FDCAN1 使用 PD0/PD1、1 Mbps、11 位标准经典 CAN、8 字节数据帧、BRS 关闭。
- USB CDC 在 Windows 中实测枚举为 COM7。
- KEY1 为 PA15、外部上拉、按下低电平；5 ms 采样并要求连续稳定 20 ms。
- 当前运动权限只属于 KEY1。USB 运动、使能、失能、回零和选择命令均返回 `err key-only-control`，不会改变目标。
- 每次上电最多受理一次按键；启动未完成、运动中和完成后的按键均不会累计。
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

然后短按并松开 KEY1。串口阶段应依次进入 `ENABLING → MOVING → HOLDING`。理论匀速一圈约 12.57 秒，驱动器自身加减速会使实际时间略长；30 秒未到达会锁存故障。

## USB 命令

每条命令以 CR、LF 或 CRLF 结束。

| 命令 | 作用 |
|---|---|
| `#PING` | 返回 `ok PONG` |
| `#ECHO text` | 回显最多 95 个字符 |
| `#GETSTATE` | 查询阶段、模式/参数/反馈/使能掩码、位置、反馈年龄、故障和 CAN/USB 状态 |
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
  -c "program E:/Desktop_E/TCG/Aethor_robo_fw/aethor_robo_v1/MDK-ARM/CtrBoard-H7_FDCAN/CtrBoard-H7_FDCAN.hex verify reset exit"
```

当前 PA15 镜像已完成主机测试和 Keil 构建，但尚未由本轮烧录，按键、CAN 反馈和双电机一圈运动仍属于待完成的实机验收。烧录前应可靠固定两台空载电机，并准备可立即切断 24 V 电机电源的措施。
