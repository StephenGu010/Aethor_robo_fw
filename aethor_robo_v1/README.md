# Aethor 七自由度机械臂固件

当前分支实现 PRD Phase 0 安全基线：STM32H723 工程已经切换到静态内存的 `App/` 分层入口，启动状态固定为 `BOOT → SELF_TEST → FAULT(CONFIG_INCOMPLETE)`。由于真实机械参数尚未完成逐轴验证，本固件不会进入使能或运动状态，也没有可执行的电机发送接口。

## 当前入口与范围

- 正式入口：`App/aethor_app.c`，由 `main.c` 初始化、FreeRTOS 默认任务每 4 ms 服务一次。
- 默认任务：CubeMX 静态创建，栈缓冲区为 512 words。
- 关节模型：固定7轴；ESC ID为`0x01–0x07`，Master ID为`0x11–0x17`。
- 物理参数：方向、零位/限位、速度、加速度、MIT 增益、电机量程和外部减速比均保持未验证状态，因此配置不能通过使能就绪检查。
- 旧验证代码：`User/` 原样保留作为迁移参考，但旧按键双电机和旧七轴控制链不进入当前 Keil 目标。
- 正式传输：板卡Type-C USB CDC虚拟串口；USART不作为第二套正式协议入口。
- 当前默认配置：`USB_BENCH_RELATIVE`，仅允许后续接入显式选轴的受限相对控制；生产配置仍由完整机械参数门控。
- 本阶段未实现：完整`aethor-arm-ascii-v1`、新架构电机控制、同步轨迹、DH正逆解、RGB和上位机业务逻辑。

## 分层目录

```text
App/
├─ Config/       七轴只读配置、构建身份、验证位
├─ Protocol/     上位机协议边界，Phase 0 不执行命令
├─ Arm/          状态唯一所有者与启动自检
├─ Motion/       运动类型边界，不生成设定值
├─ Motor/        电机类型边界，不发送 CAN 帧
├─ Telemetry/    固定容量事件环与诊断计数
└─ Platform/     HAL/RTOS/FDCAN/USB CDC适配契约
```

核心业务层不包含 HAL、FreeRTOS 或 USB/FDCAN 头文件；`App/` 禁止动态分配。架构规则由脚本持续检查。

## 构建与自动化验证

```powershell
powershell.exe -ExecutionPolicy Bypass -File .\Tests\host\run_phase0_tests.ps1
powershell.exe -ExecutionPolicy Bypass -File .\Tests\host\run_tests.ps1
powershell.exe -ExecutionPolicy Bypass -File .\Tests\host\check_phase0_architecture.ps1
```

Keil 工程：`MDK-ARM\CtrBoard-H7_FDCAN.uvprojx`

CubeMX 工程：`CtrBoard-H7_FDCAN.ioc`

最近一次 ARMCC 5 构建结果为 `0 Error(s), 0 Warning(s)`；该结果只证明源码和工程配置能够生成固件，不证明 USB、CAN、电机、关节方向、限位、减速比或运动精度已通过实机验证。详细证据与交接说明见 `docs/handoffs/phase-00/`。

## 后续入口

Phase 1 应先建立参数来源、审核和逐轴确认机制，再迁移 DM3520/S3519 参数读取与反馈解码。任何运动功能都必须等配置验证位、单轴台架验收和安全条件满足后才能接入。

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
  -c "program E:/Desktop_E/TCG/Aethor_robo_fw/aethor_robo_v1/MDK-ARM/CtrBoard-H7_FDCAN/CtrBoard-H7_FDCAN.hex verify reset exit"
```

当前 PA15 镜像已完成主机测试和 Keil 构建，但尚未由本轮烧录，按键、CAN 反馈和双电机一圈运动仍属于待完成的实机验收。烧录前应可靠固定两台空载电机，并准备可立即切断 24 V 电机电源的措施。

</details>
