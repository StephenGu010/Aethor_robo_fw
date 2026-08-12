# STM32H723 电机固件交接说明

## 当前运行入口

当前固件运行的是 PA15 双电机一圈演示，而不是 USB 七轴运动入口。七轴配置、协议和控制模块仍保留，后续可以重新切换。

- KEY1：PA15，按下低电平，20 ms 消抖。
- 电机：`CAN 0x1/0x2`，Master `0x11/0x12`。
- 第一次有效按下：两个目标分别为各自当前位置 `+2π rad`。
- 速度上限：`0.5 rad/s`。
- 到达后：继续发送终点位置保持。
- 每次上电：最多受理一次；复位后才重新允许。
- USB：只允许诊断查询，不能使能或改变目标。
- 烧录后先运行 `Tests\hardware\monitor_com7.ps1`，确认 `READY` 且无故障，再按 KEY1。

## 当前可复现状态

- 分支：`feature/7dof-joint-control`
- 目标芯片：STM32H723VGT6
- CubeMX 元数据：6.11.1 / STM32CubeH7 V1.11.2
- 控制周期：5 ms
- 当前总线：FDCAN1，PD0/PD1，1 Mbps，经典 CAN
- 当前 USB CDC：COM7
- 当前活动电机：J1 `CAN=0x1/Master=0x11`，J2 `CAN=0x2/Master=0x12`
- 当前按键策略：启动检查完成前忽略按键；每次上电只受理第一次有效按下，目标为两台当前位置各自 `+2π rad`。

上一版七轴入口曾在两台空载电机连接状态下完成 USB、CAN、模式/参数回读和失能反馈检查，最后一次观测为：

```text
stage=READY active=0x03 mode=0x03 ranges=0x03 feedback=0x03
pending=0x00 enabled=0x00 motor_fault=0x00 lock=1 safety=0
can_started=1 warning=0 passive=0 busoff=0 usb_drop=0 probe_drop=0
```

该记录不代表当前 PA15 固件已经烧录，也不证明当前按键运动、方向、角度精度或机械安全；当前镜像仍需重新烧录并按下文流程采集探针。

## 下一阶段必须取得的真实参数

| 项目 | J1/J2 | J3-J7 |
|---|---|---|
| CAN ID / Master ID | 已确认 `1/0x11`、`2/0x12` | 尚未从实机确认 |
| 正方向 `direction` | 未标定 | 未标定 |
| 机械零位 `joint_zero_degrees` | 未标定 | 未标定 |
| 外部减速比 | 当前占位 1.0，需按机构确认 | 未确认 |
| 关节最小/最大角 | 当前为未生效占位值 | 未确认 |
| 最大速度/加速度 | 当前空载测试上限 0.5 rad/s、60 deg/s² | 未确认 |
| DH `theta0,d,a,alpha` | 未测量 | 未测量 |
| 标定状态 | `commissioned=0` | `commissioned=0` |

修改参数的唯一入口是 `User/robot_config.c`。真实七轴启用时还要修改 `active_joint_mask`；FDCAN 过滤器会从配置表生成，不需要在 BSP 中再写死 Master ID。

## 本次 PA15 实机调试顺序

1. 两台电机空载并可靠固定，确认 CAN 终端电阻、24 V 电源和立即断电手段。
2. 烧录本次生成的 HEX，复位控制板并运行 `Tests\hardware\monitor_com7.ps1`。
3. 必须先看到 `stage=READY mode=0x03 ranges=0x03 feedback=0x03 enabled=0x00 fault=0`；未就绪时不要按键。
4. 短按并松开 KEY1 一次，观察阶段依次进入 `ENABLING → MOVING → HOLDING`。
5. 运动期间保持可立即断电；任一电机方向异常、机械干涉或声音异常时直接切断 24 V 电机电源。
6. 到达后再次按键应保持 `accepted=1` 且不再产生新的一圈；复位后才重新允许一次。
7. 若进入 `FAULT`，保留完整 `#GETSTATE` 和 `probe ... event=FAULT` 输出再定位，不要反复复位尝试运动。

七轴配置、标定脚本和控制模块仍保留，但不属于本次 PA15 双电机运行入口；重新启用七轴入口时仍需逐轴完成方向、零位、减速比、软限位和 DH 参数验证。

## 常见探针定位

| 现象 | 优先检查 |
|---|---|
| 停在 `BOOT` | 等待满 2 秒；确认 FreeRTOS 任务仍在运行 |
| 停在 `MODE_SETUP` | CAN ID/Master ID、1 Mbps、CTRL_MODE 回包、100 ms 超时与三次重试 |
| 停在 `RANGE_DISCOVERY` | PMAX/VMAX/TMAX 回包的 RID、Master ID、D0 电机 ID和数值合法性 |
| `READY` 但按键不受理 | 检查 `accepted` 是否已经为 1，以及按键是否形成稳定 20 ms 的高到低有效沿 |
| `ENABLING` 后故障 | 50 ms 内未收到状态值 1 的反馈，或电机返回故障状态 |
| `usb_drop/probe_drop` 增长 | 上位机没有及时读取、CDC 忙或发送队列饱和 |
| `warning/passive/busoff` 非零 | 终端电阻、波特率、供电、接地、CANH/CANL 或节点冲突 |
| Master ID 正确但帧被忽略 | 检查 D0 低四位电机 ID、DLC=8、标准经典数据帧 |

## 未完成边界

- 本机未找到 STM32CubeMX 可执行程序，因此本轮没有再次调用 GUI/CLI 重新生成；`.ioc`、`fdcan.c`、GPIO、FreeRTOS、USB 与 Keil 文件引用已逐项人工同步并由 Keil 编译验证。
- COM7 探针监视脚本只使用 Windows .NET，不需要额外依赖；当前 PA15 镜像尚未由本轮烧录，串口阶段和双电机一圈运动仍待本次实机验证。
- 软件构建和主机测试不能替代按键、CAN 总线和双电机运动的实机验收。
- 没有推送或合并分支。
