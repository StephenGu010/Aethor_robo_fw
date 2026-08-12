# 七轴固件交接说明

## 当前可复现状态

- 分支：`feature/7dof-joint-control`
- 目标芯片：STM32H723VGT6
- CubeMX 元数据：6.11.1 / STM32CubeH7 V1.11.2
- 控制周期：5 ms
- 当前总线：FDCAN1，PD0/PD1，1 Mbps，经典 CAN
- 当前 USB CDC：COM7
- 当前活动电机：J1 `CAN=0x1/Master=0x11`，J2 `CAN=0x2/Master=0x12`
- 当前安全状态：两台均未完成机械标定，完整使能锁开启，只允许选择单轴后进行相对使能位置 `±3°` 标定。

固件已在两台空载电机连接状态下完成实机启动检查。最后一次观测为：

```text
stage=READY active=0x03 mode=0x03 ranges=0x03 feedback=0x03
pending=0x00 enabled=0x00 motor_fault=0x00 lock=1 safety=0
can_started=1 warning=0 passive=0 busoff=0 usb_drop=0 probe_drop=0
```

该证据只证明 USB、CAN、模式/参数回读和失能状态反馈链路，不证明运动方向、角度精度或机械安全。

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

## 建议的实机标定顺序

1. 两台电机空载并可靠固定，确认 CAN 终端电阻、24 V 电源和立即断电手段。
2. 运行 `Tests\hardware\monitor_com7.ps1`，必须先看到 `READY`、`lock=1`、`enabled=0`、`safety=0`。
3. 只选择 J1，读取 `#GETJPOS` 作为使能参考，执行相对 `+3°/-3°`，确认方向、反馈和失能。
4. 更新 J1 的方向、零位、限位与外部减速比，完成代码审查和回归测试。
5. 对 J2 重复同样流程；不得同时跳过两个单轴阶段。
6. 安装到机械结构后重新校准零位与软限位。空载电机结果不能直接作为机械臂参数。
7. 逐台接入 J3-J7，确认真实 CAN/Master ID 后再扩大活动掩码。
8. 所有活动轴参数确认后才能把对应 `commissioned` 改为 1；只有全部活动轴均为 1 时完整多轴使能锁才释放。
9. 用 CAD/实测建立七轴 DH 表并验证正运动学后，才把 `dh_parameters_valid` 改为 1。
10. 最后执行七轴同步到达测试。固件不包含逆运动学，上位机必须输出关节角。

## 常见探针定位

| 现象 | 优先检查 |
|---|---|
| 停在 `BOOT` | 等待满 2 秒；确认 FreeRTOS 任务仍在运行 |
| 停在 `MODE_SETUP` | CAN ID/Master ID、1 Mbps、CTRL_MODE 回包、100 ms 超时与三次重试 |
| 停在 `RANGE_DISCOVERY` | PMAX/VMAX/TMAX 回包的 RID、Master ID、D0 电机 ID和数值合法性 |
| `READY` 但 `lock=1` | 这是未完成机械标定的预期安全状态 |
| `ENABLING` 后故障 | 50 ms 内未收到状态值 1 的反馈，或电机返回故障状态 |
| `usb_drop/probe_drop` 增长 | 上位机没有及时读取、CDC 忙或发送队列饱和 |
| `warning/passive/busoff` 非零 | 终端电阻、波特率、供电、接地、CANH/CANL 或节点冲突 |
| Master ID 正确但帧被忽略 | 检查 D0 低四位电机 ID、DLC=8、标准经典数据帧 |

## 未完成边界

- 本机未找到 STM32CubeMX 可执行程序，因此本轮没有再次调用 GUI/CLI 重新生成；`.ioc`、`fdcan.c`、GPIO、FreeRTOS、USB 与 Keil 文件引用已逐项人工同步并由 Keil 编译验证。
- Python 实机脚本需要 `pyserial`；当前 Python 环境未安装该包。COM7 探针监视脚本只使用 Windows .NET，不需要额外依赖，已实机验证。
- 没有执行 `s3519_commissioning_test.py` 的运动部分，也没有执行七轴同步测试。
- 没有推送或合并分支。
