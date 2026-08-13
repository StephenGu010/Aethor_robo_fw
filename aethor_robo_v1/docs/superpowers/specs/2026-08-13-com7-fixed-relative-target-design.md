# COM7 通用固定相对目标控制设计

## 目标

为 USB 台架配置新增通用命令 `MOVE_REL_TARGET`。该命令支持选定电机使用自定义相对角度和速度，并在命令接管时只计算一次最终绝对目标：

```text
final_target_rad = command_start_feedback_rad + delta_deg * pi / 180
```

后续控制周期始终重发同一个 `final_target_rad`，不得用运动中的新反馈再次累加目标。这样可以消除把一圈拆成 120 条小命令时产生的逐步到位容差累积。

## 协议接口

新增命令格式：

```text
REQ <request_id> MOVE_REL_TARGET motors=<list> delta_deg=<list> speed_deg_s=<list> *<crc16>
```

示例：

```text
MOVE_REL_TARGET motors=1 delta_deg=360 speed_deg_s=3
MOVE_REL_TARGET motors=3 delta_deg=90 speed_deg_s=2
MOVE_REL_TARGET motors=1,3 delta_deg=180,-45 speed_deg_s=3,1
```

接口约束：

- `motors` 仍为 1–7 的唯一升序列表。
- `delta_deg` 和 `speed_deg_s` 的列表长度必须与 `motors` 相同。
- 每轴必须满足 `0 < abs(delta_deg) <= 360`。
- 每轴必须满足 `0 < speed_deg_s <= 3`。
- 所有数值必须有限，拒绝 NaN 和 Inf。
- 命令只在 `USB_BENCH_RELATIVE` 配置可用。
- 现有 `MOVE_REL` 继续保持 `abs(delta_deg) <= 3`，用于小角度调试，语义和兼容性不变。

## 固件行为

协议层将 `MOVE_REL_TARGET` 解析为独立命令类型，避免依靠隐藏标志区分普通小步和固定长距离运动。

应用层接管命令时必须原子检查全部选中电机：

- 已发现且反馈新鲜。
- 驱动故障为 0。
- 已进入位置速度模式并处于使能状态。
- 计算后的最终绝对位置处于该电机实读 `[-PMAX, +PMAX]` 范围内。
- 目标位置和速度能够由 S3519 运行时范围正确编码。

任一电机不满足条件时，整组命令返回失败，任何选中电机都不得开始运动。超出 PMAX 时直接拒绝，不自动反转方向；电脑必须显式修改 `delta_deg` 的符号。

命令接管后，把每轴最终目标复制到活动动作的 `motion_plan.target_position_rad`。周期发送逻辑使用该快照构造的同一批 CAN 帧，直到全部选中电机的位置误差不大于 0.5 deg。运动期间的新反馈只用于到位、故障和超时判断，不参与重算目标。

## 超时与安全

活动命令期限按最大轴时长计算：

```text
deadline = accepted_time + max(abs(delta_deg) / speed_deg_s) + 0.5 s
```

例如 360 deg、3 deg/s 的期限约为 120.5 s。电脑在等待期间每 200 ms 发送 HEARTBEAT，保持现有 1000 ms 链路看门狗要求。

以下任一情况终止命令并执行现有安全停机路径：

- 反馈超时或失效。
- 驱动故障。
- CAN 发送失败或 Bus-Off。
- 到达计算期限仍未进入 0.5 deg 容差。
- USB 链路看门狗超时。

正常完成后返回 `DONE COMPLETED`。最终机械角允许保留单次到位容差，但不会再把这一容差累积 120 次。

## COM7 脚本迁移

`run_com7_output_one_turn_1_3.ps1` 的三阶段顺序保持不变，但每阶段从 120 条 `MOVE_REL` 改成一条 `MOVE_REL_TARGET`：

```text
ID1:     motors=1   delta_deg=360      speed_deg_s=3
ID3:     motors=3   delta_deg=360      speed_deg_s=3
ID1_ID3: motors=1,3 delta_deg=-360,-360 speed_deg_s=3,3
```

每条长动作的电脑侧等待时间设为计划运动时间加安全余量，至少 125 s。脚本继续执行阶段级 `INIT_MOTORS -> CLEAR_FAULT -> ENABLE -> MOVE_REL_TARGET -> STOP -> DISABLE`，异常时保留对当前电机集合的尽力而为 STOP 和 DISABLE。

如果正常 STOP 首次返回动作超时，脚本只允许再执行一次有界 STOP；重试成功后才继续 DISABLE，重试仍失败则终止后续阶段。

脚本自测从“每圈 120 步”改为验证“三阶段各一条固定目标命令”，并验证角度列表、速度列表、预计时长、命令名和方向。

## 自动化验证

主机测试先建立以下失败用例，再修改生产代码：

- `MOVE_REL_TARGET` 接受单轴 30、90、180、360 deg。
- 接受多轴不同正负角度和速度，并保持列表到电机的映射。
- 拒绝 0、超过正负 360 deg、超速、NaN、Inf、重复电机和列表长度不匹配。
- 现有 `MOVE_REL` 仍拒绝超过正负 3 deg。
- 应用接管时只计算一次 `feedback + delta`。
- 后续反馈变化时，周期重发帧中的目标保持不变。
- 最终目标越过任一电机 PMAX 时整组不发运动 CAN 帧。
- 360 deg、3 deg/s 的固件动作期限按约 120.5 s 计算。
- 串口脚本自测确认每个阶段只发送一条 `MOVE_REL_TARGET`。

随后运行现有协议、电机、Phase 0、全软件回归和 Keil Rebuild。只有全部测试通过并生成新 HEX 后才能烧录。

## 实机验收

烧录后仍按分阶段顺序执行：

1. ID1 单独 `+360 deg`。
2. ID3 单独 `+360 deg`。
3. ID1、ID3 同时 `-360 deg`。

每阶段必须记录 ACK、DONE、耗时、最终故障、CAN TX 错误、Bus-Off 和反馈有效性。串口闭环通过后，再用电机外壳标记确认输出轴误差是否处于单次 0.5 deg 到位容差附近。

## 不在本次范围

- 不开放超过正负 360 deg 的单条命令。
- 不修改 S3519 的 PMAX、VMAX、TMAX 或齿轮减速比参数。
- 不自动选择正反方向。
- 不引入 DH、机械零位、关节软限位或笛卡尔控制。
- 不删除或替换现有 `MOVE_REL`。
