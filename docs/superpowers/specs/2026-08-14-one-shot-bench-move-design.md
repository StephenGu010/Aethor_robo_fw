# Aethor 单命令电机位置速度动作设计

日期：2026-08-14
状态：已确认方案 A，待实施

## 1. 目标

在 `USB_BENCH_RELATIVE` 演示配置中增加一条可人工输入的自包含动作命令。上位机只发送一次命令，固件负责完成所选电机的初始化、控制模式准备、清故障、使能、绝对位置运动、到位确认、停止和失能。动作执行期间不要求上位机周期发送 `ping`，且不再使用固定的 `3°` 位移和 `3°/s` 速度上限。

本设计满足以下可观察结果：

1. 一条命令可为一个或多个明确电机分别指定绝对位置和速度。
2. 固件内部持续下发固定的 POS_VEL 目标，直到到位、停止、故障或动作超时。
3. 自包含动作持续超过 1000 ms 时，不会被通信看门狗取消。
4. `done` 只在所选电机已经失能后返回。
5. 位置和速度按每台电机实际发现的范围校验，不使用固定演示上限。
6. 旧命令和非自包含使能状态继续保留原有通信看门狗保护。

## 2. 当前事实与边界

- 活跃协议是可打印、按行分帧的 `aethor-text-v1`。
- 当前 `bench jog` 已经只提交一次，固件也会周期重发 CAN 目标；动作仍然失败的原因是使能或运动期间 1000 ms 未收到有效串口请求会触发 `STOP_DISABLE`。
- 当前 `AETHOR_BENCH_MAX_RELATIVE_DEGREES` 和 `AETHOR_BENCH_MAX_SPEED_DEGREES_S` 均为 `3.0F`，属于台架协议人为限制。
- 初始化流程已经逐电机读取 `PMAX`、`VMAX`、`MAX_SPD`、ACC、DEC、模式和身份信息。
- S3519 的位置速度模式使用两个小端浮点数传递绝对位置和速度限制；`PMAX`、`VMAX` 是电机映射范围，`MAX_SPD` 是驱动器最大速度参数。
- 生产七轴关节方向、软限位、减速器和机械限位仍未验证。本设计只扩大电机台架能力，不解除 `ARM_PRODUCTION` 配置门禁，不声称机械臂可以在电机全范围内安全运动。

## 3. 串口接口

### 3.1 新命令

```text
<request_id> bench move <motors> position=<degrees_list> speed=<degrees_per_second_list>
```

单电机示例：

```text
50 bench move 1 position=90 speed=30
```

多电机示例：

```text
51 bench move 1,3 position=90,-45 speed=30,20
```

接口规则：

- `motors` 使用现有的 `1` 至 `7` 电机编号和逗号列表语法，禁止重复编号。
- `position` 是电机输出端相对于本次上电零点的绝对角度，单位为度。
- `speed` 是正的速度限制，单位为度每秒；方向由目标位置和当前反馈决定。
- `position` 和 `speed` 的元素数量必须与 `motors` 完全一致，并按相同顺序一一对应；不做隐式广播。
- 所有数值必须为有限浮点数。速度必须大于零。
- 状态改变请求继续使用现有非零 `request_id` 重放语义；重复发送相同编号和正文只重放结果，不重复执行运动。

### 3.2 接受和完成回复

语法、列表和静态数值检查通过后立即回复：

```text
ok 50 bench move accepted=1
```

只有在所选电机已经停止并失能后才回复：

```text
done 50 bench move result=completed elapsed_ms=3200 motors=01
```

运行时失败使用终态回复，并给出阶段和稳定错误码：

```text
done 50 bench move result=failed stage=validate code=position_out_of_range motor=1
done 50 bench move result=failed stage=discovery code=not_ready motor=1
done 50 bench move result=failed stage=motion code=timeout motor=1
done 50 bench move result=failed stage=disable code=feedback_timeout motor=1
```

解析阶段错误继续使用现有 `error` 类型：

```text
error 50 bench move code=bad_argument field=position
error 50 bench move code=count_mismatch field=speed
```

### 3.3 范围查询

`show motor <id>` 增加本次初始化得到的可读范围字段：

```text
ok 60 show motor 1 pmax_deg=<runtime> vmax_deg_s=<runtime> max_speed_deg_s=<runtime> move_speed_limit_deg_s=<runtime>
```

其中：

- `pmax_deg` 来自 `PMAX`。
- `vmax_deg_s` 来自 `VMAX`。
- `max_speed_deg_s` 来自 `MAX_SPD`。
- `move_speed_limit_deg_s` 为 `VMAX` 与 `MAX_SPD` 的较小正值。
- 未完成发现时对应字段输出 `?`，动作命令不得用默认值代替。

## 4. 动态范围规则

每台所选电机分别执行以下检查：

1. 身份、模式、范围和 `MAX_SPD` 已由本次上电后的只读发现流程验证。
2. `abs(position_rad) <= PMAX`。
3. `0 < speed_rad_s <= min(VMAX, MAX_SPD)`。
4. 当前反馈有效、无非零故障标志。

边界值本身允许；超过边界的命令不做截断，而是明确拒绝。不同电机可以具有不同范围，整条命令采用全有或全无原则：任一电机无有效范围或任一值超限时，不使能任何所选电机。

旧 `bench jog` 保留，但删除固定 `3°` 和 `3°/s` 判断。其相对目标先按当前反馈计算绝对目标，再应用同一组逐电机动态范围规则。这样兼容旧人工命令，同时不会绕过新安全边界。

## 5. 自包含动作状态机

新命令由 `ArmControlTask` 中的单一静态动作对象拥有，不创建动态内存或额外任务。状态按以下顺序推进：

```text
VALIDATE_INPUT
  -> DISCOVER_IF_NEEDED
  -> VALIDATE_LIMITS
  -> ENSURE_POS_VEL_MODE
  -> CLEAR_FAULT
  -> ENABLE
  -> MOVE
  -> HOLD_STOP
  -> DISABLE
  -> DONE
```

阶段语义：

- `VALIDATE_INPUT`：验证列表、一一对应关系和有限数值；失败时不启动复合动作。
- `DISCOVER_IF_NEEDED`：只对未完成本次上电发现的所选电机执行现有只读发现；已验证电机直接跳过。
- `VALIDATE_LIMITS`：使用本次发现得到的实时范围和最新反馈执行逐电机动态校验；失败时不发送使能帧。
- `ENSURE_POS_VEL_MODE`：使用现有易失模式切换和读回确认。
- `CLEAR_FAULT`：发送所选电机清故障命令并等待无故障反馈。
- `ENABLE`：仅使能所选电机并等待已使能反馈。
- `MOVE`：保存一次固定绝对目标批次，并按现有调度周期反复发送；不从变化的反馈重复计算目标。
- `HOLD_STOP`：到位后以最终目标和零速度执行现有停止保持语义。
- `DISABLE`：只失能所选电机并确认反馈状态。
- `DONE`：生成唯一终态结果并释放动作对象。

动作持续时间使用每台电机的 `abs(target-current)/speed` 计算，取最大值并增加现有到位和通信余量。计算使用有限值与 64 位溢出检查，不额外设置人为位置或最大速度上限。动作被显式 `bench stop` 抢占、发生 CAN/反馈故障或超过计算期限时，进入停止和失能清理；清理完成后返回失败或取消终态。

## 6. 看门狗语义

通信看门狗不被删除，也不通过伪造 `ping` 刷新时间戳。

- 仅当当前动作是已接受的自包含 `bench move`，并且状态处于 `DISCOVER_IF_NEEDED` 至 `DISABLE` 之间时，应用层不执行通信超时取消。
- 自包含动作仍受动作期限、反馈新鲜度、CAN Bus-Off、驱动故障和显式 `bench stop` 约束。
- 动作必须在返回 `done` 前完成失能，因此串口断开不会留下无限期保持使能状态。
- 旧 `bench enable`、旧 `bench jog`、生产动作和其他使能/保持状态继续使用 1000 ms 通信看门狗，除非未来另有独立规格。

这种做法把“允许脱离上位机完成动作”的权限限制在一条有明确终点和强制失能的命令上，不扩大为全局关闭安全看门狗。

## 7. 模块边界

- `TextProtocol`：只负责现有行分帧、词和 `key=value` 解析，不了解电机硬件范围。
- `ProtocolEngine`：解析严格列表，生成新的业务命令类型，保持请求重放和可读错误格式。
- `ProtocolQueryContext`：以固定数组携带每台电机发现得到的 `PMAX`、`VMAX`、`MAX_SPD` 及有效位，供查询和早期范围检查使用。
- `AethorApp`：拥有复合状态机、动作期限、动态目标验证、内部周期发送、停止/失能清理和看门狗豁免判断。
- `MotorRuntime`：继续负责发现、模式命令、POS_VEL 编码、反馈快照和选定电机批次；增加一个集中式逐电机 POS_VEL 范围验证接口，避免 `move` 与 `jog` 重复规则。
- `S3519Codec`：保持纯编码/解码职责，不承担机械臂软限位，也不硬编码 S3519 型号默认范围。

所有新增状态、数组和结果均使用固定容量静态存储；不引入堆分配。

## 8. 兼容性

- 保留 `hello`、`ping`、`show`、`stream`、现有 `bench` 生命周期命令和 `bench jog`。
- 保留 `aethor-arm-ascii-v1` 回归解析，但不为旧协议新增复合动作别名。
- `bench move` 是 `aethor-text-v1` 的增量命令，不改变现有回复前缀和 LF/CRLF 分帧。
- 文档、manifest、兼容性向量和上位机参考客户端与固件同一提交周期更新。

## 9. 错误与清理策略

- 解析或范围失败：不发送任何电机控制帧。
- 发现、模式切换或清故障失败：尽最大努力失能所选电机，返回对应阶段错误。
- 使能后任意失败：先生成停止目标，再失能；如果反馈不可用则直接执行所选电机失能批次。
- CAN Bus-Off、全局运行时故障或控制任务连续超期：沿用现有全局紧急失能策略。
- 串口输出阻塞或断开：不影响动作状态机推进；终态结果保留在现有有界结果队列中，受队列容量约束。
- 新动作执行期间收到第二条普通动作：返回 `busy`；`bench stop` 保持高优先级抢占能力。

## 10. 验证设计

### 10.1 协议测试

- 单电机和多电机命令解析。
- 电机、位置、速度列表数量不匹配。
- 重复电机、NaN、无穷值、零速和负速。
- 非零请求编号的重放与编号冲突。
- 不同电机独立位置和速度保持正确对应。

### 10.2 动态范围测试

- `90°` 等大于原固定 `3°`、但小于实时 `PMAX` 的位置被接受。
- 恰好等于 `PMAX` 和速度较小动态上限的值被接受。
- 超过任一电机动态位置或速度上限的整条命令被拒绝，且无使能帧。
- 未发现范围不得回退到 SDK 默认值。
- `bench jog` 计算后的绝对目标不能超过 `PMAX`。

### 10.3 状态机与看门狗测试

- 从未初始化状态执行完整发现、模式、清故障、使能、运动、停止、失能流程。
- 动作持续超过 1000 ms 且没有任何新串口请求时仍继续发送固定目标并最终完成。
- `done completed` 产生时所选电机已全部失能。
- 非自包含 `bench enable` 仍在 1000 ms 无有效请求后停止并失能。
- 动作超时、反馈过期、驱动故障、Bus-Off 和显式停止均进入正确清理路径。
- 未选电机不收到控制帧，状态不改变。

### 10.4 工程与实机验证

- 运行完整 host、Phase 0、Motor、Motion、Platform、Protocol、Text Protocol 和模拟器测试矩阵。
- Keil ARMCC 构建必须为 `0 Error(s), 0 Warning(s)`，并核对生成 HEX。
- 实机只使用卸载单电机分阶段验证：先读取并记录实时范围，再用保守角度执行一条 `bench move`，主机在发送后不再发送 `ping`，等待超过 1000 ms 后仍应完成并自动失能。
- 实机演示不以运动到 `PMAX` 或 `MAX_SPD` 作为验收；最大边界由运行时寄存器值和 host 边界测试证明，避免用机械风险换取软件边界证据。

## 11. 非目标

- 不填入或猜测七轴机械软限位、方向、减速比、零点和负载能力。
- 不解除 `ARM_PRODUCTION` 的完整配置门禁。
- 不增加逆运动学、笛卡尔控制、轨迹队列、连续流式位置控制或永久保持使能模式。
- 不把通信看门狗全局关闭。
- 不要求上位机脚本代替固件复合状态机。
