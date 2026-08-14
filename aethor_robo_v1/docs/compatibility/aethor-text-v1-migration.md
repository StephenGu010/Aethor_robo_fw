# aethor-text-v1 迁移与调试说明

## 1. 当前生效入口

固件 USB CDC 收到一行文本后，由 `aethor_app_process_protocol_line()` 调用
`protocol_engine_process_text_line()`。生产入口使用 `aethor-text-v1`，不再要求上位机
拼接应用层 CRC。旧版 `aethor-arm-ascii-v1` 解析器仅保留给主机回归测试，不作为固件
应用入口。

请求使用可打印 ASCII，以 LF 或 CRLF 结尾。单条请求最多 160 字节；单条响应包含 LF
最多 256 字节。请求编号是可选十进制 `uint32`：编号 0 视为手动命令，不进入结果重放。

## 2. 可直接手工输入的安全查询

```text
hello
ping
show info
show state
show joints
show motors
show config
show diag
stream off
```

固件输出只使用以下五类首词，便于人读和上位机分派：

```text
ok
done
error
event
data
```

## 3. 编译配置

- `AETHOR_PROFILE_BENCH=1`：台架演示配置；开放 `init/enable/jog/move/stop/disable/clear`。
- `AETHOR_PROFILE_BENCH=0`：机械臂配置；开放 `align/enable/move/stop/disable/clear`。
- 两种配置都只对外提供 `POS_VEL`，避免把底层电机模式直接暴露给演示人员。
- 台架 `jog/move` 必须给出显式电机列表；位置和速度统一由本次启动发现的电机能力做运行时校验，不再使用固定 3 度或 3 度每秒上限。

## 4. 一条命令完成绝对台架运动

```text
50 bench move 1 position=90 speed=30
51 bench move 1,3 position=90,-45 speed=30,20
```

`bench move` 的位置是 S3519 输出端相对本次上电零点的绝对角度，不是机械臂关节软限位。电机、位置和速度列表必须非空且严格等长，并按列表顺序一一对应；电机编号必须唯一且位于 1–7。位置和速度必须是有限数，速度必须大于 0，不支持单值广播。

列表数量不匹配返回 `code=count_mismatch` 并指出 `field=position|speed`；重复、空或超出 1–7 的电机列表返回 `code=bad_argument field=motors`；非数值、NaN 或 Inf 返回对应字段的 `bad_argument`；零速或负速度返回 `code=out_of_range field=speed`。解析或入队失败不会产生后续动作终态。

固件只使用本次发现的 `PMAX/VMAX/MAX_SPD`：`abs(position_deg) <= PMAX`，`speed_deg_s <= min(VMAX,MAX_SPD)`。等于边界时允许；越界值直接拒绝，不截断；发现值不完整时拒绝，不回退任何默认范围。可先发送 `show motor <id>` 查看：

```text
pmax_deg=<value|?>
vmax_deg_s=<value|?>
max_speed_deg_s=<value|?>
move_speed_limit_deg_s=<value|?>
```

这些字段采用三位小数的紧凑输出，例如 20 rad/s 显示为 `1145.916` deg/s；`?` 表示本次上电尚无可用发现值。

主机只发送一次 `bench move`，收到 `ok <id> bench move accepted=1` 后等待匹配的 `done`，不发送 `ping`。固件在内部重发已固定的逐电机 CAN 目标，并依次完成发现、范围校验、POS_VEL 模式、清错、使能、运动、最终位置加零速度 HOLD、所选电机失能；只有收到全部所选电机的新鲜 disabled 反馈后才报告完成。该局部通信超时豁免不影响反馈新鲜度、非零驱动故障、控制周期故障、动作期限或 Bus-Off 安全处理。

旧 `bench enable/jog` 不属于自包含动作，带电或运动期间仍须约每 250 ms 发送一次独立 `ping`。不要周期重发动作正文来替代保活。

终态格式如下：

```text
done 50 bench move result=completed elapsed_ms=3200 motors=01
done 50 bench move result=failed stage=motion code=stale_feedback motor=1
done 50 bench move result=cancelled
done 50 bench move result=stopped
```

失败阶段为 `validate/discovery/mode/clear/enable/motion/hold/disable`；无法归属时为 `unknown`。错误码为 `not_ready/position_out_of_range/speed_out_of_range/fault_present/stale_feedback/timeout/feedback_timeout/action_failed`。`motor` 是可归属的所选电机一基编号，否则为 `?`；完成终态的 `motors` 是所选电机位掩码，电机 1 对应 bit 0。`bench stop` 可抢占活动动作，原动作报告 `cancelled`，STOP 有自己的终态；第二条普通命令返回 `busy`，不会覆盖活动动作。

## 5. COM7 分阶段调试

只读探测（默认不会使能或转动电机）：

```powershell
.\Tests\hardware\debug_com7_aethor_text_v1.ps1 -PortName COM7 -MotorList 1,3
```

小角度往返演示（脚本会周期发送 `ping`，结束时执行 `stop` 和 `disable`）：

```powershell
.\Tests\hardware\debug_com7_aethor_text_v1.ps1 `
  -PortName COM7 `
  -MotorList 1 `
  -RunMotion `
  -DeltaDegrees 0.2 `
  -SpeedDegreesPerSecond 1
```

必须先完成只读探测并确认返回 `protocol=aethor-text-v1`、目标电机在线，再执行运动命令。
通信失败、超时或脚本异常时，脚本会尽力发送 `stop` 与 `disable`；固件在电机使能或
运动期间超过 1000 ms 未收到有效保活时也会停止并失能。

## 6. 动作只提交一次，保活独立发送

`bench jog` 等动作请求只发送一次。固件返回 `ok <request_id> ... accepted=1` 后，
上位机保持该请求编号并等待匹配的 `done`，不要用新的请求编号周期重复提交相同动作。
对于非零请求编号，完全相同的编号和正文只会重放近期结果，不会再次执行动作；编号相同
但正文不同会返回 `request_conflict`。

台架动作未到位时，固件在内部周期重发动作开始时生成的固定绝对 CAN 目标批次。上位机
不负责重发运动目标。旧 `bench enable/jog` 在电机已使能或正在运动时每 250 ms 或更快发送
独立的 `ping`，现有调试脚本采用 200 ms；自包含 `bench move` 不发送 `ping`。全部电机失能
且无运动时，通信看门狗不触发停止/失能，因此人工只读调试可以只发送一次查询。

## 7. 上位机迁移边界

- 删除请求尾部 CRC 字段和 `!/#/>/@/&/$` 类前缀分派。
- 按行读取并以首词分派 `ok/done/error/event/data`。
- 动作收到 `accepted=1` 后等待同一请求编号的 `done`，不要把动作重发当作保活。
- 旧 `bench enable/jog` 带电期间每 250 ms 发送一次 `ping`；`bench move` 只发送一次且不启动保活。
- `stream` 默认关闭；重新设置流时覆盖旧设置，拥塞时丢旧值、保留最新值。
- 兼容性常量和硬件身份以 `aethor-text-v1-manifest.json` 为机器可读基线。

软件解析、主机测试和构建只证明合同实现，不证明输出方向、外部减速比、真实角速度、机械软限位、带载能力、七轴联动或真实硬件已经验证。
