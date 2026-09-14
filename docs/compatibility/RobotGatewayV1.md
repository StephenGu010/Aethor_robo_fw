# RobotGatewayV1 对接清单

本文是上位机实现首组七自由度机械臂适配器的最小现行契约。正式固件入口使用 `aethor-text-v1`；传输、容量、Profile 和看门狗常量以 `aethor-text-v1-manifest.json` 为机器可读基线。旧 `aethor-arm-ascii-v1` 只用于回归测试。

## 传输与连接

- 在明确选择的 Windows COM 口上打开 STM32 Type-C USB CDC；串口波特率参数不代表 USB 总线速率。
- 请求只包含可打印 ASCII，以 LF 或 CRLF 结束；最大请求正文为 160 字节（不计 CR/LF），不附加应用层 CRC。
- 每次连接先发送 `hello`，确认回复中的 `protocol=aethor-text-v1`、`profile`、`dof=7`、`boot` 和 `watchdog_ms=1000`。
- 随后发送 `stream off` 和只读 `show` 查询，确认目标设备、配置、状态和电机掩码后再开放动作按钮。
- `boot` 改变或 USB 重新连接后，清空上位机正在等待的请求和旧目标，并重新执行只读探测；正式机械臂 Profile 还需重新确认参考位状态。

推荐的只读探测：

```text
hello
stream off
show info
show config
show state
show joints
show motors
show diag
```

### Windows 手工串口设置

常见串口调试助手按以下值配置：

- 串口：STM32 USB CDC 实际枚举的 COM 口；当前台架为 COM7。
- 显示和发送：ASCII；115200、8 数据位、1 停止位、流控 `NONE`、串口校验位 `NONE`。
- 应用层校验算法：`无`，不附加 CRC、校验和或帧头。
- 自动附加指令结束符：CRLF，HEX 为 `0D 0A`；也可只附加 LF `0A`。
- 关闭循环发送。自包含 `bench move`、`bench mode` 和 `bench mit` 正文只发送一次。

固件只有收到行结束符才会解析并响应。若调试工具已多次发送不带结束符的正文，应先复位板卡或发送一个单独换行清空残留半行，然后重新打开串口并从 `1 hello` 开始。确认只读响应后，可用新的非零编号执行和回零：

```text
1 hello
2 stream off
3 show motor 1
70 bench move 1 position=30 speed=5
71 bench move 1 position=0 speed=5
72 bench move 1,3 position=35,100 speed=5,5
73 bench move 1,3 position=0,0 speed=5,5
```

`position=0` 表示运动到本次上电电机坐标系的绝对零位，不执行机械零点标定，也不写入零偏。每条新正文使用新的非零请求编号；旧编号与相同正文只重放结果，旧编号与不同正文返回 `request_conflict`。

## 请求与输出格式

请求格式：

```text
[request_id] command [subcommand] [positionals...] [key=value...]
```

- `request_id` 是可选十进制 `uint32`。省略编号时按 `0` 处理，适合手工调试且不保存重放结果。
- 自包含 `bench move/bench mode/bench mit` 必须显式使用 `1..4294967295` 的编号；`0` 是内部活动动作哨兵，不接受为这些命令的编号。
- 正式上位机应为业务请求分配非零编号，并在至少 60 秒内避免给不同正文复用同一编号。
- 同一非零编号和相同正文会返回近期结果，不会再次执行动作；相同编号配不同正文会返回冲突错误。
- 字段名使用小写；重复字段、制表符、不可打印字符和超出容量的请求会被拒绝。

固件输出只使用五类首词：

```text
ok <request_id> ...
done <request_id> ...
error <request_id> ...
event <sequence> ...
data <sequence> ...
```

- 查询以匹配编号的 `ok` 或 `error` 结束。
- 动作在校验或入队失败时直接返回 `error`；受理后先返回 `ok <id> ... accepted=1`，随后以同一编号的 `done` 结束，并通过 `result=completed/stopped/cancelled/failed` 表示终态。
- `ok ... accepted=1` 只表示命令进入有界业务队列，不表示电机动作完成。
- `event` 是故障、状态或链路事件；`data` 是显式开启的状态流。上位机不能把遥测到达当作保活。

## 命令边界

通用命令：

```text
hello
ping
help
help bench
show info
show state
show joints
show motors
show motor 1
show config
show config 1
show diag
show diag can
show diag motion
stream joints rate=20
stream motors rate=10
stream off
```

台架 Profile：

```text
10 bench init 1,3
11 bench clear 1,3
12 bench enable 1,3
13 bench jog 1,3 delta=0.2 speed=1
50 bench move 1 position=90 speed=30
51 bench move 1,3 position=90,-45 speed=30,20
60 bench mode 1 mode=mit
61 bench mit 1 action=hold kp=1 kd=1 torque_ff=0 duration_ms=1000
62 bench mit 1 action=move position=5 speed=2 kp=1 kd=1 torque_ff=0 duration_ms=1000
63 bench mode 1 mode=pos_vel
14 bench stop 1,3
15 bench disable 1,3
```

- `bench init/mode/enable/jog/stop/disable/clear` 的电机列表必须是严格升序、无重复的 `1..7` ESC/CAN ID。`bench move` 单独允许任意唯一顺序，其电机、位置和速度列表必须非空且严格等长，并严格按调用者列表顺序一一映射，不支持广播。`bench mit` 当前只允许一个电机，禁止把未标定 MIT 参数同时施加到多个关节。
- `bench mode` 只接受 `mode=mit` 或 `mode=pos_vel`。固件会完成所选电机发现、失能、低速门控、易失性模式寄存器写入和读回确认，成功后仍保持失能；它不会自动 `ENABLE`，也不写电机 Flash。
- `bench mit` 的 `kp` 范围为 `0 < kp <= 500`，`kd` 范围为 `0 < kd <= 5`，`duration_ms` 范围为 `100..10000`。`position/speed` 与 `bench move` 一样是 S3519 输出端角度和角速度；`torque_ff` 是输出端 N·m。固件使用发现读回的 `PMAX/VMAX/TMAX/MAX_SPD` 拒绝越界参数，不截断、不使用静态量程回退。
- `action=hold` 以受理时的新鲜输出端位置为目标，持续发送 MIT 帧到 `duration_ms`；`action=move` 由 STM32 本地生成五次时间缩放轨迹，`speed` 是轨迹最大期望速度上限，而不是 Windows 串口发帧频率。动作结束后按目标位置保持 `duration_ms`，然后自动失能。
- `kp=1 kd=1 torque_ff=0` 仅来自厂商 MIT 示例，作为无负载首次上电的低能量起点，不代表当前机械臂已标定。参数实调步骤和记录表见 [S3519 MIT 参数标定与调试指南](S3519-MIT参数标定与调试指南.md)。
- `bench jog` 的 `delta/speed` 使用与固件相同的普通十进制正常 `float32` 语法，不接受指数；`delta` 可正、可负或为 `0`，`speed` 必须大于 `0`。确定性模拟器不为 legacy jog 伪造发现能力上限；真实固件仍在动作执行阶段依据本次发现的 `PMAX/VMAX/MAX_SPD` 做运行时校验。
- `bench move` 使用相对本次上电零点的 S3519 输出端绝对角度，不是机械臂关节软限位。位置和速度只接受不含指数的十进制正常 `float32`；位置还允许 `0`，速度必须大于 `0`。
- 位置边界来自本次发现的 `PMAX`，速度边界来自 `min(VMAX,MAX_SPD)`；边界值允许，越界拒绝且不截断，任一发现值缺失时不回退默认值。`show motor <id>` 通过 `pmax_deg/vmax_deg_s/max_speed_deg_s/move_speed_limit_deg_s` 显示这些值，不可用时显示 `?`。
- 对动作编号 `13` 只发送一次。收到 `ok 13 bench jog accepted=1` 后等待 `done 13 ...`；不要通过新编号重复发送同一动作。
- 固件在电机未到位时内部周期重发同一批固定绝对 CAN 目标，上位机不负责重发动作目标。

正式机械臂 Profile：

```text
20 arm align 0,0,90,0,0,0,0
21 arm enable
22 arm move 0,-15,30,0,20,0,5 speed=5
23 arm stop
24 arm disable
25 arm clear
```

- `arm align` 和 `arm move` 必须一次提供恰好七个有限关节值，不能拆成七条单轴命令。
- 正式 Profile 只有在七轴物理字段、身份、模式和反馈均满足安全门控后才允许真实使能；当前生产验证掩码仍未放行。
- DH/URDF、逆运动学、笛卡尔控制、动力学和碰撞规划属于上位机范围，不进入当前固件协议。

## 动作与保活

```text
上位机                       固件                         S3519
50 bench move ... ---------> 校验并入队
              <------------- ok 50 bench move accepted=1
                             发现、使能并发送固定目标 ----->
                             未到位则内部重发目标 ------>
                             HOLD、所选电机失能 ---------->
              <------------- done 50 bench move result=completed ...
```

- 新 `bench move` 只提交一次并等待同一编号的 `ok` 后接唯一 `done`，其等待期间不发送 `ping`。固件负责发现到失能清理全过程，完成仍要求新鲜 CAN 反馈、无驱动故障、控制周期安全且无 Bus-Off，并在所选电机收到新鲜 disabled 反馈后才报告 `completed`。
- `bench mode` 和 `bench mit` 采用同一自包含生命周期：上位机只提交一次，不发送 `ping`；`bench stop <motor>` 可以抢占，任何失败均进入所选电机失能清理。模式切换动作完成后保持失能，MIT 动作完成后也必须收到新鲜 `disabled` 反馈才报告 `completed`。
- 旧 `bench enable/jog` 不是自包含动作：带电或运动期间上位机每约 250 ms 发送一次独立 `ping`；调试脚本采用 200 ms。不要周期重发任何动作正文来代替保活。
- 连续 1000 ms 没有有效请求时，固件执行停止和失能并发布链路超时事件。
- 全部电机失能且无运动时，看门狗不触发停止/失能，因此只读手工调试不需要周期发送指令。
- 串口异常、脚本中断或动作失败时，上位机仍应尽力依次发送 `bench stop` 和 `bench disable`。

`bench move` 的稳定终态如下；完成结果的 `motors` 是所选电机位掩码，电机 1 对应 bit 0。失败阶段只允许 `validate/discovery/mode/clear/enable/motion/hold/disable/unknown`，错误码只允许 `not_ready/position_out_of_range/speed_out_of_range/fault_present/stale_feedback/timeout/feedback_timeout/action_failed`；`motor` 必须是可归属的所选电机，否则为 `?`。

```text
done 50 bench move result=completed elapsed_ms=3200 motors=01
done 50 bench move result=failed stage=motion code=stale_feedback motor=1
done 50 bench move result=cancelled
done 50 bench move result=stopped
done 60 bench mode result=completed elapsed_ms=18 motors=01
done 62 bench mit result=completed elapsed_ms=5700 motors=01
done 62 bench mit result=failed stage=motion code=stale_feedback motor=1
```

`bench stop` 可抢占活动动作，原动作报告 `cancelled` 且 STOP 产生自己的终态；活动期间第二条普通命令返回 `busy`。相同非零编号和相同正文重放近期结果，相同编号配不同正文返回 `request_conflict`。

## 模型与安全门控

- 实体模型只消费 `show joints` 或 `data ... joints` 中的七轴反馈，不直接使用电机原始弧度。
- 幽灵模型只表示用户目标；只有匹配请求编号的终态结果才能释放动作编排门控。
- `show motors` 提供 `present/enabled/moving/holding/stale/fault` 掩码；`show config` 提供映射、验证掩码和 `enable_ready`。
- 遥测拥塞时允许丢旧保新，因此 `data` 序号可以出现间隙；`ok/done/error/event` 不能按遥测方式丢弃。
- 上位机不得用默认值绕过方向、零位、限位、速度、加速度、驱动量程、减速比或 MIT 增益验证位。
- 故障清除只能在去使能、无运动且故障源消失后进行；清除后重新读取状态和反馈，再决定是否允许下一动作。

## 联调验收

1. 验证 LF/CRLF、分片、粘连、最大长度、非法字符和重复字段处理。
2. 验证 `hello`、`boot`、Profile、`show config` 映射和七轴 ID 顺序。
3. 验证查询 `ok` 与动作 `ok accepted=1`、终态 `done/error` 的不同生命周期。
4. 验证相同非零请求重放不会重复动作，冲突正文会被拒绝。
5. 验证 `bench move/bench mode/bench mit` 仅发送一次且不启动保活；验证 MIT 轨迹由固件内部连续更新，以及旧 `bench enable/jog` 的周期 `ping` 不会重复执行业务动作。
6. 验证 `stream joints/motors` 的频率边界、序号间隙和丢旧保新。
7. 验证 `stop`、`disable`、1000 ms 链路超时、USB 断开、重连和 `boot` 改变。
8. 逐电机验证 `MIT -> POS_VEL -> MIT` 只在失能且接近零速时切换，切换后不自动使能；记录读回模式和最终失能反馈。
9. 保存 Aethor Studio V2 的版本或 commit、固件 commit、精确命令、串口记录和结果。

外部上位机尚未完成上述验收时，只能声明“固件软件与兼容性测试包完成，外部上位机待验收”。
