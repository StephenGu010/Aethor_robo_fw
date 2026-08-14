# RobotGatewayV1 对接清单

本文是上位机实现首组七自由度机械臂适配器的最小现行契约。正式固件入口使用 `aethor-text-v1`；传输、容量、Profile 和看门狗常量以 `aethor-text-v1-manifest.json` 为机器可读基线。旧 `aethor-arm-ascii-v1` 只用于回归测试。

## 传输与连接

- 在明确选择的 Windows COM 口上打开 STM32 Type-C USB CDC；串口波特率参数不代表 USB 总线速率。
- 请求只包含可打印 ASCII，以 LF 或 CRLF 结束；最大请求长度为 160 字节，不附加应用层 CRC。
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

## 请求与输出格式

请求格式：

```text
[request_id] command [subcommand] [positionals...] [key=value...]
```

- `request_id` 是可选十进制 `uint32`。省略编号时按 `0` 处理，适合手工调试且不保存重放结果。
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
14 bench stop 1,3
15 bench disable 1,3
```

- 电机列表是升序、无重复的 ESC/CAN ID；每条动作必须显式给出，未选电机保持原状态和目标。
- `bench jog` 的位移绝对值不得超过 `3°`，请求速度不得超过 `3°/s`。
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
13 bench jog ...  ---------> 校验并入队
              <------------- ok 13 ... accepted=1
                             发送固定 CAN 目标批次 ----->
ping              ---------> 刷新通信看门狗
              <------------- ok <id> ping ...
                             未到位则内部重发目标 ------>
              <------------- done 13 ... result=completed
```

- 动作请求与保活是两类独立请求：动作只提交一次，`ping` 在等待期间周期发送。
- 任一电机已使能或存在活动运动时，上位机每 250 ms 或更快发送一次有效 `ping`；调试脚本采用 200 ms。
- 连续 1000 ms 没有有效请求时，固件执行停止和失能并发布链路超时事件。
- 全部电机失能且无运动时，看门狗不触发停止/失能，因此只读手工调试不需要周期发送指令。
- 串口异常、脚本中断或动作失败时，上位机仍应尽力依次发送 `bench stop` 和 `bench disable`。

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
5. 验证动作仅发送一次、固件内部目标重发，以及周期 `ping` 不会重复执行业务动作。
6. 验证 `stream joints/motors` 的频率边界、序号间隙和丢旧保新。
7. 验证 `stop`、`disable`、1000 ms 链路超时、USB 断开、重连和 `boot` 改变。
8. 保存 Aethor Studio V2 的版本或 commit、固件 commit、精确命令、串口记录和结果。

外部上位机尚未完成上述验收时，只能声明“固件软件与兼容性测试包完成，外部上位机待验收”。
