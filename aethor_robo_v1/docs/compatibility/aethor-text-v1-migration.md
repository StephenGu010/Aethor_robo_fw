# aethor-text-v1 迁移与调试说明

## 1. 当前生效入口

固件 USB CDC 收到一行文本后，由 `aethor_app_process_protocol_line()` 调用
`protocol_engine_process_text_line()`。生产入口使用 `aethor-text-v1`，不再要求上位机
拼接应用层 CRC。旧版 `aethor-arm-ascii-v1` 解析器仅保留给主机回归测试，不作为固件
应用入口。

请求使用可打印 ASCII，以 LF 或 CRLF 结尾。单条请求最多 160 字节；单条响应包含 LF
最多 192 字节。请求编号是可选十进制 `uint32`：编号 0 视为手动命令，不进入结果重放。

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

- `AETHOR_PROFILE_BENCH=1`：台架演示配置；开放 `init/enable/jog/stop/disable/clear`。
- `AETHOR_PROFILE_BENCH=0`：机械臂配置；开放 `align/enable/move/stop/disable/clear`。
- 两种配置都只对外提供 `POS_VEL`，避免把底层电机模式直接暴露给演示人员。
- 台架 `jog` 必须给出显式电机列表，绝对位移不超过 3 度，速度不超过 3 度每秒。

## 4. COM7 分阶段调试

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

## 5. 动作只提交一次，保活独立发送

`bench jog` 等动作请求只发送一次。固件返回 `ok <request_id> ... accepted=1` 后，
上位机保持该请求编号并等待匹配的 `done`，不要用新的请求编号周期重复提交相同动作。
对于非零请求编号，完全相同的编号和正文只会重放近期结果，不会再次执行动作；编号相同
但正文不同会返回 `request_conflict`。

台架点动未到位时，固件在内部周期重发动作开始时生成的固定绝对 CAN 目标批次。上位机
不负责重发运动目标，只在电机已使能或正在运动时每 250 ms 或更快发送独立的 `ping`；
调试脚本采用 200 ms。全部电机失能且无运动时，通信看门狗不触发停止/失能，因此人工
只读调试可以只发送一次查询。

## 6. 上位机迁移边界

- 删除请求尾部 CRC 字段和 `!/#/>/@/&/$` 类前缀分派。
- 按行读取并以首词分派 `ok/done/error/event/data`。
- 动作收到 `accepted=1` 后等待同一请求编号的 `done`，不要把动作重发当作保活。
- 使能期间每 250 ms 发送一次 `ping`，不要依赖高频遥测替代保活。
- `stream` 默认关闭；重新设置流时覆盖旧设置，拥塞时丢旧值、保留最新值。
- 兼容性常量和硬件身份以 `aethor-text-v1-manifest.json` 为机器可读基线。
