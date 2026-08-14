# COM7 双电机台架调试脚本设计

## 目标

为当前 `USB_BENCH_RELATIVE` 固件新增一个 Windows PowerShell 调试脚本，通过 COM7 对 CAN ID `0x01`、`0x03` 两台空载 S3519 电机执行可复现的分阶段调试。脚本使用当前 `aethor-arm-ascii-v1` 协议，不再发送旧版 `#PING`、`!START` 或 `@...` 命令。

默认运行只完成 USB CDC、协议、配置、状态和电机发现检查。只有用户显式传入 `-RunMotion` 时，脚本才允许清错、使能和小角度运动。

## 命令行接口

脚本路径固定为：

```text
Tests/hardware/debug_com7_motors_1_3.ps1
```

支持以下参数：

- `-PortName`：默认 `COM7`。
- `-MotorList`：默认 `1,3`，必须唯一且升序，并限制在 `1..7`。
- `-RunMotion`：默认关闭；打开后才执行使能和运动。
- `-DeltaDegrees`：默认 `0.2`，必须大于 `0` 且不超过固件台架限制 `3.0`。
- `-SpeedDegreesPerSecond`：默认 `1.0`，必须大于 `0` 且不超过 `3.0`。
- `-SelfTest`：不打开串口，只运行 CRC、帧解析和参数校验测试。

## 数据流与协议

脚本直接使用 Windows `.NET System.IO.Ports.SerialPort`，避免新增 `pyserial` 依赖。串口配置使用 ASCII、115200、8N1、LF 读取和 CRLF 发送。

每个请求按以下格式编码：

```text
REQ <request_id> <operation> [fields...] *<CRC16>\r\n
```

CRC 使用 `CRC-16/CCITT-FALSE`。脚本验证每一条固件回复的 CRC，并区分：

- 查询：等待同一请求号的 `RSP` 或 `ERR`。
- 动作：先接受 `ACK`，再等待同一请求号的 `DONE` 或 `ERR`。
- 心跳：从 `HELLO` 回复提取运行时 `session`，动作等待期间每 250 ms 发送一次新的 `HEARTBEAT` 请求。

请求号由脚本统一递增，避免同一会话内发生 `REQUEST_ID_CONFLICT`。

## 分阶段执行

所有运行首先执行：

1. 打开 COM7，并在端口被占用时明确提示关闭 UartAssist 的串口连接。
2. `HELLO` 建立会话，提取 `session` 和 `boot_id`。
3. 关闭自动遥测流，依次执行 `GET_INFO`、`GET_CONFIG`、`GET_STATE`、`GET_MOTORS` 和 `GET_DIAG`。
4. 执行 `INIT_MOTORS motors=1,3`，等待 `DONE COMPLETED`。
5. 再次执行 `GET_MOTORS`，保存发现结果。

未传入 `-RunMotion` 时，脚本在这里结束，不发送清错、使能或位置目标。

传入 `-RunMotion` 时继续：

1. `CLEAR_FAULT motors=1,3`。
2. `ENABLE motors=1,3`，必须等待 `DONE COMPLETED`。
3. 第一次 `MOVE_REL`：J1 为 `+0.2°`，J3 为 `-0.2°`，两台速度均为 `1.0°/s`。
4. 第二次 `MOVE_REL`：J1 为 `-0.2°`，J3 为 `+0.2°`，使电机回到起始位置附近。
5. `STOP motors=1,3`。
6. `DISABLE motors=1,3`。
7. 查询最终 `GET_MOTORS` 和 `GET_DIAG`。

对于其他合法 `MotorList`，脚本按升序交替生成正负位移，第二次运动使用相反符号。

## 安全与错误处理

- 默认不运动；运动必须显式使用 `-RunMotion`。
- 所有动作按顺序执行，上一动作未返回成功 `DONE` 时不发送下一动作。
- `HELLO` 后自动维持 250 ms 心跳，避免 1,000 ms 通信看门狗触发。
- 任意 CRC 错误、超时、`ERR`、失败 `DONE`、端口异常或用户中断都会进入清理路径。
- 清理路径在会话和端口仍有效时尽力发送 `STOP`、再发送 `DISABLE`；随后关闭并释放 COM7。
- 脚本不修改电机永久参数、不保存零位、不改变 CAN ID 或 Master ID。
- 脚本不能替代物理急停；运行前仍需保证两台电机空载、固定和可断电。

## 日志和验收

控制台逐行记录时间、方向、请求正文和已验证回复，但不伪造硬件成功。脚本只有在以下条件全部满足时才输出运动通过：

- 握手、配置、状态和初始化请求成功。
- 两台电机的清错、使能、两次运动、停止和失能均收到成功终态。
- 最终查询没有协议级错误。
- COM7 已正常关闭。

自动化验证采用测试驱动顺序：先添加会因脚本尚不存在或功能缺失而失败的自测入口，再实现最小脚本使 CRC 参考向量、帧解析、参数边界和安全默认值测试通过。真实 COM7 运行结果与纯脚本自测分开记录；端口被 UartAssist 占用时不强制关闭其进程。
