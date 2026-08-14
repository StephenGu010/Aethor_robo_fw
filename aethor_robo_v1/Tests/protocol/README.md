# 协议测试资产

当前固件正式入口是 `aethor-text-v1`。`aethor-text-v1-vectors.json` 固定可打印 ASCII 请求、可选十进制请求编号、LF/CRLF、分片/粘连、一次提交的 `bench move` 和统一 `ok/done/error/event/data` 输出示例；正式传输不包含应用层 CRC。

对应的主机测试：

```powershell
.\Tests\host\run_text_protocol_tests.ps1
.\Tests\host\run_text_protocol_engine_tests.ps1
.\Tests\host\run_text_protocol_arm_profile_tests.ps1
.\Tests\host\run_simulator_tests.ps1
```

模拟器测试还验证 `help bench` 与 manifest 命令一致、`show motor <id>` 先发现动态运动边界再提交一次性移动，以及仅 C 协议接受的有效请求和精确重放刷新通信看门狗；所有拒绝请求均不得延后超时。

- `run_text_protocol_tests.ps1` 验证有界文本解析、空格、大小写、字段和错误边界。
- `run_text_protocol_engine_tests.ps1` 验证查询、流、台架 Profile、动作 `ok/done`、请求重放和通信看门狗。
- `run_text_protocol_arm_profile_tests.ps1` 验证正式七轴 Profile、七值关节命令和配置安全门控。
- `run_simulator_tests.ps1` 验证确定性文本模拟器和只读参考客户端。

共享向量中的 `bench_move_build_cases` 固定以下一一映射请求：

```text
50 bench move 1 position=90 speed=30
51 bench move 1,3 position=90,-45 speed=30,20
```

`bench_move_invalid_cases` 固定数量不匹配的 `count_mismatch` 和保留编号 `0` 的 `bad_argument` 分类；`bench_move_terminal_cases` 固定 `completed/failed/cancelled/stopped` 四种终态；`bench_move_request_sequences` 固定相同请求重放和相同 ID 不同正文的 `request_conflict`。模拟器测试会加载这些向量，验证参考客户端的严格等长列表生成、本地唯一 1–7/正常 `float32`/正速度检查、单次写入、无 `ping` 等待以及终态字段解析。固件 C 协议引擎测试继续直接覆盖解析、入队、重放、冲突、busy 和 STOP 抢占。

参考客户端把固件 `strtof` 可接收的正常 `float32` 编码为不含指数的纯十进制定点文本，保留可往返的非零小数并把位置 `-0` 规范为 `0`。非零绝对值必须位于 `FLT_MIN..FLT_MAX`，位置另允许 `0`，速度必须大于 `0`；请求编号必须位于 `1..UINT32_MAX`。完整 ASCII 请求正文不得超过 160 字节；该限制与 `TEXT_PROTOCOL_MAX_REQUEST_LINE_LENGTH` 相同且不计 CR/LF。动作超时必须是有限正数且不超过 `4294967.295` 秒，使内部毫秒预算可表示为 `uint32`。客户端在调用串口写入前拒绝非法编号/数值、非法动作超时以及超长的单轴或多轴组合。

台架动作只由上位机提交一次；固件在未到位时内部重发固定 CAN 目标。自包含 `bench move` 不启动 `ping`，最终 HOLD、所选电机失能并收到新鲜 disabled 反馈后才完成；旧 `bench enable/jog` 带电期间仍通过约 250 ms 的独立 `ping` 保活。该边界由协议引擎、应用门面和主机测试共同覆盖，不由 JSON 向量单独证明。

以下资产属于迁移前 `aethor-arm-ascii-v1`，保留用于回归，不是固件正式协议入口：

- `aethor-arm-ascii-v1-golden.json`
- `aethor-arm-ascii-v1-compatibility-vectors.json`
- `generate_golden_header.ps1`
- `run_protocol_tests.ps1` 使用的 CRC 与旧帧格式测试

旧资产继续固定 CRC-16/CRC-32、`REQ` 操作名、键值字段和旧输出封帧，避免重构意外破坏兼容代码。修改旧回归资产不等于修改 `aethor-text-v1` 协议版本。

所有主机测试均在 `Tests/host/build` 下生成临时产物；生成文件不是协议源，不应手工编辑。测试通过只证明软件契约，不代表机械软限位、方向、减速比、实际角速度、USB CDC、CAN、电机负载、七轴联动或机械臂实机验收完成。
