# 协议测试资产

当前固件正式入口是 `aethor-text-v1`。`aethor-text-v1-vectors.json` 固定可打印 ASCII 请求、可选十进制请求编号、LF/CRLF、分片/粘连和统一 `ok/done/error/event/data` 输出示例；正式传输不包含应用层 CRC。

对应的主机测试：

```powershell
.\Tests\host\run_text_protocol_tests.ps1
.\Tests\host\run_text_protocol_engine_tests.ps1
.\Tests\host\run_text_protocol_arm_profile_tests.ps1
.\Tests\host\run_simulator_tests.ps1
```

- `run_text_protocol_tests.ps1` 验证有界文本解析、空格、大小写、字段和错误边界。
- `run_text_protocol_engine_tests.ps1` 验证查询、流、台架 Profile、动作 `ok/done`、请求重放和通信看门狗。
- `run_text_protocol_arm_profile_tests.ps1` 验证正式七轴 Profile、七值关节命令和配置安全门控。
- `run_simulator_tests.ps1` 验证确定性文本模拟器和只读参考客户端。

台架动作只由上位机提交一次；固件在未到位时内部重发固定 CAN 目标。电机使能或运动期间，上位机通过独立 `ping` 保活。该边界由协议引擎、应用门面和 COM7 调试脚本测试共同覆盖，不由 JSON 帧向量单独证明。

以下资产属于迁移前 `aethor-arm-ascii-v1`，保留用于回归，不是固件正式协议入口：

- `aethor-arm-ascii-v1-golden.json`
- `aethor-arm-ascii-v1-compatibility-vectors.json`
- `generate_golden_header.ps1`
- `run_protocol_tests.ps1` 使用的 CRC 与旧帧格式测试

旧资产继续固定 CRC-16/CRC-32、`REQ` 操作名、键值字段和旧输出封帧，避免重构意外破坏兼容代码。修改旧回归资产不等于修改 `aethor-text-v1` 协议版本。

所有主机测试均在 `Tests/host/build` 下生成临时产物；生成文件不是协议源，不应手工编辑。测试通过只证明软件契约，不代表 USB CDC、CAN、电机或机械臂实机验收完成。
