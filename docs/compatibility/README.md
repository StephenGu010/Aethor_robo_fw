# 上位机兼容性测试包

本目录交付独立于 Aethor Studio V2 的共享契约。当前固件正式入口是 USB CDC 上的 `aethor-text-v1`：可打印 ASCII、LF/CRLF 行边界、无应用层 CRC。这里的自动化资产可以验证固件协议和客户端实现，但不能替代真实 USB CDC、CAN、电机或外部上位机验收。

## 当前生效资产

- `aethor-text-v1-manifest.json`：传输、身份、Profile、命令边界、流和看门狗的机器可读基线。
- `aethor-text-v1-migration.md`：人工串口调试、旧协议迁移和安全边界。
- `RobotGatewayV1.md`：上位机请求、输出、动作生命周期和验收要求。
- `state-and-command-lifecycle.md`：正式机械臂、台架动作、请求重放和断线语义。
- `compatibility-result-template.md`：外部上位机恢复联系后的验收记录模板。
- `Tests/protocol/aethor-text-v1-vectors.json`：文本行、请求编号、LF/CRLF、分片/粘连和一次提交 `bench move` 向量。
- `tools/aethor_text_simulator.py`：包含一次提交 `bench move` 生命周期的确定性 `aethor-text-v1` 模拟器。
- `tools/aethor_reference_client.py`：模拟器或显式 COM 口的只读查询客户端，并提供一次写入、无保活的 `bench_move_once` 集成方法。
- `Tests/hardware/debug_com7_aethor_text_v1.ps1`：默认只读、显式 `-RunMotion` 才运动的分阶段台架脚本。

`aethor-arm-ascii-v1-schema.json`、旧 Golden/Compatibility Vectors 和 `tools/aethor_host_simulator.py` 仅用于迁移前协议的回归测试，不是固件正式入口。

## 一键验证

```powershell
powershell.exe -ExecutionPolicy Bypass -File .\Tests\host\run_text_protocol_tests.ps1
powershell.exe -ExecutionPolicy Bypass -File .\Tests\host\run_text_protocol_engine_tests.ps1
powershell.exe -ExecutionPolicy Bypass -File .\Tests\host\run_text_protocol_arm_profile_tests.ps1
powershell.exe -ExecutionPolicy Bypass -File .\Tests\host\run_simulator_tests.ps1
powershell.exe -ExecutionPolicy Bypass -File .\Tests\host\test_com7_dual_motor_debug_script.ps1
```

这些命令覆盖文本解析、Profile 门控、查询/动作生命周期、请求重放、看门狗、模拟器、参考客户端和调试脚本自测。测试通过只证明软件契约，不代表墙钟长稳、硬件长稳或机械标定完成。

真实 USB CDC 只读探测需要先安装 `pyserial`，并明确传入端口：

```powershell
python .\tools\aethor_reference_client.py --port COM7 --output .\aethor-com7-transcript.txt
```

该客户端只发送 `hello`、`show` 查询和 `stream off`，不使能、不运动。

需要进行台架运动时，使用 `debug_com7_aethor_text_v1.ps1` 并逐级放行。新 `bench move` 使用非零 `uint32` 请求编号，只发送一次；收到匹配的 `ok ... accepted=1` 后等待唯一匹配 `done`，上位机不启动 `ping`，固件内部负责重发未到位的固定 CAN 目标。旧 `bench enable/jog` 不属于该自包含生命周期，带电期间仍需约每 250 ms 发送一次独立 `ping`；不要周期重发动作正文。软件测试通过不能替代机械负载、方向、减速比、七轴联动或真实硬件验证。
