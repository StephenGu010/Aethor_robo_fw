# 上位机兼容性测试包

本目录交付独立于 Aethor Studio V2 的共享契约。它可以验证固件协议和客户端实现，但不能替代真实 USB CDC、CAN、电机或外部上位机验收。

## 内容

- `aethor-arm-ascii-v1-schema.json`：协议、身份、Profile 和能力清单。
- `RobotGatewayV1.md`：上位机接口和验收要求。
- `state-and-command-lifecycle.md`：状态机、命令和重连语义。
- `compatibility-result-template.md`：外部上位机恢复联系后的记录模板。
- `Tests/protocol/aethor-arm-ascii-v1-compatibility-vectors.json`：CRC、响应、事件、遥测、分包、粘包和坏帧向量。
- `tools/aethor_host_simulator.py`：确定性七电机固件主机模拟器。
- `tools/aethor_reference_client.py`：模拟器或 COM 口的脚本化安全查询客户端。

## 一键验证

```powershell
powershell.exe -ExecutionPolicy Bypass -File .\Tests\host\run_simulator_tests.ps1
```

该命令还执行 8 小时静态遥测与 2 小时运动/停止/恢复的逻辑时钟长稳。它在几十秒内推进模拟时间，只证明状态、缓存和队列有界，不代表墙钟长稳或硬件长稳。

真实 USB CDC 只读探测需要先安装 `pyserial`，并明确传入端口：

```powershell
python .\tools\aethor_reference_client.py --port COM7 --output .\aethor-com7-transcript.txt
```

该客户端只发送 HELLO、查询和关闭遥测，不使能、不运动。
