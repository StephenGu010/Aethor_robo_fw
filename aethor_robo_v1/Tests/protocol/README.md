# aethor-arm-ascii-v1 共享协议测试资产

`aethor-arm-ascii-v1-golden.json` 是固件与 PC 端协议测试共同使用的可移植数据源。它固定请求正文、CRC、行结束兼容方式和预期结构化解析结果；修改协议格式时必须先更新该文件并同步提高协议版本或兼容性说明。

固件侧执行：

```powershell
.\Tests\host\run_protocol_tests.ps1
```

脚本在 `Tests/host/build` 中生成临时 C 头文件并运行严格 C11 测试。生成文件不是协议源，不应手工编辑。

当前覆盖的是帧边界、CRC-16、CRC-32、请求号、操作名、键值字段、LF/CRLF 和统一输出封帧。命令语义、重复请求生命周期、USB CDC 队列和运动状态机由后续测试切片接入；本测试通过不代表 USB、CAN 或电机实机验证完成。
