# LCD-MIT 2026-09-14 固定发布

此处是图标UI改造前的当前发布基线，不会被后续构建自动覆盖。

- [LCD-MIT.hex](LCD-MIT.hex)：已记录烧录读回的固件。
- [verification.json](verification.json)：27项检查结果、对应源码与HEX哈希、原视图渲染基线。
- [操作与发布说明](../../docs/debug-ui/release-20260914.md)。

HEX SHA256：`c55bca02f9dabae20263f4df16707875e9324192bccbd041cdebc571ea34d326`。

历史Flash读回249848字节一致。本次上传不烧录。图标UI尚未实现在此工件；软件验证不等于整臂/带载验收。更换电机后重新核对坐标、发现参数与配置，不能直接沿用7号电机兼容值。
