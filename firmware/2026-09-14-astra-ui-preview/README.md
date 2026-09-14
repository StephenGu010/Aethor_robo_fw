# Astra 界面 B 固件预览

分支：`feature/lvgl-astra-ui`。基于保留的图标版 A 提交 `2d801f9bce308955861f0d1120d5f2b86d3277ea`；文字版发布基线仍为 `f33fec1e8185baff43d0be1ea08317d7d5f035cc`。

**未烧录。** 三个 HEX 来自同一次完整软件验证，28 项检查与四个 Keil 目标全部通过，零错误、零警告。源码摘要在验证期间不变，归档复制后重新核对哈希。详细状态及源码清单见 [verification.json](verification.json)，操作路径、内存比较和证据边界见 [Astra 界面 B 说明](../../docs/debug-ui/astra-ui-verification.md)。

| 文件 | SHA-256 |
|---|---|
| [LCD-ReadOnly.hex](LCD-ReadOnly.hex) | `52c6772e998d4db2c003de4cdace3a8d15be090e0050a2a46731c0acf0f45f63` |
| [LCD-POS.hex](LCD-POS.hex) | `a6a06b6d826393463f2079f0411c7c353fa3d8782f4e845fb91fe37c961ee5d9` |
| [LCD-MIT.hex](LCD-MIT.hex) | `f9e8f9d49df91bf20aa585278a3ca605c42ebd45d4ab668d7430f942880e60aa` |

89 张真实 LVGL 软件渲染页面保存在 `Tests/ui/artifacts-astra`。这些图片使用模拟数据，不代表板上显示、运动或20 ms刷新负载验收。

上一版 A 的分支、[三个归档 HEX](../2026-09-14-icon-ui-preview/README.md) 与 `Tests/ui/artifacts` 原页面保持不变。本版未合入或推送到 `main`。
