# 图标界面固件预览 · 2026-09-14

在已上传的文字界面基线 `f33fec1` 之后，于 `feature/lvgl-icon-ui` 实现。此目录保存本轮四目标构建中的三个LCD工件，**未烧录、未进行实机操作验收**。原基线仍在 `../2026-09-14`，不被此预览替换。

27项软件检查全部通过，Keil四目标均为0错误、0警告。LCD静态RAM为173904 B，比基线减少45120 B；LVGL池32 KiB。LCD-MIT的Code+RO为312652 B；大字库使该项比基线增加63056 B。

[操作和验证说明](../../docs/debug-ui/icon-ui-verification.md) · [机器可读验证摘要](verification.json) · [63页截图清单](../../Tests/ui/artifacts/render_manifest.json)

| 固件 | SHA-256 |
| --- | --- |
| LCD-ReadOnly | `1db39fa9acfbee8ead86bf537eff7a7a88639e1e5f5ff091b801b5ff968ce6c8` |
| LCD-POS | `3d09d725d565508d73e2bbecee1ca51299dc938e97e22152f81b22173850066a` |
| LCD-MIT | `600cce3686adb7a2133e9ce7fb1310721f8a31375503b20c8d2fb6386bc43143` |

三个目标宏门控不同：ReadOnly拒绝全部本地控制，POS开放POS，MIT开放POS与MIT；所有运动仍需要配置与运行时授权。
