# Astra 参数弹窗对齐版固件

基于B1提交 `2bb39316abdb0900ba735358830a6fe36743016e`，仅修正10种参数弹窗的字体、居中布局、圆角和滑轨。原B1源码保留在 `archive/lvgl-astra-ui-b1`，原[固件归档](../2026-09-14-astra-ui-preview/README.md)不变，A版及文字版也保留。

**可用于现有 STM32H723 / 280×240 LCD 配置的烧录验证，尚未烧录。** 固件源码提交为 `5844fd7169d2053473df6c8117ff07afb878b8af`。该源码的28项完整检查和四目标Keil编译全部通过，零错误、零警告；验证期间源码摘要不变，发布前再次核对当前源码与三个HEX的SHA-256。具体源码、内存布局和检查结果见 [verification.json](verification.json)，变更与证据边界见 [弹窗对齐说明](../../docs/debug-ui/astra-popup-alignment.md)。

三个HEX均已生成，可直接选择相应文件烧录，无需重新编译：`LCD-ReadOnly` 仅显示；`LCD-POS` 开放电机7本地POS入口；`LCD-MIT` 同时包含电机7本地POS和MIT入口。沿用此前POS/MIT调试配置时选择 `LCD-MIT.hex`。现有运行时授权与运动门控仍然有效。

本版仅完成软件验证；最新界面的实屏效果、硬件时序和任务栈实际余量仍待上板确认。本次发布操作不执行烧录。

| 文件 | SHA-256 |
|---|---|
| [LCD-ReadOnly.hex](LCD-ReadOnly.hex) | `e4ecb7ec4a0fa8b863d21abf84b9681dffeb355d3979711cab4d5f2fcc0fbed2` |
| [LCD-POS.hex](LCD-POS.hex) | `546a50e67c8e902978150d0dc0ad6f2bfb2f15f920b9f082464b42b628a2550b` |
| [LCD-MIT.hex](LCD-MIT.hex) | `6d79a7837db5daa895f67ee8e9f9336fee1d78982877444e7397830fcb60f020` |

模拟页面在 `Tests/ui/artifacts-astra`：10张参数弹窗更新，其余79张与B1的像素一致。图片与主机检查不替代实屏或运动验收。
