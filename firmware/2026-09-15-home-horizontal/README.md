# 系统总览横向导航固件（2026-09-15）

系统总览左右循环选择，中键或下键进入。其他页面、物理方向映射、ID1～7配置和电机控制逻辑保持不变，见 [操作说明](../../docs/debug-ui/home-horizontal-20260915.md)。

三个固定HEX分别为 `LCD-ReadOnly.hex`、`LCD-POS.hex`、`LCD-MIT.hex`。当前使用POS和MIT调试时对应 `LCD-MIT.hex`。上一版源码 `a65d3435f46c76ba5604d237f4a98886a306c388` 保留于 `archive/lvgl-seven-motor-preset`，旧工件保留在 [七电机预置目录](../2026-09-15-seven-motor-preset/README.md)。

## 软件验证

- 28项检查最终通过，包括四个Keil目标，均为0错误、0警告。
- 首轮UI评审回归的方向映射测试使用旧首页操作序列而失败；更新测试后完整复跑通过。`verification.json` 的 `resolved_regression` 保留首轮退出码与复跑哈希，复跑原始日志见 `review-rerun.txt`。
- 89张真实LVGL模拟页面全部通过，只有六张首页状态图改变；首页示例见 [预览](../../Tests/ui/artifacts-astra/01_overview.png)。
- LCD目标静态RAM均为174464字节，较上一版不增加；MIT的Code+RO为297232字节，增加48字节。LVGL采样池占用仍为10160字节。
- 构建期间生产源码哈希未变化，归档时再次校验源码和HEX哈希。完整记录见 [verification.json](verification.json)。

## 烧录状态

本版未烧录。2026-09-15烧录前检查中，ID7显示 `state=absent`、无有效反馈，控制器报告 `fault=transport`；STOP未确认，故停止烧录。待连接和失能状态恢复后，可使用本目录固定HEX继续校验烧录。本版没有执行电机运动，也未完成实物按键验收。
