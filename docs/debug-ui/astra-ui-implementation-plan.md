# Astra 风格固件实施计划

> For agentic workers: Use superpowers:subagent-driven-development. 用户已确认设计并授权实施；持续完成，无烧录。

**Goal:** 保留方案A，完整实施已确认的方案B菜单、弹窗与共享视图，输出真实LVGL全页并验证固件。

**Architecture:** 原C/LVGL与单一控制所有者；菜单树/焦点/弹窗属于无分配模型；视图消费快照，共享对象。数值字库裁剪，DMA和运动参数不变。

**Tech Stack:** LVGL8.3.11、C99/ARMCC5、GCC主机回归、Pillow字库/画廊。

## 1 保留与分支
- [x] 核对工作区干净，方案A分支feature/lvgl-icon-ui保持2d801f9。
- [x] 创建feature/lvgl-astra-ui，保存已确认设计。
- [x] 完成交付前再次核对A版三个HEX、原预览和分支提交不变。

## 2 模型与行为（独立子任务）
文件：App/DebugUi/debug_ui_model.h/.c、Tests/host/debug_ui_model_test_main.c、debug_ui_soak_test_main.c、Tests/ui/debug_ui_readonly_test.c、必要的输入集成用例。
- [x] 先测试菜单树、父页焦点恢复、字段弹窗保存/取消、只读/STOP/800ms不回归。
- [x] 实施六项电机操作菜单、参数列表、模式/恢复/诊断分类、共享数值和消息状态。
- [x] 仅确认完成提交；MIT保持不编辑角度/速度；目标、epoch、输入变化取消弹窗/确认。
- [x] 不改执行层参数/控制算法，相关用例先红后绿，增加导航状态<512B断言。

## 3 数值字库（独立子任务）
文件：Ui/fonts/ui_font_numeric_28.c、来源清单/生成脚本、Ui/lvgl_sources.txt、MDK工程字体组、lv_conf.h。
- [x] 从原版Montserrat28提取所需数字/单位/符号，保留原文件和许可证，不删文件。
- [x] 不编译完整28号字库，校验全部使用字形与摘要，保持16/20号图标字体。

## 4 共享视图与动画（主任务）
文件：Ui/debug_ui_view.h/.c、Ui/debug_ui_task.c、中文子集及Tests/ui/ui_render_smoke.c、task_display_regression.c、astra场景。
- [x] 黑白Tile四角焦点、五行List反白、模式状态、共享编辑/确认/结果弹窗、清晰运行页。
- [x] 页面切换不创建销毁对象；Tile 140ms 动画仅闲置运行，List 即时反白/滚动，STOP及显示故障不等待动画。
- [x] 只读叶子无可执行焦点，返回路径和实际活动目标正确，未知/过期不冒充当前值。
- [x] 真实渲染覆盖55个设计状态和必要变体；文字宽高、截图差异、对象数与池占用、OOM/DMA回归。

## 5 审查、构建、文档与交付
- [x] 独立规范符合性及代码审查，修复所有实质缺陷。
- [x] Windows PowerShell运行 tools/verify_debug_ui.ps1 -IncludeKeil，完整检查/四目标/map通过，验证期间源码摘要不变。
- [x] 保存独立Astra预览HEX与可移植验证JSON、真实页面画廊；更新README和操作指南。
- [x] 报告RAM/Flash、动画/硬件证据边界，提交独立分支；不烧录，不覆盖A版。
