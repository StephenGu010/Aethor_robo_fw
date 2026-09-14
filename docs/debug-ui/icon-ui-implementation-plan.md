# LVGL 图标界面实现计划

> **For agentic workers:** Use superpowers:subagent-driven-development for independent tasks, then review together. 禁止烧录；保留已发布基线。

**Goal:** 按已确认的42页设计，实现轻量图标UI、输出真实LVGL全页预览并验证内存及原控制合同。

**Architecture:** 视图共用一套静态对象/文本槽，模型追加图标菜单导航。所有运动请求仍走原模型、邮箱和应用所有者；不改控制算法、参数或硬件。复用LVGL原生符号和现有OFL中文字库。

**Tech Stack:** C99 / ARMCC5 / LVGL8.3.11 / GCC主机渲染 / Python-Pillow截图整理。

## 0 发布旧基线

- [x] 更新根README、LCD说明、操作文档，保存HEX和可移植验证摘要。
- [x] 核对91个构建输入与27项通过归档完全一致；补测旧UI池使用41256 B。
- [x] 生成发布提交f33fec1，包含本地未发布的固件、LVGL依赖、测试及文档。
- [x] 推送并确认远端main等于发布提交；此项完成后才开始实现。

## 1 图标菜单模型与行为测试

文件：`App/DebugUi/debug_ui_model.h/.c`、`Tests/host/debug_ui_model_test_main.c`、`Tests/ui/debug_ui_readonly_test.c`。

- [x] 先增加详情中/右只进菜单、不提交请求的测试；八个动作按上下线性循环、四项为一屏、左回详情；只读拒绝运动；活动STOP仍抢占。
- [x] 运行`Tests/host/run_debug_ui_model_tests.ps1`，旧模型须在新增导航断言失败。
- [x] 在枚举末尾追加`DEBUG_UI_PAGE_ACTIONS`，保留既有状态编号；详情只切菜单，原八动作分派搬到菜单分支。

关键合同断言示意（使用测试文件已有事件辅助函数和合成模型）：

```c
assert(model.page == DEBUG_UI_PAGE_ACTIONS);
assert(!debug_ui_model_take_request(&model, &request));
/* Eight actions retain their existing operations and mode gates. */
```

- [x] 原草稿/确认返回路径以操作菜单为动作选择层，结果已读后返回电机详情；全部原STOP、只读、控制权和按键测试通过。

## 2 共享视图与图标版式

文件：`Ui/debug_ui_view.h/.c`、`Tests/ui/ui_render_smoke.c`、`Tests/ui/task_display_regression.c`。

- [x] 先给真实渲染测试加共享视图预算断言：`assert(sizeof(DebugUiView) < 4096U);`，旧版须失败。
- [x] 单个共享`DebugUiViewPage`替代11页常驻。保留语义行文本便于原断言检查，但每次只绘制当前页；新增少量复用图标标签和确认进度对象。
- [x] 主页三图标条、详情大数值、动作2×2图标页、单字段大数值编辑、简洁确认、独立结果/失能状态、只读与诊断模板。
- [x] 图标使用`LV_SYMBOL_*`及已编译Montserrat字体，中文沿用裁剪字体。显示未知/过期和实际活动目标，不用焦点冒充STOP目标。
- [x] 新旧语义回归均检查当前共享文本槽；对可见对象坐标/字形做真实渲染检查。

## 3 内存、字库与构建布局

文件：`Ui/lv_conf.h`、`MDK-ARM/aethor_memory.sct`、`Tests/host/check_debug_ui_map.py`、`Ui/fonts/*`。

- [x] 先在64KiB池测共享视图使用量，再尝试32KiB；数组用`LV_MEM_SIZE`统一大小。
- [x] 保留26880 B双DMA缓冲，scatter池区缩至0x8000，后续AXI区边界同步；map检查应验证32768 B。
- [x] 新字符用既有`Tests/ui/generate_font_subset.py`与Noto CJK来源字体生成，不带入Windows系统字体。检查新增符号的Montserrat字形。
- [x] 长时间切页检查对象数和池使用不增长，覆盖OOM局部退出与DMA等待。

## 4 全页输出、回归与交付

文件：`Tests/ui/ui_render_smoke.c`、`Tests/ui/verify_render_artifacts.py`、`docs/debug-ui/icon-ui-verification.md`。

- [x] 真实渲染所有操作页面及设计的42个关键场景；共享模板的模式/字段变体也输出，不能只保留旧25页。
- [x] `powershell -NoProfile -ExecutionPolicy Bypass -File tools/verify_debug_ui.ps1 -IncludeKeil`：全部检查和四目标通过。
- [x] 对照原图标设计逐页审查，修复缺字、拥挤、截断、错误操作提示；报告真实map RAM/Flash与主机池峰值的不同含义。
- [x] 更新当前操作文档、实际图片画廊与验证结果，保留发布基线HEX；本次不烧录。

## 审查要求

导航实现与UI布局可由不同agent分别负责，禁止同时编辑同一个文件。实现后由独立审查核对设计符合性与控制合同，再由主任务运行整体验证。逐文件更改，不删除文件或目录，不用测试通过代替完整页面和内存验收。
