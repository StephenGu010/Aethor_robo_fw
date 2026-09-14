# LVGL 调试界面接入记录

实现位于当前工作树；不包含硬件验收或电机运行证据。四个编译 target 的配置由公共 `App/Config/debug_ui_config.h` 和父任务维护，默认 UI/POS/MIT 均为 0，未标定 profile 不允许动作。

## 工程文件

- 上游编译清单：`Ui/lvgl_sources.txt`，70 个软件渲染及基础源文件；不编译 demos、examples、assets、GPU 驱动、未用 lru 和字体文件加载器。
- 工程自身源文件：`Ui/debug_ui_task.c`、`debug_ui_view.c`、`debug_ui_graphics_guard.c`、`lv_port_disp.c`、`lv_port_indev.c`、`fonts/ui_font_16.c`，以及 `App/DebugUi/debug_ui_model.c`。
- 包含目录：`Ui`、`Middlewares/Third_Party/LVGL`，定义 `LV_CONF_INCLUDE_SIMPLE`；C99。
- `debug_ui_model.c` 和 `debug_ui_view.c` 保留可读中文 UTF-8，并带 BOM，已用 ARMCC5 单文件编译验证。工程也可采用父任务验证的 `--no_multibyte_chars`。
- `StartDebugUiTask(void const *)`：Low 优先级，静态 2048-word 栈。Core 提供头文件声明的通知、单调微秒和控制时序 hook。

## 内存和调用归属

双 `280×24` RGB565 缓冲各 13,440 B，总计 26,880 B，32 B 对齐于 `.lcd_dma`；LVGL 64 KiB 静态池置于 `.lvgl_pool`，通过上游 `LV_MEM_POOL_ALLOC` 扩展点提供。scatter 地址和最终 map 由父任务核验。

颜色 `LV_COLOR_16_SWAP=1`，直接满足平台 MSB-first SPI 像素合同。所有 LVGL 操作只在 UiTask 的本地断言边界内；平台完成 hook 例外只调用 `lv_disp_flush_ready()`。`wait_cb` 每轮服务传输期限、ADC、快照、终态、STOP、50 ms 健康通知并 `osDelay(1)`，不递归 timer handler、不改控件树。显示平台确认释放所有权后才归还缓冲。正常主循环 `osDelay(5)`；LVGL 脏区计时器 50 ms，数值文本 100 ms。

控件、页面和固定字符串存储均预建。内存耗尽通过 `setjmp/longjmp` 退出当前 UiTask 图形调用并撤销健康，保留平台/输入/STOP 服务；不调用全局 fatal。无活动/待执行/待终态请求且平台已释放缓冲时，显示可尝试一次重初始化；恢复不申请本地授权。

## 输入和动作语义

每次 poll 后优先消费 `debug_ui_input_emergency_event()`，随后消费普通事件。运行态 M2/M3 的普通中键 PRESS 和独立 EMERGENCY_STOP 不依赖焦点或本地授权；EMERGENCY_STOP 空闲时忽略。M1 编译开关或只读快照禁止所有状态变更，包括 STOP，画面明确提示只读。

草稿、确认预览、已提交动作各有独立结构。中键进入确认页后必须释放，再按住确认；进度取 `min(event.held_ms, event.timestamp_ms-review_pressed_ms)`，使用输入层重新稳定后计算的连续保持时间。一次长按只提交一次。参考/授权变化、过期反馈、输入/显示失效会取消编辑。STOP 单独优先，未送出的动作会被取消，避免 STOP 后再发旧运动。已受理和最终完成分别处理，以完整来源/epoch/request_id 匹配结果；不会自动重发运动。

显示单位为输出轴 °、°/s、Nm，内部仍为 rad、rad/s、Nm。无反馈显示 `--`，过期反馈保留有限末值并标记“过期”。切模草稿及确认明确显示实际模式到 requested_mode；不使用位置预估代替切模内容。

`debug_ui_model_active_target()` 优先使用实际 active_motor_mask，然后活动快照 target，再使用待执行本地请求或已捕获的停止目标；不使用浏览焦点或历史提交补一个远端 STOP 目标。多机显示“ID3等2台”，位置行标明该 ID，避免把单个位置样本理解为全部电机。

`DEBUG_UI_REASON_STOP_LATCHED=15` 是同步保留的独立停止意图，不为此 ID 产生 admission/completion。模型立即清除该 STOP 的 `stop_waiting`，使用 `stop_latch_state` 等待实际快照；原已受理请求仍独立等待自己的终态。捕获提交时的目标 mask，并合并清理期间快照报告的 mask。仅当更晚且有效的快照中 active/pending/stop_pending 都清除，才结束锁存等待；无需必须观察中间 pending=1。所有目标的反馈必须有效、年龄小于 150 ms、推算采样时间晚于 STOP 提交且处于失能，才展示“反馈已确认失能”；否则显示故障和失能未确认，不伪造终态、不自动解锁。此处理与结果型 STOP 的完整身份匹配相互独立。

健康 `display_error` 是当前显示有效性的故障标志。平台 READY 且本地图形恢复有效后，此标志归零；累计 SPI、初始化和 DMA 错误继续保留在诊断中，不能用历史错误计数永久撤销健康。

锁存失能确认还要求每台电机的原始 `driver_state=0` 且 `fault_flags=0`。`enabled=0` 也可能由故障态映射而来，不能单独用作确认依据。持久模型回归覆盖故障态8、未知态2，以及状态0但故障标志非零；这些均显示未确认并保持锁定。

## 第三方来源

LVGL 官方 v8.3.11 标签指向提交 `74d0a816a440eea53e030c4f1af842a94f7ce3d3`。归档、标签对象和提交记录位于 `Middlewares/Third_Party/LVGL/UPSTREAM.json`，MIT 许可为 `LICENCE.txt`，逐源文件比对为 `SOURCE_MANIFEST.json`。源 ZIP 保留在 workspace output/deps，未删除。

中文采用 Noto CJK Sans `Sans2.004` 的可分发子集：`Ui/fonts/OFL.txt`、`SOURCE.json`（含原版权及字体 SHA）、`charset.txt` 和生成脚本 `Tests/ui/generate_font_subset.py`。没有使用系统商业字体。生成字体已改名为 `ui_font_16`。

## 可复现验证

Windows 仅用进程级执行策略：`powershell.exe -NoProfile -ExecutionPolicy Bypass -File <脚本绝对路径>`。

- `Tests/host/run_debug_ui_model_tests.ps1`：模型行为及编译期 M1 STOP/运动门禁。
- `Tests/ui/run_input_model_integration.ps1`：真实输入+模型，跨方向 STOP、溢出 STOP、间断长按、600/40/500 ms 连续保持反例、0..49 ms 延迟样本。
- `Tests/ui/run_review_regressions.ps1`：持久化独立审查案例，真实任务健康恢复、缓冲持有期间 STOP/健康服务、切模内容区别、活动 mask 优先、结果无回调的 STOP 锁存、原终态交错和新鲜失能验证。host shim 在 `Tests/ui/stubs/`，不进入固件工程。
- `Tests/ui/run_ui_render_smoke.ps1`：真实 LVGL 软件光栅和生产双缓冲移植层，所有实际文案字形与文字宽度检查，输出 `Tests/ui/artifacts/*.ppm` 与 `.png`；像素清单记录于 `render_manifest.json`。图中始终有“模拟”标识。

2026-09-09 新增 17/18 切模确认、19 远端 ID3、20 停止锁存、21/22 失能确认/未确认和 23 多机范围，共 23 张画面。仍是原七个生产 C 文件，不需要新增工程源条目；字体子集随新增文案重生，保留 OFL 许可和来源证据。

上述均为主机软件证据。实屏方向/偏移/字节序、五向键物理方向和 ADC 分布、SPI/ADC 时序、实际任务负载、台架增益及动作清理仍须由父任务按硬件关卡验收。
