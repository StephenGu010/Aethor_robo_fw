# UI 软件验证记录 — 2026-09-09

工作树：`D:\download\TCG\Aethor_robo_fw\.worktrees\flatten-project-root`。本记录不表示连接过实屏、主控或电机。

## 主机行为

`Tests/host/run_debug_ui_model_tests.ps1`：`DEBUG_UI_MODEL_TESTS_OK`、`DEBUG_UI_COMPILETIME_READONLY_OK`。

覆盖七台电机五行滚动、未知/过期/NaN 数值、内部 rad 与显示度换算、草稿与冻结请求分离、先释放再连续 800 ms 确认、长按不重复、来源完整身份匹配、门禁/参考变化取消编辑、远端运行结束退出、再次远端动作 STOP、M1 不提交任何 STOP、M2/M3 无本地授权仍可 STOP、清理后显式重新申请本地控制。

测试先于模型实现落地；后续远端运行退出、M1 STOP 门禁和 LOCAL_FAULT 重新申请回归均记录过对应断言失败，再实现修正。

`Tests/ui/run_input_model_integration.ps1` 使用真实 `debug_ui_input.c` 与 `debug_ui_model.c`，输出：

```text
MODEL cross_key_STOP request=1 input_valid=1
MODEL queue_full_remote_STOP request=1 input_valid=0 fault=4
MODEL fragmented_hold_submits request=0 operation=0 elapsed=0 input_valid=1
CONTINUOUS_800MS_CONFIRMATION_OK
INPUT_MODEL_INTEGRATION_OK
```

600 ms CENTER → 40 ms UNKNOWN → 500 ms CENTER 不提交，恢复后连续稳定保持达 800 ms 才提交。0/30/39/40/44/45/49 ms 固定延迟样本均为 armed=valid=1、fault=0；这些是合成 ADC 测试，不是实际采样时延测量。

## 真实 LVGL 渲染

`Tests/ui/run_ui_render_smoke.ps1` 链接固定 LVGL v8.3.11 及生产 `lv_port_disp.c`，假 SPI 回调在复制完像素后才释放双缓冲。生成 23 张 280×240 PPM/PNG，页面都带“模拟”标识。包含总览、滚动列表、详情、过期反馈、三类诊断、POS/MIT 草稿、两种二次确认、运行、STOP 清理、完成、故障、只读运行，以及新增切模确认、远端 ID3、锁存停止及失能确认/未确认、多机范围。

`ALL_UI_GLYPHS_OK` 检查字库清单中的每个字符都能取得实际 glyph 描述及位图；生成器/验证器同时扫描所有实际 UI/模型字符串，保证新增文案没有漏出清单。每个渲染标题、行和页脚检查文字宽度不超过可用宽度；PNG 还检查尺寸、非空白和像素哈希互不重复。

当前子集 340 glyph，位图 36,189 B。原字体为 Noto CJK Sans `Sans2.004`，版权 `© 2014-2021 Adobe`，SIL OFL 1.1，许可/来源/哈希均随 `Ui/fonts/` 保存。

23 页测试记录 187 次 flush、164 次 wait；LVGL 64 KiB 池已用 34,488 B、余 31,048 B。这是 64 位主机上的软件统计，不能充当 ARM 板上 RAM/栈余量。

真实内存池耗尽测试发现上游部分对象数组 realloc 失败无检查；配置控制的 `LV_MEM_FAILURE_HANDLER` 在分配失败处退出 UiTask 图形边界。修复后输出 `LVGL_OOM_LOCAL_ESCAPE_OK`，无全局 fatal、无提前释放显示缓冲。上游差异逐项记录于 `Middlewares/Third_Party/LVGL/PATCHES.md`。

## 独立审查回归 — 2026-09-09

先补 `Tests/ui/model_review_regression.c`、`task_display_regression.c` 和 `run_review_regressions.ps1`，再修生产代码。前四条均实际产生断言失败：

| 用例 | 修复前 RED | 当前 GREEN |
|---|---|---|
| READY 恢复且历史 SPI 错误为 1 | display_error=1，App_healthy=0 | display_error=0，App_healthy=1；诊断 spi_errors=1 |
| SET_MODE POS/MIT | 七行内容完全相同，非相同断言失败 | 明确实际模式到请求模式；无无关预计位置 |
| active_mask=4，target=0/旧 lease=1 | STOP target 为 ID1，期望 ID3 断言失败 | 两种快照均 STOP ID3 |
| STOP_LATCHED 同步返回 | 普通失败页，锁存等待断言失败 | 等待反馈；不等待不存在的终态；新鲜逐机失能确认或未确认故障 |
| 多机 mask=0x14 | 缺少目标数量/位置 ID，日志 `build/review/multi_target_red.log` | “ID3等2台”，位置明确 ID3；23 号截图断言通过 |

锁存回归还覆盖：旧快照不能结束等待、全部目标均须有效反馈、已失能但采样早于 STOP 仍不确认、原请求终态先到不能结束锁存、未观察中间 stop_pending=1 也能依据更晚的一致快照结束、无自动解锁及无重复提交。实际 UiTask 的 submit→模型→snapshot 路径也运行该无异步结果案例。

任务与显示回归沿用独立 reviewer 的真实 LVGL/生产 UiTask 用例，持久化 host shim 位于 `Tests/ui/stubs/`。输出保留于 `Tests/ui/build/review/green.log`：

```text
ACTIVE_MASK_TARGET_PASS remote_ID3_selected_ID1_old_submission_ID1
STOP_LATCH_SNAPSHOT_PASS no_terminal_wait_multi_motor_fresh_disable_or_fault
STOP_LATCH_INTERLEAVING_PASS original_terminal_preserved_no_auto_unlock
TASK_DMA_WAIT_STOP_PASS requests=1 while_owned=1 buffers=2 immutable_checks=609 health=11
SET_MODE_CONFIRMATION_DIFFERENT_PASS
RECOVERED_DISPLAY_HEALTH ready=1 input=1 ui=1 display_error=0 App_healthy=1 protocol_valid=1
TASK_STOP_LATCHED_PASS dispatch_snapshot_no_async_result_no_duplicate
TASK_ASSERT_LIVE_DMA_PASS local_escape=1 STOP_before_release=1 health_withdrawn=1 cleanup_completed=1
```

新增图片是 `artifacts/17_review_set_pos.png` 到 `23_remote_multiple_targets.png`。所有 23 张真实渲染均检查字形和宽度；340 字符清单实际 glyph 描述与位图均存在，源文案完整清单再次校验通过。已查看切模、已确认失能及多机画面，未发现方框或裁切。

## ARMCC 与已生成 Keil 证据

最终独立复验另发现：`enabled=0` 不等于驱动态明确失能，故障态8也会映射为未使能。已在 `update_stop_latch()` 同时检查 `driver_state=0` 和 `fault_flags=0`；持久用例扩至7种反馈结果，先记录失败断言，再修复通过。独立原探针的5种结果重新编译执行，`failures=0`，编译和测试退出码均为0，8个编译输入的哈希前后一致。最终独立结论在 `output/ui-review-20260908/FINAL_REVIEW.md`，前轮失败证据保留。

2026-09-09 首轮UI审查修正后，对 `debug_ui_task.c`、`debug_ui_view.c`、`debug_ui_model.c`、`ui_font_16.c` 分别采用LCD-ReadOnly/POS/MIT定义和包含路径执行ARMCC5单文件编译，共12次，全部退出0、无诊断。使用C99、Cortex-M7.fp.dp和 `--no_multibyte_chars`，日志为 `Tests/ui/build/review/armcc.log`。该记录早于最后的失能判定修复，最终版本以随后完成的整机矩阵为准。

本任务逐文件 ARMCC5 编译全部七个自有 UI/模型 C 文件，以及七个上游补丁文件，无错误/警告；保留 C99、中文 UTF-8 和公开平台接口。父任务负责工程文件及最终四配置矩阵。

2026-09-09 09:57（UTC+8）主任务最终完整矩阵已结束：27组检查全部退出0，包括23页渲染、真实任务回归及四个Keil目标。最后的失能判定修复包含在该轮重建中，90个自有源码/工程输入哈希前后一致。`LCD-ReadOnly`最终为Code=178720、RO-data=61592、RW-data=768、ZI-data=214448，0 errors、0 warnings；详细四目标结果见[实施记录](../../docs/debug-ui/implementation-record.md)。

`MDK-ARM/LCD-ReadOnly/LCD-ReadOnly.map` 确认：

| 符号/段 | 地址 | 大小 | 属性 |
|---|---|---|---|
| draw_buffer_first | 0x24000000 | 13440 B | `.lcd_dma`，Zero RW |
| draw_buffer_second | 0x24003480 | 13440 B | `.lcd_dma`，Zero RW |
| debug_ui_lvgl_pool | 0x24008000 | 65536 B | `.lvgl_pool`，Zero RW |

两个绘图缓冲互不重叠，均 32 B 对齐，合计 26880 B，落在预留 AXI 32 KiB 内。ARMCC 对象的段表也确认 `SHT_NOBITS`、alignment=32，证据保存为 `Tests/ui/build/render/armcc_section_evidence.json`。

## 边界

没有接线、点屏、ADC 实测、SPI 实测、板上控制任务时序、负载、实际栈高水位或电机动作验证。默认 profile 仍由公共配置保持 invalid。编译成功和模拟画面不开放硬件动作资格。
