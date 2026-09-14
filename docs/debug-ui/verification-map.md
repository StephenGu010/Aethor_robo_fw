# 实施计划验证索引

本索引对应原计划8.1的行为编号；复核日期2026-09-11。最新完整运行证据是`Tests/host/build/verification/summary.json`（03:31:47 UTC完成），27项退出码均0，验证前后源码哈希一致；再次核对91个一方源码文件，变化数为0。当前归档为`D:\download\TCG\output\register-position-20260911\verification-cpu-stack-7790eb2c.zip`，含CPU任务栈诊断增量。此前`verification-final-4523074a.zip`保留为前一版本证据。下面的“软件通过”只指测试输入和断言覆盖，不代表实物按键、RTOS延迟或电机验收。

| 编号 | 当前可核对的实现测试 | 证据边界 |
|---|---|---|
| K01 | `debug_ui_input_test_main.c`的nominal/enter boundaries、exit hysteresis、median spikes/bounce测试 | 合成ADC输入的软件识别通过；实物六状态分布未测 |
| K02 | input的startup_release、power_on_center；model的review_once_and_identity | 上电按住及未松开长按不提交的软件门禁通过 |
| K03 | input的center_hold_is_coalesced；model在799/800ms及持续长按后检查唯一请求 | 一次长按只提交一次的软件行为通过 |
| K04 | input的UNKNOWN、ADC stale、跨键、overflow、wraparound测试 | 失效清理及计时边界软件通过；物理延迟未测 |
| U01 | App的init_terminal_verification_snapshot、snapshot_actual_target；25页真实LVGL模拟渲染 | 未知/过期/错误文本与目标身份软件通过；实屏专项边界仍待验收 |
| U02 | App的relative_start_and_queued_expiry；model的gates_and_cancellation | 新反馈起点、参考代次取消旧草稿软件通过 |
| A01 | App的engine_identity、usb_local_arbitration；model的completion来源检查 | 同号不同来源不串受理/终态的软件行为通过 |
| A02 | App的usb_local_arbitration、local_stop_remote_and_full_queue | 顺序模拟交错提交时BUSY与单一动作所有者通过；不是同时执行两任务的物理测量 |
| A03 | App的usb_session_disconnect_and_result_retention | hello/断开及未消费结果的软件合同通过 |
| A04 | App的default_profile、local_rejections和五配置构建；非法宏组合检查 | 只读、生产配置、未标定及未授权拒绝通过 |
| A05 | App的stop_backpressure、stop_all_result_queues_full；mailbox独立STOP测试 | 邮箱/结果满时停止意图保留及清理软件通过 |
| A06 | App的local_rejections、relative_start_and_queued_expiry、results_and_idle_authority | 250ms请求期限、旧代次、30s空闲授权过期软件通过 |
| A07 | App的usb_local_arbitration、stop_coalesced_scope、stop_preserves_interrupted_scope；Phase0停止与延迟结果回归 | 来源终态、重复STOP、清理范围的软件断言通过 |
| A08 | App的local_rejections、local_disable_actual_mode_and_fresh_ack | 忙态拒绝直接DISABLE、实际模式编码及新失能反馈要求通过 |
| A09 | App的independent_health_cleanup | UI与Protocol两路分别失活，在八个动作阶段进入清理的软件测试通过；实际150ms调度预算待实机 |
| A10 | App的missing_disable_feedback_and_reacquire、fault_target_lock；任务停止锁存回归 | 缺失失能反馈不冒充成功，故障和原目标锁定保留的软件测试通过 |
| D01 | `lcd_transport_test_main.c`的eot_once、timeout_and_late_irq、abort/start_error；fake HAL和UiTask等待DMA回归 | 重复/晚到/超时与所有权软件通过；30min物理图形压力待补 |
| D02 | transport的bounds测试；渲染双缓冲与map检查 | 区域、长度、对齐和静态布局通过；物理SPI波形未测 |
| M01 | App的mit_limits、local_rejections及model门禁；Phase0/Motor参数范围测试 | 非有限值、零增益、未标定、越界和非零前馈拒绝通过 |
| M02 | Phase0的public_mit_mode_and_hold_lifecycle、public_mit_stop_uses_mode_correct_hold、public_mit_move_uses_local_trajectory；共享动作阶段清理测试 | MIT轨迹/保持/停止及共用清理有软件证据；不能据此宣称MIT每阶段真实电机验收 |
| R01 | 四目标Keil、App五配置、非法宏组合、架构与map检查 | UI关闭资源隔离、只读拒绝、生产本地运动关闭软件通过 |

上述C测试文件均位于`Tests/host/`；图形和任务级补充位于`Tests/ui/`。完整运行入口是`tools/verify_debug_ui.ps1 -IncludeKeil`，不再使用原方案中尚未创建的`run_debug_ui_tests.ps1`名称。

## 新增寄存器读取的专项证据

- Motor Core先复现、再修正普通反馈与0x33/0x50字节碰撞误拦截；覆盖未开始查询及查询进行中的普通反馈。
- 错误Master/ESC/RID、操作码、长度、非有限float、超时、晚回包和忙时取消均有断言。
- App五配置检查只读台架启用查询，其他配置不自动读；寄存器样本不更新标准反馈和标定标志。
- 只读导航及30分钟逻辑时间回归明确访问新增页，本地控制请求保持0。
- 25页渲染包含小原始值0.000219124和500ms以上的过期值；字形和页面尺寸检查通过。

## 尚不能由软件证据关闭的验收项

新增CPU栈诊断覆盖启动未知、七任务容量、零剩余、无效容量、UI关闭、快照临界区及256字节响应边界。7790eb2c实机30分钟18000条查询全回复，CPU报告七任务最小余量均超过25%；该工况不覆盖物理页面操作和运动。Flash回读、短测、长测及无UI基线分别记录在验收台账中，不能把旧版负载与新版资源数字合并成一次试验。

仪器缺失已由用户确认；接线逐触点测量、SPI波形和物理STOP/显示延迟仍未验收。实物各键100次、十次真实冷启动、M2二十组动作以及M3标定和运动也不能由上述主机测试代替。当前实时台架结果、工件哈希和剩余条件以[验收台账](acceptance-record.md)为准。
