# LCD-MIT 与 ADRC 单固件集成实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 生成一份保留 LCD-MIT 流畅度与 POS/MIT 操作、又能在独占控制权下执行选定轴 ADRC 的离线固件。

**Architecture:** 新目标沿用 LCD-MIT 的默认应用路径，增加独立 ADRC 编译开关与控制权状态机。USB/LCD 只提交请求，4 ms 控制任务核对失能反馈及发送队列后切换所有权；发送路径按所有者选择且 STOP 总能进入禁能路径。

**Tech Stack:** STM32H723、ARMCC 5.06、FreeRTOS、经典 CAN、LVGL 8.3.11、C11 主机测试、MATLAB R2026a 已验证生成控制器。

设计依据：`docs/superpowers/specs/2026-09-23-adrc-lcd-mit-integration-design.md`。工作树：`D:/download/TCG/Aethor_robo_fw/.worktrees/s3519-adrc-lcd-mit`。当前板上 LCD-MIT 不改写。

### Task 1: 锁定纯控制权状态机

**Files:** Create `App/Adrc/adrc_lcd_ownership.h`, `App/Adrc/adrc_lcd_ownership.c`; create `Tests/host/adrc_lcd_ownership_test_main.c`, `Tests/host/run_adrc_lcd_ownership_tests.ps1`.

- [ ] **Step 1: 写失败测试。** 覆盖初始 LCD、仅满足 `lcd_idle && queue_idle && feedback_fresh && disabled && no_fault && stationary && mit_discovered` 时取得 ADRC、STOP 进入 RELEASING、发送回执和较新失能反馈后归还 LCD；每一项缺失必须拒绝。
- [ ] **Step 2: 执行 `Tests/host/run_adrc_lcd_ownership_tests.ps1`，确认因缺失状态机而失败。**
- [ ] **Step 3: 实现显式 `LCD / ACQUIRING / ADRC / RELEASING` 状态和纯函数输入/输出；对旧时间戳、溢出、重复请求保持保守拒绝。** 所有新增 C/H 文件写文件级及函数级注释。
- [ ] **Step 4: 重跑状态机测试及 `Tests/host/run_adrc_app_tests.ps1`；提交状态机。**

### Task 2: 构建单固件目标

**Files:** Modify `App/Config/adrc_build_config.h`, `App/Adrc/adrc_app_bridge.c`, `App/Adrc/adrc_app_bridge.h`; create `MDK-ARM/LCD-MIT-ADRC.uvprojx`; modify `Tests/host/test_debug_ui_build_contract.py`.

- [ ] **Step 1: 添加构建门禁测试。** 新目标必须包含 LCD-MIT 原有 `AETHOR_DEBUG_UI_ALLOW_MOTION=1`、`AETHOR_DEBUG_UI_ALLOW_MIT=1`、七电机兼容掩码、`AETHOR_ADRC_LCD_INTEGRATED=1`、生成 C 和 ADRC 发送通道；原 LCD-MIT、ADRC-Bench 宏不变。
- [ ] **Step 2: 执行构建门禁测试，确认目标缺失导致失败。**
- [ ] **Step 3: 新建目标和开关；桥接层在集成目标初始化时不抢占 LCD 的发现流程，由所有权切换后显式开始目标轴发现。**
- [ ] **Step 4: 构建四个原目标及新目标；无错误、无警告后提交。**

### Task 3: 应用入口与协议互斥

**Files:** Modify `App/aethor_app.c`, `App/aethor_app.h`, `App/Adrc/adrc_protocol.c`, `App/Adrc/adrc_protocol.h`; modify `Tests/host/adrc_app_test_main.c`; create集成应用测试入口与运行脚本于 `Tests/host/`。

- [ ] **Step 1: 写失败测试。** 默认 LCD 原命令和本地 POS/MIT 入口正常；ADRC 状态/硬件诊断只读；`adrc acquire motor=7` 在 LCD 忙、旧反馈、非 MIT、未失能、队列未空时拒绝；取得后 LCD 运动请求拒绝；重复请求 ID 不重新切换。
- [ ] **Step 2: 运行测试确认失败原因是缺失集成路由。**
- [ ] **Step 3: 在 ProtocolTask 只入队 acquire/release；控制任务核对条件并切换 `aethor_app_service`、发现帧、结果输出与 LCD 入口。保留旧目标条件编译的原有行为。**
- [ ] **Step 4: 重跑集成、ADRC App、Motor、Phase 0 与 LCD 主机测试；提交。**

### Task 4: CAN 与 STOP 边界

**Files:** Modify `App/Platform/stm32_platform.c`, `App/Platform/stm32_platform.h`, `Core/Src/freertos.c`, `App/aethor_app.c`; modify `Tests/host/platform_io_test_main.c` 与集成应用测试。

- [ ] **Step 1: 写失败测试。** CAN 软件队列或硬件发送邮箱未空时不能转交；ADRC 持有期间旧控制组不会出队；LCD STOP 和 USB STOP 都令当前所有者请求禁能；发送失败不构成失能确认。
- [ ] **Step 2: 运行测试确认失败。**
- [ ] **Step 3: 增加只读 CAN idle 快照；4 ms 控制任务按 owner 调用 ADRC 发送回执或传统发送服务，STOP 优先于普通帧；释放只接受较新失能反馈。**
- [ ] **Step 4: 跑 CAN 通道、平台、主机回归及 ARMCC 构建；提交。**

### Task 5: 离线发布门禁

**Files:** Modify `docs/adrc/execution-status.md` 与 `output/adrc` 中本次运行记录；新增忽略的构建/测试结果，不改板上固件。

- [ ] **Step 1: 跑 `Tests/host/run_tests.ps1`、ADRC 全套、LCD 传输/UI 回归和模型生成 C 重放；失败即停止发布。**
- [ ] **Step 2: Keil 全量重建 `LCD-MIT-ADRC`，记录零错误零警告、Code/RO/RW/ZI 与 HEX SHA-256。**
- [ ] **Step 3: 复查所有权拒绝路径、源文件差异、已有目标行为和设计覆盖；把剩余实机资格明确标为未验证。**
- [ ] **Step 4: 提交文档与离线工件说明；保持 24 V 关闭且不烧录。**
