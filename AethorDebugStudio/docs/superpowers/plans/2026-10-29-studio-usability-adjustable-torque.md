<!-- @file 已确认的上位机易用性与可调转矩策略实施计划；离线执行，不烧录。 -->
# 上位机易用性与可调转矩实施计划

> 使用subagent-driven-development拆分固件与界面工作，独立复审后集成。用户已确认设计，无需再次询问执行方式。

**Goal:** 重排完整调试流程、补齐新手手册，实现3.50～6.24 N·m受保护可调边界，完成离线编译测试。

**Architecture:** 现有React页面保留通信与事务锁定，分组呈现主流程与高级操作。网关白名单扩展可调配额请求；独立固件候选负责强制范围、冷却维护门禁和实际保护。旧协议保持既有5.46 N·m维护兼容，配置以板端读回为准。

**Tech Stack:** React/TypeScript/Vite、C# .NET8、C11/STM32H723/Keil、Node tests/Playwright/GCC离线测试。

## 任务1：冻结起点

- [x] 按02f1ee45发布清单逐文件校验复制到`D:/download/TCG/studio-usability-torque-candidate-20261029`，不复制旧output。
- [x] 保存主机基线清单列出的85文件快照（不是完整历史diff）；交付另冻结全部当前源码。当前仓库无HEAD，不整体提交此前暂存内容。

## 任务2：固件可调策略

文件：新候选`App/DebugUi/debug_ui_torque_envelope.h`、`debug_ui_torque_envelope_app.inc`、`debug_ui_remote_app.inc`、`App/aethor_app.c`与对应`Tests/host`。

- [x] 先写并运行失败回归：允许3500/5460/6240毫牛米，拒绝3499/6241、无冷却、失能缺失、忙状态、错误/重复令牌；保持6.24下累计约6.58秒退出与上电未授权。
- [x] 加入选定峰值；`ui torque`增加`policy_v=2 min_mnm=3500 max_mnm=6240 peak_mnm=<整数> blockers=<掩码>`；保持原有字段。`ui torque_reset token=<n> cold=1 peak_mnm=<n>`原子恢复，旧两字段格式只恢复既有5.46值。
- [x] 在门禁全部通过前不得修改策略或账本；令牌一次消费、操作不发CAN。首故障保存当时线`tq_limit`，历史解释不受后改影响。
- [x] 跑`Tests/host/run_torque_envelope_tests.ps1`及远程故障释放、app配置、协议回归；Keil构建留待集成冻结。

## 任务3：网关和上位机策略

文件：`Backend/Studio.Gateway/RemoteControlProtocol.cs`及Tests；`Frontend/src/torqueEnvelope.ts`、`components/TorqueEnvelope.tsx`及tests。

- [x] 新请求参数`PeakMilliNm`为可空整数，仅用于torque_reset；新版本带范围，旧版本不发送新字段。拒绝越界值和错误操作组合。
- [x] 解析版本与精确边界，不用缺失字段代填；草稿、读回和当前授权分开显示。3.50/5.46/6.24预设；恢复前新取令牌并冻结所选值，响应后只读核验，不重复写。
- [x] 状态变化撤销声明与旧显示；阻塞条件中文呈现，原始字段可展开；温度/时间/预算共同显示。
- [x] 测试先覆盖新版合法/非法策略、旧协议、未知版本、双击、断连、丢回执、恢复拒绝与读回不符，再运行`npm.cmd test`和新目录C#测试DLL。

## 任务4：页面与新手手册

文件：`App.tsx`、`catalog.ts`、`types.ts`、`components/RemoteControl.tsx`、`DeviceDebug.tsx`、相关CSS、独立手册数据/页面、`docs/user-manual.md`。

- [x] 主流程按设备检查、参数、保护、审核启动、结果/恢复排序；高级操作和协议详情折叠。设零与运动回零单独说明。
- [x] 新手手册覆盖全部已接入功能与未接入边界；首次路径、参数表、常见故障恢复和动作影响明确。文案与按钮名称一致。
- [x] 修正帮助和功能目录过期说明；首页有明确起步入口，侧栏分组保留全部原页。
- [x] 浏览器拦截全部API，覆盖真实模式/离线/回放、手册导航、参数到启动顺序、会话恢复、可调策略；320/768/1024/1440/1920截图及axe检查。

## 任务5：复审、构建和交付

- [x] 独立审查固件门禁、跨层协议和用户流程，逐条处理有效发现；沿用用户此前不追加外部模型的选择。
- [x] `npm.cmd test`、`npm.cmd run build`、C#构建/测试、相关固件GCC/Keil构建全部通过后冻结。
- [x] 发布清单列出源码/产品哈希、测试日志、手册、截图与逐项目标审计；`flashed=false hardware_qualified=false`。

## 已验证起点

- 前端88/88通过：`artifacts/usability-torque-baseline-frontend-20261029.log`。
- 网关38/38通过，构建0错误0警告：`artifacts/usability-torque-baseline-gateway-*.log`。
- 原策略GCC单元测试通过：`artifacts/usability-torque-baseline-policy-20261029.log`。
