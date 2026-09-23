# ADRC 实施状态与续接入口

更新日期：2026-09-23。已完成真实 ADRC 模型、生成 C、独立实验目标及 LCD-MIT/ADRC 单固件的离线集成；集成目标尚未烧录或进行电机验收。下文保留 9 月 21 日独立目标的证据，新集成目标状态见下一节。

## LCD-MIT/ADRC 单固件离线集成（2026-09-23）

- 独立工作树：`D:/download/TCG/Aethor_robo_fw/.worktrees/s3519-adrc-lcd-mit`，分支 `feature/s3519-adrc-lcd-mit`。先合入已验证的 LCD 流畅度版本，再建立单独的 `MDK-ARM/LCD-MIT-ADRC.uvprojx`；用户板上的 LCD-MIT 固件保持原样。
- 上电默认 LCD-MIT。USB 的 `adrc status`、`adrc hardware`、`adrc limits`、`adrc feedback` 可只读；当前目标只接受 `adrc acquire motor=7`。取得所有权前，控制任务检查 LCD 运动/结果队列和发现序列为空、传统 CAN 软件队列与 FDCAN FIFO 已排空并经过 4 ms 静默、收到静默起点之后的新鲜禁能零故障低速反馈，且实际发现的电机 7 身份、MIT 模式和全部参数一致。
- ADRC 持有时，LCD POS/MIT 请求和旧 CAN 发送入口被拒绝，LCD 与 USB STOP 都作用于 ADRC 单轴。`adrc release` 强制重新发送 DISABLE；只有匹配的真实 CAN 发送回执及更新的禁能反馈都到达后才交还 LCD。失败或超时保持锁定，`adrc status` 的 `owner`/`handoff` 字段可查询结果。
- `Tests/host/run_adrc_lcd_integration_tests.ps1` 与所有权状态机测试通过；覆盖默认 LCD、错误轴、未取得控制权拒绝运行、旧 LCD 控制组/新 CAN 提交阻止接管、LCD/USB STOP、发送失败和旧反馈不能交还、已失能监督器仍需新 DISABLE。原 `run_tests.ps1`、ADRC App/bench/协议/通道/监督器、LCD UI 与传输回归通过；`test_debug_ui_build_contract.py` 的 24 组门禁与 13 组 ADRC 发布门禁通过。
- 最新集成目标 ARMCC 构建 0 错误、0 警告，Code=216216、RO=100956、RW=848、ZI=233496 字节；HEX SHA-256 为 `894F68686145F747E4BB97104631FADA8D73FB9004ADF78C3A3043FA5B22351D`。构建日志在 `MDK-ARM/LCD-MIT-ADRC/LCD-MIT-ADRC.build_log.htm`，HEX 位于同目录。模型及生成代码仍是此前验证的三个 SLX 和 ERT 快照，本轮未改模型。
- 集成版 MATLAB 客户端新增显式 `acquireMotor(7)`/`releaseMotor()`，仅在收到匹配 ACK 且轮询到 `owner`/`handoff` 的目标状态后返回。用户开启的 R2026a 桌面 Automation Server 在沙箱外可连接；纯内存传输测试通过，输出 `ADRC_CLIENT_TESTS_PASSED checks=45 hardwareOpened=0`，COM 客户端退出码为 0，测试目录为 `output/adrc/client/integrated-com`。独立 `-batch` 也完成了这 45 项，但 MATLAB 在输出后退出时发生 access violation，故该批处理不作为正常退出证据。未连接串口或硬件。
- 当前硬件资格仍为空，USB 不能自行授权 ADRC 运动；集成版尚无由实测辨识结果驱动的本地资格注入路径，因此这份 HEX 只代表已链接的离线集成版，不能作为单电机 ADRC 运行验收。先做只读接管/释放和反馈时序核对，再完成位置、速度、转矩映射与 b0 实测、模型重验和资格配置，最后才分阶段开放有限期运动。本轮未连接串口、未烧录、未给电机通电。

## 文件与版本

- 独立 ADRC-Bench 历史工作树：`D:/download/TCG/Aethor_robo_fw/.worktrees/s3519-adrc`，分支 `feature/s3519-adrc`。
- 原基线：`a19fcfe`。原 `flatten-project-root` 工作树保持干净；备份目录 `D:/download/TCG/output/adrc_baseline_20260921` 的 11 项 SHA-256 已重新匹配。
- `App/Adrc/`：监督器、协议、单轴试验管理、应用连接层与生成接口适配器。
- `App/Platform/stm32_adrc_channel.*`：独立 CAN buffer 0 与真实发送回执。
- `Models/Adrc/`：controller / plant / validation 的重建源、仿真与生成代码重放流水线。
- `output/adrc/`：每次构建、仿真和验证的独立目录。失败产物保留，不作为通过证据。

## 独立 ADRC-Bench 行为

默认 `AETHOR_ADRC_BENCH=0`。实验目标开启后，只接受选定轴的 ADRC 流程与只读命令；LCD 保留 STOP。默认硬件资格为空，USB 命令无法自称取得资格。真实生成算法缺失时不发布完整实验目标；当前已由通过哈希复核的真实生成算法发布独立实验目标。

监督器区分准备、运行、停止中、已失能和故障；反馈超时、异常周期、发送失败、超速/越界/温度与长时间无法确认失能均有处理。运行命令使用单调 ID 和保留的异步结果，防止重放重新启动。改变轴或坐标映射会撤销对应资格。

辨识采用 20/40/80 ms 有限期限，到期或提前 STOP 直接请求失能，禁止转矩尾坡。总线发送成功只证明 CAN 帧已发送，不证明电机已执行；较新的驱动反馈才用于确认失能。迟到发送不冒充按期成功，取消请求未完成时不复用缓冲区。

生产 MIT 编码后重新核对名义转矩幅值和相邻真实已发送值的变化量；编码网格过粗、无合法码点、资格量程超过发现量程时拒绝运行。

## 验证证据

整套离线验证入口为 `tools/verify_adrc_offline.ps1 -IncludeLegacy`。它保存各项退出码、日志及运行前后的源码哈希；同时明确记录 `generated_controller_verified=false`、`adrc_firmware_linked=false`、`hardware_tested=false`，避免把主机测试等同于实机验收。

最新运行 `output/adrc/verification/20260921T081446669Z` 的 `manifest.json` 确认 `offline_subset_passed=true`、`sources_unchanged=true`。监督器 276 检查、bench 239 检查、协议/应用/CAN 通道、13 个发布证据门测试、6 个字体行尾兼容测试及 ARMCC 严格对象编译通过；旧功能回归 28 项通过，包含四个 Keil 目标与内存映射检查。上述两个生成/链接布尔字段只表示该验证脚本排除这些范围，其真实通过证据在下一节单独记录。

此前 `20260921T073624794Z` 因电脑重启未完成，仍保留为历史记录。Motor 分层和字体 CRLF 校验问题已修正；字体源和生成字体文件均未改写。基线目标测试产生的 HEX 已另存到最新验证目录，原受版本管理的 HEX 恢复为 HEAD 内容。原基线工作树干净，备份 11 项哈希再次全部匹配。

应用主机测试明确使用控制器测试桩，验证监督、协议、编码与停止流程，不证明 PI/LADRC 生成算法的闭环数值结果。真实 HAL 和 ARMCC 对象编译用于发现主机编译器未暴露的兼容问题；它也不能替代完整实验固件链接。

## 模型与固件实际验收

用户启用 MATLAB Automation Server 后，成功复用正常 R2026a 桌面会话。最新模型运行 `output/adrc/models/com_20260921T081022747Z` 的三个模型、ERT 生成、2,002 样本控制器 MIL、38 个合成闭环场景、59,040 样本真实生成 C 重放全部通过，模型与源码哈希一致，会话环境恢复成功。独立 MATLAB 进程的间歇性原生崩溃尚未确认根治。

交付入口：[三个实际 SLX 模型](../../Models/Adrc/VerifiedOffline/README.md)。这是离线合成对象，不是 S3519 实测辨识结果。

`MDK-ARM/ADRC-Bench.uvprojx` 已使用最新生成快照完成 ARMCC 链接，0 错误、0 警告。证据：`output/adrc/verification/final_target_20260921T081022747Z/build.log` 与 `target_report.json`。Code=180760、RO=100276、RW=816、ZI=233336 字节；链接通过不证明运行时栈余量或 4 ms 任务时序。

## 剩余阶段

MATLAB 客户端已完成，入口见 `Models/AdrcClient/README.md`。9 月 21 日初版在 R2026a 会话执行纯内存测试 35 项通过，证据：`output/adrc/client/20260921T082813219Z/client_report.json`（`hardwareOpened=false`）；集成版新增所有权测试的 45 项结果见本文件开头。测试覆盖显式调用、心跳、超时、STOP/失能区分、断线、拒绝、ID 耗尽、冻结 trace 和 uint64 时间戳。串口适配仅实现及静态检查，尚无真实 USB 验收。首轮测试固定行号断言错误已修正，失败日志保留。

离线软件阶段已完成。硬件阶段仍需按顺序完成：

1. 单电机供电但保持失能，读取身份、模式、量程与驱动看门狗；测量反馈时序与任务运行时间。
2. 分别核对位置、速度、转矩映射与量化，不把历史位置比例自动套用到速度或转矩。
3. 经有限脉冲辨识 b0，重复及保留数据验证后，更新 plant 与控制参数并重新执行模型验收。
4. 进行有限期 PI/LADRC 对照，保存速度、已发送名义转矩、ESO 状态、扰动恢复与最终失能证据。

现在不需要连接硬件或给电机供电。接线和分阶段供电、辨识与调参细节见 [调试与调参步骤](commissioning-and-tuning.md)。未经实测的参数和仿真结果均不得用于解锁电机运行。
