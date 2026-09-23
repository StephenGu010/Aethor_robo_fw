# ADRC 实施状态与续接入口

更新日期：2026-09-23。已完成真实 ADRC 模型、生成 C、独立实验目标及 LCD-MIT/ADRC 单固件集成；集成目标已三次烧录并通过回读与复位后的 USB 检查。当前版本的上电纯只读发现已确认电机 7 原始模式为 2，参数读取正常，但没有新鲜标准控制反馈；尚未进行运动验收。下文保留 9 月 21 日独立目标的证据，新集成目标状态见下一节。

## LCD-MIT/ADRC 单固件集成（2026-09-23）

- 独立工作树：`D:/download/TCG/Aethor_robo_fw/.worktrees/s3519-adrc-lcd-mit`，分支 `feature/s3519-adrc-lcd-mit`。先合入已验证的 LCD 流畅度版本，再建立单独的 `MDK-ARM/LCD-MIT-ADRC.uvprojx`；原板上 LCD-MIT 固件的完整 Flash 已在替换前备份。
- 上电默认 LCD-MIT。USB 的 `adrc status`、`adrc hardware`、`adrc limits`、`adrc feedback` 可只读；当前目标只接受 `adrc acquire motor=7`。取得所有权前，控制任务检查 LCD 运动/结果队列和发现序列为空、传统 CAN 软件队列与 FDCAN FIFO 已排空并经过 4 ms 静默，且实际发现的电机 7 身份、MIT 模式和全部参数一致。接管期间通过独立 CAN 缓冲区发送一次 `0x7FF/0xCC` 只读反馈查询，只有确认发送并取得查询之后的新鲜禁能、零故障、低速反馈才转移所有权；100 ms 内没有取得反馈则超时返回 LCD。
- ADRC 持有时，LCD POS/MIT 请求和旧 CAN 发送入口被拒绝，LCD 与 USB STOP 都作用于 ADRC 单轴。`adrc release` 强制重新发送 DISABLE；只有匹配的真实 CAN 发送回执及更新的禁能反馈都到达后才交还 LCD。失败或超时保持锁定，`adrc status` 的 `owner`/`handoff` 字段可查询结果。
- `Tests/host/run_adrc_lcd_integration_tests.ps1` 与所有权状态机、独立 CAN 通道测试通过；覆盖默认 LCD、错误轴、未取得控制权拒绝运行、旧 LCD 控制组/新 CAN 提交阻止接管、只读查询发送和超时、查询失败保持 LCD、LCD/USB STOP、发送失败和旧反馈不能交还、已失能监督器仍需新 DISABLE。原 `run_tests.ps1`、ADRC App/bench/协议/监督器、LCD UI 与传输回归通过；`test_debug_ui_build_contract.py` 的 24 组门禁与 13 组 ADRC 发布门禁通过。
- 当前板上已烧录提交 `7db5d7f` 的集成目标，ARMCC 构建 0 错误、0 警告，Code=217240、RO=101180、RW=868、ZI=233500 字节；HEX SHA-256 为 `A02EC16DC65220190E0778C3B620CA1320603D7A41C76D4FDCDF1C8EB5AE570D`。完整构建日志在 `output/adrc/hardware/integrated_gate_flash_20260923/discover_keil_final_20260923.log`，HEX 位于 `MDK-ARM/LCD-MIT-ADRC/`。模型及生成代码仍是此前验证的三个 SLX 和 ERT 快照，本轮未改模型。
- 集成版 MATLAB 客户端新增显式 `acquireMotor(7)`/`releaseMotor()`，仅在收到匹配 ACK 且轮询到 `owner`/`handoff` 的目标状态后返回。用户开启的 R2026a 桌面 Automation Server 在沙箱外可连接；纯内存传输测试通过，输出 `ADRC_CLIENT_TESTS_PASSED checks=45 hardwareOpened=0`，COM 客户端退出码为 0，测试目录为 `output/adrc/client/integrated-com`。独立 `-batch` 也完成了这 45 项，但 MATLAB 在输出后退出时发生 access violation，故该批处理不作为正常退出证据。客户端测试本身未连接串口或硬件。
- 2026-09-23 在板上仍为 LCD-MIT 固件时，COM4 与电机 7 做了上电前后检查。24 V 关闭时，`adrc status` 返回 `unknown_command`；24 V 开启后，`show state` 为 `enabled=00 moving=0`，`bench init 7` 完成，身份/模式/量程/版本掩码均为 `40`。随后连续 10 次 `show motor 7` 仍为 `state=absent age_ms=4294967295`，`show motors present=00`；参数读取成功不能证明实时反馈存在。记录位于 `output/adrc/hardware/usb_readonly_20260923.txt`、`usb_powered_precheck_20260923.txt`、`bench_init_7_readonly_20260923.txt`、`motor7_feedback_poll_20260923.txt`。源码复核确认 `bench init` 在参数发现后还发送非持久化 CTRL_MODE=2 写入并回读；这些历史日志文件名中的 `readonly` 仅表示未使能、未运动，不表示整段 CAN 交互没有写寄存器。检查结束后用户已确认关闭 24 V。
- 用户确认可在 24 V 关闭时替换固件。烧录前 COM4 仍显示原固件 `adrc status code=unknown_command`、`enabled=00 moving=0`，CMSIS-DAP 唯一编号为 `07000001000000000000000000000000a5a5a5a597969908`，芯片 ID 为 `0x10016483`。`e863524` 的 HEX 与 ELF 对应的 317972 个 Flash 字节一致；旧固件完整 1 MiB Flash 备份的 SHA-256 为 `5408D1B10CFE9C6E0DA10AA3FF40B7274D6CD2F83453FE0F90BF44D692A1C935`。新 HEX 烧录后独立回读 317972 字节全部相等，复位后核心为 `State.RUNNING`。工件、原固件备份和 `flash-result.json` 位于 `output/adrc/hardware/integrated_flash_20260923/20260923T112752/`；烧录脚本和控制台日志位于其上级目录。
- 复位后 COM4 自动重新枚举；只读 `adrc status` 返回 `owner=lcd handoff=pending qualified=0 active_id=0 last_id=0`，`show state` 返回 `enabled=00 moving=0 fault=none`，`show motors present=00`，`adrc feedback` 返回 `seen=0 fresh=0`。完整 USB 记录为 `output/adrc/hardware/integrated_flash_20260923/20260923T112752/postflash_usb_readonly.txt`。本次未发送使能、运动、接管或释放指令，也未重新开启 24 V；`disabled=0` 是缺少新鲜驱动反馈，不能解释为已使能。
- 首版集成固件在用户确认 24 V 开启、驱动未使能时，电机 7 的 `bench init` 完成，返回 `identity=40 mode=40 ranges=40 version=40`；这是字段掩码，不是实际模式值。该命令在参数发现后还发送非持久化 CTRL_MODE=2 写入并回读，不能当作纯只读发现使用。随后 `show motors present=00`，`show motor 7 state=absent age_ms=4294967295`，CAN 计数为 `rx=15 tx=19 error=0 busoff=0`。记录位于 `output/adrc/hardware/integrated_flash_20260923/integrated_bench_init_7_readonly_20260923.txt`。
- 在同一上电窗口，只进行了一次有界、非运动的 `adrc acquire motor=7` 请求：命令回执为 `accepted`，后续 `adrc status` 为 `owner=lcd handoff=unsafe`。请求前后 CAN 计数均为 `rx=15 tx=19`，未进入 `0xCC` 查询，`enabled=00 moving=0`；LCD 显示“控制权：远端/只读”。因此只证明安全门阻止了接管，尚不能判定具体被哪一项门限拦截。记录位于 `output/adrc/hardware/integrated_flash_20260923/integrated_handoff_probe_20260923.txt`。此轮之后用户确认关闭 24 V。
- 为定位拒绝原因，新增纯只读、无 CAN 发送的 `adrc gate motor=7` 诊断，输出 LCD/CAN 空闲、实际 MIT 模式和字段验证、新鲜反馈、禁能与故障状态以及队列活动。集成/所有权/CAN 通道/ADRC App 主机测试与 13 组发布门禁通过；新目标构建结果见上。用户确认 24 V 关闭时，已备份首版固件完整 1 MiB Flash，SHA-256 为 `70841974CA4A2B3CFFA8AFE0F45500517A4AFFD91A8FC5C8B0C04BAA3CEE1594`；新 HEX/ELF 对应的 318460 个 Flash 字节相符，烧录后独立回读 318460 字节全部相等，复位后核心为 `State.RUNNING`。报告及备份位于 `output/adrc/hardware/integrated_gate_flash_20260923/20260923T114954/`。
- 诊断版复位后 COM4 只读检查返回 `enabled=00 moving=0 owner=lcd`，`adrc gate motor=7` 为 `lcd_idle=1 can_idle=0 mit_ready=0 mode=0 fields=0000 verified=00 fb_fresh=0`，CAN `rx=0 tx=4 error=0 busoff=0`；10 秒后 `can_idle=0` 仍在。该检查时 24 V 关闭，启动期 CAN 请求可能未被应答，所以此读数不能代表上电后的门限原因。记录位于 `output/adrc/hardware/integrated_gate_flash_20260923/20260923T114954/postflash_usb_gate_readonly.txt`。
- 诊断版在用户重新开启 24 V 后，第一次纯只读查询仍为 `show motor 7 state=absent`、`can_idle=1 lcd_idle=1 mit_ready=0 mode=0 fields=0000 fb_fresh=0`，CAN `rx=0 tx=4`；初始记录为 `output/adrc/hardware/integrated_gate_flash_20260923/powered_gate_preinit_20260923.txt`。随后执行一次 `bench init 7`，结果 `identity=40 mode=40 ranges=40 version=40`，CAN 增至 `rx=15 tx=19`。源码确认此命令包含 CTRL_MODE=2 写入与回读，虽然没有使能或运动，也不能视为纯只读；记录为同目录的 `powered_bench_init_7_readonly_20260923.txt`，文件名沿用测试脚本命名。
- 该轮 `bench init` 后，诊断版 `adrc gate motor=7` 连续两次为 `lcd_idle=1 can_idle=1 mit_ready=0 mode=2 fields=1fff verified=40 fb_fresh=0 disabled=0 no_fault=0 stationary=0`；`show motor 7` 仍为 `state=absent age_ms=4294967295`，`show state enabled=00 moving=0 fault=none`，`adrc status owner=lcd qualified=0`。`mode=2` 是目前可确认的接管拒绝条件，缺少新鲜反馈也是后续接管必须解决的条件；`disabled=0` 在此只表示没有可用反馈，不能推断驱动已使能。记录为同目录的 `powered_gate_readonly_20260923.txt`。本轮没有发送接管、使能或运动命令，结束后用户确认关闭 24 V。
- 新增单独的 `adrc discover motor=7`：仅在 LCD 拥有、LCD/CAN 空闲时启动常规参数发现，不执行 `bench init` 随后的 POS_VEL 模式写入。发现期间 LCD 操作显示忙碌，普通 USB 命令排队，STOP 保持原高优先级路径；完成后用 `adrc gate motor=7` 核对原始模式和字段。集成测试先因缺命令失败，再通过首帧 `0x33` 读取、错误轴和忙碌拒绝、并发命令等待等断言；调试 UI App 各配置、文本协议、ADRC App/CAN 通道、构建合同及 13 组发布门禁通过。实机只读结果见下文，离线测试本身不证明驱动反馈能力。
- 用户确认 24 V 关闭后，烧录前 COM4 只读核对 `enabled=00 moving=0 owner=lcd`，记录位于 `output/adrc/hardware/integrated_gate_flash_20260923/discover_preflash_off_20260923.txt`。随后先备份上一版固件完整 1 MiB Flash，SHA-256 为 `8BC14E6DA2072B5E2D3AD11E3F9BB3F56E50B7AE6D98D4080EECD61B0CF497A9`；新 HEX/ELF 对应 318676 字节一致，烧录后独立回读 318676 字节全部相等，复位后核心为 `State.RUNNING`。报告及备份位于 `output/adrc/hardware/integrated_discover_flash_20260923/20260923T131421/`。复位后 COM4 再枚举，24 V 关闭状态下 `enabled=00 moving=0 owner=lcd`，错误轴 `adrc discover motor=1` 返回 `code=bad_argument`，CAN `rx=0 tx=4` 未变化。记录为 `output/adrc/hardware/integrated_discover_flash_20260923/postflash_usb_readonly_20260923.txt`；未发送有效发现、模式写入、接管、使能或运动命令。
- 用户重新开启 24 V 后，当前版本执行一次 `adrc discover motor=7`。上电前置检查为 `enabled=00 moving=0 fault=none owner=lcd`，LCD/CAN 空闲；发现后 CAN 计数由 `rx=0 tx=4` 增至 `rx=13 tx=17`，`adrc gate motor=7` 为 `mode=2 fields=1fff verified=40 fb_fresh=0`，`show motor 7 state=absent age_ms=4294967295`，最终仍为 `enabled=00 moving=0 owner=lcd qualified=0`。这次只发送 13 个参数读请求，故模式 2 是原始实测值，并非 `bench init` 写入的结果。记录为 `output/adrc/hardware/integrated_discover_flash_20260923/powered_discover_motor7_20260923.txt`；没有发送接管、模式写入、使能或运动指令。测试结束后用户确认 24 V 已关闭。
- 历史上电记录 `output/can_id_commissioning_20260909/lcd-status-refresh-4byte-no-response.log` 与 `lcd-status-refresh-8byte-no-response.log` 显示：`bench init` 和 `bench disable` 后曾取得短暂的 `state=disabled` 反馈，但 4/8 字节 `0xCC` 查询都未持续刷新，约 300 ms 后状态变为 `absent`。因此不重复相同的 `0xCC` 查询，也不把参数读响应当成失能证据。已准备 `Tests/hardware/adrc_motor7_disable_feedback_probe.ps1`，下次上电先检查原始模式和空闲状态，再限定电机 7 发送一次非运动 POS_VEL DISABLE，记录命令完成、发送后新反馈和失效时间；这是 CAN 控制命令，不归类为只读。脚本已通过 PowerShell 语法检查，尚未通电运行。只有该步取得可信反馈，才设计失能状态下的临时 MIT 模式切换与回读试验；切模后仍须重新取得新鲜失能反馈，不能凭模式寄存器回读开放 ADRC 接管。
- 当前硬件资格仍为空，USB 不能自行授权 ADRC 运动；集成版尚无由实测辨识结果驱动的本地资格注入路径。烧录和 USB 启动通过不等于单电机 ADRC 验收。当前阻塞项是原始模式 2 和缺少新鲜标准反馈。先验证单轴 DISABLE 后的反馈寿命，再确定无需使能的模式切换和反馈刷新路径；之后仍需完成位置、速度、转矩映射与 b0 实测、模型重验和资格配置，最后才分阶段开放有限期运动。

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

1. 集成目标的三次烧录、回读、USB 检查和上电纯只读 `adrc discover` 已完成，原始模式 2、参数可读而无新鲜标准反馈。电机 7 的有界 DISABLE/反馈寿命脚本已离线准备，确认 24 V 开启后才执行；成功后再单独设计模式 1 临时切换、回读和反馈试验。只有反馈闭环证据完整且门限明确才重试非运动接管/释放；释放会发送 DISABLE，不能称为纯只读操作。
2. 分别核对位置、速度、转矩映射与量化，不把历史位置比例自动套用到速度或转矩。
3. 经有限脉冲辨识 b0，重复及保留数据验证后，更新 plant 与控制参数并重新执行模型验收。
4. 进行有限期 PI/LADRC 对照，保存速度、已发送名义转矩、ESO 状态、扰动恢复与最终失能证据。

当前 24 V 按用户最近确认保持关闭，板上为带 `adrc gate` 与 `adrc discover` 的 LCD-MIT/ADRC 集成固件。接线和分阶段供电、辨识与调参细节见 [调试与调参步骤](commissioning-and-tuning.md)。未经实测的参数和仿真结果均不得用于解锁电机运行。
