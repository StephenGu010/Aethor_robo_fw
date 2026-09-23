# ADRC 实施状态与续接入口

更新日期：2026-09-23。已完成 ADRC 模型、生成 C、独立实验目标及 LCD-MIT/ADRC 单固件集成。电机 7 原始模式为 2，失能状态可临时切入模式 1 并恢复模式 2；4/8 字节 `0xCC` 查询已实际发送，但未取得查询后的新接收帧。首次 100 ms MIT HOLD 脚本运行时 OUTPUT 关闭，不计作带电证据；随后在确认 24 V 供电下重做，取得 26 帧连续使能态反馈、25 个 4000 μs 帧间隔及最终失能/模式恢复证据。当前仍未完成 ADRC 接管或闭环运动验收；速度与转矩量化是阻塞项。下文保留 9 月 21 日独立目标的证据，新集成目标状态见下一节。

## LCD-MIT/ADRC 单固件集成（2026-09-23）

- 独立工作树：`D:/download/TCG/Aethor_robo_fw/.worktrees/s3519-adrc-lcd-mit`，分支 `feature/s3519-adrc-lcd-mit`。先合入已验证的 LCD 流畅度版本，再建立单独的 `MDK-ARM/LCD-MIT-ADRC.uvprojx`；原板上 LCD-MIT 固件的完整 Flash 已在替换前备份。
- 上电默认 LCD-MIT。USB 的 `adrc status`、`adrc hardware`、`adrc limits`、`adrc feedback` 可只读；当前目标只接受 `adrc acquire motor=7`。取得所有权前，控制任务检查 LCD 运动/结果队列和发现序列为空、传统 CAN 软件队列与 FDCAN FIFO 已排空并经过 4 ms 静默，且实际发现的电机 7 身份、MIT 模式和全部参数一致。接管使用单次电机 7 DISABLE 挑战；须确认实际发送成功和提交后的新鲜禁能、零故障、低速反馈，100 ms 内未取得则超时返回 LCD。此路径已刷入但尚未做带电接管实测。
- ADRC 持有时，LCD POS/MIT 请求和旧 CAN 发送入口被拒绝，LCD 与 USB STOP 都作用于 ADRC 单轴。`adrc release` 强制重新发送 DISABLE；只有匹配的真实 CAN 发送回执及更新的禁能反馈都到达后才交还 LCD。失败或超时保持锁定，`adrc status` 的 `owner`/`handoff` 字段可查询结果。
- `Tests/host/run_adrc_lcd_integration_tests.ps1` 与所有权状态机、独立 CAN 通道测试通过；覆盖默认 LCD、错误轴、未取得控制权拒绝运行、旧 LCD 控制组/新 CAN 提交阻止接管、禁能挑战发送和超时、失败保持 LCD、LCD/USB STOP、发送失败和旧反馈不能交还、已失能监督器仍需新 DISABLE。原 `run_tests.ps1`、ADRC App/bench/协议/监督器、LCD UI 与传输回归通过；`test_debug_ui_build_contract.py` 的 24 组门禁与 13 组 ADRC 发布门禁通过。
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
- 历史上电记录 `output/can_id_commissioning_20260909/lcd-status-refresh-4byte-no-response.log` 与 `lcd-status-refresh-8byte-no-response.log` 显示：`bench init` 和 `bench disable` 后曾取得短暂的 `state=disabled` 反馈，但 4/8 字节 `0xCC` 查询都未持续刷新。当前版本已用 `Tests/hardware/adrc_motor7_disable_feedback_probe.ps1` 在用户确认 24 V 开启后复测：前置 `enabled=00 moving=0 fault=none owner=lcd`，重新只读发现后仍为 `mode=2 fields=1fff`；`bench disable 7` 一次返回 `done ... result=completed enabled=00`。其后 `show motor 7` 在反馈年龄 35、66、97 ms 时为 `state=disabled`，129 ms 时已为 `absent`，直至 475 ms 未再刷新。CAN `rx=13 tx=17` 增至 `rx=27 tx=31`，`error=0 busoff=0`；最终 `enabled=00 moving=0 owner=lcd qualified=0`。原始记录为 `output/adrc/hardware/disable_feedback_probe_20260923/motor7_disable_feedback_20260923T054951769Z.txt`。原脚本末尾的 `adrc feedback` 报 `motor=1`，因为 ADRC 草稿轴尚未配置为 7，不是本次电机 7 的反馈证据；脚本随后移除此无关查询。本轮没有模式写入、使能、运动或 ADRC 接管，用户结束后确认 24 V 已关闭。
- 当前硬件资格仍为空，USB 不能自行授权 ADRC 运动；集成版尚无由实测辨识结果驱动的本地资格注入路径。烧录和 USB 启动通过不等于单电机 ADRC 验收。当前阻塞项是原始模式 2 与缺少持续的新鲜标准反馈。新增 LCD 持有控制权时的 `adrc probe motor=7`：仅在模式 1 已回读、LCD/CAN 空闲时经独立缓冲区发送一次 `0x7FF/0xCC`，`adrc probe` 可区分已发送、新反馈、超时和失败；诊断不接管 ADRC。`Tests/hardware/adrc_motor7_mode_feedback_window.ps1` 已离线准备：一次上电内完成只读发现、失能切到模式 1、单次查询、恢复模式 2 和最终失能，任何异常保留日志并尝试恢复/失能。集成、所有权、ADRC App、Phase 0 和文本协议主机测试通过，Keil 集成目标构建 0 错误、0 警告。即便模式 1 读回成功，也需实测禁能及运行状态下的反馈更新间隔、速度分辨率和延迟，不能凭单帧放开 4 ms 闭环。之后才进行映射、b0 实测、模型重验和有限期运动。
- 上述 `adrc probe` 固件随后在用户确认 24 V 关闭状态下完成烧录。烧录前 COM4 只读为 `enabled=00 moving=0 fault=none owner=lcd`；目标提交 `b838bc3`，HEX SHA-256 为 `233AB1FFCC593E39A67720939FA38F1B8DD335E3E4F28DF885A23C4EB2EFBE24`，Keil 完整重建 0 错误、0 警告。旧 Flash 1 MiB 备份 SHA-256 为 `3FC7C287B7033C422975E9BF9BB951BC144B3E8741F5C3F3A1CB93573414143A`，新映像 319560 字节独立回读全部一致，复位后核心为 `State.RUNNING`。报告、备份与构建日志位于 `output/adrc/hardware/mode_feedback_probe_20260923/20260923T142711/`。复位后 COM4 返回 `enabled=00 moving=0 fault=none owner=lcd`、`adrc probe motor=7 state=idle tx=0`，错误轴请求得到 `code=bad_argument`，CAN `rx=0 tx=4 busoff=0`；记录见同目录上级的 `postflash_usb_readonly.txt`。烧录阶段没有发送使能或运动命令。
- 用户随后开启 24 V，执行一次 `adrc_motor7_mode_feedback_window.ps1` 并已再次关闭 24 V。原始模式 2 的只读发现完成；`bench mode 7 mode=mit` 在 13 ms 内完成并回读模式 1，`enabled=00 moving=0 owner=lcd`。`adrc probe motor=7` 的专用 CAN 发送回执为 `tx=1`，但 `sample_after_tx=0`，100 ms 内超时；原有禁能反馈的年龄从 25、57、95 ms 增到 135 ms 时变为 `absent`。随后 `bench mode 7 mode=pos_vel` 在 15 ms 内完成并回读模式 2，最终 `bench disable 7` 返回 `completed enabled=00`，收尾仍为 LCD 控制权、无故障和 bus-off。原始记录是 `output/adrc/hardware/mode_feedback_probe_20260923/motor7_mode_feedback_20260923T063141935Z.txt`；断电后 COM4 只读记录是同目录的 `post_window_poweroff_readonly.txt`。这证明当前 4 字节 `0xCC` 路径在模式 1 的禁能状态也没有取得可用新反馈，不证明驱动完全没有发帧，因为 `show diag can` 经 100 ms 诊断任务采样。未使能或运动，未取得 ADRC 接管资格。
- 对照随附厂商 H7 SDK 与 Python u2canfd SDK，4 字节 `0x7FF/{07,00,CC,00}` 与当前固件一致；另一份 Python u2can SDK 使用同前缀的 8 字节零填充帧。厂商 V1.4 协议手册第 32 页说明普通控制反馈应为 8 字节、以 Master ID 发出，但未明确当前固件在禁能时对 `0xCC` 的应答条件。对照版加入 8 字节查询选项，以及查询发送后直接进入 STM32 接收路径的帧数、有效反馈数和拒绝数；同一上电窗口对照 4/8 字节，避免继续用延迟更新的 `show diag can` 计数判断即时收帧。相关主机测试通过，Keil 完整重建 0 错误、0 警告；新版本尚未上电实测。
- 对照版随后在 24 V 关闭状态下完成烧录。烧录前 COM4 为 `enabled=00 moving=0 fault=none owner=lcd`；目标提交 `c1622c3`，HEX SHA-256 `F7B514412B9E150F0E11EA9AF342A1BAE1AEAC164525A36CE50774B30A6E0458`。旧 Flash 1 MiB 备份 SHA-256 `F6EC121EE6F898999C55349B4890A96A6449E01625B2F125232A5817329A0EAC`，新映像 320144 字节逐字节回读一致，复位后核心为 `State.RUNNING`。构建日志、备份和 `flash-result.json` 位于 `output/adrc/hardware/mode_feedback_probe_20260923/20260923T144859/`；复位后 COM4 返回 `enabled=00 moving=0 owner=lcd`、`adrc probe motor=7 state=idle query_len=4 rx_after_tx=0`，错误轴仍被拒绝，记录为同目录上级 `postflash_rx_compare_readonly.txt`。对照版尚未上电实测，也没有发送使能或运动命令。
- 用户再次开启 24 V 运行同一有界脚本，随后确认关闭。模式 1 回读、模式 2 恢复和最终 `DISABLE completed enabled=00` 均通过，始终未使能或运动。4 字节查询 `tx=1`，100 ms 内 `rx_after_tx=0 rx_valid=0 rx_rejected=0`；旧禁能反馈仍变为过期。8 字节请求虽返回 `probe=accepted`，但紧接着 `state=failed tx=0`，故没有发到电机，不能推断其响应能力。原始记录为 `output/adrc/hardware/mode_feedback_probe_20260923/motor7_mode_feedback_20260923T065256719Z.txt`。源码定位到 `stm32_adrc_channel_submit` 把专用探针硬限制为 4 字节；现已离线改为仅允许 4 字节或末四位全零的 8 字节 `0xCC`，并按实际长度选择 FDCAN DLC。先失败再通过的专用通道测试、集成测试和 Keil 完整重建均通过，修正版尚未烧录或实测。
- 修正版在用户确认 24 V 关闭后完成烧录。烧录前 COM4 为 `enabled=00 moving=0 fault=none owner=lcd`，目标提交 `a7d8e1a`，HEX SHA-256 `1502E9F5B369B5BB30E6D2461FA33029350186E8123C944105225AC91A4F1E5E`；旧 Flash 1 MiB 备份 SHA-256 `AEC17C71E36FCD92738CB95AD0A1793545388FA4AC0868160EB9B28D2345F17B`，新映像 320164 字节逐字节回读一致，复位后核心 `State.RUNNING`。报告、备份与构建日志在 `output/adrc/hardware/mode_feedback_probe_20260923/20260923T145921/`，复位后 COM4 只读为 `enabled=00 moving=0 owner=lcd`、`adrc probe state=idle query_len=4 rx_after_tx=0`，错误轴仍被拒绝；原始 USB 记录在同目录上级 `postflash_channel_fix_readonly.txt`。没有发送使能或运动命令；8 字节实际发帧及反馈尚待新一轮上电确认。
- 用户确认 24 V 开启后运行修正版单窗口脚本，并在完成后确认关闭。原始模式 2 的只读发现完成，失能状态切到模式 1 并回读；4 字节查询与 8 字节零填充查询都得到专用 CAN 发送回执 `tx=1`，各自 100 ms 窗口内均为 `rx_after_tx=0 rx_valid=0 rx_rejected=0 sample_after_tx=0`，原有禁能反馈随时间过期。随后模式 2 恢复并回读，最终 `bench disable 7` 为 `completed enabled=00`；收尾 `show state enabled=00 moving=0 fault=none`、`adrc status owner=lcd`、CAN `busoff=0`。原始记录为 `output/adrc/hardware/mode_feedback_probe_20260923/motor7_mode_feedback_20260923T070531089Z.txt`。本轮无使能、运动或 ADRC 接管；结果表明当前失能状态下这两种查询均未产生可用反馈，不能据此推断使能后反馈行为，也不能放开 ADRC 运动资格。下一步先离线核对厂商反馈条件和现有驱动的安全启停路径，准备有界的使能后反馈试验，再决定是否进入该物理试验；不重复相同的失能查询或烧录。
- 离线复核现有 `bench mit ... action=hold`：只有预检 DISABLE 取得新的禁能反馈、MIT 模式回读、清故障后取得新反馈，才发送 ENABLE；ENABLE 后需取得新的 enabled 反馈才开始保持；保持使用读回位置、`kp=1 kd=1 torque_ff=0`，反馈过期或故障进入失能清理，终态需新的 disabled 反馈。已单独准备 `Tests/hardware/adrc_motor7_enabled_feedback_pilot.ps1`：要求显式 24 V、使能试验批准及电源限流确认开关，预检和位置锚点，单次 100 ms HOLD，轮询速度/转矩/位移并在主机侧越限时 STOP，之后尝试 DISABLE、恢复模式 2、最终再次 DISABLE 和状态核对。PowerShell 语法检查及缺少确认开关时拒绝执行的离线检查通过；**首次脚本执行未完成带电试验，后续带电结果见本节末尾**，主机轮询次数不等于独立 CAN 样本数，也不能替代 4 ms 反馈周期测量。该试验可能产生短时保持力矩或小幅运动，须在确认电源限流和人工断电条件后单独决定是否执行。
- 用户曾允许上述**一次** 100 ms 试验。电源背面铭牌为 MAISHENG MS-6010C、输出额定 0–60 V / 0–10 A；OUTPUT 关闭时面板显示 24 V / 1.7 A，开启后用户报告实际 24 V / 0.097 A。1.7 A 可记录为面板预设读数，但尚无独立限流动作校准。S3519 V1.1 说明书第 20 页列出的 9.2 A/8.6 A 是额定相电流/电源电流，第 10 页的“1 A 以上电源”只针对校准流程；二者都不能代替本台架设定值。
- 已运行一次 `Tests/hardware/adrc_motor7_enabled_feedback_pilot.ps1`，原始记录为 `output/adrc/hardware/enabled_feedback_pilot/motor7_enabled_feedback_20260923T072542072Z.txt`。脚本记录的三个确认开关为 True，MIT HOLD 命令报告 `completed elapsed_ms=121`（配置的保持时间为 100 ms），7 条主机轮询看到 `state=moving` 且反馈年龄 0–4 ms；结束时模式 2 已恢复，最终 DISABLE 报告完成，`enabled=00 owner=lcd busoff=0`。但用户随后明确确认**脚本运行时 OUTPUT 已关闭**，并且未观察电机。因此该次不能证明带电使能、机械运动、输出电流、持续反馈或 4 ms 独立 CAN 更新周期；不能用日志中的 `power24v_confirmed=True` 替代实际供电证明。反馈帧的供能条件未查明，保留原始日志，不把这次计入实机闭环资格。用户已确认目前 24 V 关闭。
- 原脚本汇总的 `fresh_enabled_polls=0` 是主机统计字段的错误：公开 `show motor` 将使能反馈显示为 `moving` 或 `holding`，不会显示 `enabled`。原始 7 条活动态轮询均为年龄小于 100 ms；脚本后续改为统计 `fresh_active_polls`。这仅修正日志解释，不改变上段供电状态结论。日志速度恒为 -2.799°/s，位置仅在约 ±0.011° 间变化；由 `vmax_deg_s=11459.156` 和 12 位速度编码计算，单码格约 5.597°/s（0.09768 rad/s），因此这些速度读数及文本 `moving` 不能独立证明轴在转动。
- 当前离线仿真把速度量化设为 0.001 rad/s，约比上述原始反馈码格细 98 倍；固件资格要求 `velocity_quantum_rad_s <= 0.015`，同时又要求配置值不小于按发现量程计算的真实码格。默认坐标映射为 1 时，两项无法同时满足。这是**已验证的模型/资格不匹配**，不能通过仅调大 ESO 带宽或填写资格标志绕过。下一步应先确定可靠的供电状态记录、反馈帧实际到达间隔与位置/速度映射，再选定速度估计方法和控制周期，重建量化/延迟仿真及资格门限；只有通过后才设计新的带电限时试验。
- 已离线加入电机运行时的**最新连续使能态反馈段**统计：只有通过解码和身份/时间戳检查的帧才计数，禁能或故障帧结束当前段；下一段从 1 重新计数。`adrc gate motor=7` 只读返回 `active_samples`、`active_intervals`、`active_min_us`、`active_max_us`，100 ms 试验脚本在最终失能后保存该结果。统计时间取自 STM32 CAN 接收任务解码时刻，反映控制软件实际接收节奏，不等同于总线物理到达时刻，也不能证明 24 V OUTPUT 状态。Motor 核心测试、LCD/ADRC 集成测试和 PowerShell 语法检查通过；集成 Keil 完整重建 0 错误、0 警告，日志为 `output/adrc/hardware/enabled_feedback_pilot/timing_build_20260923.log`。用户确认 24 V 关闭后，已刷入提交 `79ecd83` 的诊断版。烧录前只读为 `enabled=00 moving=0 owner=lcd`；HEX SHA-256 为 `461B553807FDC277DE81C7810FC2D99E222A2CDC1366AA2C0A818C194F6B1E3C`，与 ELF 对应的 320396 个 Flash 字节一致。旧版完整 1 MiB Flash 已备份，SHA-256 为 `16050BC12EF97778F52A07D9392F50E3FA4E96A3791D2F0ADF62F9F4FC2EFB08`；新映像逐字节回读 320396 字节一致，复位后核心为 `State.RUNNING`。备份与报告位于 `output/adrc/hardware/enabled_feedback_pilot/20260923T154457/`。复位后 COM4 只读为 `enabled=00 moving=0 fault=none owner=lcd`，`adrc gate motor=7` 返回四个新增字段均为 0；记录为同目录上级的 `preflash_poweroff_readonly_20260923.txt` 和 `postflash_poweroff_readonly_20260923.txt`。没有发送发现、接管、使能或运动命令，也没有重新开启 24 V；无供电时 `mode=0 fields=0000` 不代表驱动参数改变。

- 用户在脚本运行前明确确认 OUTPUT 已开启、面板实际为 24 V / 0.096 A、可以观察黑色转子且可立即断电。新的带电原始记录为 `output/adrc/hardware/enabled_feedback_pilot/motor7_enabled_feedback_20260923T075222068Z.txt`：原始模式 2 发现和预检 DISABLE 完成；100 ms、Kp=1、Kd=1、前馈转矩 0 的 MIT HOLD 报告 `completed elapsed_ms=121`；接收侧连续使能反馈 `active_samples=26 active_intervals=25 active_min_us=4000 active_max_us=4000`。主机 6 条活动态轮询反馈年龄 1–4 ms，位置从预检 0.055° 到末尾 0.011°，观察到的最大绝对反馈转矩为 0.051 Nm；速度读数始终 -2.799°/s，受 5.597°/s 原始码格限制，不能用它判定实际转动。最终 DISABLE、模式 2 恢复、LCD 控制权和 `busoff=0` 均有回执。用户随后确认 OUTPUT 已关闭，转角太小未见明显转动；未取得试验期间电源 CC 指示或电流瞬态记录。该结果只验收一次短时使能态反馈接收节奏与安全收尾，不验收供电电流峰值、长期反馈时延或 ADRC 闭环。

- 24 V 关闭后，以已刷入的 AXF 符号定位并只读 STM32 RAM 中电机 7 的发现快照，得到 `PMAX=12.5 rad`、`VMAX=200 rad/s`、`TMAX=10 Nm`、`mode=2 fields=1fff`；前两项、模式和字段与此前 COM4 发现结果一致。记录为 `output/adrc/hardware/enabled_feedback_pilot/motor7_range_ram_read_20260923.txt`。这不是新一次驱动寄存器查询。`TMAX=10` 对应的 12 位转矩码格约 0.004884 Nm，超过现有 4 ms、0.5 Nm/s 斜率门限允许的 0.002 Nm；合成模型的 0.001 Nm 也偏乐观。因此除速度量化外，转矩量化也是当前 ADRC 资格阻塞项。厂商 S3519 V1.1 手册将 `VMAX(0x16)`、`TMAX(0x17)` 列为可读写映射范围，寄存器写入立即生效但未发送存储命令时掉电丢失。暂以 `VMAX=20 rad/s`、`TMAX=4 Nm` 作为**待验证的临时映射候选**，理论码格分别约 0.009768 rad/s、0.001954 Nm；尚未写入电机，也不能把 `TMAX` 当成电源限流。实施前需完成失能独占、写入回读、主控量程同步、失败恢复原值和模型重验。

- 接管路径已调整：`adrc acquire motor=7` 在 MIT 模式回读、LCD/CAN 空闲后，经专用 FDCAN buffer 发送一次**精确的电机 7 DISABLE 帧**；只有确认本次帧实际发送成功，且收到时间戳晚于该帧提交时刻的新鲜、禁能、无故障、低速反馈，才交接 ADRC。CAN 回复可能早于下一个 4 ms 周期的发送回执采样，因此反馈不要求晚于回执采样时刻。独立的 LCD 诊断 `adrc probe` 仍发送 4/8 字节 `0xCC`。专用通道仅额外放行 CAN ID `0x007`、数据 `FF FF FF FF FF FF FF FD` 的电机 7 DISABLE，拒绝字节变体和其他运动帧。此接管版本 `9c74b53` 已在 24 V 关闭时烧录：先备份原 Flash 1 MiB，回读验证新镜像 320508 字节完全一致，复位后核心运行。记录在 `output/adrc/hardware/enabled_feedback_pilot/20260923T162042/flash-result.json`。COM4 只读检查为 `enabled=00 moving=0 owner=lcd qualified=0 busoff=0`，原始记录在同目录 `postflash_usb_readonly.txt`；没有发送接管、使能或运动指令。

- 离线复核发现释放也需按 DISABLE **提交时刻**而非下一周期回执采样时刻判断反馈新鲜度，否则一次快速禁能回复可能被误判为旧反馈，导致 LCD 控制权无法交还。现已新增专用缓冲区提交时间记录；交还仍要求匹配的实际发送成功、提交后的新鲜禁能/无故障/低速反馈和 CAN 空闲。所有权、集成、专用 CAN 通道、ADRC App 主机测试已通过，Keil 集成完整重建 0 错误、0 警告，构建日志为 `output/adrc/hardware/enabled_feedback_pilot/handoff_release_build_20260923.log`。释放修正版 `cc3bb3f` 已在 24 V 关闭时烧录：原 Flash 1 MiB 已备份，新镜像 320632 字节回读一致，复位运行；报告为 `output/adrc/hardware/enabled_feedback_pilot/20260923T163039/flash-result.json`。COM4 只读验收为 `enabled=00 moving=0 owner=lcd qualified=0 busoff=0`，原始记录在同目录 `postflash_usb_readonly.txt`。尚未带电实测接管/释放；即使通过，速度与转矩量化资格仍阻止 ADRC 运动。

- 已准备 `Tests/hardware/adrc_motor7_handoff_window.ps1`，用于下一次**单次上电窗口**的非运动接管/释放验收。脚本只允许电机 7 参数发现、失能切模式、`adrc acquire`、`adrc release`、只读状态、模式 2 恢复及最终 DISABLE；无 ENABLE、转矩或速度命令。每步检查 LCD 控制权、MIT 模式和 CAN 状态，若释放后仍非 LCD 所有者，保持锁定、不尝试旧模式命令，并在原始记录中标记需要手动断电。脚本已通过 PowerShell 语法检查，尚未在带电电机上执行。

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

1. 集成目标的既往烧录、回读、USB 检查，以及模式 2/模式 1 禁能反馈试验已完成。模式 1 能临时切入并恢复模式 2；4/8 字节 `0xCC` 查询均实际发帧，但禁能状态下没有新接收帧。首次 100 ms MIT HOLD 脚本运行时 OUTPUT 关闭，不计作带电证据；随后在用户确认 OUTPUT 为 24 V / 0.096 A 且可观察转子的条件下重做一次，测得 26 帧连续使能态反馈及 25 个 4000 μs 接收间隔，模式 2 恢复和最终失能完成。下一步先在 24 V 关闭时烧录新 DISABLE 接管版本并只读验收，再在一次受控上电窗口验证非运动接管和释放；不因此批准 ADRC 运动。
2. 分别核对位置、速度、转矩映射与量化；当前实测范围的速度、转矩码格与离线模型及固件资格门限不匹配。先解决测量链与模型/门限一致性，不把历史位置比例自动套用到速度或转矩。
3. 经有限脉冲辨识 b0，重复及保留数据验证后，更新 plant 与控制参数并重新执行模型验收。
4. 进行有限期 PI/LADRC 对照，保存速度、已发送名义转矩、ESO 状态、扰动恢复与最终失能证据。

当前 24 V 按用户最近确认保持关闭，板上为提交 `cc3bb3f` 的 LCD-MIT/ADRC 集成固件，已包含电机 7 DISABLE 接管挑战、DISABLE 提交后反馈释放门禁、`adrc gate`、`adrc discover`、`adrc probe` 和使能态反馈帧间隔统计。接线和分阶段供电、辨识与调参细节见 [调试与调参步骤](commissioning-and-tuning.md)。未经实测的参数和仿真结果均不得用于解锁电机运行。

## 主线位置与工期估算

| 主线阶段 | 当前状态 | 通过条件 | 估算工作量 |
| --- | --- | --- | --- |
| 模型、生成代码、集成固件、烧录与只读发现 | 已完成离线验证及当前板上检查 | 不能替代实机闭环验收 | 已投入，不计剩余 |
| 模式与连续反馈资格 | **正在进行**；原始模式 2，禁能查询不持续反馈；带电 100 ms 保持期间 26 帧、25 个 4 ms 接收间隔已测得，低速分辨率仍不合格 | 模式 1/恢复模式 2 均回读；失能和后续运行反馈的更新间隔、时延及分辨率支持选定控制周期 | 1–2 个工作日，取决于驱动反馈行为 |
| 坐标、量程、看门狗及本地资格 | 未开始实测 | 位置/速度/名义转矩映射与失联停止可复现，固件仅接受绑定此电机的证据 | 1–2 个工作日 |
| 小脉冲辨识与模型重验 | 未开始实测 | 正负有界激励辨识 b0，重复及留出数据通过，Simulink/生成代码用实测参数重验 | 1–2 个工作日 |
| 空载速度控制与 PI 对照 | 未开始实测 | 低速限时运行、STOP/失能、日志及重复性能可核对 | 1–2 个工作日 |
| 可重复负载扰动恢复 | 缺少已验收的可重复加载装置 | 同一负载工况下比较 PI/LADRC 的速度跌落与恢复 | 装置就绪后另需约 1–3 个工作日 |

上述是后续工程工作量估算，不是已经约定的完成日期。若模式 1 仍没有足够频率或分辨率的反馈，必须先调整采样周期/估计方法或硬件测量链并重跑模型与安全检查；此分支不能按表中工期保证。为避免反复通断电，下一次上电只在离线脚本、通过/停止条件和恢复步骤准备好后进行。

## 减少 24 V 手动通断电的目标

当前实际接入 Windows 的串口为 COM4（STM32 USB CDC，VID 0483/PID 5740）和 COM3（CMSIS-DAP 接口，VID 0D28/PID 0204）；工程 `.ioc` 中没有已命名的电机 24 V 电源使能或继电器输出，项目资料也没有记录外部电源型号、远程接口或功率回路开关接线。因此目前只能通过 CAN 请求驱动失能，不能把它当作电脑已经切断 24 V 的证据；不会凭未确认的 GPIO 或串口发送电源开关指令。

按用户最新决定，24 V 由用户手动通断，暂缓电源自动化。先采用单次上电窗口：断电时完成代码审查、脚本、测试和可恢复固件准备；用户上电一次后，脚本依次执行预检、有限项非运动诊断、模式/反馈检查、恢复原模式、最后 DISABLE 与状态记录；任一步异常即停止后续动作并要求物理断电。下轮不为每个只读查询单独要求通断电。

若当前电源具备明确的 USB/串口/网口远程输出控制与实际输出状态回读，或功率回路已有额定适配的独立受控开关，可在确认型号、接线、默认断电行为和人工急停后增加软件控制。自动上电前必须核对电压/限流、控制板身份、单轴目标和无运动任务；断线、脚本异常或超时应退出本轮试验并留下可见状态。电源设备型号与接线尚待用户提供，远程通断电未实现或验收。
