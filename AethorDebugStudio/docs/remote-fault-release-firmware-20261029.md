# 已失能的远程故障会话无法结束：修复候选

## 根因

现有e1f234a6固件收到失败终态后，无论是否已经确认失能，都会进入LOCAL_FAULT并推进本地会话代次；远程release却仍只接受LOCAL_ARMED和故障前owner_epoch。故障退出与释放准入不一致，导致失败后无法用上位机结束会话。代理持有标志清除后，旧能力接口又将本地故障与LCD控制一并报告为owner=1。

排查期间用户确认重启或重新测试过，因此启动号改变后的旧事务不能沿用。本轮只进行只读能力/终态查询；未发送STOP、DISABLE、ENABLE、运动、配置或实机会话释放命令。

## 修复范围

- 能力接口新增authority及remote_fault_release，区分本地故障并明确是否允许提交安全结束审核；旧固件缺少该能力时保持关闭。
- 仅原上位机ID7单轴运动类事务、准入成功、FAILED终态且disabled=1，才可进入审核。保留原事务身份，不借用成功事务或真实LCD控制。
- 原控制代次必须与代理owner_epoch匹配，当前代次必须恰为故障后的下一非零代。绑定原请求的协议session、hello nonce、USB原子连接代数及STOP代数；重连、hello、STOP或LCD接管后不能沿用旧证据。
- capability、提交和真正执行release时均复查。执行阶段仅接受本次release的唯一自有mailbox保留项，不能借用其他事务。
- 真实平台专用探针要求软件槽/队列、独立发送缓冲区、取消请求、RX inbox及两个RX FIFO均已清空；控制器必须实时无故障、未使能、未运动。未处理运输故障、应急/回退帧或残留控制组均阻断。
- 复用原LCD执行器的release准入与结果，不生成CAN控制帧，不清除转矩配额，不改保护阈值，也不自动开始下一次测试。
- 租约过期时保留原STOP意图；已在LOCAL_FAULT时不重复推进故障代次。

## 固定候选及验证

- 候选目录：`D:\download\TCG\session-recovery-candidate-20261029`。
- 目标：LCD-POS-Load-Test。
- HEX：`output/session-recovery/keil-final/LCD-POS-Load-Test.hex`。
- HEX SHA256：`1526a4e7c10212945f41c64773ac3f87e65f8045d30aaadb5fba15381d07b48d`。
- AXF SHA256：`ffd2fc5511316a6412ea43d3416be9c934026482b571680228941063cca2a522`。
- 固定934个源输入，HEX/ELF对应521204个占用字节；预期当前镜像为e1f234a6，部署前必须全量比对并备份。
- Keil最终构建：0错误、0警告。10种App配置及非法配置拒绝检查通过。
- 转矩策略/真实路由回归、17类故障结束阻断以及提交后新运输/控制器故障回归通过。
- 真实Platform/HAL fake严格探针及原静载平台回归，在locked、single、seven、combined四种配置通过。
- 上位机87项单元测试、16项全API拦截浏览器测试通过；TypeScript与生产构建通过。
- 独立复审提出的协议/USB边界、硬件运输债务、独立TX与RX积压、实时控制器故障均已修复并覆盖相应检查。
- 部署适配器默认只做离线核验，缺少本次断电确认不进入硬件路径，失败不重试；4项部署门控测试通过。

## 设备状态与部署

用户已确认本次24V断开、臂段及负载独立承托、LCD空闲、USB及烧录器连接，随后仅执行一次烧录。

- 旧e1f234a6的520048个占用字节全匹配；USB/SWD设备身份一致。
- 完整1 MiB Flash备份SHA256：`b643699c1cabfdbe222cc1cb76a9551e9a76c84ce04ae8eae7c2696467c67ce5`。
- 新1526a4e7的521204个占用字节读回一致；正常复位完成，CPU为RUNNING，error=null。
- 实际部署报告：`D:\download\TCG\session-recovery-candidate-20261029\output\adrc\hardware\20261029T072835697159Z_poweroff_deploy\report.json`。冻结离线清单保留原有flashed=false快照，部署事实以此报告为准。
- COM4重新连接后只读取caps及torque，已收到authority、remote_fault_release新字段。启动号3667573924，epoch=1，owner=0、authority=0、active_id=0、terminal_id=0。
- 24V断开时模式未知、无电机反馈，remote_fault_release=0符合没有原失败事务的初始状态；不把该值当作实机恢复失败或资格证明。
- 配额authorized=0、reason=0、can_reset=0，保持3.5 N·m基线，未恢复。
- 实际页面已打开`http://127.0.0.1:5179/?release=1526a4e7`；POS草稿-50°、50°/s、10秒已恢复为草稿，未审核或提交。

本轮未发送实机STOP、DISABLE、ENABLE、运动、配额恢复或会话释放命令。带电故障恢复尚待后续实机记录核验；不得将烧录成功和离线回归等同于带载资格。
