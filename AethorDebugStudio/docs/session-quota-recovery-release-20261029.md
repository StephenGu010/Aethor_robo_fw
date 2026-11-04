# UI会话握手与首次转矩退出诊断修复

本候选配套更新STM32固件和Studio.Gateway，修复测试失败且确认失能后仍不能结束原远程会话的问题。保护阈值、运动参数、LCD执行器和配额恢复条件保持原值。

## 根因与修复

1526a4e7的故障释放审核要求原协议session非零，但既有网关从不发送hello，实际保留的失败事务session为0。直接放行0不能充分隔离串口关闭后新客户端重新打开的情况，因此保留非零审核门控，修正连接握手。

- 网关每次显式连接生成独立GUID挑战，由唯一串口线程发送一次`hello ui=<32位小写hex>`，核对原请求号、挑战、协议、bench/7轴以及非零boot/session/nonce。握手失败关闭连接，不自动重试，不处理排队远程动作。
- 固件新增可选的UI身份握手，仅旋转协议身份并回显挑战，不激活旧USB协议看门狗、不清队列或结果、不改变流设置或LCD控制代次。原无字段hello及普通命令重放语义不变。
- 相同ID、不同挑战命中旧缓存时明确冲突，网关关闭连接；旧成功ACK的挑战不匹配时忽略，不能假装建立了新会话。
- 原事务、接纳、失能、原控制代次、USB连接代数、hello nonce、STOP代数、实时通信空闲等审核全部保留。结束会话只释放原控制权，不生成运动帧，不恢复配额。
- 首次转矩保护发生前保存授权状态、账本读数，绑定原动作epoch/request ID，并保留实际保护分支及反馈转矩。覆盖运动、保持和普通失能阶段；cleanup不能改写，其他事务不能借用。
- 原失败终态增加可选`tq_reason/tq_auth/tq_used/tq_nm`，页面区分3.5基线、5.46峰值、10秒窗口、累计预算。缺失或不完整证据继续只显示通用原因，不能根据故障后的账本倒推首因。

## 固定产物与离线证据

- 固件目录：`D:\download\TCG\session-quota-recovery-candidate-20261029`。
- 目标：LCD-POS-Load-Test；HEX：`output/session-quota-recovery/keil-final/LCD-POS-Load-Test.hex`。
- HEX SHA256：`02f1ee453db560fb9022e60ac1cc23a178d99ba4854ea50f937951008d6b40e3`。
- 离线清单：`output/session-quota-recovery/release-offline-20261029.json`，SHA256：`752973037445c2b59d017e2fd1351f41da8690f7207c2589c678c767bd325e13`。
- 固定934个固件输入、85个宿主输入/产品；HEX/ELF对应521764个占用字节。预期板上旧固件为1526a4e7，部署前完整比对并备份1 MiB Flash。
- Keil构建0错误、0警告；10种App配置和非法配置拒绝检查通过；原文本协议引擎回归通过。
- 真实App代理/转矩适配回归验证四种退出分支、普通失能首故障、清理后保留、身份/运输/STOP/真实LCD阻断和只释放不恢复配额。
- 网关38项、前端88项单元测试通过。12项全API拦截浏览器测试通过，包括四种首故障显示和原有会话恢复门控；没有连接真实电机。
- 4项部署门控测试通过：缺本次断电确认不准备、默认离线、失败不重试、旧/新映像与所有固定文件精确匹配。
- 独立源复审及部署复审均无未解决阻断。按用户既有选择，未追加外部模型。

测试日志分别保留在候选根目录`output-session-quota-*.log`及上位机`artifacts/session-quota-*.log`。App脚本末尾故意编译非法配置，原生LASTEXITCODE可留为非零；实际10项通过及`DEBUG_UI_INVALID_CONFIGS_REJECTED`均已逐项核对。

## 部署边界

用户已确认本次24V断开、机械臂和负载独立承托、LCD空闲、USB与烧录器连接，随后执行一次部署，已完成：

- 旧1526a4e7的521204个占用字节匹配，USB/SWD身份一致。
- 完整1 MiB备份SHA256：`40ef13adafad216ab11bae83c87f5c7175174db70de57639f24cb1cf4d3231d3`。
- 新02f1ee45的521764个占用字节读回一致，正常复位完成，CPU为RUNNING、error=null。
- 部署报告：`D:\download\TCG\session-quota-recovery-candidate-20261029\output\adrc\hardware\20261029T085903237741Z_poweroff_deploy\report.json`。冻结清单中的`flashed=false`保留离线快照语义，部署事实以此报告为准。
- 配套新网关PID21160，普通用户medium完整性；原网关产品保留于`output/session-quota-old-gateway-20261029`。新网关产品位于`output/session-quota-gateway-release`，未覆盖旧产品。
- 新COM4连接握手实测：UI session=604856869、nonce=1、boot=3441103743。控制权owner=0/authority=0，active_id=0、terminal_id=0；没有原失败事务时remote_fault_release=0符合预期，不能把它当作故障释放实测。
- 配额authorized=0、reason=0、can_reset=0，基线3.5、实验峰值5.46、窗口10000 ms；未执行恢复。
- 实际页面已打开`http://127.0.0.1:5179/?release=02f1ee45`，法兰-50°、50°/s、10秒仅恢复为未审核草稿；只点击了读取能力和读取转矩策略。
- 页面截图：`artifacts/session-quota-postflash-live-20261029.png`；本轮未发实机STOP、DISABLE、ENABLE、运动、回零、预检、会话释放或配额恢复请求。

固件与网关须配套使用。旧固件不认识挑战握手时，新网关会关闭连接，不降级成无会话调试；若保留旧固件，应配用原版本网关。

烧录后先核验新握手、控制权和反馈；本轮不启动测试、使能、运动、回零、预检或配额恢复。24V断开时电机无实时反馈属于现场状态，不能判为该修复失败。

3.5 N·m基线、5.46 N·m瞬时线、10秒超额窗口及累计预算均未提高。主控重启后实验配额默认关闭；后续需要用户单独完成冷却承托声明、有效失能维护证据及显式恢复，不能把烧录成功当成已恢复5.46 N·m或完成带载资格验证。
