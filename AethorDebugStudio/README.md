# Aethor 调试工作台

Windows本地调试工作台 v0.2.0，前端React/TypeScript，网关.NET 8；默认离线演示。仓库中的配套固件目标为 `../MDK-ARM/LCD-POS-Load-Test.uvprojx`。

当前已接入ID7单轴静载、POS带载、静载RAM参数、软件法兰零点及回零、会话释放、最近60秒示波和可调试验转矩退出线。动作只在用户明确操作后提交，未知回执不自动重试。先阅读[新手手册](docs/user-manual.md)和[本次源码发布说明](../docs/releases/2026-11-04-studio-pos-quota.md)。

单轴静载与POS带载共用3.50～6.24 N·m试验退出线（电机输出轴口径），不等于目标转矩或驱动限流。充分冷却并独立承托后，页面允许一次失能取样、原门禁审核、配额应用及读回。具体按钮顺序见[配额操作说明](docs/quota-operations.md)。

2026-11-03已部署209e5fc5固件，随后单次失能及5.46 N·m配额应用通过实机读回；没有以此声明较大转矩POS承载资格已通过。模式2待机仍可能只有位置寄存器持续更新，实测速度/转矩/温度过期时保持未知。见[固件部署记录](docs/deployment-209e5fc5-20261103.md)及[配额准备修复记录](docs/pos-quota-preparation-20261103.md)。

## 打开

从本仓库的 `AethorDebugStudio` 目录双击 **Start-Studio.cmd**。首次构建需要联网下载前端和.NET固定依赖。启动完成后打开 http://127.0.0.1:5179 。已有构建可运行 `./Start-Studio.ps1 -SkipBuild`。启动不自动打开串口或恢复试验配额。

环境：Node.js 22.12或更新版本（本轮22.23.2）、.NET SDK 8（本轮8.0.426）、Windows。PowerShell 7优先；脚本含BOM以兼容Windows PowerShell读取中文。

关闭网页不会改变设备状态。需要释放串口请先点击“连接设备→断开只读连接”；关闭后台可运行 **Stop-Studio.ps1**。该脚本只关闭有匹配启动回执的项目进程，不发送硬件STOP。

## 已交付与边界

| 页面 | 本版能力 | 真实控制状态 |
|---|---|---|
| 工作台总览 | 七轴状态表、选中轴、来源和曲线 | USB只读链路已验证；按当前反馈判断 |
| 手动调试 | 普通POS、静载RAM参数、分步控制及停止/失能 | ID7明确操作后执行，其他轴不自动开放运动 |
| 单轴静载 | 当前位姿保持、前馈、增益、保持时间 | 板端控制权/模式/反馈/保护门禁审核 |
| POS带载 | 法兰相对角度、速度、到位保持 | ID7专用POS带载路径，支持已授权的试验退出线 |
| 七轴调试 | 逐轴状态与静载流程入口 | 演示展示选中轴，不宣称真实七轴验证 |
| 实时示波 | 分单位曲线、事件、首故障、播放/拖动 | 真实选中轴为主机轮询采样 |
| 驱动与维护 | 量程公开值、设备身份、维护步骤 | 驱动PMAX和EEPROM维护仍未接入页面；静载RAM下发在手动调试 |
| 记录与诊断 | 工作台/板端trace导入、JSON与CSV导出 | 文件来源声明不等于采集签名认证 |
| 实验与扩展 | 可搜索/筛选的功能登记表 | ADRC资格与远程接口单独验收 |

真实设备的STOP已接入固定板端命令；发送请求不等于失能已确认，必须核对原事务结果。发布时只运行离线验收，不自动重放此前实机动作。历史文档按各自日期阅读，不用初版“远程未接入”状态覆盖当前能力。

## 离线操作

1. 保持顶部“离线演示”。进入单轴静载或POS页面设置参数；单位、坐标、解释与输入就近显示。
2. “预检与审核”冻结本次参数，再“开始离线演示”。全局/运行区按钮立即停止当前演示。
3. 演示完成或停止后，记录保留在当前浏览器会话内，最多20条；请导出需要长期保存的文件。
4. 在“记录与诊断”导入JSON进入回放；“实时示波”可播放、暂停、拖动并切换位置/速度/转矩/温度。

所有默认值、示意图与合成曲线仅用于体验界面。ID7 POS演示使用法兰相对角度，静载使用输出轴保持；不对未标定的真实轴自动套用减速比。长POS演示的移动预览最多压缩到10秒，页面和事件明确说明。

## 真实只读连接

显式点击“连接设备”，枚举后选择USB CDC端口并连接。固定115200，DTR/RTS关闭，不自动重连；同一端口只能由一个程序持有。网关先建立板端UI会话，再执行固定只读查询；连接本身不使能、不运动。远程动作使用独立的固定白名单接口，没有通用原始命令入口。连接失败可查[COM4排查记录](docs/com4-connection-20261026.md)，端口编号以当前系统枚举为准。

设备公开响应中的位置/速度保留为“固件原始反馈”，未知模式、控制权与标定不自行推断。反馈年龄为设备报告年龄加主机等待时间；超过1000ms在显示层标过期，不作为控制安全阈值。曲线不绘制未知/过期值。单轴最多保留3000个主机轮询点，切换轴重新开始当前采样窗口；这不是4ms控制轨迹或连续CAN采集证明。通信故障关闭连接，需用户重新连接。

## 记录格式

工作台：`aethor-studio-recording/v1`，包含source、coordinate、motorId、createdAt、samples、events。
板端：`static-startup-capture-v1`（现有export_static_startup_trace.py输出）。只将accepted_rx事件中的应用采纳快照作为反馈曲线，raw_rx不被误读为已解码反馈。保留CAN十六进制、序号缺口及永久首故障槽。CAN顺序不能证明驱动内部目标接受。

文件最大2MB、最多20,000样本/5,000事件；数值、坐标、ID和时间顺序均校验。缺口断线。导入/导出全在本机，不上传云端。CSV只输出固定列和数字，未知留空。

## 验证

```powershell
# 从克隆仓库根目录开始
cd AethorDebugStudio
cd Frontend
npm ci --ignore-scripts
npm run browsers
cd ..
./scripts/Restore-Gateway.ps1
dotnet build Backend/Studio.Gateway.Tests --no-restore
./scripts/verify.ps1
```

浏览器测试只使用页面与拦截的假串口快照，不打开COM。报告和截图在artifacts中，默认不提交Git。HTTP边界可运行 `node scripts/verify-http.mjs`，要求网关已启动。

仅验证配额及会话恢复、完全不启动真实网关时，可先运行 `npm run browsers`，再从Frontend运行 `npm run test:e2e -- --config playwright.quota.config.ts`。固件生产POS配额回归从仓库根运行 `./AethorDebugStudio/scripts/run-pos-quota-regression.ps1`；需要GCC，编译器默认路径见板端测试脚本，可在对应固件runner中用Compiler参数指定。

`scripts/deploy_*.py`、`read_*trial.py`、`freeze_usability_torque_release.py`是历史现场部署/证据工具，依赖当时的候选目录、哈希及本地证据，保留用于追溯，不属于克隆后的启动步骤。不要把历史路径替换成新固件后直接执行。当前启动仅使用Start-Studio脚本；软件验收使用verify.ps1及上述离线回归。

NuGet CLI在本轮环境中无法读取在线索引，因此Restore-Gateway.ps1从NuGet官方HTTPS固定版本地址下载7个包，在项目内计算SHA256清单、使用本地源和项目内缓存恢复；没有关闭TLS/证书验证或改变全局配置。离线恢复不执行NuGet在线漏洞审计，不能据此声称.NET依赖在线审计通过。前端npm审计独立执行。

Vite8对lucide-react的两处“use client”产生构建提示；本项目是纯客户端SPA，不使用RSC，提示保留在日志中，不能描述为零告警构建。前端和网关均不依赖外部CDN或在线字体。

## 开发与后续接口

前端：Frontend/src/domain.ts（纯逻辑）、catalog.ts（能力登记）、useStudio.ts（来源隔离）、pages/（任务页面）。
网关：Backend/Studio.Gateway（固定查询、串口独占、同源保护）。绑定127.0.0.1:5179；变更HTTP请求要求自定义头并限制Origin，拒绝未知Host；没有任意路径或命令发送接口。

固件远程入口通过UI会话、能力协商和控制权仲裁复用板端保护，不能伪装LCD控制方。七轴远程动作、驱动PMAX持久化和其他未开放功能仍按能力关闭；旧固件缺少能力时不会强行接管。

参考：[已批准设计](docs/superpowers/specs/2026-10-26-debug-studio-design.md)、[实施计划](docs/superpowers/plans/2026-10-26-debug-studio.md)、[验证记录](docs/verification-20261026.md)。

技术依据：[Vite构建](https://vite.dev/guide/build)、[React](https://react.dev/reference/react)、[System.IO.Ports](https://learn.microsoft.com/en-us/dotnet/api/system.io.ports.serialport?view=net-8.0)。
