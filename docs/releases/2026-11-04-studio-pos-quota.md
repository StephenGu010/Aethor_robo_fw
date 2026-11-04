<!-- @file LCD/POS固件与Windows上位机源码发布说明；区分离线验证、部署和实机承载资格。 -->
# LCD/POS带载与Aethor Debug Studio源码发布

发布目标为`StephenGu010/Aethor_robo_fw`的main分支，基于`06dc439`整理。此前main主要保留较早LCD/ADRC代码；本次同步本地已维护的LCD静载/POS、反馈、远程控制和保护逻辑，并首次收录独立上位机。旧模型、文档、固件归档及七轴功能不删除，源码更新不代表所有机械轴已完成实机资格验证。

## 目录与版本

| 入口 | 用途 |
| --- | --- |
| `MDK-ARM/LCD-POS-Load-Test.uvprojx` | 当前LCD/POS带载、静载及远程维护组合目标 |
| `MDK-ARM/LCD-SingleStatic-Test.uvprojx` | 独立单轴静载目标 |
| `MDK-ARM/LCD-MIT-ADRC.uvprojx`、`ADRC-Bench.uvprojx` | 保留实验入口，补齐现有源码依赖；不授予ADRC硬件资格 |
| `AethorDebugStudio/` | Windows本地前端、.NET串口网关、离线回归和手册 |
| `App/DebugUi/`、`App/Motor/` | 会话仲裁、静载/POS流程、待机反馈、坐标及转矩配额 |

2026-11-03已烧录固件HEX SHA-256：`209e5fc5788078e02d1dba0348a5513efd6c6ac02f0f25e3b273b080e489b118`。本次从发布树重新构建POS目标，HEX与该部署版本相同。原冻结清单共934项输入，其中908项为维护源码/工程/工具输入，另26项为本地候选清单及历史output文件；源码条目摘要见[发布输入清单](2026-11-04-source-manifest.json)。906项维护输入与冻结版本完全一致，另外两项是本次补齐依赖的ADRC工程生成器及其测试，运行时源码未变。本次还补入三个既有专用工程及对应静载RTE头文件。未把本地候选目录或原始现场数据作为克隆后的依赖。

## 当前功能和约束

- LCD与上位机共用板端控制权、模式准备、预检、反馈与停止保护。上位机可执行ID7静载、POS带载和静载RAM参数下发；没有通用任意串口命令入口。
- POS角度与速度显示为法兰口径，RID50按已核实契约换算；静载保持当前位置。转矩始终使用电机输出轴口径，不折算法兰负载能力。
- 法兰软件零点按设备/坐标身份绑定，提供明确的设零和回零操作。保存页面参考不等于修改驱动编码器绝对零点，回零属于运动。
- 实时示波显示最近60秒；待机降低位置查询频率，运动期间按现有控制需求采样。模式2待机的标准速度/转矩/温度仍可能未知，位置寄存器不能替代这些证据。
- 24V恢复后的身份重识别、健康卡反馈年龄、未确认失能及故障会话恢复链路已纳入当前实现；异常不得通过重复启动或网页刷新掩盖。
- ID7静载/POS带载试验退出线可选3.50～6.24 N·m。默认未授权时按3.50基线保护；冷却且承托后可显式单次失能采样，通过原门禁再一次应用所选值并读回。STOP、断连、身份变化及未知维护结果均阻断后续写入。
- 保留连续超额10秒、累计175.616 (N·m)²·s、任一路温度超过55℃退出、恢复时两路≤35℃、实验20ms反馈/服务要求及原有位置/速度/失能保护。6.24恒定超额的满预算理论约6.58秒，不是厂家许可持续时间或驱动限流值。

操作步骤见[完整手册](../../AethorDebugStudio/docs/user-manual.md)与[配额按钮说明](../../AethorDebugStudio/docs/quota-operations.md)。历史初版文档保留当时状态，应按日期阅读。

## 从克隆仓库启动上位机

Windows需要Node.js 22.12+和.NET 8 SDK及对应ASP.NET运行时。在`AethorDebugStudio`目录双击`Start-Studio.cmd`，首次下载固定依赖、构建后打开`http://127.0.0.1:5179`。已有构建可用`./Start-Studio.ps1 -SkipBuild`。默认离线演示，不自动连接串口或开启配额。

若环境中同时安装多个.NET根目录，确认`dotnet --list-runtimes`包含8.x。启动脚本只为网关子进程指定与选中的dotnet命令一致的根目录。普通用户桌面启动即可；Low完整性沙箱可能被Windows拒绝访问COM，不能据此认定端口被占用。

连接时选择实际枚举到的USB CDC端口；历史COM4不是固定设备身份。停止后台前先完成原流程、核对失能并释放控制权，再断开串口。`Stop-Studio.ps1`不发送硬件STOP；关闭浏览器也不等于机械臂已停止。

## 离线构建与检查

Keil目标使用已注册的ARMCC 5.06 update 4。默认Keil/GCC路径沿用维护脚本，可通过相应脚本参数适配本机。固件构建命令仅编译，不烧录：

```powershell
# 仓库根目录
./tools/build_pos_load_test.ps1
./tools/build_single_static_test.ps1
./Tests/host/run_torque_envelope_tests.ps1
./Tests/host/run_remote_fault_release_tests.ps1
./Tests/host/run_single_static_load_tests.ps1
./Tests/host/run_pos_load_app_tests.ps1
./AethorDebugStudio/scripts/run-pos-quota-regression.ps1
```

上位机离线检查：

```powershell
cd AethorDebugStudio/Frontend
npm ci --ignore-scripts
npm test
npm run build
npm run browsers
npm run test:e2e -- --config playwright.quota.config.ts
cd ..
./scripts/Restore-Gateway.ps1
dotnet build Backend/Studio.Gateway.Tests --no-restore
dotnet Backend/Studio.Gateway.Tests/bin/Debug/net8.0/Studio.Gateway.Tests.dll
```

配额浏览器配置只启动隔离前端，测试API全部拦截，不打开真实串口。NuGet恢复使用官方HTTPS固定包和项目内源，当前项目关闭在线NuGet审计；不能把离线构建通过描述为依赖无漏洞。前端保留lucide的两处`use client`构建提示。

本次发布树验证结果（2026-11-04）：

| 检查 | 结果 |
| --- | --- |
| 前端单元测试、类型检查与生产构建 | 100/100通过；构建成功，保留两处依赖提示 |
| .NET网关 | 43/43通过，构建0错误0警告 |
| 配额/会话/STOP浏览器回归 | 25/25通过；API模拟，不打开设备 |
| 固件GCC生产路径回归 | 配额、POS ENABLE/MOVE/HOLD正负转矩、远程故障释放、文本协议、静载locked/single/combined和POS带载通过 |
| ADRC工程生成器 | 13/13通过，包含新增源依赖断言 |
| 独立静载构建器 | 5/5通过，模拟外部编译器，不调用硬件 |
| Keil四工程重建 | LCD-POS-Load-Test、LCD-SingleStatic-Test、LCD-MIT-ADRC、ADRC-Bench均0错误0警告 |
| POS HEX比对 | 与已部署209e5fc5完整SHA-256一致 |
| 提交范围和文档 | 独立复审关闭发现的问题；未加入现场日志、备份、依赖缓存或新构建二进制；新增/更新说明的相对链接有效 |

首次Keil受限进程未识别用户已注册的ARMCC5；改用普通用户编译环境完成上述构建，工程编译选项未改。浏览器首轮用例均通过后停在测试服务器回收，使用同一隔离服务器复跑得到25通过、退出码0，不把重复执行累计为50项。`git diff --check`保留了原维护源码末尾空行及Keil生成RTE头部的空白提示，没有为了消除提示改写已冻结运行时源码。

发布期间没有设备动作、烧录或再次恢复实机配额。本次只重建上述四个维护目标，不将其扩大为所有历史目标、模型或实机资格均已重新验收。

## 实机证据与限制

固件209e5fc5已在2026-11-03完成522,492占用字节Flash读回验证。后续经用户明确确认承托和冷却，通过与页面相同的生产函数执行一次ID7失能、一次5.46 N·m配额应用，读回`authorized=1 peak_mnm=5460 used=0`；没有使能或POS运动。待机标准反馈再次过期时，配额读回仍为授权，但再次恢复门禁变为32，两个状态需要分别显示。

这次单次维护成功不证明高转矩带载、10秒连续工况、温升或ADRC闭环已通过。主控重启默认关闭高转矩；旧实机确认和旧进程号不能当作当前现场状态。

原始日志、设备标识、Flash备份、二进制构建产品、依赖缓存和临时工作树留在本地。文档提及的`artifacts/`、`output/`是现场证据位置，不随Git上传。`tools/deploy_*`及`AethorDebugStudio/scripts/deploy_*`、历史冻结和实机采集脚本保留追溯用途，依赖当时的现场身份/路径/哈希，不能作为新克隆的自动部署入口；本发布不自动运行它们。
