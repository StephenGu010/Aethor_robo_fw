<!-- @file 已批准零点、60秒示波与采样策略的实施/验证计划。 -->
# 法兰零点、60秒示波和自适应轮询实施计划

> For agentic workers: 使用subagent-driven-development按下列边界实施；每个任务先写失败回归，再最小实现，检查规格和代码质量。

**Goal:** 软件参考零点、受保护固定目标回零、最近60秒示波及有证据的轮询策略，离线验证后部署。

**Architecture:** ID7零点随当前连接及坐标身份有效，原始遥测不覆盖；独立回零协议在板端新鲜起点处计算目标差。现有运动保护、单次事务与STOP优先级不变。固件新候选在D:/download/TCG/flange-zero-candidate-20261027，旧candidate保持原样。

**Tech Stack:** React/TypeScript、.NET8、STM32H723 C/Keil、host C/Python tests、Playwright API拦截。

## 任务1：固件固定目标回零及采样策略调查

- [x] 新增失败测试：固定绝对输出轴目标在准备前后起点变化时不漂移；身份过期、超范围、非有限值拒绝；STOP和新鲜反馈门禁保持。
- [x] App/DebugUi/debug_ui_remote_app.inc、请求类型和原POS执行器增加独立`home`能力。现有八字段协议中home仅将delta字段解释为绝对输出轴目标，其他操作保持相对语义；能力home=1，目标绑定当前boot及坐标参考。新建回归验证实际执行目标。
- [x] 调查App/aethor_app.c的寄存器轮询间隔和负载，统计每秒帧数与经典CAN上界。需要调整时让已确认空闲低频，动作准备/保持/运动维持原频率；不降低保护反馈频率，不把未知状态当空闲。
- [x] Tests/host对应测试通过，tools/build_pos_load_test.ps1构建新独立产物。

## 任务2：上位机软件零点和回零操作

- [x] Frontend/src/flangeZero.ts及tests/flange-zero.test.ts先测试角度偏移、速度不偏移、有限数、跨连接/启动/坐标身份失效。
- [x] useStudio共享零点，RemoteControl增加设零/清除/回零速度与审核按钮；保持静载/POS切页一致，刷新失效，执行/未知结果禁止改变参考。
- [x] remoteControl.ts/remoteWorkflow.ts支持home能力、新旧固件分支与单次事务。home草稿的delta为绝对目标；mode准备不漂移目标。缺少新能力不得下发回零。
- [x] Browser测试拦截API，验证设零无动作、相对坐标和参数、旧固件禁用、失败锁定、回零单发。

## 任务3：网关协议和60秒示波

- [x] Backend/Studio.Gateway/RemoteControlProtocol.cs和RemoteControlTests.cs先测试home固定八字段生成与非法值拒绝；保留白名单与旧操作兼容。
- [x] Frontend/src/components/Chart.tsx增加显式实时60秒窗口；对应调用只对live开启。原始recording导出不截短，回放保留完整记录。窗口横轴为当前时间前60秒，纵轴和悬停只取窗口内点。
- [x] 图表失败回归：70秒记录无0秒点、只取最近60秒、未知值和间断保持；边界点不生成虚假插值。

## 任务4：集成、复核与部署

- [x] npm test、额外新单元测试、npm run build；dotnet run测试程序；相关Playwright隔离测试。
- [x] 独立规格与质量复核，修复后针对性回归；记录轮询决定、开销估算及未验证的硬件层。
- [x] 冻结全新构建/输入manifest并验证HEX/ELF一致；复用USB/SWD配对、1MiB备份和完整读回比较。
- [x] 烧录前需要当前现场24V断开和独立承托状态；用户已授权烧录，无需重复询问授权，但不能用此前接通24V的状态执行复位。
- [x] 烧录完成只读核验，物理回零/电机动作不在本轮执行范围。

逐文件编辑，不删除文件，不批量暂存或提交现有改动。验证失败逐项修复，不以构建通过替代物理资格。

实施完成：见docs/flange-zero-release-20261027.md。最终固件3cf419f2已烧录并完整读回，24V保持断开；实机运动/回零不在本次执行范围。
