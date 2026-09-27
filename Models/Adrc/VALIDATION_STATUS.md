# 离线模型验证状态：PASSED（未进行硬件验收）

日期：2026-09-21。用户开启 Automation Server 后，复用现有 MATLAB R2026a 会话，完成实际模型生成、仿真、ERT C 生成和代码重放。

## 最新有效验收

- 运行：`output/adrc/models/com_20260921T081022747Z`，`pipeline_report.json` 为 `stage=completed`、`passed=true`、`verificationHashesMatch=true`；`job_status.json` 确认成功且恢复会话环境。
- 已交付三个真实 `.slx`：见 [交付入口](VerifiedOffline/README.md)。控制器 MIL 2,002 样本通过，最大归一化误差 `4.826142429026348e-7`；38 个合成闭环场景通过；真实生成 C 的 59,040 个重放样本与 MIL 最大归一化误差为 0。
- 四个维护源、三个模型、生成 C/H 的文件集合与 SHA-256 在验收前后保持一致；发布脚本再次复核后建立独立 `MDK-ARM/ADRC-Bench.uvprojx`。
- 最新目标完整 ARMCC 编译链接通过：`output/adrc/verification/final_target_20260921T081022747Z/build.log`，0 errors / 0 warnings；`target_report.json` 保存目标和 AXF/HEX/MAP 哈希。
- 源参数 `CombineOutputUpdateFcns=off` 修正了 plant 模型引用输出/更新合并问题；38 场景指标采用预分配结构体。正常 EOF 哈希读取已通过真实流水线验证。
- 以上是合成对象上的离线验证和固件链接，不证明真实 S3519 参数、任务实时性、驱动看门狗或电机运行。未烧录、未启用电机。

以下保留早期故障记录，均为恢复前的历史状态，不代表最新结果。独立进程启动的间歇性原生崩溃尚未确认根治；目前成功路径是复用正常桌面会话。

## 历史：电脑重启后的复测（15:51—15:54）

- `output/adrc/models/20260921T075104562Z`：MATLAB 成功执行脚本，流水线在源文件 SHA-256 检查处失败。原因是分块 `fread` 越过正常 EOF 后，`ferror` 被当作文件损坏。已改为先取文件长度、按剩余字节精确读取，并验证每次实际读取数量；修正版通过 `mlint`，但尚未再次运行到此步骤。
- `output/adrc/models/20260921T075241224Z`：启动退出 1，无 `job_status.json`，未进入修正版流水线。
- `output/adrc/session_20260921T075320924Z`：独立持久 worker 也未写出 `ready.json`。新崩溃记录 `matlab_crash_dump.11692-1` 仍是 GTP 后台线程 Access violation、RAX=0、指令地址末尾 `1aa7`。
- 正常用户上下文查询现有 `Matlab.Application` 自动化对象，返回 `MK_E_UNAVAILABLE`。系统仍有此前启动的 MATLAB 进程，但不能据此断言其桌面状态或命令窗口可用。

结论：重启使一次启动成功，但尚未消除间歇性原生崩溃。当前仍没有通过的 ADRC `.slx` 集、生成 C 或 MIL。可在用户确认正常的桌面会话中开启 Automation Server 后复用该会话，避免继续重复新建进程；不关闭用户现有进程。

## 已执行与尚未执行

| 项目 | 证据状态 |
| --- | --- |
| 三个 MATLAB 源文件静态分析 | R2026a `bin/win64/mlint.exe -id` 检查全部 `.m`，退出 0，零诊断消息 |
| Unit Delay 参数修正 | 首次实际运行报不支持 `OutDataTypeStr`；删除该参数后后续运行越过此步骤 |
| controller/plant SLX 保存 | 两次运行保存了中间 SLX；validation 更新失败，不能称完整模型通过 |
| 人工代数环修正 | plant 输出改为存储状态；再按官方文档关闭 `SingleOutputUpdateFcns`、开启 `ModelReferenceMinAlgLoopOccurrences`。最终配置尚未运行 |
| ADRC 控制器 C 生成 | 未执行至 `slbuild`；没有真实 `adrc_controller.h/.c` |
| 控制器和闭环 MIL | 未执行，报告保持失败 |
| 生成 C 逐样本一致性 | 未执行；直接 GCC 检查重放器因缺少 `adrc_controller.h` 而退出 1，未伪造头文件或算法 |
| 硬件相关操作 | 未连接、未刷写、未启用电机 |

单独的独立数值预检用于检查所选合成测试阈值是否合理，不是 Simulink 仿真或生成代码证明，未计入通过状态。

## 启动与模型运行记录

工作树根目录：`D:/download/TCG/Aethor_robo_fw/.worktrees/s3519-adrc`。下列路径均相对工作树。

| 输出或日志 | 实际结果 |
| --- | --- |
| `output/adrc/models/pipeline_matlab.log` | 进入脚本；Unit Delay 参数错误；MATLAB 退出 1 |
| `output/adrc/models/pipeline_matlab_second.log` | 启动前退出 1，日志为空 |
| `output/adrc/models/20260921T065106212Z` | 公共隐藏启动器启动前退出 1；无 job_status，日志为空 |
| `output/adrc/models/20260921T065512426Z` | 从早先成功目录启动并进入脚本；validation 人工代数环，job_status 记录失败 |
| `output/adrc/models/20260921T065720651Z` | 使用相同启动目录仍在启动前退出 1；无 job_status |
| `output/adrc/models/20260921T065846667Z` | shell 工作目录改为顶层 TCG，仍启动前退出 1 |
| `output/adrc/models/20260921T065914716Z` | 进入脚本；plant 已输出 UnitDelay 状态，Model 引用默认输出/更新合并仍导致人工环 |
| `output/adrc/models/20260921T070211979Z` | 添加官方人工环配置后，MATLAB 启动前再次崩溃，未运行修正版 |

启动失败时，用户 TEMP 中的 `matlab_crash_dump.34508-1`、`31848-1`、`13460-1` 等记录后台 GTP 线程原生访问冲突，RAX=0、指令地址尾部为 `1aa7`。早期带符号堆栈指向 MATLAB Home Session Manager / Recent Artifacts Service。更深内部根因尚未确定；没有证据说明这是控制器算式、许可证、GPU 或 Java 的错误。

正常用户上下文、直接/隐藏启动、`-r`、`-nodesktop`、`-noFigureWindows` 和不同工作目录曾有成功与失败，不能把某一次成功组合当成稳定修复。主任务后续尝试的持久 worker 与 automation 启动也未建立稳定运行条件；未修改全局配置、安装目录或服务。

## 与最小 probe 成功的区别

早先独立诊断目录 `D:/download/TCG/output/adrc_environment_20260921` 中，`environment_report.json` 记录最小单输入/增益/延迟模型的 GRT 与 ERT 成功，日志含 `ENVIRONMENT_DIAGNOSTIC_COMPLETED`，该次 MATLAB 退出 0。该最小 ERT C 也通过了 ARMCC 5 对象编译。

这些事实证明本机安装具有一次成功的最小生成能力；它们**不能证明 ADRC 模型、真实控制器 C、MIL、ARMCC 完整固件链接或硬件运行通过**。两个证据集必须分开引用。

## 恢复条件

稳定 MATLAB 运行后，重新执行 `run_adrc_pipeline`，要求 `pipeline_report.json` 的 `stage=completed`、`passed=true`，并同时确认公共启动器的 job 成功与进程退出 0。随后依据真实生成头文件适配固件并进行 ARMCC 编译；不得用预期接口名、历史 probe 或手写算法补足这道验收门。
