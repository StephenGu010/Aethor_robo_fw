# S3519 离线 Simulink 控制器

本目录包含可重建的模型源脚本。MATLAB 运行产物写入 `output/adrc/models/`，不写硬件资格、串口或 CAN，不包含可解锁硬件的仿真标志。

## 入口与产物

已验证的三个 `.slx` 见 [交付入口](VerifiedOffline/README.md)。本机独立 MATLAB 启动存在间歇性原生崩溃；成功路径是在用户启用 Automation Server 后复用现有 R2026a 桌面会话，通过 `tools/run_adrc_session.m` 调用流水线。此函数恢复当前目录、搜索路径、环境 PATH、随机数状态和生成目录配置，不退出桌面。已加载的同名模型会阻止重建，以免覆盖用户修改。独立启动入口 `tools/run_adrc_matlab.ps1` 保留用于环境正常时调用。以实际 `job_status.json` 和 `pipeline_report.json` 为准。

流水线按顺序执行：

1. `build_adrc_models` 创建 `adrc_controller.slx`、`adrc_plant.slx`、`adrc_validation.slx`，并更新模型检查结构。
2. 仅对 controller 执行 ERT `slbuild`，产生真实自动生成 C。
3. 模型与独立双精度递推式比较，检查复位、正反向、丢样、NaN/Inf 和复位恢复。
4. 运行 PI/LADRC × b0 的 0.5/1/2 倍 × 0/1/2 周期附加反馈延迟 × 有/无噪声，共 36 个合成闭环场景；另加两个饱和恢复压力场景。
5. GCC 编译真实生成 C 与 `adrc_generated_replay.c`，对开环和全部闭环记录逐样本比对。归一化误差 `abs(C-MIL)/max(1,abs(MIL)) <= 1e-4`。
6. 重新计算维护源、三个 SLX 及生成目录全部 `.c/.h` 的 SHA-256；文件集合或任一内容变化均拒绝通过。

模型使用标准 Simulink 离散块，没有 MATLAB Function Block、MEX 依赖或手写 C 控制算法。重放 C 文件仅装载端口、调用生成入口、比较结果。GCC 需要在 PATH 中可用。代码生成关闭示例 main 和动态对象，不需要编译宿主 MEX。

流水线保存阶段报告、SLX、仿真参数 MAT、闭环完整时间序列 MAT、生成 C、逐样本 CSV、GCC 日志与比对 JSON。全部中间失败保留 `passed=false` 及错误信息，不能作为硬件验收通过证据。

开始执行时，`verificationSources` 记录本目录四个维护源（`build_adrc_models.m`、`run_adrc_pipeline.m`、`adrc_simulation_parameters.m`、`adrc_generated_replay.c`）的绝对 `path` 和 `sha256`。真实代码生成完成后，`verificationArtifacts` 用同一结构记录三个 SLX 和 BuildDirectory 全部 `.c/.h`。MIL 与 C 重放完成后重新枚举并核对文件集合和哈希，只有全部一致且原验收通过才设置 `verificationHashesMatch=true`、`passed=true`。哈希使用公共 Java `MessageDigest` 流式读取二进制文件，因此要求启用 JVM；此检查已在最新完整流水线中实际通过。固件准备入口还必须在写入前独立复核这些哈希，不能只信任报告布尔值。

## 控制器接口

固定采样周期 0.004 s，算法数据与状态为 single/float32。输入顺序与 `App/Adrc/adrc_experiment.h` 一致：

| 端口 | 类型 | 语义 |
| --- | --- | --- |
| mode | uint8 | 1=PI，2=LADRC；监督器拒绝其他闭环模式 |
| reference_rad_s | single | 监督器已斜坡处理的输出轴速度参考 |
| velocity_rad_s | single | 输出轴速度反馈 |
| prev_sent_nm | single | 上周期真正发送且经协议反解的名义转矩 |
| b0 | single | 已冻结对象增益，必须有限且非零 |
| wc_rad_s / wo_rad_s | single | 已冻结正控制/观测带宽 |
| reset | uint8 | 非零时 z1=当前速度，z2=0，PI 积分=0，前周期 raw 记忆=0 |
| new_sample | uint8 | 仅新反馈执行 ESO 校正与 PI 误差积分 |

输出为 `torque_raw_nm`、`z1`、`z2`，均 single。控制器不做幅值/斜率/协议量化，不访问执行器。模式与参数必须在运行前冻结，输入合法性、有限输出检查、停止与失能由监督器/适配层负责。NaN/Inf 新反馈会产生非有限结果，适配层必须拒绝；复位可恢复状态。

生成配置为 `Nonreusable function`，用于一个序列化的单轴实例。预期公开入口为 `adrc_controller_initialize()`、`adrc_controller_step()`，端口对象为 `adrc_controller_U` 和 `adrc_controller_Y`；实际生成头文件是适配依据，不能仅凭预期名称集成。此配置不支持多轴同时共享全局状态。

## 离散算法与 PI 初值

ESO：`z1p=z1+Ts*z2+b0*Ts*prev_sent`，`z2p=z2`；`p=exp(-wo*Ts)`，`l1=1-p²`，`l2=(1-p)²/Ts`。仅新样本时，以 `velocity-z1p` 校正两状态。复位直接使用当前速度，不执行预测或校正。

`kc=(1-exp(-wc*Ts))/Ts`，LADRC 原始输出为 `(kc*(reference-z1)-z2)/b0`。

PI 设计初值采用理想积分对象的临界阻尼参数化：`Kp=2kc/b0`、`Ki=kc²/b0`、`Kaw=kc`。积分每步加入 `Ts*Kaw*(prev_sent-previous_raw)`，新样本再加入 `Ts*Ki*(reference-velocity)`。输出 `Kp*error+integral`。这只是模型设计初值，没有实测调参依据；相同 wc 不表示两个控制器具有相同实际带宽，也不证明公平最优。

## 合成对象与验收边界

`adrc_plant` 是 `velocity[k+1]=velocity[k]+Ts*(actual_b0*torque-damping*velocity+disturbance)` 与位置积分器，完全由 `adrc_simulation_parameters.m` 的合成参数驱动。输出始终为当前存储状态，plant 的同步复位清零下一状态；每次新仿真从零状态开始。此连接消除模型引用的人工直接馈通，没有额外添加反馈或控制延迟。b0=10、阻尼=1、wc=4、wo=20 均不是 S3519 辨识结果。

validation 使用同一外层斜率限制、幅值限制、合成均匀最近舍入量化，以及实际限幅/量化输出的一周期回馈。量化步长 0.001 是合成压力测试值，**不是已确认 MIT 编码量程或舍入规则**；生产 MIT 编码/反解需由协议测试和实际量程证据验证。

36 场景检查轨迹有限、共同幅值/斜率限制，以及两个名义无噪声无附加延迟场景最终误差 <=0.01 rad/s。两个额外压力场景直接施加参考跳变以触发饱和，要求饱和确实发生且归零后最终速度 <=0.02 rad/s；这种参考跳变不属于硬件指令流程。记录全部场景最终误差、峰值速度、raw/sent 峰值和 RMS 跟踪误差；通过不意味着所有鲁棒场景都达到实机指标。

未包含实测反馈基础延迟、真实负载扰动或转矩标定，最新独立 ADRC-Bench 目标已通过 ARMCC 链接，但尚未证明任务耗时、运行时栈余量、硬件看门狗与电机运行。硬件资格只能由独立实测流程建立。

## 公共 API 依据

- [程序化创建 Simulink 模型](https://www.mathworks.com/help/simulink/programmatic-modeling.html)
- [代码接口包装配置](https://www.mathworks.com/help/rtw/ref/codeinterfacepackaging.html)
- [模型引用人工代数环与输出/更新分离](https://www.mathworks.com/help/simulink/ug/modeling-considerations-with-algebraic-loops.html)：plant 关闭 `CombineOutputUpdateFcns`，开启 `ModelReferenceMinAlgLoopOccurrences`；controller 保留单一 step 接口。
- [Windows MATLAB 启动参数](https://www.mathworks.com/help/matlab/ref/matlabwindows.html)

## 当前验证状态

2026-09-21：**离线模型、真实生成代码和独立固件链接通过**。最新运行 `output/adrc/models/com_20260921T081022747Z` 完成三个模型更新、ERT 生成、2,002 样本控制器 MIL、38 个合成闭环场景，以及 59,040 样本真实生成 C 重放（最大 C/MIL 归一化误差 0）。维护源与产物哈希一致，现有 MATLAB 会话环境恢复成功。

`MDK-ARM/ADRC-Bench.uvprojx` 使用通过报告对应的真实生成代码，ARMCC 完整编译链接 0 错误、0 警告。尚未烧录或运行电机，仿真示例参数不能解锁硬件。早期失败目录保留为诊断历史，不作为交付版本。详见 [验证状态与环境证据](VALIDATION_STATUS.md)。
