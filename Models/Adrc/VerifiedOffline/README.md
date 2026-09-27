# 已验证的离线模型交付

当前交付版本：`20260921T081022747Z`，MATLAB/Simulink R2026a。

| 文件 | 用途 |
| --- | --- |
| [adrc_controller.slx](20260921T081022747Z/adrc_controller.slx) | 4 ms 离散 PI/LADRC 控制器；用于生成 STM32 集成的算法 C |
| [adrc_plant.slx](20260921T081022747Z/adrc_plant.slx) | 合成的转矩—速度—位置对象；尚未代入实测辨识参数 |
| [adrc_validation.slx](20260921T081022747Z/adrc_validation.slx) | 闭环仿真，包括限幅、斜率、量化和反馈延迟 |
| [pipeline_report.json](20260921T081022747Z/pipeline_report.json) | 完整通过报告，包含源文件、模型、生成代码的原始路径和哈希 |
| [job_status.json](20260921T081022747Z/job_status.json) | 实际 MATLAB 执行及会话恢复状态 |

打开时，将 MATLAB 当前文件夹切换到本版本目录，让 validation 能解析同目录的 controller/plant 模型引用。若已打开其他版本的同名模型，应先自行保存需要保留的修改，再切换版本；交付过程没有覆盖用户已打开的模型。

通过记录：控制器 MIL 2,002 样本；合成闭环 38 场景；真实生成 C 重放 59,040 样本，C/MIL 最大归一化误差 0。模型副本与原始验收模型 SHA-256 相同。历史版本 `20260921T080454015Z` 保留供追溯；当前版本改善引用块端口显示。

完整重建和仿真入口见 [上级 README](../README.md)。原始逐样本记录和生成代码位于工作树 `output/adrc/models/com_20260921T081022747Z`。固件快照由该报告发布，目标 `MDK-ARM/ADRC-Bench.uvprojx` 已完成 ARMCC 链接，0 错误、0 警告。

这些结果不构成硬件许可。b0、阻尼、wc、wo 等当前均为仿真示例，尚未辨识实际 S3519；硬件资格保持未启用。当前无需电机供电。
