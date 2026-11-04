<!-- @file 24V恢复后的待机反馈、零点入口和60秒示波只读验收；不修改控制软件或零点。 -->
# 2026-10-27通电只读检查

用户确认24V接通、LCD待机，要求不回零、不运动。本轮没有改软件、烧录、设置/清除零点、参数提交、模式写入、STOP、DISABLE或ENABLE。

## 启动识别与恢复

初始USB连接正常，但ID7模式和身份未知：`pm_contract=0`、标准反馈年龄4294967295、寄存器无样本，CAN `rx=0 tx=4`。当前启动发生于24V断开时；身份识别失败后，待机寄存器读取因要求完整识别而未开放。只接通24V没有自动重新触发识别。

核对已部署源码后，使用已冻结工具`check_mode2_position_readonly.py --refresh-discovery --remote-capability`，仅执行一次`adrc discover motor=7`。该路径只读13个身份/模式/范围寄存器，不执行bench init的模式写入阶段。工具先确认LCD持有且空闲、控制器enabled=00/moving=0/active=0、无故障；暂时释放COM4，结束后恢复网关连接。

重查成功：模式2、fields=1fff、verified=40，寄存器契约1，boot=2234713070，coord=14。原始回执位于固件候选`output/idle-feedback/mode2-readonly-1793087207046117200.json`及`output/flange-zero/powered-readonly-discovery-20261027.log`。CAN最终rx=74、tx=78，drop/error/busoff均0。

## 实际采样与零点入口

通过网关缓存GET连续观察65.031秒，共253个不同接收时刻，253/253位置有效。位置有效年龄包含主机收到回复后的等待，为31.256～455.976ms，均小于500ms。法兰角度范围6.301565～6.302405°；坐标身份始终为上述boot/coord。速度仅为板端差分估算。

页面读取远程能力后，`当前位置设为零点`入口已开放，原始法兰角度约6.302°。软件零点仍未设置，相对零点读数为空、清除和回零审核保持禁用；未点击这些按钮。caps显示home=1、mode=2、owner=0、last/active_id/terminal_id均0、fresh=0/defer=1，表示板端可接收受保护准备意图，不能将其当成运动或实测失能资格。

253份证据：`artifacts/flange-zero-powered-65s-20261027.json`。最终只读身份/事务状态：`artifacts/flange-zero-powered-final-20261027.json`。设零入口截图：`artifacts/flange-zero-powered-panel-20261027.png`。

## 最近60秒窗口

实际示波器已切换为法兰坐标，出现约6.302°的连续位置曲线。截图横轴2131.6～2191.6秒，跨度60秒，起点已不再固定为0。只切换显示坐标，没有改动零点或控制参数。

截图：`artifacts/flange-zero-powered-scope-20261027.png`。速度为估算值；标准状态、转矩、温度及实测失能确认仍未知，界面保持缺失而非填零。

## LCD只读模型与残留问题

使用当前3cf419f2的AXF生成字段偏移，按attach-only方式读取LCD模型5次，主控始终RUNNING，没有halt/reset/resume/RAM写入。五次display_valid、snapshot_seen、graphics_running均1；last_view_ms从2836374推进到2837618，位置寄存器年龄48～225ms。模型行显示`位置14.70°`、`估算速度0.00°/s`，输出轴口径对应上位机法兰6.302°；转矩/温度/标准状态未知。该证据验证运行模型及刷新，不替代实拍LCD屏幕。

LCD回执：固件候选`output/idle-feedback/engineering-lcd-readonly-1793087730273943500.json`。

仍有两处问题，不在本轮只读检查中修改：

1. 24V后于主控启动接通时，需要本次手动只读重查才能恢复身份与待机采样；尚无自动恢复。
2. 健康提示卡用250ms更新的studio.now判断刚收到的新快照；now早于receivedAt时，feedbackAge返回null，可能误显示“超过500ms”，而实际位置读数使用Date.now正常。已用同一119/230ms样本在落后10ms时复现误报，使用当前墙钟则显示正在更新。证据：`artifacts/flange-zero-health-clock-evidence-20261027.json`。

检查结束保持COM4只读连接，24V为用户已接通状态，未执行回零或电机动作。
