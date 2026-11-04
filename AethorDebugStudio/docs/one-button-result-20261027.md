<!-- @file 单按钮单轴控制的交付与证据；软件验收、部署和真实电机运动分别记录。 -->
# ID7 单轴静载、POS带载及手动调试

上位机服务已更新：`http://127.0.0.1:5179/`，网关PID4172。3d731bd1已烧录并完整回读；用户随后接通24V，COM4只读身份/显示核验通过，尚未执行电机动作。实时模式下，单轴静载、POS带载测试、手动调试三个导航均进入设备控制页面。页面加载和编辑参数不发送动作。

## 操作流程

- 单轴静载：设置Kp、Kd、前馈和保持秒数，点击“按参数启动一次”。软件按设备能力完成所需模式切换、失能预检、RAM参数配置、控制权申请、限时保持及释放控制权。
- POS带载：设置输出轴相对角度、速度和到位后保持秒数，点击同一启动按钮。使用专用`POS_LOAD_TEST`执行器；普通POS相对运动是手动入口中的独立操作。
- 手动调试：选择固定操作并设置相关参数，可单按钮启动，也保留能力查询、审核和单次提交入口。支持固定MIT/POS模式切换、静载RAM配置/撤回、预检及控制权操作。

界面角度为电机输出轴度，速度为度/秒，保持时间为秒；协议由软件转换为输出轴rad、rad/s和ms。Kp默认120、Kd默认0.30，保留80至200、0.20至0.40范围。POS带载不使用MIT增益和前馈。每个操作只显示所需参数。

## 控制边界与审查修复

板端`defer`只允许原LCD支持的预检意图，不伪造反馈。身份、模式、量程、控制权及既有预检继续由板端验证；实际运动前仍需DISABLE后的新标准反馈。静载配置和保持没有反馈豁免。没有查明的真实速度、转矩、温度来源仍保持未知，估算速度不参与安全准入。

STOP/失能调用立即递增全局启动代次，取消还没提交的后续步骤。自动、手动和恢复查询共用终态验证；已准入但缺少必要失能证明的结果持续锁定。未知原操作不会继承上一事务的宽松规则，连接变化前的迟到恢复回复被丢弃。超时不重发动作，恢复不自动续跑原流程。

独立只读审查发现的上述P1均已修复并复审，保留红测和绿色回归证据。审查使用[requesting-code-review技能](C:/Users/Aethor_ca/.codex/skills/superpowers/requesting-code-review/SKILL.md)，没有访问硬件。

## 离线证据及运行状态

- 前端单元51/51：`artifacts/one-button-review-unit-20261027.log`。
- 全浏览器27/27：`artifacts/one-button-final-browser-20261027.log`。参数按操作显示和秒到ms转换修改后，相关9项浏览器及单项完整静载流程再次通过。
- 网关32项测试通过；真实facade测试证明准确POS带载/SET_MODE/DISABLE预检映射，缺反馈仅可进入预检而无法通过实际执行检查。
- 原POS带载完整流程回归通过，含缺失/错误TX回执、STOP、饱和清理、峰值和范围保护。日志位于固件`output/idle-feedback/remote-deferred-pos-regression-20261027.log`。
- 新固件Keil构建0错误、0警告；HEX/AXF对应515312字节一致，642个冻结输入校验通过。
- 已更新的网关健康检查正常，烧录前串口断开；烧录后COM4只读连接及新能力查询通过，记录在`artifacts/one-button-postflash-gateway-20261027.json`。更新前备份在`artifacts/host-before-one-button-20261027/`。
- 当前实际界面截图：`artifacts/one-button-static-final-20261027.png`。

## 已部署固件与未完成验收

固件目录：`D:/download/TCG/idle-feedback-candidate-20261026`。

HEX SHA256：`3d731bd1478935de2439ba52715a3dc352858a765d7961b663b581a787867cc2`。

AXF SHA256：`be69b0e091dadcf2267786cd31ab2eeb74ece189d8e7644e1de77aec72299e2f`。

冻结清单：`output/idle-feedback/one-button-release-20261027.json`。`tools/deploy_one_button_poweroff.py`默认仅离线核验，烧录须带两个显式参数；复用USB/SWD配对、旧镜像核对、完整备份及逐字节回读流程。用户本次确认24V断开后已部署本版本，515312字节完整回读一致、主控RUNNING。实际回执为固件目录下`output/adrc/hardware/20261027T051437548633Z_poweroff_deploy/report.json`；旧镜像514384字节核对通过，1 MiB备份SHA256为`d5cc05e5f80634033b86ac443737f0011ba028af023b0ad0b2578c52e98bf3b4`。冻结清单中的flashed=false保留冻结时含义。

断电后USB查询确认boot=906780155，新能力字段正常返回，fresh/mode/动作能力均0；未供电数据保持未知。LCD被动SWD读取确认5次模型有效、显示任务运行、刷新计数递增，CPU始终RUNNING，未halt/reset/resume。证据分别为固件目录下`mode2-readonly-1793078628182546700.json`及`engineering-lcd-readonly-1793078661348610200.json`（均位于`output/idle-feedback/`）。LCD显示文本为未知，不是物理屏幕照片；以上也不证明电机物理失能、静止或运动合格。

## 通电只读核验

ID7只读发现确认模式2、fields=1fff、verified=40；20次USB回复的0x50位置年龄18～225ms，全部有工程单位契约及板端估算速度，CAN drop/error/busoff均0。LCD被动运行模型五次读取均有效并持续刷新，文本显示工程角度和“估算速度”。转矩、温度及标准状态反馈仍未知。

上位机连续15.9秒采到60个不同回复，位置工程样本60/60有效，年龄包含主机等待后为21～455ms；见`artifacts/one-button-powered-verified-samples-20261027.json`。较早`one-button-powered-live-samples-20261027.json`因PowerShell日期隐式转字符串丢失毫秒，年龄统计无效；记录保留，新采样采用DateTimeOffset直接转换。网页没有该错误转换。

真实示波截图为`artifacts/one-button-powered-position-20261027.png`及`artifacts/one-button-powered-speed-20261027.png`。能力查询确认专用load、模式switch及预检意图defer均支持；fresh仍0。未发ENABLE/HOLD/STOP/DISABLE、模式写入、参数下发或运动。

当前24V已接通，仍需另行授权的单次失能反馈检查，以及条件满足后的限定单次运动验收。“先不要执行电机运动”的约束仍有效，整体目标未完成。

## 用户随后主动操作，代理保持只读

只读观察末尾出现了控制事务及位置变化，用户明确回复确认由其主动启动测试。网关缓存回执为`id=2034495321 result=completed accepted=1 disabled=1`，随后软件状态回到enabled=00/moving=0/active=0，当前位置寄存器约26.805转子rad。记录为`artifacts/one-button-passive-state-change-20261027.json`。这证明出现了用户操作及完成回执；尚未完整取得该次原始参数和关联的动作类型，不把末笔回执自动当成某种测试的完整验收。

页面草稿随后显示POS带载、相对位移0°、速度15°/s、保持10s，并有零位移校验提示；该草稿不能代替启动前的参数。页面状态保存在`artifacts/one-button-operator-action-ui-20261027.txt`。

另外通过无halt/reset/resume的被动读取保留了一份POS报告，`identity.epoch=4 request_id=2`，1秒保持、complete=1、disabled_confirmed=1。其身份未与上述网关事务绑定，不用于证明上述动作；证据为固件`output/idle-feedback/one-button-trial-readonly-1793079563060502200.json`。新增检查器`tools/check_one_button_trial_readonly.py`默认只验证编译器字段偏移，只有`--read`才读取，不发送控制请求。

当前结论：新固件已部署，位置/估算速度实时显示和远程能力已核验；用户已有主动操作，但静载、POS带载及手动调试三类实机验收尚未全部完成。代理工具未执行控制动作，不能将用户操作误归为代理只读工具产生。
