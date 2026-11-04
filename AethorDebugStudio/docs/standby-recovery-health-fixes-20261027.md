<!-- @file 待机自动只读重识别、健康卡时间误判修复与零点失效原因说明。 -->
# 修复结果与零点说明

本轮只修改软件、进行离线构建/回归和读取网关缓存；没有烧录、复位、预检、参数下发、设零、回零、STOP/DISABLE/ENABLE或电机运动。

## 24V恢复后的自动识别

根因已由上轮只读实测和本轮失败测试确认：主控在24V断开时识别失败，待机位置读取又要求完整身份，通电后不会自动重启识别。

修复位于新候选`D:/download/TCG/standby-recovery-candidate-20261027`，基于3cf419f2冻结输入，保留HOME_PREPARE及既有保护。仅在LCD拥有且空闲、没有前台任务/STOP/控制/参数维护/未确认失能等条件时，使用原13参数只读识别路径，禁止通过bench init或模式写入恢复。

识别失败冷却2秒，每轮最多2秒；成功后停止重查。已经识别的模式2仅在0x50连续3秒缺样后恢复识别；0x51恒零/继续回复不能替代0x50资格。前台忙碌会重置观测宽限，不将运动期间暂停采样误当掉电。后台被前台接管时先取消自己的租约并隔离迟到应答，不取消前台的新识别。真正长期失联使旧坐标参考失效一次，防止沿用可能已改变的编码器原点。

专项10项、原HOME/POS/MIT静载/远程代理/采样等回归通过。Keil构建0错误、0警告，工程配置未改；173项显式构建输入存在。主流程再次核对证据哈希及HEX/ELF的517764个占用字节，927项输入已冻结。

- HEX：`output/standby-recovery/reviewed-build-20261027/LCD-POS-Load-Test.hex`
- HEX SHA-256：`f82a9a72b0cf12013cca1f9037c55dc9742c603a63b453f15cd7a7463d2f3629`
- AXF SHA-256：`5cc3fc00c66b55a54db66b3b7162a3901521d31c763fd8c278280dbca657bb42`
- 完整证据：`output/standby-recovery/firmware-review-evidence-20261027.json`
- 离线发布冻结：`output/standby-recovery/release-offline-20261027.json`

该固件尚未由本轮烧录，自动恢复功能需要部署修复版后才在主控生效，不以离线测试代替晚通电实物验证。

## 健康卡偶发“过期”

旧组件用每250ms更新的studio.now判断刚收到的新快照，若UI tick落后快照10ms，原feedbackAge会拒绝负时间差，卡片误显示“超过500ms”。同一119ms寄存器样本按当前时钟则有效。

仅将`Frontend/src/components/DeviceDebug.tsx`的评估时刻改为Date.now，UI tick继续触发重绘。没有放宽500ms门限，也没有改变未来非法时间、断连及标准反馈资格判断。

隔离浏览器固定页面版本与时钟，稳定复现旧代码在实时示波页错误显示过期，修复后通过；同用例继续验证500ms仍过期、断连仍关闭，全部API拦截且POST为0。72项单元测试、8项相关浏览器回归及生产构建通过。修复后的生产资源已生成，未强制刷新用户已有页面，以免清空其会话零点。

日志：`artifacts/health-clock-red-stable-20261027.log`、`artifacts/health-clock-green-stable-20261027.log`、`artifacts/health-clock-targeted-final-20261027.log`、`artifacts/standby-health-unit-20261027.log`、`artifacts/standby-health-build-20261027.log`。隔离模拟截图：`artifacts/health-clock-fixture-fixed-20261027.png`。

## 为什么刷新/重连/重启会丢零点

用户补充：丢失发生于刷新页面、重连USB或主控重启。这三项均符合当前会话零点策略，未发现需要通过取消保护解决的问题。

`useStudio.ts`使用useState(null)保存零点，没有写入浏览器持久存储或驱动器；刷新会重新初始化。零点同时绑定连接generation、主控boot和坐标reference；USB重连、主控重启、参考变化时失配即清除。静载/POS页互切本应保留；临时反馈过期仅禁用回零，不清零。

一次HTTP轮询异常也会暂记gateway.connected=false，从而触发相同的保守失效。用户本次确认的丢失场景不需要改这个路径；若以后发现未刷新/未重连也丢失，应再绑定实际连接身份区分网络瞬断，不能直接恢复旧绝对目标。

本轮不擅自增加跨重启持久化，不把旧零点直接重新用于回零。核验期间网关被动快照的主控启动号也由2234713070变为516631035，该变化会按规则使原零点失效；本轮没有发起这个复位或现场控制事务。
