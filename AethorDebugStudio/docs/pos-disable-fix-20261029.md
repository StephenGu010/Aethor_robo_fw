<!-- @file POS启动失败的欠压原始证据、终态诊断修复及未完成的实机恢复边界。 -->
# POS启动失败排查与离线修复

本轮用户限定只读检查、不执行电机运动、不提高转矩门槛。未发送启动、预检、清故障、模式切换、STOP、DISABLE或ENABLE，未烧录/复位/暂停主控，也未解除保护。运行中检查使用固定HTTP缓存读取，以及按已部署AXF/map解析地址的attach-only SRAM读取。

## 证据与供电

原事务1091929107终态为`failed accepted=1 reason=9 disabled=0`。用户先确认实际供电12V，随后确认已改为24V。原厂S3519说明书V1.1第20页给出额定24V、驱动支持15～52V；12V低于该范围，与原始欠压反馈一致。没有通过降低UV_Value或转矩保护来接受12V供电。寄存器0x00为欠压保护设定值，不是实时母线电压，本轮没有编造实时电压数值。

保存的最后CAN帧为标准ID0x17、长度8、`97 7f ff 7f f7 ff 1c 18`。生产解码器直接读取首字节高四位，0x97因此为ID7驱动状态9；原厂第8页将其定义为欠压。SRAM中driver_state和fault_flags均为9，feedback_generation=1，POS阶段为VALIDATE、samples=0。当前保存的欠压帧已过期，不能用作24V调整后的当前电压、温度或转矩。

UI退出原因9是NOT_DISABLED：该门会统一拒绝非失能、有故障或速度不满足的反馈；与驱动状态9不是同一套编号。独立disabled=0表示没有获得安全失能证明。未发现已证实的丢ACK或将UI原因误写入驱动故障的缺陷；收到新故障反馈后正确拒绝执行，不能通过放宽门禁使其启动。

供电调整后仍仅看到旧帧。主控`enabled=00 moving=0 active=0 fault=none`是主控状态，不能覆盖原事务disabled=0。ID7的`unconfirmed_disable_mask=64`、remote_owns=1仍保持，原背景采样/识别门因此关闭。没有把缓存查询正常误认作当前驱动已恢复。

原始证据：`../artifacts/pos-disable-passive-20261029/capture.json`及同目录8个二进制缓存；解析结果`decoded.json`。24V后快照`../artifacts/pos-disable-after-24v-20261029.json`，本轮末尾快照`../artifacts/pos-disable-investigation-final-20261029.json`。

## 软件修复

上位机将“确定的事务失败”与“安全失能证明”分开表示：failed/accepted=1/disabled=0保持failed，显示原因和现有阶段/错误码，并保留原事务ID和运动锁；结果未知、缺失准入及completed但失能未确认的保守规则保持。原remoteTerminalConfirmed及所有安全解锁条件未放宽；不自动release、不重发动作。恢复旧固件的失败回执也能清晰显示，且不会因相同数字9就将UI原因解释成驱动欠压。

前端生产构建已生成。用户原有调参页没有被强制刷新；独立观察页加载新资源并只读恢复原事务，实际显示“板端已报告失败；未满足失能/无故障/低速条件（原因9）；失能未确认，运动保持锁定，仅可读取原事务结果”。浏览器曾复用旧index资源，独立观察页通过带诊断查询的URL加载已验证的新资源`index-Btytf_nI.js`，未清除用户页面零点或草稿。

固件候选在独立目录`D:/download/TCG/pos-disable-recovery-candidate-20261029`，仅追加远程终态中已有的stage/error/detail/motor/fb_us字段，未改CAN准入、执行、清理、失能资格或3.5N·m转矩保护。detail缓冲160字节，保守最大字段组合134字节；读取保留终态不清运动锁或所有权。原f82a9a72部署目录和冻结输入保持完整。

## 验证与交付状态

- 前端76项单元测试通过；新增真实失败形态用例先RED后GREEN。
- 10项隔离浏览器回归通过，覆盖失败原因、原事务保留、跨页锁定、不释放/不重发。所有API均拦截，模拟测试没有接触设备。
- 固件最大数字字段和重复终态读取用例先RED后GREEN；原远程适配回归通过。
- 10种应用配置生成的测试程序逐一exit=0，且原脚本的非法配置拒绝检查完成；原包装命令误用了脚本末尾预期失败编译留下的LASTEXITCODE，已由独立exe结果和哈希排除误判，未改测试源脚本。
- POS流程回归通过，包括新反馈、原有保护、STOP/饱和队列/严格终态传输证明。
- 独立审查未发现阻断问题；指出的NONE=0显示问题已用失败测试复现并修正。
- Keil目标LCD-POS-Load-Test，0错误、0警告；HEX/AXF的517840个占用字节一致。
- HEX SHA256：`f7a73f5e2a249c6dde1da0f4546e2651a6137a3a045e78d3d3c4d61fc70e9abb`。
- AXF SHA256：`943a4cdde4ce0b6d28bbe1a25944a57ef61c78654e1c113101204411f5ab1280`。
- 新冻结清单：候选`output/pos-disable/release-offline-20261029.json`，927项输入；相对原固件仅改变远程结果格式文件及其测试文件。新候选尚未烧录，主控仍为f82a9a72，不认定实机启动修复完成。

前端日志：`../artifacts/pos-disable-unit-final-20261029.log`、`pos-disable-browser-release-20261029.log`、`pos-disable-build-final-20261029.log`。固件回归日志在新候选output/idle-feedback/pos-failure-green。当前真实界面截图：`../artifacts/pos-failure-live-diagnostics-fixed-current-20261029.png`。

## 下一步边界

纯只读检查未获得新的标准失能确认，不能直接再次进行POS带载试验。若用户后续要求继续实机恢复，需要先确认当次独立承托，再单独授权一次有界的ID7失能确认；它会撤去保持力矩，因此不包含在本轮只读范围内。只允许该次失能事务及其新回执采集，禁止自动清故障、改模式、使能、运动或失败重试。新鲜无故障低速失能反馈及其发送/接收证据满足原门槛后，才可讨论下一步测试。本轮没有执行该动作。
