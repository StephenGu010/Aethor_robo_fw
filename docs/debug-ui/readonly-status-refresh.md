# LCD 电机7只读状态刷新

**当前结论：硬件验证未通过，已撤回轮询源码，恢复上一版控制检查页固件。以下实现描述是实验方案，不是当前默认行为。**

## 目的与协议依据

检查页和电机详情原先仅显示被动反馈快照。电机失能且没有新帧时，反馈自然过期。本阶段增加空闲只读查询，不改变反馈有效期、运动阈值、模式或标定配置。

本地官方SDK `D:\download\TCG\S3519\电机SDK\Python例程\u2can\DM_CAN.py` 的 `refresh_motor_status` 使用标准CAN标识符 `0x7FF`、8字节载荷 `07 00 CC 00 00 00 00 00` 查询电机7状态。该8字节格式在本机电调上未得到反馈（20次采样过期，CAN发送计数增长但接收未更新）。项目已有 `s3519_pack_feedback_query` 使用4字节 `07 00 CC 00`，后续复用该既有编码核对兼容性。查询不同于使能/禁用特殊帧或POS目标帧。

## 实现与边界

- 复用 `s3519_pack_feedback_query` 编码状态读，验证ID和输出参数。
- `aethor_app_next_can_frame` 在原执行/参数帧之后尝试生成低优先级查询；UiTask不直接发送CAN。
- 仅LCD已启用、运动权限关闭、台架配置生效。生产、POS、MIT版本不启用该轮询。
- 只查询 `DEBUG_UI_INITIAL_MOTOR_ID`（当前7），不是随浏览选择轮询其他电机。
- 参数身份、模式、范围已验证，且至少有一次反馈；原始反馈状态均为失能才查询。未知和过期不会被伪装成新鲜数据。
- 每50毫秒最多生成一帧；动作、命令队列、STOP、紧急禁用、未完成结果、本地请求及参数发现优先。查询发送失败仍由现有CAN诊断报告。
- 当前启动后的参数发现与一次禁用确认沿用USB流程；本阶段没有添加LCD独立发现入口。
- 角度、速度语义仍待核对，本阶段不执行运动，不将POS标定标为有效。

## 软件验证与回退

新增测试先在只读配置失败，再验证查询精确载荷、50毫秒限频、时间回退、动作/STOP/参数发现让行、其他电机已使能暂停，以及其他构建配置无轮询。App五配置、非法配置、Motor Core、Phase0、生产配置回归通过。

8字节候选版本Keil LCD-ReadOnly：0错误、0警告，内存布局检查通过；241800字节Flash回读一致。候选HEX SHA256：`d676c0475bbc90ff9d46ef04e04d19830d01e090996eaf727f77f074df66d30c`。该候选硬件刷新未通过，不作为最终验收结果。

修改前备份：`D:\download\TCG\output\can_id_commissioning_20260909\before-readonly-refresh.zip`。

Flash回读以 `lcd-status-refresh-flash-result.json` 为准，实机查询以 `lcd-status-refresh-hardware.log` 为准，均位于上述output目录。

## 实机结果与恢复

| 格式 | 参数发现/禁用 | 20次新鲜采样 | CAN诊断 | 结论 |
|---|---|---|---|---|
| 8字节 07 00 CC 00 00 00 00 00 | 完成，四验证掩码40，enabled=00 | 0/20 | rx=15 tx=101 error=0 busoff=0 | 未收到状态查询回包 |
| 4字节 07 00 CC 00 | 完成，四验证掩码40，enabled=00 | 0/20 | rx=15 tx=100 error=0 busoff=0 | 未收到状态查询回包 |

两轮均未使能或执行运动；最终查询 `enabled=00 moving=0 active=0 fault=none`。位置缓存约23.004度，不能据此判断当前角度控制精度。这里验证的是本台电调、当前模式2、失能状态；不扩大为所有达妙电机均不支持查询的结论。

4字节候选HEX为 `e230e107205dd155b47b23c54b16f16ff25f539e8c66ed126ffac79806374f9b`，241800字节回读一致。实验代码、测试、HEX/AXF保存在 `readonly-refresh-4byte-experiment.zip`，失败原始日志分别为 `lcd-status-refresh-8byte-no-response.log` 和 `lcd-status-refresh-4byte-no-response.log`。

恢复目标为上一版 `8b2c8a6ca97a01b5b225eb49d5c6b240901105032288fb8b6b2d5ea9cfd283ea`；恢复回读记录为 `lcd-status-refresh-rollback-result.json`。保留电机7默认选择和控制检查页，不保留持续无回应的查询。恢复后首轮参数发现仍需USB流程。

后续应核对本台电调型号/固件版本对应的失能状态查询支持范围，再选择独立状态查询或明确标注的一次性禁用确认操作。不能用周期禁用、自动使能或放宽过期阈值伪装成只读刷新。本轮不更新电调固件。
