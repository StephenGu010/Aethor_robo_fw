# Aethor Studio V2 外部兼容性验收记录

- 日期：待填写
- 上位机版本/commit：待提供
- 固件 commit：待填写
- Windows 版本：待填写
- COM 口：待填写
- `controller_id/arm_id`：`ctrboard-h7/arm-1`
- `boot_id`：待填写

| 验收项 | 结果 | 证据文件 |
|---|---|---|
| CRC 与 Golden Frames | 待执行 | |
| HELLO/会话/boot_id | 待执行 | |
| GET_CONFIG/map_hash | 待执行 | |
| ACK 与 DONE 分离 | 待执行 | |
| 重放安全与 ID 冲突 | 待执行 | |
| JOINT_STATE 50 Hz | 待执行 | |
| MOTOR_STATE 10 Hz/状态变化 | 待执行 | |
| 遥测拥塞丢旧保新 | 待执行 | |
| 实体模型只跟随反馈 | 待执行 | |
| STOP/DISABLE | 待执行 | |
| 断线/重连 | 待执行 | |
| boot_id 改变后清理旧状态 | 待执行 | |

## 结论

未填写并通过全部项目之前，Aethor Studio V2 保持“外部交付待契约验收”，不得计入固件仓库的软件完成声明。
