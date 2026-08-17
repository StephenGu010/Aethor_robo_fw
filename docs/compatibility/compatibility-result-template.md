# Aethor Studio V2 外部兼容性验收记录

- 日期：待填写
- 上位机版本/commit：待提供
- 固件 commit：待填写
- Windows 版本：待填写
- COM 口：待填写
- `controller_id/arm_id`：`ctrboard-h7/arm-1`
- 目标协议：`aethor-text-v1`
- `profile`：待填写
- `boot`：待填写

| 验收项 | 结果 | 证据文件 |
|---|---|---|
| 可打印 ASCII、LF/CRLF、无应用层 CRC | 待执行 | |
| 分片、粘连、超长行、非法字符和重复字段 | 待执行 | |
| `hello`/Profile/`boot`/看门狗参数 | 待执行 | |
| `show config`/映射/验证掩码 | 待执行 | |
| 查询 `ok/error` | 待执行 | |
| 动作 `ok accepted=1` 与终态 `done` 分离 | 待执行 | |
| 重放安全与 ID 冲突 | 待执行 | |
| `bench jog` 只提交一次且固件内部重发目标 | 待执行 | |
| 使能期间周期 `ping` 与 1000 ms 超时停止/失能 | 待执行 | |
| 空闲失能时无需周期保活 | 待执行 | |
| `stream joints` 1–50 Hz | 待执行 | |
| `stream motors` 1–10 Hz | 待执行 | |
| `data` 拥塞丢旧保新 | 待执行 | |
| 实体模型只跟随反馈 | 待执行 | |
| `stop/disable` 安全清理 | 待执行 | |
| USB 断线/重连 | 待执行 | |
| `boot` 改变后清理旧请求和目标 | 待执行 | |

## 结论

未填写并通过全部项目之前，Aethor Studio V2 保持“外部交付待契约验收”，不得计入固件仓库的软件完成声明。
