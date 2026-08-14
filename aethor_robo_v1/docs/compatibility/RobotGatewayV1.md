# RobotGatewayV1 对接清单

本文是上位机实现首组七轴机械臂适配器的最小契约。权威协议语义仍以 `固件开发prd/02_上位机串口协议_aethor-arm-ascii-v1.md` 为准。

## 连接与会话

- 在明确选择的 Windows COM 口上打开 STM32 Type-C USB CDC；波特率参数不代表 USB 总线速率。
- 每次连接先发送 `HELLO client=<name> protocol=1`，保存响应中的 `boot_id`、`session`、`controller_id` 和 `arm_id`。
- 每 250 ms 发送 `HEARTBEAT`。固件连续 1,000 ms 未收到有效会话请求时执行 `STOP_DISABLE`。
- 断线、重新 `HELLO` 或 `boot_id` 改变后，清空上位机正在执行的请求、旧目标和对齐状态，不复用旧 request ID 的业务含义。

## 请求生命周期

```text
查询：REQ -> RSP | ERR
动作：REQ -> ACK -> DONE | ERR
异步：EVT
遥测：TEL JOINT_STATE / TEL MOTOR_STATE
```

- `ACK` 只表示命令已被有界队列接受，不表示电机动作完成。
- UI 和动作编排只在匹配 request ID 的 `DONE` 后释放命令门控。
- 同一会话内重复发送完全相同的 request ID/正文必须得到缓存结果，不能重复执行动作；相同 ID 配不同正文视为冲突。
- 高优先级的 `ERR`、`EVT`、`DONE`、`STOP` 和 `DISABLE` 不能被遥测积压阻塞。

## 模型与运动

- 实体模型只消费 `JOINT_STATE.q_deg`，它是对齐后的七轴关节角；不能直接使用电机原始弧度。
- 幽灵模型只表示用户目标。一次 `MOVE_JOINTS` 必须发送恰好七个有限值，不能拆成七条单轴命令。
- `MOTOR_STATE` 默认 10 Hz，并在状态变化时立即输出；`JOINT_STATE` 默认 50 Hz。遥测序号允许因丢旧保新出现间隙。
- `GET_MOTORS startup=id,mode,ranges,version` 返回四个七轴验证掩码；未置位表示相应启动参数尚未获得可信响应。
- `GET_DIAG parse=bad_frame,bad_crc` 和 `motion=active_request,predicted_ms,actual_ms,max_error_mdeg` 使用紧凑定长字段，供日志和 UI 直接关联。
- DH/URDF 仅属于上位机模型。当前机械参数未知不阻止协议和关节空间软件测试，但生产使能保持锁定。

## 安全门控

- 正式配置必须满足七电机均存在、反馈新鲜、身份和驱动参数一致、已对齐、模式回读一致、物理字段已验证，才可使能。
- 台架配置的 `motors=` 每条命令显式给出、升序且不得重复；未选电机保持原状态和目标。
- 上位机不得通过默认值绕过固件的方向、零位、限位、速度、加速度、量程、减速比或 MIT 增益验证位。
- `CLEAR_FAULT` 只能在去使能、无运动且故障源已消失时成功；恢复后重新建立反馈，需要时重新对齐。

## 联调验收

1. 对共享 Golden Frames 和 CRC 向量逐项编解码。
2. 验证 `HELLO`、`GET_CONFIG`、`map_hash` 和七轴 ID 顺序。
3. 验证查询 `RSP` 与动作 `ACK/DONE` 的不同生命周期。
4. 验证相同请求重放不会重复动作，冲突请求被拒绝。
5. 验证 50 Hz/10 Hz 遥测、序号间隙和丢旧保新。
6. 验证 STOP、DISABLE、断线、重连和 `boot_id` 改变。
7. 保存 Aethor Studio V2 的版本或 commit、执行日志与结果。

上面第 7 项尚未取得外部上位机版本时，只能声明“固件软件与兼容性测试包完成，外部上位机待验收”。
