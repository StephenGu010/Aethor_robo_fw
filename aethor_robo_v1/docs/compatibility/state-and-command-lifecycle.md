# 状态机与命令生命周期回放

## 正式七轴状态

```mermaid
stateDiagram-v2
    [*] --> BOOT
    BOOT --> SELF_TEST
    SELF_TEST --> UNALIGNED: 软件与配置结构有效
    SELF_TEST --> FAULT: 配置或自检失败
    UNALIGNED --> DISABLED: ALIGN_REFERENCE 完成
    DISABLED --> ENABLING: ENABLE 已接受
    ENABLING --> READY: 七轴反馈确认使能
    READY --> MOVING: MOVE_JOINTS 已接受
    MOVING --> STOPPING: STOP controlled
    MOVING --> READY: 到位并连续稳定 200 ms
    STOPPING --> READY: 受控减速完成
    READY --> DISABLED: DISABLE 反馈确认
    MOVING --> FAULT: 驱动/反馈/CAN/实时性/链路故障
    READY --> FAULT: 驱动/反馈/CAN/实时性/链路故障
    FAULT --> DISABLED: 故障源消失且 CLEAR_FAULT 完成
```

生产参数尚未逐轴验证时，固件可以查询、模拟和完成主机测试，但 `SELF_TEST` 不会放行真实七轴使能。

## 典型动作回放

```text
PC                  STM32                 七电机
REQ MOVE_JOINTS --> 原子校验
<-- ACK accepted    固定队列接管
                    同周期排入 J1..J7 -->
<-- TEL ...         读取一致性反馈快照
<-- EVT ...         状态变化
<-- DONE COMPLETED  误差/速度/新鲜度连续满足 settle_ms
```

STOP 使用独立高优先级槽。受控停止有可用反馈时按共同减速时长执行并保持使能；反馈失效或控制帧无法原子入队时升级为快速停止和全轴失能。

## 会话重建

```text
USB 断开或 1,000 ms 超时
  -> 活动请求 DONE FAILED/CANCELLED
  -> 全部已使能电机失能
  -> LINK_TIMEOUT 事件
  -> 上位机重新连接并 HELLO
  -> 查询 GET_INFO/GET_CONFIG/GET_STATE
  -> boot_id 变化时重新 ALIGN_REFERENCE
```
