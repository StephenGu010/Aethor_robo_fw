# 状态机与命令生命周期回放

本文描述当前 `aethor-text-v1` 正式入口。旧 `REQ/RSP/ACK/ERR/EVT/TEL` 名称只属于 `aethor-arm-ascii-v1` 回归资产。

## 正式七轴状态

```mermaid
stateDiagram-v2
    [*] --> BOOT
    BOOT --> SELF_TEST
    SELF_TEST --> UNALIGNED: 软件与配置结构有效
    SELF_TEST --> FAULT: 配置或自检失败
    UNALIGNED --> DISABLED: arm align 完成
    DISABLED --> ENABLING: arm enable 已接受
    ENABLING --> READY: 七轴反馈确认使能
    READY --> MOVING: arm move 已接受
    MOVING --> STOPPING: arm stop
    MOVING --> READY: 到位并连续稳定 200 ms
    STOPPING --> READY: 受控减速完成
    READY --> DISABLED: arm disable 反馈确认
    MOVING --> FAULT: 驱动/反馈/CAN/实时性/链路故障
    READY --> FAULT: 驱动/反馈/CAN/实时性/链路故障
    FAULT --> DISABLED: 故障源消失且 arm clear 完成
```

生产参数尚未逐轴验证时，固件可以查询、模拟并完成主机测试，但 `SELF_TEST` 不会放行真实七轴使能。台架 Profile 使用显式电机子集和独立的 `bench` 命令，不代表正式七轴状态机已经通过机械验收。

## 查询生命周期

```text
上位机                         固件
7 show state  ---------------> 解析并读取一致性快照
               <-------------- ok 7 show state state=ready ...
```

- 查询使用匹配请求编号的 `ok` 或 `error` 结束。
- 省略请求编号时固件使用编号 `0`，便于人工串口输入；编号 `0` 不进入近期结果重放。
- `show` 和 `data` 都是只读输出，但遥测到达不替代通信保活。

## 台架动作生命周期

```text
上位机                         固件                         所选电机
13 bench jog ...  -----------> 校验 Profile、列表、位移和速度
               <-------------- ok 13 bench jog accepted=1
                               发送固定绝对 CAN 目标批次 --->
ping              -----------> 刷新通信看门狗
               <-------------- ok <id> ping ...
                               未到位：复位批次索引并重发 -->
                               已到位：完成动作
               <-------------- done 13 bench jog result=completed ...
```

- `bench jog` 只由上位机发送一次。收到 `accepted=1` 后，上位机保留请求编号并等待匹配的 `done`。
- 固件在动作开始时根据反馈计算一次固定绝对目标；未到位时只复位 CAN 批次读取索引，不重新解释串口命令、不改变目标。
- 上位机不能通过不断分配新请求编号来重复提交同一动作，否则会形成多个业务请求。
- 已接受动作无论成功、停止、取消或失败，都通过 `done ... result=completed/stopped/cancelled/failed` 报告终态；校验或入队失败在动作开始前返回 `error`。

## 自包含绝对台架运动

```text
上位机                                      固件                         所选电机
50 bench move 1,3
 position=90,-45 speed=30,20  -----------> 严格校验等长列表与一一映射
                         <--------------- ok 50 bench move accepted=1
                                          按需发现 PMAX/VMAX/MAX_SPD
                                          校验范围、反馈与故障
                                          POS_VEL、清错、使能        --->
                                          固定逐电机目标并内部重发    --->
                                          最终位置 + 0 速度 HOLD     --->
                                          selected disable           --->
                                          等待新鲜 disabled 反馈
                         <--------------- done 50 bench move result=completed ...
```

- `motor/position/speed` 列表非空且严格等长，按调用者列表顺序一一对应；电机是唯一的 1–7，不广播。请求编号必须为 `1..UINT32_MAX`；位置和速度只接受正常 `float32`，非零绝对值位于 `FLT_MIN..FLT_MAX`，位置另允许 `0` 且速度必须大于 `0`。
- 位置是相对本次上电零点的 S3519 输出端绝对角度，不是机械臂软限位。`abs(position)` 只与本次发现的 `PMAX` 比较，速度只与 `min(VMAX,MAX_SPD)` 比较；边界允许，越界拒绝且不截断，缺失发现值不回退默认值。
- 主机只发送一次动作正文，要求同一非零编号的 ACK 先于唯一 DONE，并且不发送 `ping`。局部看门狗豁免只覆盖已接受动作实际持有的发现到失能/清理状态；反馈新鲜度、驱动故障、控制周期、动作期限与 Bus-Off 路径仍然有效。
- 成功终态只在 HOLD、所选电机失能和全部所选电机新鲜 disabled 反馈之后产生。未选电机不接收该动作的控制帧。
- `bench stop` 可抢占活动动作：原 `bench move` 报告 `cancelled`，STOP 有独立终态。第二条普通命令立即返回 `busy`，不覆盖当前动作。

稳定终态 schema：

```text
done 50 bench move result=completed elapsed_ms=3200 motors=05
done 50 bench move result=failed stage=motion code=stale_feedback motor=3
done 50 bench move result=cancelled
done 50 bench move result=stopped
```

失败阶段为 `validate/discovery/mode/clear/enable/motion/hold/disable`，无法归属时为 `unknown`；错误码为 `not_ready/position_out_of_range/speed_out_of_range/fault_present/stale_feedback/timeout/feedback_timeout/action_failed`。可归属故障时 `motor` 是所选电机的一基编号，否则为 `?`；完成结果的 `motors` 是所选电机位掩码，电机 1 对应 bit 0。

## 请求重放

```text
13 bench jog ...  -----------> 首次受理并执行
               <-------------- ok 13 ... accepted=1
               <-------------- done 13 ... result=completed

13 bench jog ...  -----------> 相同非零编号、相同正文
               <-------------- 缓存结果；不再次驱动电机

13 bench jog ...不同正文 -----> 编号冲突
               <-------------- error 13 ... code=request_conflict
```

近期结果容量为 32 项，保留 60 秒。上位机应把重放作为链路恢复手段，而不是周期运动命令；人工编号 `0` 不提供重放保护。

## 停止与失能

- `arm stop` 使用独立高优先级槽。反馈有效时按共同减速时长执行并保持使能；反馈失效或控制帧无法原子入队时升级为快速停止和全轴失能。
- 台架异常清理依次发送 `bench stop <motors>` 和 `bench disable <motors>`，并等待各自终态；脚本退出路径只做尽力清理，不能替代物理断电能力。
- 驱动故障、反馈超时、CAN Bus-Off、传输故障和通信看门狗均可使活动动作以失败或取消终态结束。

## 保活与连接重建

```text
旧 bench enable/jog 使电机带电或运动
  -> 上位机每 250 ms 或更快发送有效 ping
  -> 1000 ms 无有效请求
  -> 固件停止并失能
  -> event ... link_timeout elapsed_ms=1000 action=stop_disable
  -> 上位机重新连接并发送 hello
  -> stream off
  -> show info/config/state/motors/diag
  -> boot 改变时丢弃旧请求与旧目标
```

自包含 `bench move` 在其已接受的发现到失能/清理生命周期内不走上述通信超时路径，也不伪造 `ping` 或刷新最后请求时间。全部电机失能且无运动时，通信看门狗同样不触发停止/失能；因此只读手工调试不需要周期重复发送指令。重新连接后必须先确认设备身份、Profile、`boot`、电机状态和故障掩码，再开放动作；正式机械臂 Profile 还需重新确认参考位。

本文描述的软件合同和主机测试不证明机械软限位、方向、外部减速比、真实角速度、带载性能、七轴联动或真实硬件已经验收。
