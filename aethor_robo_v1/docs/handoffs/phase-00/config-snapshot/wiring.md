# Phase 0 接线证据

记录日期：2026-08-12（Asia/Shanghai）

## 已由本地资料证实

| 信号 | MCU 引脚/接口 | 证据 | Phase 0 状态 |
|---|---|---|---|
| FDCAN1 RX | PD0 | `CtrBoard-H7_FDCAN.ioc`：`PD0.Signal=FDCAN1_RX` | 已配置，未做本阶段实机收发 |
| FDCAN1 TX | PD1 | `CtrBoard-H7_FDCAN.ioc`：`PD1.Signal=FDCAN1_TX` | 已配置，未做本阶段实机收发 |
| CAN 物理口 | CAN_H、CAN_L | `达妙科技DM-MC-Board02电机开发板使用说明书V1.1.pdf` 第 4 页 | 板卡具备三路 CANFD 物理接口；手册该表未给出各接口到 MCU 引脚的逐路映射 |
| USART1 TX | PA9 | 同一手册第 6 页；本地 `CtrBoard-H7_RS485.ioc` 与 `Core/Src/usart.c` | 候选正式链路，当前工程未启用 |
| USART1 RX | PA10 | 同一手册第 6 页；本地 `CtrBoard-H7_RS485.ioc` 与 `Core/Src/usart.c` | 候选正式链路，当前工程未启用 |
| USART1 参数 | 921600、8 数据位、1 停止位、无校验 | 本地 `dm-mc02/例程/CtrBoard-H7_RS485/Core/Src/usart.c` | 示例配置已证实，PC 收发和 DMA + IDLE 未验证 |
| USB FS | PA11 DM、PA12 DP | `CtrBoard-H7_FDCAN.ioc` | 当前工程已生成 USB CDC；不作为 PRD 正式 UART 链路完成证据 |

## 当前允许的连接结论

- Phase 0 不连接或驱动电机，不发送 CAN 运动控制帧。
- FDCAN1 的 MCU 侧引脚已由 `.ioc` 固定，但在没有原理图逐网标注和实机收发前，不指定三组外部 CAN 接口中的具体一组。
- USART1 的 PA9/PA10 和 921600 8N1 有板卡手册及本地官方示例双重证据，因此作为后续正式上位机链路候选。
- 当前 `CtrBoard-H7_FDCAN.ioc` 未启用 USART1；接线、电平、DMA 环形接收、IDLE 检测和 PC 往返数据均标记为未验证。

## 本地证据路径

- `E:/Desktop_E/TCG/Aethor_robo_fw/aethor_robo_v1/CtrBoard-H7_FDCAN.ioc`
- `E:/Desktop_E/TCG/dm-mc02/说明书/达妙科技DM-MC-Board02电机开发板使用说明书V1.1.pdf`
- `E:/Desktop_E/TCG/dm-mc02/例程/CtrBoard-H7_RS485/CtrBoard-H7_RS485.ioc`
- `E:/Desktop_E/TCG/dm-mc02/例程/CtrBoard-H7_RS485/Core/Src/usart.c`
