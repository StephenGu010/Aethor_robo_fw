# COM4连接失败排查与只读通信验收 · 2026-10-26

## 已确认原因

旧网关PID14108的Windows完整性RID为4096（Low）。网关能枚举COM4，但SerialPort.Open抛出Access denied。相同时间、相同端口在普通用户Medium会话中成功打开/关闭，写入0字节，排除了当时端口被独占的解释。没有更改串口ACL、系统安全策略或使用管理员权限。

COM4实际设备：VID0483/PID5740、USB序列号386B336F3134，识别为本项目STM32 USB CDC。COM3属于另一个VID0D28设备，没有打开COM3。

## 修复

- 核对旧进程路径与未连接状态后，仅重启本项目网关到普通用户会话。
- 新增GatewayExecutionContext，读取当前进程完整性级别；health公开processId与integrityLevel。
- 低权限导致的拒绝访问返回明确中文提示；普通会话中的拒绝访问仍保留“占用或权限”的可能性，不盲目断言。
- 打开串口失败时立即释放失败句柄和取消源，便于后续显式重连。
- 未更改波特率、DTR/RTS、查询白名单或任何运动接口。

## 实际测试

新网关PID6040，health返回integrityLevel=medium。显式连接COM4成功，115200、DTR/RTS关闭。

已收到show info、show state及七轴show motor的匹配响应，连接持续保持，error=null。控制板公开信息：STM32H723VGT6、usb_cdc、classic_1m、S3519/DM3520；build=unknown，不据此认定当前固件哈希。

实际控制状态查询返回unaligned、enabled=00、moving=0、active=0、fault=none。它是控制器公开状态，不替代新鲜电机失能反馈。

ID7响应state=absent，带有约178万ms以前的历史反馈；上位机已按无效处理，位置、速度、转矩和温度不显示为实时值。USB通信通过不等于电机反馈或运动测试通过。本轮未发送ENABLE、运动、模式切换、参数写入、Save或复位命令。

已将用户当前页面的数据来源切换为“真实设备·只读”，可见COM4与“串口已连接”。没有通过页面启动演示或运动。

## 软件验证与证据

- 网关修复先通过13项离线测试，构建0错误、0告警。
- 普通用户会话实际打开/关闭探针写入0字节。
- 页面和API均确认connected=true、port=COM4、error=null。
- artifacts/com4-connected-state.json：初次连续查询回执。
- artifacts/com4-connected-latest.json：后续快照。
- artifacts/com4-connected-browser.png：用户实际浏览器连接状态截图。
- artifacts/com4-connection-report.json：结构化验收摘要及源码哈希。

网关现保持COM4只读连接。若其他串口工具需要使用该端口，应先在页面显式断开；不自动抢占其他程序。
