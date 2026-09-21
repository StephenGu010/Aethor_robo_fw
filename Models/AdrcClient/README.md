# 单电机 ADRC MATLAB 客户端

`AdrcClient`、`AdrcSerialTransport` 的实例化均不会连接设备、使能电机或启动实验。`AdrcMemoryTransport` 仅用于无硬件测试。客户端不提供资格写入接口，不推断或生成电机标定证据；固件默认无资格时 `prepare`/`run` 仍被拒绝。

客户端依据 `App/Adrc/adrc_protocol.c` 的文本契约实现，面向独占一个协议连接的单个客户端。MATLAB 需要支持 `serialport`、`string`、`table`；离线测试不调用 `serialport`。

## 无硬件测试

```matlab
addpath(fullfile(repo, 'Models', 'AdrcClient'));
report = run_adrc_client_tests(outputDirectory);
```

测试使用内存传输和虚拟时间，覆盖默认禁止 RUN、资格组拒绝、单调 ID、运行期心跳、STOP、ACK/心跳超时、断连、不重试 RUN、ID 耗尽及冻结 trace 导出。测试中 `qualified=1` 仅是内存响应夹具，不能写入真实固件。报告明确标记 `hardwareOpened=false`，不构成串口、板端时序或电机验证。

## 显式连接与命令

```matlab
client = AdrcClient();                % 无 I/O
link = AdrcSerialTransport('COM7', 115200); % 仅保存明确指定的设置
% 以下均需用户明确调用；端口名应由实际设备确定。
% link.open();
% snapshot = client.connect(link.callbacks()); % 只读 status，同步 last_id
% snapshot = client.status();
% client.disconnect();
```

也可显式调用 `client.connectSerial(portName, baudRate)`。没有自动枚举端口、连接、重连、配置或启动行为。连接到已有活动实验时只读取状态；不会接管心跳。应显式调用 `stop()` 等待失能。

配置、准备和运行分别是独立方法，示例值应来自已批准且实际测量的实验参数，不由客户端生成：

```matlab
% client.configure('control', measuredControlFields);
% client.configure('mapping', measuredMappingFields);
% client.configure('limits', approvedLimitFields);
% client.configure('identify', struct('torque', measuredPulseTorque, 'pulse_ms', 20));
% client.prepare(selectedMotor);  % 对外轴号 1..7；固件仍验证资格和反馈
% terminal = client.run(4);       % 必须已有 qualified/prepared/disabled 快照
% stopped = client.stop(1);       % 等待实际失能的最新状态
% client.clearFault();            % 显式请求固件故障清除，不绕过失能门限
```

允许字段与真实协议一致：

| 组 | 字段 |
|---|---|
| control | b0, wc, wo, target, mode, duration_ms |
| mapping | pos_scale, vel_scale, torque_scale |
| limits | torque_limit, torque_slew, reference_accel, near_zero |
| identify | torque, pulse_ms |

mode 为 identify/pi/ladrc；数值请求按 float32 精度转换为不含指数的十进制，160 字符限制仍适用。最终范围与硬限制由固件裁决。没有 evidence 组，没有旧式使能或运动命令。

## 等待、心跳与停止语义

每个变更命令先递增非零 uint32 ID，再仅写一次。连接和显式 `status()` 使用 `last_id` 同步最高已接受 ID，ID 不回绕。只读 status/trace 不携带 ID，避免进入重放缓存；因此一个连接只允许一个客户端且一次一个查询。命令超时不会自动换 ID 重发 RUN。明确错误、超时或未知连接状态会通过异常交给调用方。

`run()` 同步等待，使用可注入时钟，每 100ms 发送独立 ID 的 heartbeat；ACK 最多等待 200ms，heartbeat DONE 最多等待 200ms，RUN 总期限默认 4 秒且可显式设置（最多 60 秒）。这是 MATLAB 调度目标，不是硬实时保证；长时间阻塞 MATLAB 的回调可能错过期限，固件自身仍负责失联停机。

`run()` 等待期间可从 GUI/timer 回调调用 `client.requestStop()`，由循环发送 STOP。直接 `stop()` 用于客户端空闲时。STOP 自己的 DONE 仅表示已处理请求；`stop()` 还轮询 `disabled=1`、`state=disabled/fault`、`active_id=0` 才报告确认。`run()` 请求停止后，以 RUN 终端的 disabled 标志作为确认。

运行异常或中断的清理逻辑尽力发送一个新 ID 的 STOP，保存 `LastStopAttempt` 并关闭连接；没有终端/反馈确认时 `confirmed` 保持 false。通信失效时不能用关闭连接代替物理失能证明。停止方法的查询可能在总期限边界额外占用一个不超过 200ms 的查询窗口。

## 冻结 trace

```matlab
% trace = client.exportTrace(fullfile(outputDirectory, 'trial.csv'));
```

仅允许 `disabled=1`、`frozen=1`、`active_id=0`、disabled/fault 状态下导出。逐项读取后再次核对状态和首项；时间戳保留 uint64，不经过 double。已有文件不覆盖。CSV 旁保存 `.metadata.json`，注明来源、状态及转矩是协议名义命令而非真实转矩测量。导出失败可能留下部分磁盘文件，调用方应保留错误信息并选择新路径重试；客户端不批量清理文件。

## 传输接口与边界

可注入结构含函数句柄 `WriteLine(line)`、`ReadLine(timeoutSeconds)`、`IsOpen()`、`Close()`、`Now()`、`Sleep(seconds)`，可选 `Name`。`ReadLine` 返回完整一行或超时空字符；时钟必须单调，传输必须遵守有界读取。内存传输不会打开硬件；串口适配的显式 open 才调用串口 API。

当前范围是协议客户端和离线契约测试，没有 GUI、参数辨识算法、资格录入、自动重连/恢复、多个并发会话或硬件验收。真实串口路径尚需独立验证。所有设备操作必须使用已测资格和相应台架流程。
