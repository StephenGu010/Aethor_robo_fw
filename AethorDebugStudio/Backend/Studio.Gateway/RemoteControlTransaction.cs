/** @file 可注入的单次远程事务，不打开串口、不重发动作。 */
using System.Diagnostics;
namespace Studio.Gateway;
/** 仅一次发送；等待时允许调用者服务优先STOP，超时保持未知。 */
public static class RemoteControlTransaction
{
    /** 执行一个固定请求，result查询需要用户显式再次操作。 */
    public static RemoteControlReceipt Execute(uint id, RemoteControlRequest request, Action<string> write,
        Func<int> read, Action<string> log, CancellationToken cancellation, Action? serviceStop = null, int timeoutMs = 1500)
    {
        var command = RemoteControlProtocol.Build(id, request);
        if (timeoutMs is < 1 or > 1500) throw new ArgumentException("超时必须有界");
        try {
            cancellation.ThrowIfCancellationRequested(); write(command);
            var clock = Stopwatch.StartNew(); var framer = new LineFramer(); var count = 0;
            while (clock.ElapsedMilliseconds < timeoutMs && count < 128) {
                cancellation.ThrowIfCancellationRequested();
                serviceStop?.Invoke();
                if (clock.ElapsedMilliseconds >= timeoutMs) break;
                int value;
                try { value = read(); } catch (TimeoutException) { serviceStop?.Invoke(); continue; }
                if (value < 0) throw new IOException("串口已关闭");
                var line = framer.Push((char)value); if (line == null) continue;
                count++; log(line);
                var receipt = RemoteControlProtocol.Match(line, id, request);
                if (receipt != null) return receipt;
            }
        } catch (Exception error) when (error is IOException or InvalidOperationException or FormatException or OperationCanceledException or UnauthorizedAccessException) {
            return new(id, request.Operation, "unknown", new(), "结果未知：" + error.Message + "；未重发");
        }
        return new(id, request.Operation, request.Operation == "caps" ? "unsupported" : "unknown", new(),
            request.Operation == "caps" ? "设备未确认远程能力，固件可能不支持；操作保持关闭" : "终态未知；仅可读取原事务结果，不得重发动作");
    }
}
