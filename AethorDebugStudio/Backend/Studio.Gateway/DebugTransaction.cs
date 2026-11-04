/** @file 可离线验证的单次设备事务；串口由调用者独占，发送后不重试。 */
using System.Diagnostics;
namespace Studio.Gateway;
/** 把一次发送和有界回执等待封装为可注入传输的状态机。 */
public static class DebugTransaction
{
    /** 只写一次固定命令，错误和超时均保留未知结果，无后续动作。 */
    public static DebugReceipt Execute(uint requestId, DebugRequest request, Action<string> write,
        Func<int> readByte, Action<string> logLine, CancellationToken token, int timeoutMs = 4000)
    {
        if (timeoutMs is < 1 or > 4000) throw new ArgumentException("事务超时必须有界");
        var command = DebugProtocol.Build(requestId, request);
        var tracker = new DebugReplyTracker(requestId, request);
        try {
            token.ThrowIfCancellationRequested();
            write(command);
            var framer = new LineFramer();
            var clock = Stopwatch.StartNew();
            var observedLines = 0;
            while (clock.ElapsedMilliseconds < timeoutMs && observedLines < 128) {
                token.ThrowIfCancellationRequested();
                int value;
                try { value = readByte(); } catch (TimeoutException) { continue; }
                if (value < 0) throw new IOException("串口已关闭");
                var line = framer.Push((char)value);
                if (line == null) continue;
                observedLines++; logLine(line);
                var receipt = tracker.Accept(line);
                if (receipt != null) return receipt;
            }
            return tracker.Receipt("unknown", "等待设备终态超时，动作结果未知；未自动重发，请保留LCD停止入口并检查反馈");
        } catch (Exception exception) when (exception is IOException or UnauthorizedAccessException or FormatException or InvalidOperationException or OperationCanceledException) {
            return tracker.Receipt("unknown", "命令结果未知：" + exception.Message + "；未自动重试");
        }
    }
}
