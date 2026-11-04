/** @file 固定停止/失能事务；不提供使能、运动、模式切换或任意文本发送。 */
using System.Globalization;
namespace Studio.Gateway;
/** 显式设备操作请求；失能必须声明负载已有独立承托。 */
public sealed record DebugRequest(string Operation, int MotorId = 7, bool LoadSupported = false);
/** 软件事务回执；完成回执不能单独证明物理停止或驱动已失能。 */
public sealed record DebugReceipt(uint RequestId, string Operation, int MotorId, bool Accepted,
    string Result, string Detail, string[] Lines, bool FeedbackConfirmed = false);
/** 限定可发送的停止/失能命令，不复用通用字符串命令通道。 */
public static class DebugProtocol
{
    /** 验证操作、轴号及承托条件，返回协议操作名称。 */
    public static string Operation(DebugRequest request)
    {
        if (request.MotorId is < 1 or > 7) throw new ArgumentException("电机编号必须为1至7");
        return request.Operation switch {
            "stop" or "stop-all" => "stop",
            "disable" when request.LoadSupported => "disable",
            "torque-disable" when request.MotorId == 7 && request.LoadSupported => "disable",
            "torque-disable" => throw new ArgumentException("配额准备只允许已独立承托的ID7"),
            "disable" => throw new ArgumentException("失能会撤去保持力矩，请先确认独立承托"),
            _ => throw new ArgumentException("只允许停止、全部停止或选中轴失能；运动与参数写入未开放")
        };
    }
    /** 构造一次有编号的命令；不自动使能、重试或追加失能。 */
    public static string Build(uint requestId, DebugRequest request)
    {
        if (requestId == 0) throw new ArgumentException("请求编号不能为0");
        var operation = Operation(request);
        var motors = request.Operation == "stop-all" ? "1,2,3,4,5,6,7" : request.MotorId.ToString(CultureInfo.InvariantCulture);
        return $"{requestId.ToString(CultureInfo.InvariantCulture)} bench {operation} {motors}\n";
    }
}
/** 按请求号、路径、ACK/DONE顺序和电机掩码识别单次回执。 */
public sealed class DebugReplyTracker
{
    private readonly uint requestId;
    private readonly DebugRequest request;
    private readonly string operation;
    private readonly List<string> matchedLines = new();
    private bool accepted;
    /** 冻结本次命令身份，拒绝后续改轴或改操作。 */
    public DebugReplyTracker(uint requestId, DebugRequest request)
    {
        _ = DebugProtocol.Build(requestId, request);
        this.requestId = requestId; this.request = request; operation = DebugProtocol.Operation(request);
    }
    /** 处理一行完整响应；不匹配的异步数据不会完成本事务。 */
    public DebugReceipt? Accept(string line)
    {
        var parts = line.Split(' ', StringSplitOptions.RemoveEmptyEntries);
        if (parts.Length < 4 || !uint.TryParse(parts[1], out var responseId) || responseId != requestId ||
            parts[2] != "bench" || parts[3] != operation || parts[0] is not ("ok" or "done" or "error")) return null;
        var fields = new Dictionary<string, string>();
        foreach (var token in parts.Skip(4)) {
            var separator = token.IndexOf('=');
            if (separator <= 0 || !fields.TryAdd(token[..separator], token[(separator + 1)..])) throw new FormatException("设备回执字段无效或重复");
        }
        if (matchedLines.Count >= 4) throw new FormatException("设备重复回执过多");
        matchedLines.Add(line);
        if (parts[0] == "error") return Receipt("rejected", "设备拒绝命令：" + fields.GetValueOrDefault("code", "unknown"));
        if (parts[0] == "ok") {
            if (accepted || fields.GetValueOrDefault("accepted") != "1") throw new FormatException("设备接收回执无效");
            accepted = true; return null;
        }
        if (!accepted) throw new FormatException("缺少接收回执，不能采信完成回执");
        var result = fields.GetValueOrDefault("result", "unknown");
        if (result is not ("completed" or "failed" or "stopped" or "preempted")) throw new FormatException("设备终态未知");
        if (result == "completed") {
            var key = operation == "stop" ? "stopped" : "enabled";
            var expectedMask = operation == "disable" ? 0 : request.Operation == "stop-all" ? 0x7f : 1 << (request.MotorId - 1);
            if (!int.TryParse(fields.GetValueOrDefault(key), NumberStyles.HexNumber, CultureInfo.InvariantCulture, out var mask) || mask != expectedMask)
                throw new FormatException("设备终态电机掩码不匹配");
        }
        return Receipt(result, result == "completed" ? "设备报告命令完成；仍需新鲜反馈确认状态，不能据此认定物理停止" : "设备报告命令未正常完成；未自动重试");
    }
    /** 保留已收回执；超时结果未知时也不补发动作。 */
    public DebugReceipt Receipt(string result, string detail) => new(requestId, request.Operation, request.MotorId, accepted, result, detail, matchedLines.ToArray());
}
