/** @file 固定ID7远程LCD适配协议；参数使用输出轴rad、rad/s、Nm，提交永不重发。 */
using System.Globalization;
namespace Studio.Gateway;
/** 冻结设备与参数身份；home的Delta为绝对输出轴目标，Kp/Kd专用于无损boot/coord身份。 */
public sealed record RemoteControlRequest(string Operation, double Delta = 0, double Speed = 0.02,
    double Kp = 120, double Kd = 0.30, double Torque = 0, uint Duration = 1000,
    int MotorId = 7, string Coordinate = "output-rad", uint TransactionId = 0, uint Token = 0,
    bool CooledConfirmed = false, int? PeakMilliNm = null);
/** 原始板端字段保留用于审核；accepted不代表物理运动完成。 */
public sealed record RemoteControlReceipt(uint RequestId, string Operation, string Result,
    Dictionary<string, string> Fields, string Detail);
/** 所有可写字段及操作固定白名单，禁止任意串口文本。 */
public static class RemoteControlProtocol
{
    /** 验证有限数值、坐标、身份与固定ID7范围；最终保护仍由固件执行。 */
    public static void Validate(RemoteControlRequest request)
    {
        if (request.MotorId != 7 || request.Coordinate != "output-rad") throw new ArgumentException("仅支持ID7输出轴rad坐标");
        if (request.CooledConfirmed && request.Operation != "torque_reset") throw new ArgumentException("冷却确认仅用于单独的转矩配额维护");
        if (request.PeakMilliNm.HasValue && (request.Operation != "torque_reset" || request.PeakMilliNm is < 3500 or > 6240))
            throw new ArgumentException("试验转矩退出线仅用于冷却配额维护，范围3500–6240毫牛米（电机输出轴）");
        if (request.Operation is "caps" or "torque") return;
        if (request.Operation == "torque_reset") {
            if(request.Token==0||!request.CooledConfirmed)throw new ArgumentException("恢复实验配额需要板端令牌及本次充分冷却确认");
            return;
        }
        if (request.Operation is "heartbeat") { if (request.Token == 0) throw new ArgumentException("缺少远程租约"); return; }
        if (request.Operation is "result") { if (request.TransactionId == 0) throw new ArgumentException("缺少原事务编号"); return; }
        if (request.Operation is not ("acquire" or "release" or "pos" or "pos_load" or "home" or "configure" or "hold" or "lock" or "preflight" or "mode_mit" or "mode_pos")) throw new ArgumentException("远程操作未开放");
        if (request.Token == 0) throw new ArgumentException("必须先读取设备能力并冻结审核令牌");
        if (!new[] { request.Delta, request.Speed, request.Kp, request.Kd, request.Torque }.All(double.IsFinite)) throw new ArgumentException("参数必须为有限数值");
        if(request.Operation=="home") {
            if(!ValidIdentity(request.Kp)||!ValidIdentity(request.Kd)||request.Torque!=0||request.Duration!=1000||request.Speed<=0)
                throw new ArgumentException("回零需要有效boot/coord整数身份、正速度、零前馈及固定一秒到位阶段");
            return;
        }
        if (request.Operation == "pos_load") {
            if (request.Kp != 0 || request.Kd != 0 || request.Torque != 0 || request.Delta == 0 ||
                request.Duration % 1000 != 0 || request.Duration is < 1000 or > 60000 || request.Speed <= 0)
                throw new ArgumentException("POS带载需要非零位移、正速度、1至60整秒保持，Kp/Kd/前馈必须为零");
            return;
        }
        if (request.Kp is < 80 or > 200 || request.Kd is < .20 or > .40 || request.Speed <= 0 || request.Duration is < 1000 or > 60000)
            throw new ArgumentException("Kp需80–200，Kd需0.20–0.40，速度与持续时间必须大于零");
    }
    /** IEEE double可无损表示全部uint32；拒绝小数或越界身份，不做截断。 */
    private static bool ValidIdentity(double value)=>double.IsFinite(value)&&value>=1&&value<=uint.MaxValue&&Math.Truncate(value)==value;
    /** 使用不受系统语言影响的十进制生成严格字段集。 */
    public static string Build(uint requestId, RemoteControlRequest request)
    {
        Validate(request);
        if (requestId == 0) throw new ArgumentException("请求编号不能为零");
        if (request.Operation == "caps") return $"{requestId} ui caps\n";
        if (request.Operation == "torque") return $"{requestId} ui torque\n";
        if (request.Operation == "torque_reset") return request.PeakMilliNm is int peakMilliNm ?
            FormattableString.Invariant($"{requestId} ui torque_reset token={request.Token} cold=1 peak_mnm={peakMilliNm}\n") :
            $"{requestId} ui torque_reset token={request.Token} cold=1\n";
        if (request.Operation == "heartbeat") return $"{requestId} ui heartbeat token={request.Token}\n";
        if (request.Operation == "result") return $"{requestId} ui result id={request.TransactionId}\n";
        var command = request.Operation=="home"?
            FormattableString.Invariant($"{requestId} ui submit token={request.Token} op=home delta={request.Delta:0.######} speed={request.Speed:0.######} kp={(uint)request.Kp} kd={(uint)request.Kd} torque=0 duration=1000\n"):
            FormattableString.Invariant($"{requestId} ui submit token={request.Token} op={request.Operation} delta={request.Delta:0.######} speed={request.Speed:0.######} kp={request.Kp:0.######} kd={request.Kd:0.######} torque={request.Torque:0.######} duration={request.Duration}\n");
        if (command.Length > 160) throw new ArgumentException("参数超出板端报文长度");
        return command;
    }
    /** 严格关联本次请求、固定路径和schema，未知/错误旧固件不被当作能力。 */
    public static RemoteControlReceipt? Match(string line, uint requestId, RemoteControlRequest request)
    {
        var parts = line.Split(' ', StringSplitOptions.RemoveEmptyEntries);
        if (parts.Length < 4 || !uint.TryParse(parts[1], out var responseId) || responseId != requestId) return null;
        var path = request.Operation is "caps" or "result" or "heartbeat" or "torque" or "torque_reset" ? request.Operation : "submit";
        if (parts[2] != "ui" || parts[3] != path) return null;
        if (parts[0] == "error") return new(requestId, request.Operation, "rejected", new(), "设备拒绝或固件不支持远程LCD适配：" + line);
        if (parts[0] != "ok") return null;
        var fields = new Dictionary<string, string>();
        foreach (var field in parts.Skip(4)) {
            var separator = field.IndexOf('=');
            if (separator <= 0 || !fields.TryAdd(field[..separator], field[(separator + 1)..])) throw new FormatException("远程回执字段重复或无效");
        }
        if (fields.GetValueOrDefault("schema") != "1") throw new FormatException("不支持的远程协议版本");
        if (request.Operation == "result" && fields.GetValueOrDefault("id") != request.TransactionId.ToString(CultureInfo.InvariantCulture))
            throw new FormatException("原动作事务编号不匹配");
        var result = fields.GetValueOrDefault("result", request.Operation == "caps" ? "capability" : "accepted");
        return new(requestId, request.Operation, result, fields, "板端回执；不代表物理运动或失能已确认");
    }
}
