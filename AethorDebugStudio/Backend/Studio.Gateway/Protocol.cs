/** @file aethor-text-v1查询协议适配；查询与显式调试事务分离。 */
using System.Globalization;
using System.Text;
namespace Studio.Gateway;
/** 有界逐字分帧器，超长行隔离到下个换行。 */
public sealed class LineFramer
{
    private readonly StringBuilder buffer = new();
    private bool discard;
    /** 接收单个ASCII字符并仅在完整换行后返回一行。 */
    public string? Push(char character)
    {
        if (character == '\n') { var line = discard ? null : buffer.ToString(); buffer.Clear(); discard = false; return string.IsNullOrEmpty(line) ? null : line; }
        if (character == '\r') return null;
        if (character > 127 || character < 32 || buffer.Length >= 4096) { discard = true; buffer.Clear(); return null; }
        if (!discard) buffer.Append(character);
        return null;
    }
}
/** 严格查询构造与请求号关联；不提供任意帧发送。 */
public static class TextProtocol
{
    /** 将已定义的枚举转换成固定协议路径。 */
    public static string Path(ReadQuery query) => query switch {
        ReadQuery.Info => "show info", ReadQuery.State => "show state",
        ReadQuery.Motors => "show motors", ReadQuery.Motor => "show motor",
        ReadQuery.Config => "show config", ReadQuery.Can => "show diag can",
        _ => throw new ArgumentException("Unsupported read-only query")
    };
    /** 构造固定查询，不允许0编号、越界轴或命令拼接。 */
    public static string BuildQuery(uint requestId, ReadQuery query, int motorId = 1)
    {
        if (requestId == 0 || motorId < 1 || motorId > 7) throw new ArgumentException("Invalid query identity");
        return requestId.ToString(CultureInfo.InvariantCulture) + " " + Path(query) +
            (query is ReadQuery.Motor or ReadQuery.Config ? " " + motorId.ToString(CultureInfo.InvariantCulture) : "") + "\n";
    }
    /** 仅接受匹配请求号及路径的响应，错误响应不能成为成功。 */
    public static Dictionary<string, string>? Match(string line, uint requestId, ReadQuery query, int motorId)
    {
        var parts = line.Split(' ', StringSplitOptions.RemoveEmptyEntries);
        if (parts.Length < 4 || (parts[0] != "ok" && parts[0] != "error") ||
            !uint.TryParse(parts[1], out var responseId) || responseId != requestId) return null;
        var pathWords = Path(query).Split(' ');
        if (parts.Length < 2 + pathWords.Length || !parts.Skip(2).Take(pathWords.Length).SequenceEqual(pathWords)) return null;
        if (parts[0] == "error") throw new FormatException("设备拒绝只读查询：" + string.Join(" ", parts.Skip(4)));
        var fields = new Dictionary<string, string>(StringComparer.Ordinal);
        foreach (var token in parts.Skip(2 + pathWords.Length)) {
            var separator = token.IndexOf('=');
            if (separator <= 0 || !fields.TryAdd(token[..separator], token[(separator + 1)..])) throw new FormatException("响应字段缺失或重复");
        }
        if (query is ReadQuery.Motor or ReadQuery.Config && (!fields.TryGetValue("joint", out var joint) || joint != motorId.ToString(CultureInfo.InvariantCulture)))
            throw new FormatException("电机响应身份不一致");
        return fields;
    }
    /** 解析有限数字；问号、NaN与无穷始终保留未知。 */
    public static double? Number(string? value) => double.TryParse(value, NumberStyles.Float, CultureInfo.InvariantCulture, out var number) && double.IsFinite(number) ? number : null;
    /** 启动号及采样代次只接收十进制uint32；缺失或非法身份不得沿用旧值。 */
    private static uint? Identity(string? value) => uint.TryParse(value, NumberStyles.None, CultureInfo.InvariantCulture, out var identity) ? identity : null;
    /** 原始位置寄存器独立计时；缺失或500ms过期不升级为有效状态反馈。 */
    private static (double? value, double? age) Register(Dictionary<string, string> fields, string register)
    {
        var value = Number(fields.GetValueOrDefault(register + "_raw"));
        var age = Number(fields.GetValueOrDefault(register + "_age_ms"));
        return value is not null && age is >= 0 and < 500 ? (value, age) : (null, null);
    }
    /** 构建单轴快照，不根据旧反馈推断模式或标定。 */
    public static MotorSnapshot ParseMotor(int expectedId, Dictionary<string, string> fields, string raw)
    {
        if (fields.GetValueOrDefault("joint") != expectedId.ToString(CultureInfo.InvariantCulture)) throw new FormatException("电机编号不匹配");
        var age = Number(fields.GetValueOrDefault("age_ms"));
        if (age < 0 || age >= uint.MaxValue) age = null;
        var state = fields.GetValueOrDefault("state", "unknown");
        var register50 = Register(fields, "reg50");
        var register51 = Register(fields, "reg51");
        // 仅ID7确认的转子弧度契约有效；估算速度使用0x50自己的样本年龄。
        int? pmContract = expectedId == 7 && fields.GetValueOrDefault("pm_contract") == "1" ? 1 : null;
        var pmVelocity = pmContract == 1 && register50.value is not null ? Number(fields.GetValueOrDefault("pm_vel")) : null;
        var valid = state is "disabled" or "moving" or "holding" or "fault";
        if (!valid) age = null;
        return new(expectedId, state,
            valid ? Number(fields.GetValueOrDefault("pos_deg")) : null, valid ? Number(fields.GetValueOrDefault("speed_deg_s")) : null,
            valid ? Number(fields.GetValueOrDefault("torque_nm")) : null, valid ? Number(fields.GetValueOrDefault("mos_c")) : null,
            valid ? Number(fields.GetValueOrDefault("rotor_c")) : null, valid ? Number(fields.GetValueOrDefault("fault")) : null,
            age, DateTimeOffset.UtcNow, Number(fields.GetValueOrDefault("pmax_deg")), raw,
            register50.value, register50.age, register51.value, register51.age, pmContract, pmVelocity,
            pmContract == 1 ? Identity(fields.GetValueOrDefault("pm_boot")) : null,
            pmContract == 1 ? Identity(fields.GetValueOrDefault("pm_epoch")) : null,
            expectedId == 7 ? Identity(fields.GetValueOrDefault("coord_boot")) : null,
            expectedId == 7 ? Identity(fields.GetValueOrDefault("coord_ref")) : null);
    }
}
