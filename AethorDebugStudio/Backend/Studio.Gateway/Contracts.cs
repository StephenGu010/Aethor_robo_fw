/** @file 网关公开值模型；未知字段使用null，快照不授予控制资格。 */
namespace Studio.Gateway;
/** 固定只读查询分类；不接受用户提供的原始命令。 */
public enum ReadQuery { Info, State, Motors, Motor, Config, Can }
/** 单轴原始反馈和接收时间。 */
public sealed record MotorSnapshot(int Id, string State, double? PositionDeg, double? SpeedDegS,
    double? TorqueNm, double? MosC, double? RotorC, double? Fault, double? AgeMs,
    DateTimeOffset? ReceivedAt, double? PmaxDeg, string Raw,
    double? Register50Raw = null, double? Register50AgeMs = null,
    double? Register51Raw = null, double? Register51AgeMs = null,
    int? PmContract = null, double? PmVelocityRadS = null,
    uint? PmBoot = null, uint? PmEpoch = null,
    uint? CoordinateBoot = null, uint? CoordinateReference = null);
/** 本机通信会话快照；只读不意味着反馈一定有效。 */
public sealed record GatewaySnapshot(bool Connected, string? Port, string Status, string? Error,
    DateTimeOffset? ReceivedAt, Dictionary<string, string> Info, MotorSnapshot[] Motors,
    string[] Lines, bool ReadOnly = true, bool DebugAvailable = true, DebugReceipt[]? DebugReceipts = null,
    bool TorquePreparationAvailable = true);
/** 连接仅允许系统已枚举端口，波特率固定。 */
public sealed record ConnectRequest(string Port);
