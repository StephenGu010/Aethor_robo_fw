/** @file 单端口串口所有者；默认只读，显式停止/失能事务不自动重试。 */
using System.Diagnostics;
using System.IO.Ports;
namespace Studio.Gateway;
/** 串口会话与有界反馈缓存；任何通信故障终止本次读取。 */
public sealed class SerialGateway : IDisposable
{
    private readonly object sync = new();
    private readonly SemaphoreSlim lifecycle = new(1, 1);
    private readonly Queue<string> lines = new();
    private readonly MotorSnapshot[] motors = Enumerable.Range(1, 7).Select(UnknownMotor).ToArray();
    private Dictionary<string, string> info = new();
    private SerialPort? serial;
    private CancellationTokenSource? cancellation;
    private Task? worker;
    private bool connected;
    private string? port;
    private string status = "未连接";
    private string? error;
    private DateTimeOffset? receivedAt;
    private uint requestId;
    private readonly List<PendingDebug> pendingDebug = new();
    private readonly DebugPreparationGate preparationGate = new();
    private bool preparationPending;
    private readonly Queue<DebugReceipt> debugReceipts = new();
    private readonly AutoResetEvent commandReady = new(false);
    private PendingRemote? pendingRemote;
    private bool remoteBusy;
    private RemoteControlReceipt? lastRemoteReceipt;
    private uint remoteLease;
    private uint uiSessionBoot;
    private long remoteHeartbeatTick;
    private readonly HashSet<uint> consumedRemoteTokens = new();
    /** 至多一个远程请求；STOP仍使用原独立优先队列。 */
    private sealed record PendingRemote(RemoteControlRequest Request, TaskCompletionSource<RemoteControlReceipt> Completion);
    /** 保留最近远程回执，HTTP中断后可只读恢复事务编号。 */
    public RemoteControlReceipt? RemoteReceipt() { lock (sync) return lastRemoteReceipt; }
    /** 冻结一次远程请求并拒绝重复点击；不主动连接设备。 */
    public Task<RemoteControlReceipt> RemoteAsync(RemoteControlRequest request)
    {
        if (request.Operation == "heartbeat") throw new ArgumentException("心跳由唯一串口线程持有，不向HTTP开放");
        RemoteControlProtocol.Validate(request);
        var completion = new TaskCompletionSource<RemoteControlReceipt>(TaskCreationOptions.RunContinuationsAsynchronously);
        lock (sync) {
            if (!connected || cancellation?.IsCancellationRequested != false) throw new InvalidOperationException("请先连接设备");
            if (uiSessionBoot==0 && request.Operation is not ("caps" or "result" or "torque"))throw new InvalidOperationException("UI连接握手未确认或主控已重启，请断开重连；本次未发送");
            if (remoteBusy || pendingRemote != null || preparationPending) throw new InvalidOperationException("远程事务繁忙；本次未发送");
            if (request.Operation is not ("caps" or "result" or "torque") && !consumedRemoteTokens.Add(request.Token)) throw new InvalidOperationException("审核令牌已使用，禁止重发");
            pendingRemote = new(request, completion);
        }
        commandReady.Set(); return completion.Task;
    }
    /** HTTP请求只排队，由唯一串口线程实际发送。 */
    private sealed record PendingDebug(DebugRequest Request, TaskCompletionSource<DebugReceipt> Completion, long PreparationGeneration);
    /** 构建零假设的初始电机快照。 */
    private static MotorSnapshot UnknownMotor(int id) => new(id, "unknown", null, null, null, null, null, null, null, null, null, "");
    /** 拒绝未枚举路径、非COM及换行注入。 */
    public static bool IsAllowedPort(string candidate, string[] available) =>
        System.Text.RegularExpressions.Regex.IsMatch(candidate, "^COM[1-9][0-9]{0,3}$", System.Text.RegularExpressions.RegexOptions.CultureInvariant) &&
        available.Contains(candidate, StringComparer.OrdinalIgnoreCase);
    /** 返回副本，避免HTTP序列化与后台线程竞争。 */
    public GatewaySnapshot Snapshot() { lock (sync) return new(connected, port, status, error, receivedAt, new(info), motors.ToArray(), lines.ToArray(), DebugReceipts: debugReceipts.Reverse().ToArray()); }
    /** 提交固定调试操作；STOP优先于尚未发送的诊断，离线时不排队。 */
    public Task<DebugReceipt> DebugAsync(DebugRequest request)
    {
        if (request.Operation == "diagnostics") {
            if (request.MotorId is < 1 or > 7) throw new ArgumentException("电机编号必须为1至7");
        } else _ = DebugProtocol.Operation(request);
        var completion = new TaskCompletionSource<DebugReceipt>(TaskCreationOptions.RunContinuationsAsynchronously);
        lock (sync) {
            if (!connected || worker is not { IsCompleted: false } || cancellation?.IsCancellationRequested != false)
                throw new InvalidOperationException("请先连接设备");
            if (request.Operation is "stop" or "stop-all" or "disable") preparationGate.Cancel();
            if (request.Operation == "torque-disable" &&
                (uiSessionBoot == 0 || preparationPending || pendingDebug.Count != 0 || remoteBusy || pendingRemote != null || remoteLease != 0))
                throw new InvalidOperationException("设备未空闲或UI身份未确认；未发送维护失能");
            if (request.Operation is "stop" or "stop-all") {
                pendingRemote?.Completion.TrySetException(new InvalidOperationException("未发送的远程动作被STOP取消"));
                pendingRemote = null;
                remoteLease = 0;
            }
            if (pendingDebug.Count >= 2 && request.Operation is "stop" or "stop-all") {
                var replaceIndex = pendingDebug.FindLastIndex(item => item.Request.Operation == "diagnostics");
                if (replaceIndex >= 0) {
                    pendingDebug[replaceIndex].Completion.TrySetException(new InvalidOperationException("尚未发送的诊断被STOP抢占"));
                    pendingDebug.RemoveAt(replaceIndex);
                }
            }
            if (pendingDebug.Count >= 2) throw new InvalidOperationException("设备事务繁忙，请等待回执；未发送本次请求");
            var pending = new PendingDebug(request, completion, preparationGate.Generation);
            if (request.Operation == "torque-disable") preparationPending = true;
            if (request.Operation is "stop" or "stop-all") pendingDebug.Insert(0, pending); else pendingDebug.Add(pending);
        }
        commandReady.Set(); return completion.Task;
    }
    /** 只在用户明确连接后打开；每条串口连接均建立新的UI身份，不发送电机指令。 */
    public async Task ConnectAsync(string candidate)
    {
        await lifecycle.WaitAsync();
        try {
            preparationGate.Cancel();
            if (!IsAllowedPort(candidate, SerialPort.GetPortNames())) throw new ArgumentException("请选择当前已枚举的COM端口");
            if (worker is { IsCompleted: false }) throw new InvalidOperationException("请先断开当前连接");
            cancellation?.Dispose();
            cancellation = new CancellationTokenSource();
            SerialSessionCleanup.Finish(() => serial?.Dispose(), () => serial = null);
            serial = new SerialPort(candidate, 115200, Parity.None, 8, StopBits.One) {
                DtrEnable = false, RtsEnable = false, Handshake = Handshake.None, ReadTimeout = 150, WriteTimeout = 500, NewLine = "\n"
            };
            try { serial.Open(); }
            catch {
                SerialSessionCleanup.Finish(serial.Dispose, () => serial = null);
                cancellation.Dispose(); cancellation = null;
                throw;
            }
            lock (sync) {
                port = candidate; connected = true; status = "只读查询中"; error = null; receivedAt = null; info = new(); lines.Clear(); debugReceipts.Clear();
                requestId = (uint)Random.Shared.Next(1_000_000, int.MaxValue);
                consumedRemoteTokens.Clear();
                lastRemoteReceipt = null;
                remoteLease = 0;
                uiSessionBoot = 0;
                for (var index = 0; index < motors.Length; index++) motors[index] = UnknownMotor(index + 1);
            }
            var ownedSerial = serial;
            var token = cancellation.Token;
            worker = Task.Run(() => Poll(ownedSerial, token), token);
        } finally { lifecycle.Release(); }
    }
    /** 取消轮询并等待串口释放；不发送停止/失能或参数写入。 */
    public async Task DisconnectAsync()
    {
        preparationGate.Cancel();
        await lifecycle.WaitAsync();
        try {
            cancellation?.Cancel();
            if (worker != null) await worker;
            SerialSessionCleanup.Finish(() => serial?.Dispose(), () => {
                serial = null;
                lock (sync) { connected = false; status = "已断开 · 旧反馈保留"; }
            });
        } finally { lifecycle.Release(); }
    }
    /** 按固定白名单逐轴轮询，并在查询之间处理显式调试事务。 */
    private void Poll(SerialPort ownedSerial, CancellationToken token)
    {
        try {
            requestId=requestId==uint.MaxValue?1:requestId+1;
            var session=GatewayProtocolSession.Execute(requestId,Guid.NewGuid().ToString("N"),
                ownedSerial.Write,ownedSerial.ReadByte,LogLine,token);
            var identity = Exchange(ownedSerial, ReadQuery.Info, 1, token);
            identity.fields["ui_session"]=session["session"];identity.fields["ui_nonce"]=session["nonce"];
            lock (sync) {info = identity.fields;uiSessionBoot=uint.Parse(session["boot"],System.Globalization.CultureInfo.InvariantCulture);}
            while (!token.IsCancellationRequested) {
                var state = Exchange(ownedSerial, ReadQuery.State, 1, token);
                lock (sync) { info["controller_state"] = state.fields.GetValueOrDefault("state", "unknown"); info["controller_fault"] = state.fields.GetValueOrDefault("fault", "unknown"); }
                for (var motorId = 1; motorId <= 7; motorId++) {
                    ProcessDebug(ownedSerial, token);
                    ProcessRemote(ownedSerial, token);
                    ProcessRemoteHeartbeat(ownedSerial, token);
                    var response = Exchange(ownedSerial, ReadQuery.Motor, motorId, token);
                    var snapshot = TextProtocol.ParseMotor(motorId, response.fields, response.raw);
                    lock (sync) { motors[motorId - 1] = snapshot; receivedAt = DateTimeOffset.UtcNow; status = "只读连接"; }
                }
                ProcessDebug(ownedSerial, token);
                // 只读快照约4Hz，配合模式2各位置寄存器约5Hz刷新；不改变电机控制频率。
                WaitHandle.WaitAny(new[] { token.WaitHandle, commandReady }, 200);
            }
        } catch (OperationCanceledException) { }
        catch (Exception exception) when (exception is IOException or UnauthorizedAccessException or TimeoutException or FormatException or InvalidOperationException) {
            lock (sync) { error = exception is TimeoutException ? "只读查询超时，连接已关闭；未自动重试" : exception.Message; status = "通信中断"; }
        } finally {
            SerialSessionCleanup.Finish(ownedSerial.Close, () => {
                lock (sync) {
                    connected = false;
                    preparationPending = false;
                    preparationGate.Cancel();
                    foreach (var pending in pendingDebug) pending.Completion.TrySetException(new InvalidOperationException("连接已结束，本次排队操作未发送"));
                    pendingDebug.Clear();
                    pendingRemote?.Completion.TrySetException(new InvalidOperationException("连接已结束，排队远程请求未发送"));
                    pendingRemote = null;
                }
            });
        }
    }
    /** 处理一次显式事务；超时或取消保留结果未知，不补发或连带失能。 */
    private void ProcessDebug(SerialPort ownedSerial, CancellationToken token, bool stopOnly = false)
    {
        PendingDebug pending;
        lock (sync) {
            if (pendingDebug.Count == 0) return;
            if (stopOnly && pendingDebug[0].Request.Operation is not ("stop" or "stop-all")) return;
            pending = pendingDebug[0]; pendingDebug.RemoveAt(0);
        }
        try {
            if (pending.Request.Operation == "diagnostics") {
                var captured = new List<string>();
                foreach (var query in new[] { ReadQuery.Config, ReadQuery.Can, ReadQuery.State, ReadQuery.Motor }) {
                    ProcessDebug(ownedSerial, token, true);
                    var response = Exchange(ownedSerial, query, pending.Request.MotorId, token); captured.Add(response.raw);
                    if (query == ReadQuery.Motor) {
                        var motor = TextProtocol.ParseMotor(pending.Request.MotorId, response.fields, response.raw);
                        lock (sync) motors[motor.Id - 1] = motor;
                    }
                }
                CompleteDebug(pending, new(requestId, "diagnostics", pending.Request.MotorId, true, "completed", "已读取主控配置、CAN统计及缓存反馈；没有发送使能、运动或驱动参数写入", captured.ToArray()));
                return;
            }
            token.ThrowIfCancellationRequested();
            requestId = requestId == uint.MaxValue ? 1 : requestId + 1;
            Action<string> write = pending.Request.Operation == "torque-disable"
                ? command => preparationGate.Write(pending.PreparationGeneration, ownedSerial.Write, command)
                : ownedSerial.Write;
            var receipt = DebugTransaction.Execute(requestId, pending.Request, write, ownedSerial.ReadByte, LogLine, token);
            CompleteDebug(pending, receipt);
        } catch (Exception exception) when (exception is IOException or UnauthorizedAccessException or TimeoutException or FormatException or InvalidOperationException or OperationCanceledException) {
            if (pending.Request.Operation == "torque-disable") { lock(sync) preparationPending = false; }
            pending.Completion.TrySetException(new InvalidOperationException("事务未完成：" + exception.Message));
            if (exception is IOException or UnauthorizedAccessException or OperationCanceledException) throw;
        }
    }
    /** 回执留在网关缓存，浏览器刷新或HTTP超时不会抹去已执行事务。 */
    private void CompleteDebug(PendingDebug pending, DebugReceipt receipt)
    {
        lock (sync) {
            if (pending.Request.Operation == "torque-disable") preparationPending = false;
            if (debugReceipts.Count >= 10) debugReceipts.Dequeue(); debugReceipts.Enqueue(receipt);
        }
        pending.Completion.TrySetResult(receipt);
    }
    /** 在既有STOP之后发送一次远程请求，所有传输仍由唯一串口线程拥有。 */
    private void ProcessRemote(SerialPort ownedSerial, CancellationToken token)
    {
        PendingRemote pending;
        lock (sync) {
            if (pendingRemote == null || pendingDebug.Any(entry => entry.Request.Operation is "stop" or "stop-all")) return;
            pending = pendingRemote; pendingRemote = null; remoteBusy = true;
        }
        try {
            requestId = requestId == uint.MaxValue ? 1 : requestId + 1;
            var receipt = RemoteControlTransaction.Execute(requestId, pending.Request,
                ownedSerial.Write, ownedSerial.ReadByte, LogLine, token, () => ProcessDebug(ownedSerial, token, true));
            lock (sync) {
                if(uiSessionBoot!=0 && receipt.Fields.TryGetValue("boot",out var boot) && boot!=uiSessionBoot.ToString(System.Globalization.CultureInfo.InvariantCulture)) {
                    uiSessionBoot=0;remoteLease=0;
                    throw new InvalidOperationException("主控启动号已变化，请断开重连建立UI会话；未重发请求");
                }
                lastRemoteReceipt = receipt;
                if (pending.Request.Operation == "caps" && uint.TryParse(receipt.Fields.GetValueOrDefault("last"), out var lastId)) requestId = Math.Max(requestId,lastId);
                if (pending.Request.Operation == "acquire" && receipt.Result == "queued" && uint.TryParse(receipt.Fields.GetValueOrDefault("lease"),out var lease)) remoteLease = lease;
                if (pending.Request.Operation == "release") remoteLease = 0;
            }
            pending.Completion.TrySetResult(receipt);
        } catch (Exception error) { pending.Completion.TrySetException(error); }
        finally { lock (sync) remoteBusy = false; }
    }
    /** 仅已申请代理控制权时维持独立板端5秒租约；不产生使能、运动或自动重试动作。 */
    private void ProcessRemoteHeartbeat(SerialPort ownedSerial, CancellationToken token)
    {
        uint lease;
        lock(sync) {
            lease=remoteLease;
            if(lease==0 || Environment.TickCount64-remoteHeartbeatTick<1000) return;
            remoteHeartbeatTick=Environment.TickCount64;
        }
        requestId=requestId==uint.MaxValue?1:requestId+1;
        var receipt=RemoteControlTransaction.Execute(requestId,new("heartbeat",Token:lease),ownedSerial.Write,
            ownedSerial.ReadByte,LogLine,token,()=>ProcessDebug(ownedSerial,token,true));
        if(receipt.Result!="alive") {lock(sync)remoteLease=0;}
    }
    /** 保存有界原始响应，查询与控制回执统一留痕。 */
    private void LogLine(string line)
    {
        lock (sync) { if (lines.Count >= 80) lines.Dequeue(); lines.Enqueue(line[..Math.Min(line.Length, 1000)]); }
    }
    /** 发送枚举生成的只读查询，读取有时间与行数上限。 */
    private (Dictionary<string, string> fields, string raw) Exchange(SerialPort ownedSerial, ReadQuery query, int motorId, CancellationToken token)
    {
        token.ThrowIfCancellationRequested();
        requestId = requestId == uint.MaxValue ? 1 : requestId + 1;
        ownedSerial.Write(TextProtocol.BuildQuery(requestId, query, motorId));
        var framer = new LineFramer();
        var clock = Stopwatch.StartNew();
        var observedLines = 0;
        while (clock.ElapsedMilliseconds < 1500 && observedLines < 64) {
            token.ThrowIfCancellationRequested();
            int value;
            try { value = ownedSerial.ReadByte(); } catch (TimeoutException) { continue; }
            var line = framer.Push((char)value);
            if (line == null) continue;
            observedLines++;
            LogLine(line);
            var response = TextProtocol.Match(line, requestId, query, motorId);
            if (response != null) return (response, line);
        }
        throw new TimeoutException("No matching response");
    }
    /** 宿主退出时释放串口；只读网关不改变电机运行状态。 */
    public void Dispose() { cancellation?.Cancel(); worker?.GetAwaiter().GetResult(); SerialSessionCleanup.Finish(() => serial?.Dispose(), () => serial = null); cancellation?.Dispose(); commandReady.Dispose(); lifecycle.Dispose(); }
}
