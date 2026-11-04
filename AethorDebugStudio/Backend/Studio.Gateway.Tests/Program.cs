/** @file 使用合成串口响应验证白名单、分帧及未知值；不打开COM。 */
using Studio.Gateway;
using System.Text;
var failures = new List<string>();
int count = 0;
/** 执行单个验证并保留失败原因。 */
void Check(string name, Action action) { count++; try { action(); Console.WriteLine("PASS " + name); } catch (Exception error) { failures.Add(name + ": " + error.Message); } }
/** 断言条件。 */
void Require(bool condition) { if (!condition) throw new Exception("assertion failed"); }
/** 断言拒绝坏输入。 */
void Reject(Action action) { try { action(); } catch (ArgumentException) { return; } catch (FormatException) { return; } throw new Exception("unsafe input accepted"); }
Check("查询白名单只构造固定命令", () => { Require(TextProtocol.BuildQuery(7, ReadQuery.Motor, 7) == "7 show motor 7\n"); Reject(() => TextProtocol.BuildQuery(0, ReadQuery.State)); Reject(() => TextProtocol.BuildQuery(1, (ReadQuery)99)); Reject(() => TextProtocol.BuildQuery(1, ReadQuery.Motor, 8)); });
Check("响应请求号和路径严格对应", () => { Require(TextProtocol.Match("ok 7 show state state=ready", 7, ReadQuery.State, 1) != null); Require(TextProtocol.Match("ok 6 show state state=ready", 7, ReadQuery.State, 1) == null); Require(TextProtocol.Match("ok 7 show info build=x", 7, ReadQuery.State, 1) == null); Reject(() => TextProtocol.Match("ok 7 show state state=a state=b", 7, ReadQuery.State, 1)); });
Check("可识别错误响应而不当作成功", () => Reject(() => TextProtocol.Match("error 7 show state code=not_ready", 7, ReadQuery.State, 1)));
Check("分片合并、CRLF和粘包", () => { var framer = new LineFramer(); var lines = new List<string>(); foreach (var character in "ok 1\r\nok 2\n") { var line = framer.Push(character); if (line != null) lines.Add(line); } Require(lines.SequenceEqual(new[] { "ok 1", "ok 2" })); });
Check("超长帧拒绝到换行并恢复", () => { var framer = new LineFramer(); foreach (var character in new string('x', 4097)) framer.Push(character); Require(framer.Push('\n') == null); string? last = null; foreach (var character in "ok\n") last = framer.Push(character); Require(last == "ok"); });
Check("未知反馈不变成0或失能确认", () => { var motor = TextProtocol.ParseMotor(7, new Dictionary<string, string> { ["joint"] = "7", ["state"] = "unknown", ["pos_deg"] = "?", ["torque_nm"] = "0", ["age_ms"] = "4294967295" }, "raw"); Require(motor.PositionDeg == null && motor.TorqueNm == null && motor.AgeMs == null && motor.State == "unknown"); });
Check("有效反馈中的零值保持为零", () => { var motor=TextProtocol.ParseMotor(7,new Dictionary<string,string>{["joint"]="7",["state"]="disabled",["pos_deg"]="0",["torque_nm"]="0",["age_ms"]="0"},"");Require(motor.PositionDeg==0&&motor.TorqueNm==0&&motor.AgeMs==0); });
Check("错误轴和非有限字段不被采用", () => { Reject(() => TextProtocol.ParseMotor(7, new Dictionary<string, string> { ["joint"] = "1" }, "")); Require(TextProtocol.Number("NaN") == null && TextProtocol.Number("Infinity") == null); });
Check("无有效反馈的默认0不冒充实际位置", () => { var motor=TextProtocol.ParseMotor(7,new Dictionary<string,string>{["joint"]="7",["state"]="absent",["pos_deg"]="0",["torque_nm"]="0",["age_ms"]="0"},"");Require(motor.PositionDeg==null&&motor.TorqueNm==null&&motor.AgeMs==null); });
Check("待机寄存器值独立保留且不授予标准反馈", () => {
    var motor=TextProtocol.ParseMotor(7,new Dictionary<string,string>{["joint"]="7",["state"]="absent",["age_ms"]="4294967295",["reg51_raw"]="0.25",["reg51_age_ms"]="4"},"");
    var json=System.Text.Json.JsonSerializer.SerializeToElement(motor);
    Require(json.GetProperty("Register51Raw").GetDouble()==0.25 && json.GetProperty("Register51AgeMs").GetDouble()==4);
    Require(motor.PositionDeg==null && motor.TorqueNm==null && motor.AgeMs==null && motor.State=="absent");
});
Check("缺失过期及非有限寄存器值不绘图", () => {
    foreach(var pair in new[]{("?","0"),("NaN","1"),("0.5","500"),("0.5","4294967295"),("0.5","-1")}) {
        var motor=TextProtocol.ParseMotor(7,new Dictionary<string,string>{["joint"]="7",["reg51_raw"]=pair.Item1,["reg51_age_ms"]=pair.Item2},"");
        Require(System.Text.Json.JsonSerializer.SerializeToElement(motor).GetProperty("Register51Raw").ValueKind==System.Text.Json.JsonValueKind.Null);
    }
});
Check("RID50契约只接受ID7且估算速度独立于标准反馈", () => {
    var fields = new Dictionary<string,string>{["joint"]="7",["state"]="absent",["pm_contract"]="1",["pm_vel"]="2.5",["pm_boot"]="41",["pm_epoch"]="7",["reg50_raw"]="1.25",["reg50_age_ms"]="4"};
    var json = System.Text.Json.JsonSerializer.SerializeToElement(TextProtocol.ParseMotor(7, fields, ""));
    Require(json.TryGetProperty("PmContract", out var contract) && contract.GetInt32()==1);
    Require(json.GetProperty("PmVelocityRadS").GetDouble()==2.5 && json.GetProperty("SpeedDegS").ValueKind==System.Text.Json.JsonValueKind.Null);
    Require(json.TryGetProperty("PmBoot",out var boot) && boot.GetUInt32()==41 && json.GetProperty("PmEpoch").GetUInt32()==7);
    foreach(var invalidIdentity in new[]{"-1","1.5","4294967296","?"}) {
        var badIdentity = new Dictionary<string,string>(fields){["pm_boot"]=invalidIdentity};
        var identityJson=System.Text.Json.JsonSerializer.SerializeToElement(TextProtocol.ParseMotor(7,badIdentity,""));
        Require(identityJson.GetProperty("PmBoot").ValueKind==System.Text.Json.JsonValueKind.Null);
    }
    foreach(var scenario in new[]{"legacy","axis","stale","nan"}) {
        var altered = new Dictionary<string,string>(fields);
        if(scenario=="legacy") altered.Remove("pm_contract");
        if(scenario=="axis") altered["joint"]="6";
        if(scenario=="stale") altered["reg50_age_ms"]="500";
        if(scenario=="nan") altered["pm_vel"]="NaN";
        var rejected = System.Text.Json.JsonSerializer.SerializeToElement(TextProtocol.ParseMotor(scenario=="axis"?6:7,altered,""));
        Require(rejected.GetProperty("PmVelocityRadS").ValueKind==System.Text.Json.JsonValueKind.Null);
    }
});
Check("端口校验只接受已枚举COM", () => { Require(SerialGateway.IsAllowedPort("COM7", new[] { "COM7" })); Require(!SerialGateway.IsAllowedPort("COM7\n8 bench enable", new[] { "COM7" })); Require(!SerialGateway.IsAllowedPort("COM8", new[] { "COM7" })); });
Check("初始网关完全断开且无数据", () => { using var gateway = new SerialGateway(); var state = gateway.Snapshot(); Require(!state.Connected && state.ReadOnly && state.Motors.All(motor => motor.PositionDeg == null)); });
Check("Windows完整性级别区分低权限和普通用户", () => { Require(GatewayExecutionContext.ClassifyIntegrity(4096)=="low"); Require(GatewayExecutionContext.ClassifyIntegrity(8192)=="medium"); Require(GatewayExecutionContext.ClassifyIntegrity(12288)=="high"); Require(GatewayExecutionContext.ClassifyIntegrity(null)=="unknown"); });
Check("访问失败提示不把低权限错误误报成端口占用", () => { var restricted=GatewayExecutionContext.AccessDeniedMessage("COM4","low");Require(restricted.Contains("低权限")&&restricted.Contains("普通用户"));var normal=GatewayExecutionContext.AccessDeniedMessage("COM4","medium");Require(normal.Contains("占用")&&normal.Contains("权限")); });
Check("调试接口不允许使能、运动、原始命令和未承托失能", () => {
    Require(DebugProtocol.Build(8, new("stop", 7)) == "8 bench stop 7\n");
    Require(DebugProtocol.Build(9, new("stop-all", 7)) == "9 bench stop 1,2,3,4,5,6,7\n");
    Require(DebugProtocol.Build(10, new("disable", 7, true)) == "10 bench disable 7\n");
    Reject(() => DebugProtocol.Build(1, new("enable", 7)));
    Reject(() => DebugProtocol.Build(1, new("stop\nbench enable", 7)));
    Reject(() => DebugProtocol.Build(1, new("disable", 7)));
    Reject(() => DebugProtocol.Build(1, new("stop", 0)));
});
Check("调试回执必须ACK后DONE且匹配请求号和电机掩码", () => {
    var tracker = new DebugReplyTracker(12, new("stop", 7));
    Require(tracker.Accept("done 11 bench stop result=completed stopped=40") == null);
    Require(tracker.Accept("ok 12 bench stop accepted=1") == null);
    var receipt = tracker.Accept("done 12 bench stop result=completed stopped=40");
    Require(receipt?.Result == "completed" && receipt.Accepted && !receipt.FeedbackConfirmed);
    Reject(() => new DebugReplyTracker(12, new("stop", 7)).Accept("done 12 bench stop result=completed stopped=40"));
    var mismatch = new DebugReplyTracker(12, new("stop", 7));
    mismatch.Accept("ok 12 bench stop accepted=1");
    Reject(() => mismatch.Accept("done 12 bench stop result=completed stopped=01"));
});
Check("设备拒绝和失败终态不升级为停止确认", () => {
    var rejection = new DebugReplyTracker(15, new("stop", 7)).Accept("error 15 bench stop code=profile");
    Require(rejection?.Result == "rejected" && !rejection.Accepted);
    var failure = new DebugReplyTracker(16, new("disable", 7, true));
    failure.Accept("ok 16 bench disable accepted=1");
    Require(failure.Accept("done 16 bench disable result=failed enabled=00")?.Result == "failed");
});
Check("诊断查询只能读取固定配置和CAN统计", () => {
    Require(TextProtocol.BuildQuery(20, ReadQuery.Config, 7) == "20 show config 7\n");
    Require(TextProtocol.BuildQuery(21, ReadQuery.Can, 7) == "21 show diag can\n");
    Require(TextProtocol.Match("ok 21 show diag can rx=10 tx=20", 21, ReadQuery.Can, 7)?["rx"] == "10");
    Reject(() => TextProtocol.Match("ok 20 show config joint=1 verified=00", 20, ReadQuery.Config, 7));
});
Check("虚拟传输只发送一次且等待匹配终态", () => {
    var sent = new List<string>();
    var input = new Queue<char>("ok 21 bench stop accepted=1\ndone 21 bench stop result=completed stopped=40\n");
    var receipt = DebugTransaction.Execute(21, new("stop", 7), sent.Add, () => input.Dequeue(), _ => {}, CancellationToken.None);
    Require(sent.SequenceEqual(new[] { "21 bench stop 7\n" }) && receipt.Result == "completed");
});
Check("接收后超时保留未知，不重试、不追加失能", () => {
    var sent = new List<string>();
    var input = new Queue<char>("ok 22 bench stop accepted=1\n");
    var receipt = DebugTransaction.Execute(22, new("stop", 7), sent.Add, () => input.Count > 0 ? input.Dequeue() : throw new TimeoutException(), _ => {}, CancellationToken.None, 15);
    Require(sent.Count == 1 && receipt.Accepted && receipt.Result == "unknown" && !receipt.FeedbackConfirmed);
});
Check("取消和通信中断不重发，断开时不推定已失能", () => {
    var sent = new List<string>();
    using var canceled = new CancellationTokenSource(); canceled.Cancel();
    var unsent = DebugTransaction.Execute(23, new("disable", 7, true), sent.Add, () => throw new IOException(), _ => {}, canceled.Token);
    Require(sent.Count == 0 && unsent.Result == "unknown");
    var failed = DebugTransaction.Execute(24, new("disable", 7, true), sent.Add, () => throw new IOException("lost"), _ => {}, CancellationToken.None);
    Require(sent.Count == 1 && failed.Result == "unknown" && !failed.FeedbackConfirmed);
});
Check("网关未连接时不积压设备动作", () => {
    using var gateway = new SerialGateway();
    try { _ = gateway.DebugAsync(new("stop", 7)); } catch (InvalidOperationException) { return; }
    throw new Exception("disconnected operation was queued");
});
Check("USB设备消失导致Close异常仍完成会话释放", () => {
    var finalized = false;
    var closeError = SerialSessionCleanup.Finish(() => throw new IOException("device disappeared"), () => finalized = true);
    Require(finalized && closeError == "device disappeared");
});
RemoteControlTests.Run(Check);
Check("维护失能仅允许ID7且必须承托，仍只构造一次失能命令", () => {
    Require(DebugProtocol.Build(31,new("torque-disable",7,true))=="31 bench disable 7\n");
    Reject(()=>DebugProtocol.Build(31,new("torque-disable",6,true)));
    Reject(()=>DebugProtocol.Build(31,new("torque-disable",7,false)));
});
Check("排队维护失能被STOP代际取消后不写串口，新点击才可再次准备", () => {
    var gate=new DebugPreparationGate();var sent=new List<string>();var generation=gate.Generation;
    gate.Cancel();
    var canceled=DebugTransaction.Execute(32,new("torque-disable",7,true),command=>gate.Write(generation,sent.Add,command),()=>-1,_=>{},CancellationToken.None);
    Require(sent.Count==0&&canceled.Result=="unknown");
    var bytes=new Queue<int>("ok 33 bench disable accepted=1\ndone 33 bench disable result=completed enabled=00\n".Select(c=>(int)c));
    var fresh=gate.Generation;
    var complete=DebugTransaction.Execute(33,new("torque-disable",7,true),command=>gate.Write(fresh,sent.Add,command),()=>bytes.Dequeue(),_=>{},CancellationToken.None);
    Require(sent.SequenceEqual(new[]{"33 bench disable 7\n"})&&complete.Result=="completed"&&!complete.FeedbackConfirmed);
});
GatewayProtocolSessionTests.Run(Check);
TorquePreparationQueueTests.Run(Check);
foreach (var failure in failures) Console.Error.WriteLine("FAIL " + failure);
Console.WriteLine($"GATEWAY_TESTS passed={count - failures.Count} failed={failures.Count}");
return failures.Count == 0 ? 0 : 1;
