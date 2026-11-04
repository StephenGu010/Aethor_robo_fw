/** @file 纯内存远程协议验收，不枚举或打开任何真实串口。 */
using Studio.Gateway;
/** 与现有无框架测试器集成。 */
public static class RemoteControlTests
{
    /** 验证白名单、旧固件、错配、重复和超时无重发。 */
    public static void Run(Action<string, Action> check)
    {
        check("可调转矩使用整数毫牛米，拒绝越界和其他操作混入", () => {
            var request=new RemoteControlRequest("torque_reset",Token:12,CooledConfirmed:true,PeakMilliNm:6240);
            Require(RemoteControlProtocol.Build(33,request)=="33 ui torque_reset token=12 cold=1 peak_mnm=6240\n");
            Require(RemoteControlProtocol.Build(34,request with{PeakMilliNm=3500}).EndsWith("peak_mnm=3500\n"));
            foreach(var invalid in new[]{request with{PeakMilliNm=3499},request with{PeakMilliNm=6241},
                request with{Token=0},request with{CooledConfirmed=false},request with{Operation="torque",CooledConfirmed=false},
                request with{Operation="pos",CooledConfirmed=false}})Reject(invalid);
        });
        check("转矩策略只读查询与显式冷却复位使用固定命令，不产生运动", () => {
            Require(RemoteControlProtocol.Build(31,new("torque"))=="31 ui torque\n");
            Reject(new("torque_reset",Token:12));
            var restore=new RemoteControlRequest("torque_reset",Token:12,CooledConfirmed:true);
            Require(RemoteControlProtocol.Build(32,restore)=="32 ui torque_reset token=12 cold=1\n");
            Reject(restore with{Token=0}); Reject(restore with{MotorId=6});
            var receipt=RemoteControlProtocol.Match("ok 31 ui torque schema=1 result=policy peak=5.46 window_ms=10000",31,new("torque"));
            Require(receipt?.Result=="policy"&&receipt.Fields["peak"]=="5.46");
            var writes=0;
            var timeout=RemoteControlTransaction.Execute(31,new("torque"),_=>writes++,()=>-1,_=>{},CancellationToken.None,timeoutMs:1);
            Require(writes==1&&timeout.Result is "unknown" or "unsupported");
        });
        check("被动坐标身份无损保留，未知和非法身份不继承", () => {
            var bootProperty=typeof(MotorSnapshot).GetProperty("CoordinateBoot");
            var referenceProperty=typeof(MotorSnapshot).GetProperty("CoordinateReference");
            Require(bootProperty!=null&&referenceProperty!=null);
            var fields=new Dictionary<string,string>{{"joint","7"},{"state","absent"},{"coord_boot","4294967295"},{"coord_ref","16777217"}};
            var motor=TextProtocol.ParseMotor(7,fields,"");
            Require((uint?)bootProperty!.GetValue(motor)==uint.MaxValue&&(uint?)referenceProperty!.GetValue(motor)==16777217);
            fields["coord_ref"]="1.5";fields.Remove("coord_boot");motor=TextProtocol.ParseMotor(7,fields,"");
            Require(bootProperty.GetValue(motor)==null&&referenceProperty!.GetValue(motor)==null);
        });
        check("回零使用绝对目标和无损uint32身份，固定一秒且保持八字段", () => {
            var home=new RemoteControlRequest("home",Delta:0,Kp:uint.MaxValue,Kd:4294967294,Torque:0,Duration:1000,Token:7);
            var command=RemoteControlProtocol.Build(6,home);
            Require(command.Contains("op=home delta=0 ")&&command.Contains("kp=4294967295 kd=4294967294 torque=0 duration=1000"));
            Require(command.Count(c=>c=='=')==8&&command.Length<=160);
            Require(RemoteControlProtocol.Build(7,home with{Delta=-2.1}).Contains("delta=-2.1"));
            foreach(var invalid in new[]{home with{Kp=0},home with{Kd=0},home with{Kp=1.5},home with{Kd=4294967296},home with{Torque=1},home with{Duration=2000},home with{Delta=double.NaN}})Reject(invalid);
        });
        check("模式切换只接受两个固定操作，预检复用固定白名单", () => {
            foreach(var operation in new[]{"mode_mit","mode_pos","preflight"})
                Require(RemoteControlProtocol.Build(8,new(operation,Token:9)).Contains("op="+operation+" "));
            Reject(new("mode_3",Token:9)); Reject(new("mode",Token:9));
        });
        check("POS带载独立操作拒绝MIT增益、零位移和非整秒保持", () => {
            var valid = new RemoteControlRequest("pos_load", Delta:0.01, Kp:0, Kd:0, Token:7);
            Require(RemoteControlProtocol.Build(6, valid).Contains("op=pos_load"));
            Reject(valid with { Kp=120 }); Reject(valid with { Kd=.3 });
            Reject(valid with { Torque=1 }); Reject(valid with { Delta=0 });
            Reject(valid with { Duration=1500 });
        });
        check("远程请求固定ID7和单位，拒绝NaN和任意操作", () => {
            Reject(new("pos", Token:1, MotorId:1)); Reject(new("pos", Token:1, Coordinate:"flange-deg"));
            Reject(new("pos", Token:1, Kp:double.NaN)); Reject(new("enable", Token:1)); Reject(new("pos"));
            var command=RemoteControlProtocol.Build(5,new("pos",Token:7));
            Require(command.StartsWith("5 ui submit token=7 op=pos ") && command.Length<=160 && command.Count(c=>c=='=')==8);
        });
        check("远程回执必须对应路径和原事务", () => {
            Require(RemoteControlProtocol.Match("ok 5 ui caps schema=1 token=3",6,new("caps"))==null);
            Require(RemoteControlProtocol.Match("ok 5 ui result schema=1 id=2",5,new("caps"))==null);
            try { RemoteControlProtocol.Match("ok 5 ui result schema=1 id=4",5,new("result",TransactionId:3)); }
            catch(FormatException){return;} throw new Exception("accepted wrong original id");
        });
        check("旧固件拒绝与超时不重复发送", () => {
            var bytes=new Queue<char>("error 9 ui caps code=unknown_command\n");var writes=0;
            var old=RemoteControlTransaction.Execute(9,new("caps"),_=>writes++,()=>bytes.Count>0?bytes.Dequeue():throw new TimeoutException(),_=>{},CancellationToken.None);
            Require(old.Result=="rejected"&&writes==1);
            var timeout=RemoteControlTransaction.Execute(10,new("pos",Token:3),_=>writes++,()=>throw new TimeoutException(),_=>{},CancellationToken.None,timeoutMs:1);
            Require(timeout.Result=="unknown"&&writes==2);
        });
        check("断连网关拒绝远程动作且不打开串口", () => {
            using var gateway=new SerialGateway();
            try { _=gateway.RemoteAsync(new("pos",Token:2)); }catch(InvalidOperationException){return;}
            throw new Exception("disconnected request accepted");
        });
        check("连续串口字节不会饿死优先STOP", () => {
            var bytes=new Queue<char>("ok 9 ui caps schema=1 token=2\n");var stops=0;var reads=0;
            var receipt=RemoteControlTransaction.Execute(9,new("caps"),_=>{},()=>{reads++;Require(stops>=reads);return bytes.Dequeue();},_=>{},CancellationToken.None,()=>stops++);
            Require(receipt.Result=="capability"&&stops==reads);
        });
    }
    /** 统一断言失败出口。 */
    private static void Require(bool value){if(!value)throw new Exception("remote assertion failed");}
    /** 输入拒绝不得形成可发送命令。 */
    private static void Reject(RemoteControlRequest request){try{RemoteControlProtocol.Build(1,request);}catch(ArgumentException){return;}throw new Exception("unsafe request accepted");}
}
