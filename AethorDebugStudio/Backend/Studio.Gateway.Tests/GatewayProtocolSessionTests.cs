/** @file 内存模拟串口握手回归；不枚举、打开或控制任何真实设备。 */
using Studio.Gateway;
/** 验证连接挑战、旧ACK隔离、身份严格性和失败不重试。 */
public static class GatewayProtocolSessionTests
{
    private const string Challenge="0123456789abcdef0123456789abcdef";
    private const string Ack="ok 9 hello protocol=aethor-text-v1 profile=bench dof=7 boot=1234 session=47 nonce=1 ui="+Challenge;
    /** 将行为检查交给现有无框架测试器。 */
    public static void Run(Action<string,Action> check)
    {
        check("连接UI握手单次发送，忽略旧挑战ACK，不发送控制命令",()=>{
            var sent=new List<string>();
            var bytes=new Queue<int>((Ack.Replace(Challenge,"fedcba9876543210fedcba9876543210")+"\n"+Ack+"\n").Select(c=>(int)c));
            var result=GatewayProtocolSession.Execute(9,Challenge,sent.Add,()=>bytes.Dequeue(),_=>{},CancellationToken.None);
            Require(sent.SequenceEqual(new[]{"9 hello ui="+Challenge+"\n"})&&result["session"]=="47");
            Require(GatewayProtocolSession.Match(Ack,8,Challenge)==null);
        });
        check("握手拒绝错协议错型号零身份和重复字段",()=>{
            foreach(var bad in new[]{Ack.Replace("bench","arm"),Ack.Replace("dof=7","dof=6"),Ack.Replace("aethor-text-v1","other"),
                Ack.Replace("session=47","session=0"),Ack.Replace("nonce=1","nonce=0"),Ack.Replace("boot=1234","boot=4294967296"),
                Ack.Replace("nonce=1","nonce=+1"),Ack+" nonce=2"})
                MustFail<FormatException>(()=>GatewayProtocolSession.Match(bad,9,Challenge));
            Require(GatewayProtocolSession.Match(Ack+" future=1",9,Challenge)!=null);
        });
        check("握手错误和通信失败均不重试，取消时不发送",()=>{
            var sent=new List<string>();
            var bytes=new Queue<int>("error 9 hello code=busy\n".Select(c=>(int)c));
            MustFail<FormatException>(()=>GatewayProtocolSession.Execute(9,Challenge,sent.Add,()=>bytes.Dequeue(),_=>{},CancellationToken.None));
            Require(sent.Count==1);
            sent.Clear();
            MustFail<IOException>(()=>GatewayProtocolSession.Execute(9,Challenge,sent.Add,()=>throw new IOException(),_=>{},CancellationToken.None));
            Require(sent.Count==1);
            sent.Clear();
            MustFail<TimeoutException>(()=>GatewayProtocolSession.Execute(9,Challenge,sent.Add,()=>throw new TimeoutException(),_=>{},CancellationToken.None,1));
            Require(sent.Count==1);
            sent.Clear();using var canceled=new CancellationTokenSource();canceled.Cancel();
            MustFail<OperationCanceledException>(()=>GatewayProtocolSession.Execute(9,Challenge,sent.Add,()=>-1,_=>{},canceled.Token));
            Require(sent.Count==0);
            MustFail<ArgumentException>(()=>GatewayProtocolSession.Execute(9,"bad\n",sent.Add,()=>-1,_=>{},CancellationToken.None));
            Require(sent.Count==0);
        });
    }
    /** 不满足断言时中断当前纯内存检查。 */
    private static void Require(bool condition){if(!condition)throw new Exception("UI handshake assertion failed");}
    /** 验证失败路径而不吞掉其他异常类别。 */
    private static void MustFail<T>(Action action) where T:Exception
    {
        try{action();}catch(T){return;}
        throw new Exception("Expected "+typeof(T).Name);
    }
}
