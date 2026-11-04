/** @file 维护队列与完成发布时序测试；仅注入内存会话，不创建或打开串口。 */
using System.Collections;
using System.Reflection;
using Studio.Gateway;
/** 观察真实网关入队及回执发布，不用模拟实现替代取消逻辑。 */
public static class TorquePreparationQueueTests
{
    private const BindingFlags Private=BindingFlags.Instance|BindingFlags.NonPublic;
    /** 仅测试中注入未运行的内存worker，使真实入队分支可离线验证。 */
    private static void Set(SerialGateway gateway,string name,object value)=>typeof(SerialGateway).GetField(name,Private)!.SetValue(gateway,value);
    /** 读取实际队列与取消器以验证既有串口边界前的行为。 */
    private static T Get<T>(SerialGateway gateway,string name)=>(T)typeof(SerialGateway).GetField(name,Private)!.GetValue(gateway)!;
    /** 注册真实队列取消和回执可见性两项检查。 */
    public static void Run(Action<string,Action> check)
    {
        check("真实维护队列收到STOP后旧发送代际失效",()=>{
            using var gateway=new SerialGateway();var worker=new TaskCompletionSource();
            var cancellation=new CancellationTokenSource();
            Set(gateway,"connected",true);Set(gateway,"worker",worker.Task);Set(gateway,"cancellation",cancellation);Set(gateway,"uiSessionBoot",3U);
            try{
                _=gateway.DebugAsync(new("torque-disable",7,true));
                var gate=Get<DebugPreparationGate>(gateway,"preparationGate");var generation=gate.Generation;
                _=gateway.DebugAsync(new("stop-all",7));
                var pending=Get<IList>(gateway,"pendingDebug");
                var first=(DebugRequest)pending[0]!.GetType().GetProperty("Request")!.GetValue(pending[0])!;
                if(first.Operation!="stop-all"||pending.Count!=2||gate.Generation==generation)throw new Exception("STOP未抢占/取消准备");
                var written=false;
                try{gate.Write(generation,_=>written=true,"unsafe");throw new Exception("已取消维护仍可发送");}
                catch(InvalidOperationException){if(written)throw new Exception("取消后产生写入");}
            }finally{worker.SetResult();}
        });
        check("真实维护完成回执发布前解除自己的busy，后续策略查询不被拒",()=>{
            using var gateway=new SerialGateway();var worker=new TaskCompletionSource();
            var cancellation=new CancellationTokenSource();
            Set(gateway,"connected",true);Set(gateway,"worker",worker.Task);Set(gateway,"cancellation",cancellation);Set(gateway,"uiSessionBoot",3U);
            try{
                var completion=gateway.DebugAsync(new("torque-disable",7,true));
                var pending=Get<IList>(gateway,"pendingDebug");var entry=pending[0]!;pending.RemoveAt(0);
                typeof(SerialGateway).GetMethod("CompleteDebug",Private)!.Invoke(gateway,new object[]{entry,new DebugReceipt(9,"torque-disable",7,true,"completed","synthetic",Array.Empty<string>())});
                if(!completion.IsCompletedSuccessfully||Get<bool>(gateway,"preparationPending"))throw new Exception("回执先于busy解除");
                _=gateway.RemoteAsync(new("torque"));
            }finally{worker.SetResult();}
        });
    }
}
