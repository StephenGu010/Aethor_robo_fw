/** @file 维护失能的发送前取消代际；不改变已发送命令，不自动重试。 */
namespace Studio.Gateway;
/** 同步取消与实际单次写入，避免STOP先入队后仍发送旧维护失能。 */
public sealed class DebugPreparationGate
{
    private readonly object sync=new();
    private long generation;
    /** 捕获一次显式维护请求的当前代际。 */
    public long Generation { get { lock(sync)return generation; } }
    /** STOP、独立失能、重连或断连均撤销旧维护请求。 */
    public void Cancel() { lock(sync)generation++; }
    /** 在同一临界区核验代际和写入；取消后不能发出尚未发送的失能。 */
    public void Write(long expected,Action<string> write,string command)
    {
        lock(sync){
            if(expected!=generation)throw new InvalidOperationException("维护失能已被后续停止或连接变化取消，未发送");
            write(command);
        }
    }
}
