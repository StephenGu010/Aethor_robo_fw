/** @file USB热拔插时的会话清理；关闭失败不能保留已连接状态。 */
namespace Studio.Gateway;
/** 保证资源关闭异常后仍执行会话释放和排队请求拒绝。 */
public static class SerialSessionCleanup
{
    /** 捕获已知串口关闭错误；无论关闭是否成功都运行最终清理。 */
    public static string? Finish(Action close, Action finalize)
    {
        string? error = null;
        try { close(); }
        catch (Exception exception) when (exception is IOException or UnauthorizedAccessException or InvalidOperationException) { error = exception.Message; }
        finally { finalize(); }
        return error;
    }
}
