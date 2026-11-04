/** @file 为每次串口连接建立挑战绑定的UI身份；单次握手，不发送电机命令或激活旧USB看门狗。 */
using System.Diagnostics;
using System.Globalization;
using System.Text.RegularExpressions;
namespace Studio.Gateway;

/** 隔离旧连接ACK与协议缓存；调用失败时由串口所有者关闭连接，禁止重试。 */
public static class GatewayProtocolSession
{
    /** 严格核对请求号、连接挑战与板端身份；旧ACK忽略，匹配的错误立即终止。 */
    public static Dictionary<string,string>? Match(string line, uint id, string challenge)
    {
        var words=line.Split(' ',StringSplitOptions.RemoveEmptyEntries);
        if(words.Length<3 || !uint.TryParse(words[1],NumberStyles.None,CultureInfo.InvariantCulture,out var responseId) || responseId!=id || words[2]!="hello")return null;
        if(words[0]=="error")throw new FormatException("设备拒绝UI连接握手："+line);
        if(words[0]!="ok")return null;
        var fields=new Dictionary<string,string>(StringComparer.Ordinal);
        foreach(var word in words.Skip(3)) {
            var separator=word.IndexOf('=');
            if(separator<=0 || !fields.TryAdd(word[..separator],word[(separator+1)..]))throw new FormatException("UI握手字段缺失或重复");
        }
        if(fields.GetValueOrDefault("ui")!=challenge)return null;
        if(fields.GetValueOrDefault("protocol")!="aethor-text-v1" || fields.GetValueOrDefault("profile")!="bench" || fields.GetValueOrDefault("dof")!="7" ||
            new[]{"boot","session","nonce"}.Any(key=>!uint.TryParse(fields.GetValueOrDefault(key),NumberStyles.None,CultureInfo.InvariantCulture,out var value)||value==0))
            throw new FormatException("UI握手协议或设备身份无效，远程操作保持关闭");
        return fields;
    }

    /** 只发送一次身份握手；超时、取消、旧响应和通信失败均不能打开远程控制。 */
    public static Dictionary<string,string> Execute(uint id, string challenge, Action<string> write, Func<int> read,
        Action<string> log, CancellationToken cancellation, int timeoutMs=1500)
    {
        if(id==0 || !Regex.IsMatch(challenge,"\\A[0-9a-f]{32}\\z",RegexOptions.CultureInvariant) || timeoutMs is <1 or >1500)
            throw new ArgumentException("UI握手身份或超时无效");
        cancellation.ThrowIfCancellationRequested();
        write(id.ToString(CultureInfo.InvariantCulture)+" hello ui="+challenge+"\n");
        var clock=Stopwatch.StartNew();var framer=new LineFramer();var count=0;
        while(clock.ElapsedMilliseconds<timeoutMs && count<128) {
            cancellation.ThrowIfCancellationRequested();
            int value;
            try{value=read();}catch(TimeoutException){continue;}
            if(value<0)throw new IOException("UI握手期间串口已关闭");
            var line=framer.Push((char)value);if(line==null)continue;
            count++;log(line);
            var result=Match(line,id,challenge);
            if(result!=null)return result;
        }
        throw new TimeoutException("UI连接握手未确认，连接关闭；未重试或发送电机命令");
    }
}
