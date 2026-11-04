/** @file 只读检测Windows进程完整性级别，解释串口拒绝访问；不改变权限或ACL。 */
using System.Runtime.InteropServices;
using System.Runtime.Versioning;
using System.Security.Principal;
using Microsoft.Win32.SafeHandles;
namespace Studio.Gateway;

/** 当前网关运行环境诊断；普通用户权限即可访问本项目USB CDC。 */
public static class GatewayExecutionContext
{
    public static string IntegrityLevel { get; } = OperatingSystem.IsWindows()
        ? ClassifyIntegrity(ReadWindowsIntegrityRid()) : "unknown";

    /** 将Windows强制完整性RID映射为可公开诊断标签。 */
    public static string ClassifyIntegrity(int? rid) => rid switch {
        null => "unknown", < 0 => "unknown", < 8192 => "low",
        < 12288 => "medium", < 16384 => "high", _ => "system"
    };

    /** 区分已识别的低权限环境与普通会话中的占用/权限可能性。 */
    public static string AccessDeniedMessage(string portName, string integrity) =>
        integrity == "low"
        ? $"{portName}无法打开：当前网关运行在Windows低权限环境。请从桌面双击Start-Studio.cmd，以普通用户会话启动网关；无需管理员权限。"
        : $"{portName}访问被拒绝：端口可能被其他程序占用，或当前用户没有访问权限。请先释放该端口，再重新连接。";

    /** 只读取当前进程令牌的Mandatory Label，不查询其他用户或修改令牌。 */
    [SupportedOSPlatform("windows")]
    private static int? ReadWindowsIntegrityRid()
    {
        IntPtr buffer = IntPtr.Zero;
        try {
            using var identity = WindowsIdentity.GetCurrent(TokenAccessLevels.Query);
            NativeMethods.GetTokenInformation(identity.AccessToken, 25, IntPtr.Zero, 0, out var length);
            if (length is <= 0 or > 65536) return null;
            buffer = Marshal.AllocHGlobal(length);
            if (!NativeMethods.GetTokenInformation(identity.AccessToken, 25, buffer, length, out _)) return null;
            var sid = Marshal.ReadIntPtr(buffer);
            if (sid == IntPtr.Zero) return null;
            var countAddress = NativeMethods.GetSidSubAuthorityCount(sid);
            if (countAddress == IntPtr.Zero) return null;
            var count = Marshal.ReadByte(countAddress);
            if (count == 0) return null;
            var ridAddress = NativeMethods.GetSidSubAuthority(sid, (uint)(count - 1));
            return ridAddress == IntPtr.Zero ? null : Marshal.ReadInt32(ridAddress);
        } catch (Exception error) when (error is UnauthorizedAccessException or System.Security.SecurityException) {
            return null;
        } finally {
            if (buffer != IntPtr.Zero) Marshal.FreeHGlobal(buffer);
        }
    }

    /** 最小只读Win32接口声明。 */
    private static class NativeMethods
    {
        /** 读取指定令牌字段；此处仅使用TokenIntegrityLevel。 */
        [DllImport("advapi32.dll", SetLastError = true)]
        [return: MarshalAs(UnmanagedType.Bool)]
        internal static extern bool GetTokenInformation(SafeAccessTokenHandle token, int informationClass,
            IntPtr information, int informationLength, out int returnLength);
        /** 读取SID子权限数量。 */
        [DllImport("advapi32.dll")]
        internal static extern IntPtr GetSidSubAuthorityCount(IntPtr sid);
        /** 读取SID指定RID地址，不复制或发布整个用户SID。 */
        [DllImport("advapi32.dll")]
        internal static extern IntPtr GetSidSubAuthority(IntPtr sid, uint index);
    }
}
