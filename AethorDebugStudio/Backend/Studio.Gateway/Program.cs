/** @file 本机HTTP网关；默认不连接硬件，显式开放固定停止/失能和查询诊断。 */
using Studio.Gateway;
using Microsoft.Extensions.FileProviders;
using System.IO.Ports;
var builder = WebApplication.CreateBuilder(args);
builder.WebHost.UseUrls("http://127.0.0.1:5179");
builder.Services.ConfigureHttpJsonOptions(options => options.SerializerOptions.MaxDepth = 16);
builder.Services.ConfigureHttpJsonOptions(options => options.SerializerOptions.UnmappedMemberHandling = System.Text.Json.Serialization.JsonUnmappedMemberHandling.Disallow);
builder.Services.AddSingleton<SerialGateway>();
var app = builder.Build();
app.Use(async (context, next) => {
    context.Response.Headers["X-Content-Type-Options"] = "nosniff";
    context.Response.Headers["Referrer-Policy"] = "no-referrer";
    context.Response.Headers["Content-Security-Policy"] = "default-src 'self'; script-src 'self'; style-src 'self' 'unsafe-inline'; img-src 'self' data:; connect-src 'self'; object-src 'none'; frame-ancestors 'none'; base-uri 'none'";
    if (context.Request.Host.Host is not ("127.0.0.1" or "localhost")) { context.Response.StatusCode = 403; return; }
    if (context.Request.Path.StartsWithSegments("/api")) {
        context.Response.Headers.CacheControl = "no-store";
        if (context.Request.Method != "GET") {
            var origin = context.Request.Headers.Origin.ToString();
            var allowedOrigin = origin is "" or "http://127.0.0.1:5179" or "http://localhost:5179" or "http://127.0.0.1:5178" or "http://localhost:5178";
            if (!allowedOrigin || context.Request.Headers["X-Aethor-Client"] != "studio-v1") {
                context.Response.StatusCode = 403; await context.Response.WriteAsJsonAsync(new { error = "请求来源被拒绝" }); return;
            }
        }
    }
    await next();
});
app.MapGet("/api/health", () => new { service = "aethor-debug-gateway", version = "0.2.0", readOnly = true, debugAvailable = true, debugCommands = new[] { "diagnostics", "stop", "stop-all", "disable" }, hardwareTested = false, processId = Environment.ProcessId, integrityLevel = GatewayExecutionContext.IntegrityLevel });
app.MapGet("/api/ports", () => SerialPort.GetPortNames().OrderBy(value => value).ToArray());
app.MapGet("/api/state", (SerialGateway gateway) => gateway.Snapshot());
app.MapPost("/api/connect", async (HttpContext context, SerialGateway gateway) => {
    if (!context.Request.HasJsonContentType()) return Results.BadRequest(new { error = "连接请求必须为JSON" });
    if (context.Request.ContentLength is null or > 1024) return Results.BadRequest(new { error = "连接请求大小无效" });
    ConnectRequest? request;
    try { request = await context.Request.ReadFromJsonAsync<ConnectRequest>(); } catch (System.Text.Json.JsonException) { return Results.BadRequest(new { error = "连接请求格式错误" }); }
    if (request?.Port == null || request.Port.Length > 16) return Results.BadRequest(new { error = "请选择COM端口" });
    try { await gateway.ConnectAsync(request.Port); return Results.Ok(gateway.Snapshot()); }
    catch (UnauthorizedAccessException) { return Results.BadRequest(new { error = GatewayExecutionContext.AccessDeniedMessage(request.Port, GatewayExecutionContext.IntegrityLevel), code = "PORT_ACCESS_DENIED", integrityLevel = GatewayExecutionContext.IntegrityLevel }); }
    catch (Exception error) when (error is ArgumentException or InvalidOperationException or IOException or UnauthorizedAccessException) { return Results.BadRequest(new { error = error.Message }); }
});
app.MapPost("/api/disconnect", async (SerialGateway gateway) => { await gateway.DisconnectAsync(); return Results.Ok(gateway.Snapshot()); });
app.MapPost("/api/debug", async (HttpContext context, SerialGateway gateway) => {
    if (!context.Request.HasJsonContentType() || context.Request.ContentLength is null or > 1024)
        return Results.BadRequest(new { error = "调试请求必须为不超过1024字节的JSON" });
    try {
        var request = await context.Request.ReadFromJsonAsync<DebugRequest>();
        if (request == null) return Results.BadRequest(new { error = "缺少调试请求" });
        return Results.Ok(await gateway.DebugAsync(request));
    } catch (System.Text.Json.JsonException) { return Results.BadRequest(new { error = "调试请求格式错误或含未开放字段" }); }
    catch (ArgumentException exception) { return Results.BadRequest(new { error = exception.Message }); }
    catch (InvalidOperationException exception) { return Results.Conflict(new { error = exception.Message }); }
});
// Existing loopback-only listener and Host/Origin/X-Aethor-Client middleware also protect this route.
// Reject non-loopback peers explicitly so an environment URL override cannot expose remote control.
app.MapGet("/api/remote", (SerialGateway gateway) => Results.Json(gateway.RemoteReceipt()));
app.MapPost("/api/remote", async (HttpContext context, SerialGateway gateway) => {
    if (context.Connection.RemoteIpAddress == null || !System.Net.IPAddress.IsLoopback(context.Connection.RemoteIpAddress) ||
        context.Request.Headers["X-Aethor-Client"] != "studio-v1")
        return Results.Json(new { error = "远程调试仅允许本机Studio调用" }, statusCode: 403);
    if (!context.Request.HasJsonContentType() || context.Request.ContentLength is null or > 1024)
        return Results.BadRequest(new { error = "远程请求必须为不超过1024字节的JSON" });
    try {
        var request = await context.Request.ReadFromJsonAsync<RemoteControlRequest>();
        if (request == null) return Results.BadRequest(new { error = "缺少请求" });
        return Results.Ok(await gateway.RemoteAsync(request));
    } catch (System.Text.Json.JsonException) { return Results.BadRequest(new { error = "请求含未开放字段或格式错误" }); }
    catch (ArgumentException error) { return Results.BadRequest(new { error = error.Message }); }
    catch (InvalidOperationException error) { return Results.Conflict(new { error = error.Message }); }
});
app.Map("/api/{**path}", () => Results.Json(new { error = "接口未开放；请使用固定查询与停止/失能入口" }, statusCode: 404));
var frontend = Path.GetFullPath(Path.Combine(app.Environment.ContentRootPath, "../../Frontend/dist"));
if (Directory.Exists(frontend)) {
    var files = new PhysicalFileProvider(frontend);
    app.UseDefaultFiles(new DefaultFilesOptions { FileProvider = files });
    app.UseStaticFiles(new StaticFileOptions { FileProvider = files });
    app.MapFallbackToFile("index.html", new StaticFileOptions { FileProvider = files });
} else app.MapGet("/", () => Results.Text("请先在Frontend执行npm run build。", "text/plain; charset=utf-8"));
app.Run();
