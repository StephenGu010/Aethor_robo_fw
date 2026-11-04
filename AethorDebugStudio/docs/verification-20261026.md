# 调试工作台软件验收记录 · 2026-10-26

## 交付范围

独立项目D:/download/TCG/AethorDebugStudio。已交付九页界面、离线参数审核与模拟运行、即时停止演示、记录导入/回放/导出、能力登记，以及.NET 8只读串口网关实现。原固件工作树和旧版Studio未修改。

这是“界面＋离线回放＋只读网关”首版。真实远程动作、驱动参数写入/持久化、硬件STOP和ADRC闭环均未在本版开放，不称为完整实机控制交付。

## 本轮验证

最终完整命令：./scripts/verify.ps1。完整日志artifacts/full-verification.log；成功标记STUDIO_SOFTWARE_VERIFICATION_PASS hardware_connected=false motion_sent=false。

| 验证 | 当前结果 |
|---|---|
| TypeScript类型检查及Vite生产构建 | 通过；两条lucide-react客户端指令提示保留 |
| 前端领域测试 | 11通过、0失败 |
| .NET协议/边界测试 | 11通过、0失败 |
| Playwright真实Chromium流程 | 9通过、0跳过、0非预期、0不稳定 |
| HTTP同源、Host、JSON类型和未开放控制接口 | 10项通过，在生产页面浏览器用例中执行 |
| npm依赖审计 | 0漏洞 |
| 总览/参数页axe规则 | 本次执行规则无违规，不宣称完整无障碍认证 |
| 320/768/1440视口 | 无整页横向溢出，截图已保存 |
| 一键启动 | 从无服务到健康检查成功；默认未连接 |
| 真实硬件 | 未连接COM、未烧录、未使能、未运动 |

HTTP用例仅提交固定INVALID端口，服务端在打开串口前拒绝。浏览器里的COM_TEST与COM99是路由拦截的合成测试数据，不是真实连接。

## 已核对的行为

- 非有限数、超范围和未知映射拒绝；POS的ID7合成记录使用法兰坐标，静载使用输出轴坐标。
- 原始RX不充当采纳反馈，永久首故障槽被保留；显式序号缺口和时间缺口让曲线断开。
- 有效反馈中的0被保留；absent/unknown响应附带的默认0转换为null。
- 真实与回放模式不能启动演示；已连接只读会话不能静默切换来源或导入历史记录。
- 已导入记录的轴编号同步到选中轴。输入在审核后冻结；模拟完成与停止有独立结果。
- 真实采样按轴有界缓存，未知/过期字段不绘制，主机轮询数据不冒充连续CAN控制轨迹。
- 网关查询白名单、请求号/轴关联、分片、粘包、超长行隔离与错误响应均覆盖。
- 实际模式、控制权、标定、掉电保存及硬件失能无新证据时标未知/未完成。

## 运行与环境

本轮Node 22.23.2，.NET SDK 8.0.426；前端及System.IO.Ports版本见锁文件和项目文件。

NuGet CLI索引访问失败后，使用官方HTTPS固定包和项目本地源恢复。曾遇到用户级缓存写入受限，已改为项目内缓存。没有关闭证书校验或修改全局代理配置。离线恢复未进行NuGet在线漏洞审计，不将npm审计结果扩展到.NET。

apphost首次启动发现继承的DOTNET_ROOT指向另一套.NET 10，缺少.NET 8运行时。启动脚本现仅对新子进程设置与当前dotnet.exe一致的根目录，并恢复父进程环境，随后启动与健康检查通过。

Playwright浏览器缓存固定在项目.playwright，避免沙箱进程LocalAppData路径变化。通过保持启动会话存活验证本轮预览服务；普通用户双击启动脚本的生命周期仍由Windows会话管理。停止脚本只允许匹配本项目可执行文件的进程，不提供硬件停止能力。

## 工件

- artifacts/overview-desktop.png：总览完整页面。
- artifacts/production-overview.png：通过.NET网关加载的生产页面。
- artifacts/static-workflow.png：单轴演示结果。
- artifacts/layout-320.png、layout-768.png、layout-1440.png：响应式布局。
- artifacts/browser-results.json：结构化浏览器结果。
- .packages-feed/manifest.json：官方下载地址与本地包SHA256。
- artifacts/source-manifest.json：交付源码与提交标识（归档时生成）。

记录和截图不增加硬件资格，当前网关状态核查connected=false。

本地新Git仓库尚未配置作者身份，提交尝试被Git拒绝；未编造作者或修改全局身份。交付文件与暂存内容保留，源码清单的commit为null。
