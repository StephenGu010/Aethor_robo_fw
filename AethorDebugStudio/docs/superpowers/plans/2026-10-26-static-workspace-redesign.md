# 按F最新进展重设计调试工作台

**Goal:** 重做单轴静载工作区，区分发布基线、离线草稿、精度指标与保护，同时更新总览、维护和能力清单。
**Architecture:** 保留九页导航与只读网关；新增纯策略模型和静载专用页面。历史发布证据独立呈现，不冒充当前设备识别。
**Tech Stack:** 现有React/TypeScript/SVG/Playwright，不增加依赖。

依据：F聊天最新回复及static_window_release_20261026T073102Z/deployment_status.md、mit_parameters.md。已部署3281d1a8：ID7保持1°输出轴保护，精度0.5°独立观察；启动接管0.5°，等效Kp80/Kd0.2，转矩3.5N·m。高增益未部署，实际空间和动态资格未知。此更新是用户授权的现有页面重设计，不开放远程控制或修改固件。

- [x] 新增策略纯逻辑负例，验证0.5/1°边界、启动与保持区分、未知坐标拒评、预算不是硬限。
- [x] staticPolicy.ts：发布基线、候选分离、预算计算和绑定记录观察；types/domain/demo记录可选策略上下文。
- [x] StaticWorkspace.tsx：参数草稿、发布基线、三窗口图、执行审核、精度/保护分离结果；StaticInsights.tsx提供误差图与独立预算。
- [x] ReleaseBaseline.tsx：总览与维护页共享历史发布卡；更新目录能力描述和参数默认语义。
- [x] npm test、build、真实浏览器流程及320/768/1440/1920截图与axe；网关原测试回归，保持只读。
- [x] 说明验证结果和未开放能力，更新README；不重试Git身份设置，不连接硬件，不删除文件。
