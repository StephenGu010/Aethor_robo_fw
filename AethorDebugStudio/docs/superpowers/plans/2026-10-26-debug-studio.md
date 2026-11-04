# Aethor调试工作台实施计划

> 执行方式：当前会话逐文件实施，每一阶段自动验证并报告；不启动子agent，不进行硬件操作。

**Goal:** 交付九页调试工作台、离线流程/记录回放和默认断开的只读Windows串口网关。
**Architecture:** 前端领域模型与界面分离，.NET网关独占串口，固件保护保持现状。
**Tech Stack:** React、TypeScript、Vite、SVG曲线、.NET 8、System.IO.Ports、Playwright。

- [x] 1. 建立独立Git仓库与配置，保存批准设计，不改变固件工作树。
- [x] 2. 先写领域负例：错误数字/单位/过期/导入超限/真实动作拒绝；运行确认失败。实现types.ts、domain.ts、catalog.ts与demo.ts；运行npm test。
- [x] 3. 编写只读协议测试，先验证缺失实现失败。实现Protocol.cs、SerialGateway.cs、Program.cs、Contracts.cs；运行dotnet run --project Backend/Studio.Gateway.Tests。
- [x] 4. 实现Shell、总览、参数流程、示波、维护、记录与实验页面；统一参数解释、状态标签、错误和空状态。
- [x] 5. 接入API和离线回放；真实模式只读，演示状态与真实快照分离；npm run build和网关build通过。
- [x] 6. 浏览器检查九页导航、参数异常、模拟完成/停止、导入导出与320/768/1440宽度；生成artifacts中的截图。
- [x] 7. API集成负例和依赖审计；说明已实现/未接入/未实机验证；提供一键启动脚本、操作说明与验证报告。

## 文件职责与执行命令
Frontend/src/domain.ts：纯数据校验、记录导入、坐标/反馈时效；Frontend/tests/domain.test.ts：node --import tsx --test tests/*.test.ts。
Frontend/src/demo.ts：只生成显式演示数据，不访问网络。Frontend/src/api.ts：同源固定API客户端。
Frontend/src/App.tsx：数据源、导航、会话生命周期。components/*与pages/*各自承担一种页面职责。
Backend/Studio.Gateway/Protocol.cs：查询白名单与有界逐行解码。SerialGateway.cs：单端口所有者和快照发布。Program.cs：本机API/同源保护/静态资源。
验证顺序：npm test → npm run build → dotnet run --project Backend/Studio.Gateway.Tests → dotnet build Backend/Studio.Gateway → npm run test:e2e → powershell -File scripts/verify.ps1。

## 开发约束
逐文件一次性提交完整编辑块；所有源码含文件注释，函数和类含说明。禁止批量删除。只读串口只做代码与替身测试，不打开真实COM。依赖与上游资料版本记录在README中。浏览器测试不配置或访问真实串口。
