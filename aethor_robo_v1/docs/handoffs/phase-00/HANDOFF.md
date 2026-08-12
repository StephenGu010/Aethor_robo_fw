# PRD Phase 0 固件交接

## 交付状态

- 分支：`refactor/prd-phase-00`
- 需求基线：`E:\Desktop_E\TCG\固件开发prd` 0.1.0-draft
- 实现范围：仅 Phase 0 安全框架，不驱动电机
- 当前状态机：`BOOT → SELF_TEST → FAULT(CONFIG_INCOMPLETE)`
- Keil 构建：`0 Error(s), 0 Warning(s)`
- 硬件验收：未执行

## 提交序列

| 提交 | 内容 |
|---|---|
| `1922bbd` | 定义 PRD Phase 0 固件架构 |
| `b6b7ab7` | 编写可执行实施计划 |
| `49609c2` | 固化工程、工具链、接线和内存基线 |
| `6e54663` | 增加七轴配置与验证位模型 |
| `48368ce` | 增加诊断环和安全启动自检 |
| `312d355` | 增加 App 分层契约与统一入口 |
| `995174a` | 同步 CubeMX USB 依赖 |
| `3361965` | 切换静态默认任务和 4 ms 服务入口 |
| `e614a25` | 同步 Keil 分组并生成 Phase 0 镜像 |
| `0281cac` | 增加架构守卫 |
| `8b5636d` | 记录自动化测试、构建和内存证据 |

## Phase 0 验收项

| 验收项 | 结果 | 证据 |
|---|---|---|
| App 七层边界存在 | 通过 | `App/`、架构守卫 |
| 固定七轴配置与 ID 唯一性校验 | 通过 | `arm_config.*`、Phase 0 主机测试 |
| 未验证物理参数阻止使能 | 通过 | `verified_fields=0`、`CONFIG_INCOMPLETE` 测试 |
| 启动不调用电机发送接口 | 通过 | 状态机测试、源码守卫、Keil 源列表 |
| App 不使用动态内存 | 通过 | 源码守卫、固定对象和 map |
| 默认 RTOS 任务静态创建 | 通过 | `.ioc`、`osThreadStaticDef`、map |
| CubeMX 与 Keil 同步 | 通过 | CubeMX 6.11.1 生成差异、Keil XML 与构建 |
| 新旧主机测试 | 通过 | `test-results.txt` |
| ARMCC 5 构建 | 通过 | `build.txt` |
| USB/CAN/电机实机验收 | 未执行 | 明确留给后续台架阶段 |

## 配置与安全边界

生产配置固定包含 J1–J7：ESC ID 为 1–7，Master ID 为 11–17。这些是当前唯一固化的关节映射。以下每一项都必须取得可追溯的机械/电气验证后，才能设置相应有效位：

- 方向；
- 机械零位和软限位；
- 最大关节速度和加速度；
- MIT Kp/Kd；
- 电机 PMAX/VMAX/TMAX；
- 外部减速比。

任何一项缺失都会使 `arm_config_is_enable_ready()` 返回 false。当前应用没有 enable、position、velocity 或 CAN transmit API，因此按键、USB 输入和 CAN 反馈均不能触发运动。

## 内存与并发策略

- 默认任务使用 512 words 静态栈和静态 TCB，周期为 4 ms。
- 诊断使用固定 64 项环形缓冲；满时覆盖最旧事件并累计丢弃计数。
- App 对象均为静态生命周期，不使用 malloc/free 或 FreeRTOS heap API。
- FreeRTOS 15,360 字节 heap_4 仍为中间件兼容保留，但 Phase 0 map 显示其容量没有增长。
- Arm 层是状态唯一所有者；ISR、Protocol 和 Platform 不能直接写控制状态。

## 工具链与重新生成

- STM32CubeMX：工程元数据 6.11.1
- STM32CubeH7：1.11.2
- Keil uVision：5.43.1.0
- ARMCC：5.05.0.106

CubeMX 重新生成时必须保留并复核：

1. `FREERTOS.Tasks01` 为 Static，缓冲区为 `defaultTaskBuffer/defaultTaskControlBlock`；
2. PA15 标签为 `USER_KEY`；
3. FDCAN1、USB HS Device CDC、TIM23 时基没有被意外改变；
4. `main.c` 和 `freertos.c` 用户代码块仍调用 `aethor_app_init/service`；
5. Keil 分组仍只包含一份 USB 中间件源，并继续排除旧控制器。

生成后先运行架构守卫，再运行主机测试和 Keil Rebuild。不要用 CubeMX 覆盖 `App/`。

## 验证命令与产物

```powershell
powershell.exe -ExecutionPolicy Bypass -File .\Tests\host\check_phase0_architecture.ps1
powershell.exe -ExecutionPolicy Bypass -File .\Tests\host\run_phase0_tests.ps1
powershell.exe -ExecutionPolicy Bypass -File .\Tests\host\run_tests.ps1
```

Keil 工程：`MDK-ARM\CtrBoard-H7_FDCAN.uvprojx`

固件：`MDK-ARM\CtrBoard-H7_FDCAN\CtrBoard-H7_FDCAN.hex`

Map：`MDK-ARM\CtrBoard-H7_FDCAN\CtrBoard-H7_FDCAN.map`

## Phase 1 首个任务

先建立“参数来源与确认机制”，不要直接实现运动：定义逐轴配置清单、测量方法、复核人、版本号和写入来源；为每个有效位建立可重复测试。随后才能迁移电机参数读取和反馈解码，并按一次一轴、物理失能优先的台架流程验收。七轴同时使能必须继续锁定，直到每轴方向、零位、限位、量程和减速比全部校准完成。

## 明确声明

本交付已经完成软件架构、主机测试、CubeMX/Keil 同步和固件构建；没有烧录并验证当前 Phase 0 镜像，也没有验证 USB 枚举、CAN 收发、电机反馈或任何机械运动。不得把 `0 Error(s)`、Host 测试通过或历史 PA15 示例结果表述为七轴硬件完成。
