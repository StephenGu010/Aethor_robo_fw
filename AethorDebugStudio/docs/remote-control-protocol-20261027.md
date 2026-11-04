# ID7远程LCD适配（离线候选）

页面参数使用电机输出轴角度 °、速度 °/s；提交时明确换算为rad、rad/s。Kp范围80–200 N·m/rad、Kd范围0.20–0.40 N·m·s/rad，默认120/0.30。模式、轴、边界与标准反馈资格最终由原LCD准入和执行保护判定。0x50寄存器位置及其差分速度不授予运动资格。

协议保持原有160字节、8字段限制。以下是协议说明，不能把示例当作实机执行指令。

| 固定路径 | 作用 |
| --- | --- |
| `ui caps` | 只读能力，返回schema、boot、epoch、fresh、mode、owner、pos/static能力、POS范围和单调动作ID下界last；生成30秒审核token |
| `ui submit` | 固定8字段token/op/delta/speed/kp/kd/torque/duration，提交一次不可修改的请求；不重发 |
| `ui result` | id为原提交编号；只读获取pending或保留终态，可重复查询最近终态 |
| `ui heartbeat` | token为申请时确认的远程lease；仅维持独立远程健康，不刷新LCD健康或电机反馈 |

操作限acquire、release、pos、configure、hold、lock，固定ID7。configure/lock复用既有单轴静载RAM安装/撤回；不会写驱动Flash，不自动失能、使能或切模。pos复用LCD相对运动；hold复用既有静载保持和保护。动作前必须显式申请远程控制权；LCD已有控制权时拒绝，远程持有时LCD非STOP拒绝。两种STOP入口仍可抢占。

代理在固件内部使用LOCAL_UI受保护执行路径，但专用高位ID域、独立最高ID、private remote_proxy标记和回执隔离。外部协议没有origin/proxy字段，LCD公开提交函数拒绝伪造标记。远程完成不使LCD低位ID失效。原执行器约60处保护判断不改变语义。远程持有ID7期间，任何识别到的STOP/失能意图（包括另一轴或随后被原解析器拒绝的参数）都先锁存ID7安全停止，再撤销远程租约；不会出现先关闭看门狗却未安排停止的空窗。

远程动作外部ID在板端单调增加；网关读取caps.last后同步计数，重连不能复用已接受动作ID。审核token一次消费；STOP、epoch变化、30秒超时、初始化都会使旧审核失效。终态至少保留最近一条。网关GET `/api/remote`只读保留最近回执，HTTP中断后可恢复原事务编号；真正重连或网关重启后，可显式读取caps.boot/active_id/terminal_id恢复板端原事务，再只读查询结果，不补发动作。

远程控制权有独立5秒lease，网关持有租约后每秒通过唯一串口线程发送只读heartbeat；LCD心跳不会代替它。失联超时在原控制任务置STOP intent并撤销权限，随后走现有fail-safe失能保护。它可能撤去保持力矩，负载必须有独立承托。软件生成0xFD帧不代表真实CAN发送或物理失能已确认。所有请求/等待均不重发动作，STOP在持续响应字节之间也优先服务。

HTTP仅绑定本机，继承Host/Origin及X-Aethor-Client来源限制；控制端点另检查远端IP必须loopback。页面不会自动连接COM，能力读取/审核/下发/结果查询均为显式操作。旧固件无法确认schema时保持关闭。

本次验证均为内存、合成反馈、主机固件编译及浏览器API拦截。未连接真实COM发送参数或动作，未烧录，未停止既有服务。实机仍需部署新固件及网关并具备新鲜标准反馈；当前仅寄存器位置可见不能放行。
