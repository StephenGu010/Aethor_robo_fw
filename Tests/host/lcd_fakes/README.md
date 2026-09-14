# LCD/ADC 平台测试与集成记录

验证日期：2026-09-08。仅软件验证；没有连接主控、LCD 或电机，没有烧录。

## 可复现验证

在固件根目录执行：

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File Tests/host/run_lcd_transport_tests.ps1 -CompileArmcc
```

本轮结果：GCC 14.3 的 `-std=c99 -Wall -Wextra -Werror -pedantic` 主机测试通过；ARMCC 5.06 update 4 build 422 对 `lcd_transfer.c`、`lcd_st7789.c`、`lcd_key_adc.c` 在 UI=0/UI=1 下的 6 个目标文件均编译通过，警告按错误处理。目标编译使用父代理补齐的 HAL 1.11.3 与当前 H723 CMSIS，没有使用假 HAL。完整固件链接/map/Keil 工程检查由父代理负责。

输出仅写入本目录 `build/`，不修改执行策略、不执行 Git、不删除文件。

已实际观察的先失败后通过顺序：有效区域拒绝断言；平台初始化未实现断言；ADC 初始化未实现断言；ADC 校准卡住断言；EOT 后停止确认超时断言；旧 SPI 错误回调断言；调用者先取时间、ADC IRQ 后发布导致样本误失效断言。

覆盖范围：

- 包含式窗口、空指针、错误长度、最大 13440 字节、32 字节对齐、AXI 专区边界和 DTCM 拒绝。
- DMA TC 与 SPI EOT 分离；CS 和缓冲只在停止确认后释放；恰好一次完成。
- 50 ms 超时、时间回绕、重复完成、旧 token、旧完成/错误回调、连续事务。
- DMA 不能停止时保留缓冲、拒绝下一笔及重初始化；停止后清除已排队的旧 DMA/SPI 标志与 NVIC pending。
- TX cache clean 的地址/长度、X+20 窗口、MADCTL=0x70、COLMOD=0x55、SPI 分频和模式。
- 短命令失败、DMA TE、SPI UDR、恢复与累计错误计数。
- ADC 12 位/通道19、校准模式、5 ms 请求节流、IRQ raw/seq/ms 发布、重复回调、50 ms 失效、回绕、OVR 与启动失败。
- ADC 校准卡住在 10 ms 内失败；真实驱动另有 100000 次轮询上限，应对 HAL tick 失效。

假 HAL 是可控事件模型，不能验证总线电气时序、缓存硬件行为、真实 NVIC 调度或目标链接布局。`platform_test_main.c` 将合成 AXI 地址传给假 DMA，假 HAL 不解引用该地址。

## 独立审查修复与冻结记录（2026-09-09）

本轮仅修复独立复现的三项问题。2026-09-08 18:06:26（UTC+8）先将探针转入持久回归，在两个未修改的驱动上成功编译，再分别运行并观察以下行为失败；没有将编译错误当作失败证据。

| 持久回归参数 | 旧驱动首次失败 | 修复后行为与覆盖 |
| --- | --- | --- |
| `stop-order` | 测试第186行：DMA EN仍为1，SPI SPE已为0；退出码1 | abort、超时、DMA错误、SPI错误四种入口先请求停DMA并读回EN；EN未清时保留SPE、TXDMAEN、CS及缓冲，拒绝新flush和重初始化。跨三次60秒服务仍不强制归还，模拟总线访问完成后才停SPI、清请求与旧事件、恰好通知一次；恢复后下一flush成功。 |
| `dma-enables` | 测试第251行：FEIF存在、FEIE=0，TC却触发提前失败；退出码1 | 按CR中的TEIE/DMEIE和FCR中的FEIE筛选错误事件；禁用的FEIF继续TC→EOT，TC不归还缓冲。还覆盖TE/DME使能门禁、启用错误时失败、背光与累计统计、连续flush复用HAL状态。 |
| `cal-preempt` | 测试第288行：ADCAL读取忙后被抢占，校准已完成，初始化却失败；退出码1 | 到10ms deadline或100000次轮询边界时重新读取ADCAL，仅仍忙才失败；覆盖抢占、tick回绕和轮询边界。ADCAL真卡住与tick冻结仍有界失败，正常校准后能发布转换样本。 |

依据：RM0468 Rev3 §15.3.19（第631–632页）规定EN写0不立即生效，必须读回确认停止；§55.4.14（第2220页）规定关闭次序为DMA、SPI、SPI的DMA请求。当前HAL 1.11.3 `stm32h7xx_hal_dma.c` 第1225–1261、1311–1384行按中断源使能处理错误/TC；`stm32h7xx_hal_adc.c` 第3725–3737行使用超时后重读以避免抢占造成假超时。假HAL据此补充延迟EN确认、事件使能、DMA回调与忙/锁状态，不模拟完整芯片。

2026-09-09 09:12:52（UTC+8）恢复会话后，重新运行上方原有 `run_lcd_transport_tests.ps1 -CompileArmcc`，退出码0：纯C六组、平台测试及三个持久回归全部通过。ARMCC实际版本为 **5.06 update 4 build 422**；三个源文件各在UI=0/UI=1下重新生成共六个目标文件，使用真实H723/HAL头文件。GCC `-Werror`、ARMCC `--diag_error=warning` 均启用，实际编译诊断 **0 warning、0 error**。日志包含六个新目标的时间戳、大小与SHA256，并确认受测输入在运行期间未变。

证据保存在固件根目录的 `output/lcd-driver-review-20260908/`：

- [旧驱动三项失败原始日志](../../../output/lcd-driver-review-20260908/persistent-regressions-red.log)
- [最终主机与ARMCC完整日志](../../../output/lcd-driver-review-20260908/persistent-regressions-green.log)
- [最终受测输入SHA256](../../../output/lcd-driver-review-20260908/verified-input-hashes.json)
- [冻结时间与文件SHA256清单](../../../output/lcd-driver-review-20260908/frozen-files.json)

构建后可用 `Tests/host/lcd_fakes/build/lcd_platform_tests.exe stop-order` 单独运行第一项；其余参数为 `dma-enables`、`cal-preempt`。无参数默认执行全部回归。

本轮冻结的六个修改文件为 `App/Platform/lcd_st7789.c`、`App/Platform/lcd_key_adc.c`、本目录的 `lcd_fake_hal.h`、`lcd_fake_hal.c`、`platform_test_main.c`、`README.md`。`lcd_transfer.c/.h` 和原测试脚本保持原样；Ui未修改，驱动错误累计统计保持累计语义。没有增加全局关中断，没有调整ADC时钟/校准预算，没有Git写入、删除或烧录。父代理以冻结清单对应版本进行四目标统一重建；这里的ARMCC对象编译不代表完整固件链接或实机验收。本轮驱动工作至此结束。

## 父代理集成合同

公开接口以 `App/Platform/lcd_st7789.h`、`lcd_key_adc.h`、`lcd_transfer.h` 为准。所有普通入口只有 UiTask 一个调用者；完成 hook 可以在 IRQ8 或同步失败路径执行，不得重入驱动。

- 初始化：`lcd_st7789_init(hook, context)`，再 `lcd_st7789_start(HAL_GetTick())`；每 ≤5 ms 服务，等待 `LCD_ST7789_READY` 后才交给 LVGL 刷新。ADC 用 `lcd_key_adc_init/service/latest`。
- 完成 hook 签名：`void (*)(void *, uint32_t token, LcdTransferResult)`。可调用 `lv_disp_flush_ready()`，不得进行其他 LVGL 控件操作。
- `begin_flush` 返回 1 表示已接管缓冲，hook 恰好一次，可能在函数返回前发生；返回 0 表示未接管且不调用 hook，由 UI 完成自己的拒绝路径。
- `lcd_st7789_abort()` 返回 0 或状态中 `buffer_owned=1` 时，缓冲仍不可复用。继续调用 service，包含 LVGL `wait_cb`；不能只强行通知 flush-ready。故障状态需 UI 按动作清理规则显式恢复。
- Core IRQ 转发：`DMA1_Stream0_IRQHandler → lcd_st7789_dma_irq`；`SPI1_IRQHandler → lcd_st7789_spi_irq`；`ADC_IRQHandler → lcd_key_adc_irq`。
- 平台唯一持有 SPI1、DMA1_Stream0 和 ADC1 句柄并初始化 GPIO/RCC/DMA/NVIC；不生成另一份 hspi1/hadc1，不重复 MSP 或 MX_SPI1/ADC1 初始化。HAL 使用当前弱 MSP 默认函数；强完成回调由平台文件定义，集成时不要重复定义。
- 编译 `lcd_transfer.c`、`lcd_st7789.c`、`lcd_key_adc.c`；SPI/ADC 两个平台实现按 `AETHOR_DEBUG_UI_ENABLE` 整块门禁。HAL 启用 SPI、ADC、DMA，并链接匹配的 spi/spi_ex/adc/adc_ex/dma/dma_ex 及现有 RCC/GPIO/Cortex 模块。
- SPI1：PB3/PD7 AF5，CS PE15、DC PD10、RESET PB11、BL PB10。现有 PLL1Q 80 MHz，分频16为5 MHz；不改系统时钟树。PB3 不能同时用作 SWO。
- ADC1：PA5 INP19，独立单通道12位，单次中断；PLL2 M2/N16/P2/Q2/R2、异步分频16，加 H723 ADC12 固定除2后为3 MHz，采样810.5周期。当前 ADC2/PLL2 无其他使用者；后续增加使用者需重新协调 ADC12 reset 和时钟所有权。
- IRQ 优先级统一8；不使用全局关中断来保护 LCD/ADC，也不禁用全局 Cache。
- 两块 13440 字节像素缓冲由父代理分配在 `.lcd_dma` 的 `[0x24000000,0x24008000)`，32 字节对齐、整个末尾缓存行也须独占。平台只做 TX clean、不 invalidate。高字节先发 RGB565，LVGL v8 配 `LV_COLOR_16_SWAP=1`。
- LCD/ADC 时戳均用 HAL_GetTick 的32位毫秒；不要传入微秒或未换算 RTOS tick。ADC `latest` 容许读取时间比刚完成的 IRQ 样本略早，比较间隔应小于2^31 ms。

## 原始证据与待验收项

ST7789V2 本地手册第44–45、56页：SDA 在 SCL 上升沿采样，CS 下降时 SCL 可高可低，因此选择 LOW/1EDGE（mode0）；厂商 HIGH/1EDGE 并非同一采样沿。第226页 COLMOD 注2支持16位写入0x55。厂商 LCD `User/lcd.c` 提供 MADCTL=0x70、X+20/Y+0 和电源/Gamma 参数；KEY `adc.c/main.c` 提供通道及单端 offset 校准依据。

HAL 1.11.3 `SPI_DMATransmitCplt` 在 NORMAL 模式只启用 EOT IRQ；`HAL_SPI_IRQHandler` 在 EOT 路径关闭外设后调用 TxCplt。平台在 HAL 清除状态之前确认 EOT，并仅接受该 IRQ 内的完成。DMA 停止采用 EN 清零并读回确认，再清本流标志与 pending；EN 未清不归还内存。

尚未完成的硬件验证：模块版次/连接器方向和连续性；5V/3.3V/接地；复位/背光极性；逻辑分析仪测 SCK/DC/CS/EOT 与5MHz稳定性；横屏边界、20像素偏移、RGB/BGR、字节序；真实 AXI 缓冲和缓存一致性压力测试；ADC 六种状态分布、实际左右映射及阈值；冷启动/长时间并发、4ms控制周期、栈高水位。电机动作与台架验收均未进行。
