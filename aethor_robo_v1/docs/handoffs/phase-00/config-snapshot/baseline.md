# Phase 0 固件基线

记录日期：2026-08-12（Asia/Shanghai）
基线提交：`b6b7ab75ff5434b1ad739d55735c681d428ba4ea`
工作分支：`refactor/prd-phase-00`

## 工程与芯片

- Git 根目录：`E:/Desktop_E/TCG/Aethor_robo_fw`
- CubeMX/Keil 工程目录：`E:/Desktop_E/TCG/Aethor_robo_fw/aethor_robo_v1`
- MCU：STM32H723VGT6，来源为 `CtrBoard-H7_FDCAN.ioc` 的 `Mcu.CPN`。
- CPU 时钟：480 MHz，来源为 `.ioc` 的 `RCC.CpuClockFreq_Value=480000000`。
- HAL 时基：TIM23，来源为 `.ioc` 的 `NVIC.TimeBaseIP=TIM23`。
- RTOS：FreeRTOS CMSIS-RTOS V1；默认任务为 Normal 优先级、512 words。
- 当前默认任务由 CubeMX 以 `Dynamic` 方式配置，`freertos.c` 使用 `osThreadDef` 和 `osThreadCreate`。

## 当前外设

- FDCAN1：Classic CAN，标准帧，标称 1 Mbps；PD0 为 RX、PD1 为 TX。
- FDCAN1 标称时序：Prescaler 1、TimeSeg1 59、TimeSeg2 20，标准过滤器数量 7。
- USB_OTG_HS：Device Only FS，PA11/PA12，已生成 USB CDC 中间件。
- 正式上位机链路已冻结为现有 USB CDC；当前 `.ioc` 未启用 UART，USART1 不属于正式控制入口。

## 当前固件入口

- `Core/Src/main.c` 当前调用 `aethor_application_init()`。
- `User/aethor_application.c` 当前包含 `dual_motor_controller.h`，读取用户按键并驱动双电机验证流程。
- 七轴相关源码已存在于 `User/`，但不是当前正式运行入口。
- Phase 0 将保留旧源码文件，只从 Keil 正式目标中隔离旧运动入口，不删除文件。

## 工作树边界

执行 Phase 0 前，唯一已存在的未提交变更为：

```text
 M aethor_robo_v1/MDK-ARM/CtrBoard-H7_FDCAN.uvprojx
```

差异仅为 uVision `<LayerInfo>` 元数据。该变更视为用户/IDE 所有，不纳入 Phase 0 提交，也不覆盖。

## 基线验证

```text
命令：Tests/host/run_tests.ps1
结果：HOST_TESTS_PASSED
退出码：0
```

此结果证明现有主机侧软件回归通过，不代表 USB、CAN、电机或七轴机械臂实机验证完成。
