# Phase 0 工具链快照

记录日期：2026-08-12（Asia/Shanghai）

| 工具 | 可执行文件 | 已验证版本 |
|---|---|---|
| Keil uVision | `E:/keil/UV4/UV4.exe` | File/Product 5.43.1.0 |
| ARM Compiler 5 | `E:/keil/ARM/ARMCC/bin/armcc.exe` | File 5.05.0.106，Product 5.05.0 |
| STM32CubeMX | `E:/STM32CubeMX/STM32CubeMX.exe` | 可执行文件元数据 `>6.11.1-RC2` |
| STM32CubeMX 工程格式 | `CtrBoard-H7_FDCAN.ioc` | `MxCube.Version=6.11.1` |
| STM32CubeH7 | `CtrBoard-H7_FDCAN.ioc` | `STM32Cube FW_H7 V1.11.2` |

CubeMX 可执行文件的 Windows 版本资源带有 `>` 前缀，因此生成兼容性以 `.ioc` 中的 6.11.1 元数据和实际生成差异为最终判断依据。重新生成后必须检查用户代码块、FDCAN1、USB、TIM23、FreeRTOS 和 Keil 引用，不能只依据版本字符串判断成功。
