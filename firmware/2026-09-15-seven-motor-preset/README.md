# ID1～7 同型号S3519预置固件

用户确认ID1～6与ID7同型号、同驱动器、同Gr=19.2032。本版将现有LCD台架参数、输出轴换算和基础ID特殊命令扩展到七台电机。范围与身份仍按每台的实际发现结果独立生效；ID1～6未接入，仅做软件验证。

- [LCD-MIT.hex](LCD-MIT.hex)：七电机POS/MIT本地调试入口，每次只控制所选单台。
- [LCD-POS.hex](LCD-POS.hex)：七电机POS本地调试入口。
- [LCD-ReadOnly.hex](LCD-ReadOnly.hex)：七电机同型号显示/通信兼容预置，本地运动关闭。

LCD默认10°/s、步进1°/s，角度步进1°，Kp=80/Kd=0.2，反馈转矩停止门限3.5 Nm，沿用此前调试行为。参数表和接入步骤见 [七电机预置说明](../../docs/debug-ui/seven-motor-preset-20260915.md)。开机仍只自动发现ID7；新接入ID需先通过USB `bench init N` 初始化，再在LCD选中它并申请本地控制。ESC ID为1～7，Master ID为0x11～0x17，应在各驱动器上对应设置。

28项完整检查通过，四目标Keil编译0错误、0警告。新增回归逐个覆盖七ID的独立发现、未发现时锁定、单位换算、单目标命令、CAN路由及参数失效；另检查七电机只读配置和24组编译门控组合。

RAM维持：普通目标98680字节，各LCD目标174464字节。相对此前速度修正版，Code+RO变化为普通目标+24、LCD-ReadOnly不变、LCD-POS/MIT各+32字节。完整源码/HEX摘要和内存布局见 [verification.json](verification.json)。该文件记录软件验证状态，不代表未接入六台电机已实机通过。

前一版源码保留在 `archive/lvgl-astra-speed-b2`，原 [速度修正版HEX](../2026-09-15-astra-speed-preview/README.md) 保持不变。本版板上烧录记录独立保存于本机 `D:/download/TCG/output/seven-motor-flash-20260915`。
