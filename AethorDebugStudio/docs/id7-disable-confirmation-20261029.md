<!-- @file 一次ID7失能请求及新标准反馈的证据；不授权后续运动。 -->
# ID7单次失能确认

用户本次明确确认机械臂及负载独立承托、24V接通，授权仅一次ID7失能并读取新反馈，禁止使能、运动及自动重试。

原COM4已中断，先恢复只读连接并核对USB序列386B336F3134。主控启动号变为2241977010、坐标参考11、模式2；无活动/终态事务，原故障会话未被沿用。失能前没有标准反馈，位置寄存器采样有效。

唯一一次请求经固定`/api/debug`入口提交`operation=disable,motorId=7,loadSupported=true`，对应编号958137198、固定串口命令`bench disable 7`。收到ACK及DONE：accepted=1、result=completed、enabled=00。没有第二次失能请求，没有清故障、模式写入、使能或运动请求。

随后标准反馈缓存由从未收到样本变为有时间戳的新样本，故障值为0。按HTTP接收时刻减板端年龄估算，该新样本晚于本次请求提交；该估算不作为精确CAN发送时序。进一步只读校验已知向量字节及MCU/USB配对，读取运行中的缓存，确认driver_state=0（失能）、driver_fault=0、feedback_generation=1、feedback_timestamp_us=167174000、速度解码值-0.048843rad/s，满足原0.05rad/s门限。主控全程RUNNING，无额外电机命令。该速度值接近协议零点量化值，不据此认定机械实体精确静止。

固件本次DISABLE完成路径要求有效的选中轴失能反馈；结合失能前无标准样本、本次新反馈和独立驱动状态读取，可以确认该次事务内获得了失能证明。网关通用receipt中的feedbackConfirmed=false并非此次独立复核结果，它本身不采集额外标准反馈，不能单独据此宣称确认或否认物理状态。

标准反馈只有一次回复，后续没有持续上报，现已过期。12秒只GET观察的58次轮询中，位置寄存器均持续有效；其数据不授予运动资格。没有执行后续POS或保持测试，也没有为再次获取新鲜标准样本而重发DISABLE。下一次动作仍需重新满足板端新鲜反馈与所有原有保护。

本次证据目录：`../artifacts/id7-disable-confirm-20261029T025647256Z/`。包含intent.json、before.json、receipt.json、immediate.json、readonly-followup.json、cached-driver-proof.json、summary.json。summary明确区分事务内确认与当前标准反馈过期。
