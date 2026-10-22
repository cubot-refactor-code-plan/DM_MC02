# BSP 层

> 引脚：[引脚分配与冲突](../../Docs/hardware/引脚分配与冲突.md)　
> `.dma_buffer`：[链接脚本改动](../../Docs/build/链接脚本改动.md)　
> 编码：[编码规范](../../Docs/spec/编码规范.md)

BSP 只搬外设数据：报文 / 字节 / 电平进软件缓冲，数据 / 占空比出硬件；不认识设备与业务。

## 模块文档

| 模块 | 文档 | 内容 |
| --- | --- | --- |
| `bsp_cfg` | [bsp_cfg.md](bsp_cfg.md) | 全局实例、`bsp_init()`、启动时序与中断使能点、上下文模型、反模式 |
| `bsp_uart` | [bsp_uart.md](bsp_uart.md) | 收发示例、接收/发送中断内部、诊断与恢复、新增串口 |
| `bsp_can` | [bsp_can.md](bsp_can.md) | 收发示例、FIFO 中断、Bus-Off 恢复、诊断、新增总线 |
| `bsp_pwm` | [bsp_pwm.md](bsp_pwm.md) | 占空比/脉宽/频率换算、示例、新增通道 |
| `bsp_key` | [bsp_key.md](bsp_key.md) | 消抖状态机、示例 |
| `bsp_dwt` | [bsp_dwt.md](bsp_dwt.md) | CYCCNT 计时、回绕处理、示例 |
| `bsp_gpio` | [bsp_gpio.md](bsp_gpio.md) | 电平控制接口、示例 |

## 层规则

- 可放：外设句柄、寄存器、中断回调、流/报文缓冲、诊断、板级常量。
- 不可放：协议解析、节点注册、设备地址表、业务状态机、任何 `#include "device_*.hpp"`。
- 判据：本层文件出现上层头文件即放错层。
- 依赖：`App → Module → Device → Protocol → Bsp`；BSP 只依赖 HAL、FreeRTOS、`Service/status.hpp`。

## 与代码注释的分工

- 代码保留：`@brief`、单位 / 范围 / 前置条件、就地踩坑。
- 文档覆盖：启动链、收发时序、诊断表、新增外设参数、上下文模型。
- 纯注释改动后 `FLASH` / `DTCMRAM` 字节数应不变。
