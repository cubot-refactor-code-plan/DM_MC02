# Device 层

> 代码：`User/Device/`　只依赖 Bsp（及 Protocol / Service 的纯工具）。
> 引脚：[引脚分配与冲突](../../Docs/hardware/引脚分配与冲突.md)　
> EmmV5 阅读顺序：[emmv5与uart阅读顺序](../../Docs/guide/emmv5与uart阅读顺序.md)

Device 把帧 / 字节 / 占空比换成物理量，或把物理量换成协议帧。
一个实例 = 一台设备（或一条总线上的设备组）。

## 模块文档

| 模块 | 文档 | 内容 |
| --- | --- | --- |
| `device_cfg` | [device_cfg.md](device_cfg.md) | 层规则、实例表、三种驱动骨架、`device_init()`、新增设备、参数表、反模式 |
| `dji_motor_group` | [dji_motor_group.md](dji_motor_group.md) | 位置/电调 ID/控制帧、反馈解析与算例、线程模型 |
| `device_emmv5_group` | [device_emmv5_group.md](device_emmv5_group.md) | 地址表、按地址分发、MMCL、结果接口；单台电机就把地址表写成单元素 |
| `device_servo` | [device_servo.md](device_servo.md) | 角度换算、梯形规划、单写者 |
| `device_r9ds` | [device_r9ds.md](device_r9ds.md) | SBUS 切帧、通道表、在线判定 |
| `device_buzzer` | [device_buzzer.md](device_buzzer.md) | 音调 / 音量 / 阻塞鸣叫 |

## 与代码注释的分工

- 代码保留：`@brief`、单位 / 范围 / 前置条件、就地坑。
- 文档覆盖：位置-ID 对照表、解析公式与算例、线程模型、MMCL 算术、新增设备清单。
