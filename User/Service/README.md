# Service 层

> 代码：`User/Service/`　不属于 `App → Module → Device → Protocol → Bsp` 主链，以工具 / 服务形式贯穿各层。

## 模块文档

| 模块 | 文档 | 内容 |
| --- | --- | --- |
| `status` | [status.md](status.md) | `Status` 取值、`EventState` 位分配 / 八个接口 / 生命周期 / 谁能用 |
| `online_check` | [online_check.md](online_check.md) | `Online` 链表、三件套、毫秒语义、调度器前构造 |
| `motor_definition` | [motor_definition.md](motor_definition.md) | `MotorData` 字段与「设备轴 → 输出轴」约定 |
| `service_cfg` | [service_cfg.md](service_cfg.md) | `sys_state`、`service_init()`、初始化闸门、新增模块、反模式 |

## 层规则

- 可放：跨设备公共设施（状态码、在线检查、通用数据结构）。
- 不可放：具体协议解析、外设操作、业务状态机、任何 Bsp / Device 头文件。
- 更深的 `EventState` 原理见 [系统状态标志原理](../../Docs/guide/系统状态标志原理.md)。

## 与代码注释的分工

- 代码保留：接口的单位 / 返回值 / 前置条件、位分配（`static_assert` 就在旁边）。
- 文档覆盖：谁该用、生命周期、`Online` 时间语义、初始化顺序、新增模块清单。
