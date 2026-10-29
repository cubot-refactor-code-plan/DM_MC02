# corexy

> 代码：`corexy.hpp` / `corexy.cpp`　实例：`corexy`（`module_cfg.cpp`）
> 两台电机按 CoreXY 布局驱动平面机构：世界坐标运动学 + 双矩形限位 + 45°/90° 分段。

## 0. 快速使用

```cpp
// 1) 头文件：业务侧只需 module_cfg.hpp
#include "module_cfg.hpp"

// 2) 实例：module_cfg.cpp —— 只给组、地址、两块限位，几何参数固定在类内
CorexyT<decltype(emm_group)> corexy({
  .group  = &emm_group,
  .addr1  = addr1,
  .addr2  = addr2,
  .theory = theory_rect,   // 理论限位：机械真实行程
  .safe   = safe_rect,     // 安全限位：允许作为目标的范围
});

// 3) 别名 + 声明：module_cfg.hpp
using CorexyModule = CorexyT<decltype(emm_group)>;
extern CorexyModule corexy;

// 4) 显式实例化：corexy.cpp 顶部
template class CorexyT<DeviceEmmv5GroupT<BspUartT<uart_detail::DmaTransport, 128, 128>>>;

// 5) 初始化：api_main.cpp 的 all_init()，排在 device_init() 之后
module_init();

// 6) 运行：corexy_io_task 启动时 zero_here() + enable(true)，之后 10 ms 一次 update()
corexy.update();
```

本工程实际填入的数值见 [`module_cfg.md`](module_cfg.md)。

**调用示例**

```cpp
#include "module_cfg.hpp"

corexy.zero_here();                       // 当前位置记为 (0,0)，必须在任务上下文调用
corexy.enable(true);                      // 内部：静默 2 ms → 电机 1 → 2 ms → 电机 2
corexy.move_to({50.0f, 20.0f});           // 绝对移动：先 45° 再轴向
corexy.move_by({0.0f, -10.0f});            // 相对移动（方向核对可用控制台 g <0..7>）
corexy.update();                          // 10 ms 周期，直到 state() == Idle
```

### 0.1 踩坑速查（改代码前必看）

| 现象 | 根因 | 位置 |
| --- | --- | --- |
| 电机高速跑飞 | 绝对模式写负目标，被当成无符号大数 | 统一“相对模式 + 方向字节 + 正幅值” |
| 命令成功但不走 | 相对基准用了书面目标值而不是实测 counts | `_send_segment` 取实测反算 |
| 第一段能走、第二段不执行 | 到位那一拍刚发 `0x36`，0 ms 后又发 MMCL，半双工撞车 | `BUS_GAP_MS` 静默期 |
| 两轴失步、贴边 `BAD_ARG` | 减速段末尾被当成“到位”，新命令叠在未完成的运动上 | `ARRIVE_CONFIRM` 连续 2 拍 |
| 贴边走被误杀 | 理论矩形包围盒容差用了 `arrive_tol`（0.2 mm） | 改用 `OVERRUN_MM`（2 mm） |
| 停止/使能连发不生效 | `send()` 只入队，DMA 把两帧粘连 | 帧间 `BUS_GAP_MS` |
| 只有一台失能 / 位置到了却一直 `Moving` | 无应答命令（使能/清零/停止）发送前没静默，被在途 `0x36` 应答吃掉 | 统一 `_quiet()` + 增量式到位判定 |
| 第二段同一个 MMCL 里只有一台先动 | 上一段没停稳就发了下一条 MMCL；或读位置不是“一问一答” | MMCL → 2 ms → 读 1 → 2 ms → 读 2 → 两台位置都对 → 下一条 MMCL |

完整复盘（现象/定位过程/根因/修复）见 [`Docs/plan/CoreXY调试复盘.md`](../../Docs/plan/CoreXY调试复盘.md)。

## 1. 坐标与换算

世界系：原点 `(0,0)`，`+X`、`+Y` 相互垂直，单位 mm。两台电机记作 1、2。

设 `k = 脉冲/mm`，`c = counts/mm`：

```text
反馈：X = -(c1 - c2) / (2c)          c1、c2 = 两台电机的反馈位置
      Y = -(c1 + c2) / (2c)

指令：p1 = -k · (X + Y)
      p2 = +k · (X - Y)
```

几何参数是**类内私有常量**（本模块只驱动 ZDT 闭环步进）：

| 常量 | 含义 | 值 |
| --- | --- | --- |
| `MM_PER_REV` | 电机一圈对应的机构行程 | 40 mm |
| `PULSES_PER_REV` | 位置指令一圈的脉冲数 | 3200（16 细分） |
| `COUNTS_PER_REV` | 位置反馈一圈的 counts | 65536 |

由它们派生出 `PULSES_PER_MM = 80`、`COUNTS_PER_MM = 1638.4`、`COUNTS_PER_PULSE = 20.48`，
全部在编译期算好，运行期不再折算。换非 ZDT 驱动器时改这三个 `constexpr` 即可。

### 实例参数

本工程只驱动 ZDT 闭环步进，几何固定；两块限位与速度由 [`module_cfg.md`](module_cfg.md) 传入，
实际数值与来历也见该文档。

## 2. 电机转向 ↔ 世界方向

| 世界方向 | 电机 1 | 电机 2 |
| --- | --- | --- |
| `+X` | ↓ | ↑ |
| `-X` | ↑ | ↓ |
| `+Y` | ↓ | ↓ |
| `-Y` | ↑ | ↑ |
| `+X +Y`（45°） | ↓↓ | 不动 |
| `+X -Y`（45°） | 不动 | ↑↑ |
| `-X -Y`（45°） | ↑↑ | 不动 |
| `-X +Y`（45°） | 不动 | ↓↓ |

即：**轴向两台同时动，45° 对角只动一台**。核对用控制台命令 `g <0..7> [mm]`。

## 3. 限位模型

两块轴对齐矩形：

| 名称 | 作用 | 当前值 |
| --- | --- | --- |
| `theory` | 机械行程，运动越界即停机（防撞） | `(0,0) ~ (190,200)` |
| `safe` | 允许运动的目标范围 | `(1,1) ~ (188,198)`，187 × 196 |

- `move_to` / `move_by` / `step` 只按 `safe` 校验，越界返回 `BAD_ARG`；
- 运动过程中实测坐标越出 `theory`（外扩容差 = `OVERRUN_MM` = 2 mm）→ 广播停止并进入 `Fault`；
  容差不能用 `arrive_tol`：正常减速过冲就有 1 mm 量级，贴着 `y=0` 的段会被误杀；
- 越出 `safe` 但仍在 `theory` 内只打印警告，不停机（用于观察标定残差）；
- 安全限位是凸矩形，45°/90° 分段的中转点由两端凸组合得到，因此无需逐段复判。

## 4. 45°/90° 分段

`move_to(p)` 以当前**实测**坐标为起点：

1. `dx == 0`、`dy == 0` 或 `|dx| == |dy|` → 单段；
2. 否则 `d = min(|dx|,|dy|)`，先走 45° 段（两轴各 `d`），再走轴向段到目标；
3. 每段发一条 **MMCL 总帧**（两台子命令）；子命令为相对模式 + 方向字节 + 正幅值；
   增量基准用**实测位置**（`counts / counts_per_pulse`），不维护“上一条指令目标”；
   发出后只读位置、不发新命令，直到两台读回的位置都进入到位窗口才发下一段；
4. 速度按脉冲数分摊，保证同时到达：

```text
maxΔp = max(|Δp1|, |Δp2|)
rpm_i = clamp(rpm · |Δp_i| / maxΔp, 1, 5000)
acc   两台相同（加速时间与 rpm 成正比，比例关系自动保持）
```

`|Δp_i| == 0` 的电机不参与本段（45° 因此只有一条子命令）。

**每段必须先确认到位，才发下一段**（不依赖驱动器的到位帧，那个回报默认不开）：

1. MMCL 发出后先静默 `BUS_GAP_MS`（2 ms），**不能紧接着读**；
2. 然后依次读两台：读 1 → `BUS_GAP_MS`（2 ms）→ 读 2；
3. 两台读回的 `0x36` 位置都落在本段目标的 `arrive_tol` 内、**相邻两次读回几乎不变**（≤ `STILL_COUNTS`，证明已停稳）、并连续 `ARRIVE_CONFIRM`（=2）拍成立才算到位；
4. 到位后再发下一条 MMCL，同样走「2 ms → 读两台」的流程。

**段兜底**：本段发出后 `SEG_DONE_MS`（3 s）还没判到位，就**按已到位处理**（跳下一段 / 结束），
不会进 `Fault`。跳下一段前会先发一次停止，避免新命令叠在没跑完的运动上。
要人为中断始终用 `stop()`。

### 4.1 两种下发模式

| 模式 | 每段下发 | 两轴单帧长度 | 特点 |
| --- | --- | --- | --- |
| `MoveMode::Position`（默认） | `addr FD dir vel acc clk raF snF 6B`（13 B） | 31 B | 一条帧带全信息，最稳；方向走独立的 `dir` 字节 |
| `MoveMode::Quick` | 先 `addr F1 vel acc mode 00 6B`（7 B），再 `addr FC clk(4B) 6B`（7 B） | 19 B（参数有变化时 33 B） | 帧更短、驱动器少解析一套参数，执行更快 |

- 切到快速模式：`set_move_mode(MoveMode::Quick)`，或上位机 `CMD 0x0A` 负载 `01`；
- 模块会缓存每台已下发的 `0xF1` 参数，**只有转速或加速度变化时才补发**参数帧；
  45° 段只有一台动、两段速度相同的情况下，第二段就只有 `0xFC`；
- **`0xFC` 没有方向字节**，方向靠脉冲的符号（负 = 反转）。这一点与 `0xFD` 的
  “方向字节 + 正幅值”不同，换固件后要实机确认符号约定；`stop()` / 切模式会把
  参数缓存作废，下一段重新下发。
- 分段进行中不允许切模式（返回 `BUSY`），否则两台会拿着不同参数动作。
## 5. 状态机与读位置时序

一次移动的完整时序：

```text
move_to(p)                 校验安全限位 → 拆段 → 立刻发第 1 段
  ├─ 静默 BUS_GAP_MS      等上一条 0x36 的应答收完（半双工）
  ├─ 组 MMCL 总帧          两台子命令在**同一帧**里，一次发出
  └─ 置 _read_hold         MMCL 后 BUS_GAP_MS（2 ms）内一条查询都不发

update() 周期一次（建议 10 ms）
  ├─ 读 1                 发 0x36 → 等到应答（最多 REPLY_TIMEOUT_MS）
  ├─ 隔 BUS_GAP_MS        两条查询之间也留 2 ms
  ├─ 读 2                 发 0x36 → 等到应答
  ├─ 越界 / 超程检查       → 越界进 Fault
  └─ 到位判定（连续确认） → 发下一段 / 进 Idle
```

> 读位置是"一问一答"：发一台、等它回完，再发下一台；两台同时只允许一条查询在途。“到位帧”（驱动器主动上报的 `FD 9F`）默认不开，判到位只用 `0x36` 读回的位置。

| 状态 | 含义 | 转移 |
| --- | --- | --- |
| `Idle` | 无待走分段 | `move_*` → `Moving` |
| `Moving` | 走当前分段 | 两轴到位 → 下一段 / `Idle`；越界或发送失败 → `Fault` |
| `Fault` | 已停机 | 只能 `stop()` 清除 |

**原点门控**：`zero_here()` 的命令无应答，必须靠读数复核。连续 3 拍位置都在 ±0.5 mm 内
才置 `origin_verified()`；未复核时 `move_to` 返回 `NOT_INIT`，控制台提示先执行 `home`。
`pos` 输出末尾的 `z=ok/NO` 就是该标志。

`update()` 每次做三件事：

1. 读两台位置：一台一台来（发请求 → 等应答 → 隔 2 ms → 下一台）；`_read_hold` 之前一律不读；
2. 收位置后重算世界坐标，并做原点复核；
3. 推进状态机：到位判定 → 下一段；越界 / 发送失败 → `Fault`。

到位判据（每段独立）：发完 MMCL 隔 2 ms 开始读位置，**一次一台**（读 1 → 2 ms → 读 2）；
两台都读到应答、各自的 `|反馈 - 本段目标| ≤ arrive_tol`
（目标 = 段起点实测 counts + 本段指令增量，不走世界坐标，所以编码器零位有偏差也判得准），
**而且相邻两次读回的 counts 相差 ≤ `STILL_COUNTS`（=10，约 0.006 mm）**，
并**连续 `ARRIVE_CONFIRM`（=2）拍**成立才算到位。

为什么必须加“不再变化”：`arrive_tol` 的 0.2 mm 窗口在减速尾巴上就会成立（此时还有
≈28 ms 才真正停住），只判窗口的话下一段 MMCL 会在电机还在减速时发过去，
驱动器把新命令当成下一条目标排队，表现为“同一个 MMCL 里只有一台先动”。
读数不再变化才能证明已经停稳。

以 `m 100 50`（起点 `0,0`）为例：先到 `50,50`（45°，只电机 1 动）→ 两台读回位置都对 →
再到 `100,50`（+X，两台都动）→ 两台读回位置都对 → `done`。

## 6. 上位机串口协议（`corexy_protocol`）

`bsp_uart1`，二进制帧，不再有 ASCII 命令行。多字节一律**大端**。

> 帧定义、命令表、应答/事件与**可直接跑的测试案例**见 [`corexy_protocol.md`](../Protocol/corexy_protocol.md)。
> 下面只是摘要。

### 6.1 帧格式

```text
A5 5A | CMD | LEN | PAYLOAD[LEN] | SUM
SUM = CMD ^ LEN ^ PAYLOAD[0] ^ ... ^ PAYLOAD[LEN-1]
```

- 坐标单位 **0.01 mm**（i16），范围 ±327.67 mm；
- 帧头错位会自动重新同步；`LEN > 24` 或校验错的帧整帧丢弃。

### 6.2 应答

每条命令都回一帧（`CMD | 0x80`）：

```text
通用   A5 5A [CMD|0x80] 05 | status | x:i16 | y:i16 | SUM
限位   A5 5A 89 LEN | status | 理论 x0,y0,x1,y1 | 安全 x0,y0,x1,y1 | SUM   (LEN=17)
```

### 6.3 事件（主动上报）

```text
A5 5A 7F 06 | evt | status | x:i16 | y:i16 | SUM
```

| evt | 含义 |
| --- | --- |
| `0x01` | 上电自检完成（status = OK 表示原点已复核，NOT_INIT 表示未复核） |
| `0x02` | 一段移动完成 |
| `0x03` | 进入 Fault（status = 具体原因：TIMEOUT / BAD_ARG / IO_ERROR） |

### 6.4 命令表

| CMD | 名称 | 请求负载 | 说明 |
| --- | --- | --- | --- |
| `0x01` | 使能/失能 | `on:u8` | 两台一起，1=使能 0=失能 |
| `0x02` | 绝对位置 | `x:i16 y:i16` | 世界坐标，受安全限位校验 |
| `0x03` | 相对位置 | `dx:i16 dy:i16` | 世界坐标增量 |
| `0x04` | 归零 | — | 当前位置记为 `(0,0)`，需等原点复核 |
| `0x05` | STOP | — | 立即停止并清故障 |
| `0x06` | 速度/加速度 | `rpm:u16 acc:u8` | 运行期生效，不影响已下发的段 |
| `0x07` | 快速位置参数 | `addr:u8 rpm:u16 acc:u8 mode:u8` | `mode` 0=相对 1=绝对 |
| `0x08` | 快速位置 | `addr:u8 pulses:i32` | 相对/绝对由 `0x07` 决定 |
| `0x09` | 查询限位 | — | 返回理论 + 安全两块矩形 |
| `0x0A` | 分段下发模式 | `mode:u8` | 0 = 位置帧(`0xFD`)，1 = 快速位置(`0xF1`+`0xFC`) |

说明：

- **不提供速度模式命令**：速度模式只在设备层保留（`DeviceEmmv5GroupT::set_velocity`），
  上位机不需要它；
- `0x07`/`0x08` 绕过 CoreXY 运动学，直接给单台电机下发脉冲，用于标定与单轴核对；
- 动作类命令（`0x02`/`0x03`）在 `Fault` 状态下返回 `BAD_ARG`，必须先 `0x05` 清故障。

## 7. 任务与并发

| 任务 | 职责 | 优先级 / 周期 |
| --- | --- | --- |
| `cxy_con`（`corexy_console_task`） | 读 bsp_uart1 → 解析 ASCII 命令码 → 投递 `CorexyCmd`（默认） | +4 / 10 ms |
| `cxy_pro`（`corexy_task`） | 读 bsp_uart1 → `cxy::feed()` 解帧 → 投递 `CorexyCmd` | +4 / 阻塞 10 ms |
| `cxy_io`（`corexy_io_task`） | 队列消费 + 独占电机总线 + `corexy.update()` | +5 / 10 ms |

- **输入源二选一**：`app_task.hpp` 的 `APP_COREXY_PROTOCOL`，`0` = 命令码终端（默认，手工测试），
  `1` = 二进制协议；两者共用 USART1，只能开一个；
- 两个 CoreXY 任务通过 `corexy_cmd_queue`（`corexy_protocol.hpp`）通信，队列元素是语义命令，不是字节帧；
- `corexy.update()` / `move_*` 只能由 `cxy_io` 调用（MMCL 缓冲、总线均为单所有者）；
- 模块不建任务、不创建锁（总线互斥借用设备组的递归锁）；对象必须是全局/静态生命周期；
- RS485 半双工：连续 `send()` 只是入队，DMA 会把两帧连成一串；分开的帧之间留 2 ms；
- **发任何控制命令前静默 2 ms**（`BUS_GAP_MS`，统一走 `_quiet()`）：等上一条 `0x36`
  的应答收完，否则驱动器正在发送、收不进字节，整帧被丢弃；静默期内也不发查询。
  覆盖范围：分段位置帧、使能/失能、清零、停止；上层直发的 `0xF1`/`0xFC` 诊断帧没有静默
  （`quiet()` 已收进私有），只能在空闲时用；
- 一个常数管五处间隔：`BUS_GAP_MS = 2 ms`（帧间、发命令前静默、命令族之后到开始读、MMCL 后到首读、读周期）；
- **MMCL 发出后 2 ms（`BUS_GAP_MS`）才能开始读位置**，两条查询之间也留 2 ms；
  使能 / 清零 / 停止后同样先把 `_read_hold` 推开一段时间；
- 故障/停止后无需额外重同步：增量基准始终取实测位置；
- **到位判定是增量式的**：`目标 = 段起点实测 counts + 本段增量`，不引用编码器零位，
  所以清零命令丢失也不会让运动“一直停在 `Moving` 等不到那个永远不存在的绝对目标”；
- **段兜底 `SEG_DONE_MS = 3 s`**：到点还没判到位就按“已到位”处理（不是故障）；要中断只能手动 `stop`；
- 上电由 `cxy_io` 完成停止 / 清零 / 使能 / 原点复核。

### 7.1 模板参数 `Group` 的接口契约

`CorexyT<Group>` 是鸭子类型（模板编译期检查），`Group` 只需提供下列成员：

```cpp
bool    is_ready() const;
Status  enable(uint8_t addr, bool on);
Status  zero_position(uint8_t addr);
Status  stop(uint8_t addr);
Status  stop_all();
bool    get_position(uint8_t addr, int32_t &out);   // 取走最新 0x36 应答
Status  request_position(uint8_t addr);             // 发一次 0x36 请求

void    multi_clear();
Status  queue_position(uint8_t addr, emmv5::Direction dir, uint16_t rpm,
                       uint8_t acc, int32_t pulses, emmv5::PositionMode mode);
Status  queue_quick_position_params(uint8_t addr, uint16_t rpm, uint8_t acc,
                                    emmv5::PositionMode mode);   // 仅 Quick 模式
Status  queue_quick_position(uint8_t addr, int32_t pulses);      // 仅 Quick 模式
Status  multi_send(uint8_t target = emmv5::BROADCAST);
```

> 总线锁（`lock_bus()` / `unlock_bus()`）由设备组内部使用，模块层不直接调用：
> `multi_clear` / `queue_*` / `multi_send` 各自已经是总线事务。

换驱动器时，实现这组接口即可复用整个模块层；换串口 / 换机械只改 `Config` 数值。

## 8. 错误与诊断

| 现象 | 可能原因 | 排查 |
| --- | --- | --- |
| `move_to` 返回 `BAD_ARG` | 目标超出 `safe`，或处于 `Fault` 未 `stop()` | `lim` 看限位；`stop` 清故障 |
| `move_to` 返回 `BUSY` | 上一段还没走完 | `pos` 看状态，等 `Idle` |
| `Fault` + `BAD_ARG` / `IO_ERROR` | 实测坐标越出理论限位 / 单轴超出预期行程 / 发送失败 | 看 `pos` 与 `dump`，`stop` 后重新对零 |
| `Fault` + `IO_ERROR` | 单轴偏离本段预期行程 > 2 mm（反向或跑飞） | 看 `pos` 的 c1/c2 变化方向；若反向需修正接线/电机方向 |
| `Fault` + `BAD_ARG` | 实测坐标越出 `theory` | 检查是否上电未回位、皮带丢步 |
| 原点偏差 > 0.5 mm | 上电位置不在左下角，或清零命令未执行 | 手动回位后 `home` |
| 目标方向与预期相反 | 电机接线 / 转向定义变化 | 用 `g 0`、`g 2` 核对 §2 表 |

## 9. 反模式

- 在 `all_init()` 里调 `zero_here()`：调度器尚未启动，串口阻塞发送不可靠；
- 在第二个任务里调 `update()`：会与 `corexy_task` 抢位置请求，两台电机的应答互相覆盖；
- 把 `PULSES_PER_REV` 当成反馈单位：到位判定会永远不满足；
- 直接把 `safe` 设成 `theory`：失去防撞余量，装配公差会撞边；
- 上电前把机构停在行程中段：清零后的世界坐标与机械位置不符。

## 10. 实机验证清单

当前状态：**单轴 / 45° / 分段双电机运动已实机走通**（2026-10-23）。
烧录后按以下顺序核对，判据为串口输出：

1. 上电后 `pos`：两台 counts 应接近 0，坐标接近 `(0.00, 0.00)`；
2. `en 1` → `g 0 10` … `g 7 10`：逐档核对 §2 的方向表与行程；
3. `m 100 100`：观察是否先 45° 再轴向，两段都停在目标 ±0.2 mm；
4. `m 0.5 50` / `m 190 100`：应回 `reject: outside safe limit`；
5. `m 1 1`、`m 188 198`：应能走到（误差 ≤ 0.2 mm）；`m 189 199` 现在应被拒；
6. 移动中 `stop`：两台立即停；
7. `home` 后 `pos` 重新为 `(0.00, 0.00)`，随后 `m 10 10` 正常。
