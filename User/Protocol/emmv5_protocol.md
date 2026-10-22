# emmv5_protocol

> 代码：`emmv5_protocol.hpp` / `emmv5_protocol.cpp`
> 张大头 EmmV5.0 串口协议：只做编码 / 解析，不碰串口、不建任务、不持有设备状态。

## 边界

| 放这里 | 不放这里 |
| --- | --- |
| 语义参数 → 字节帧（编码） | 发送帧、读串口、开关发送 |
| 字节流 → 应答（解码 / 跨块拼帧） | 地址表、重发、超时、在线判定 |
| 协议常量、功能码查表、工厂函数 | 任何 BSP / Device 头文件 |

**Device 不拼字节，Protocol 不发字节。**

## 导出符号

| 符号 | 作用 |
| --- | --- |
| `CHECKSUM = 0x6B` | 每条命令固定校验字节（未实现 XOR / CRC-8） |
| `BROADCAST = 0x00` | 广播地址 |
| `FRAME_MAX_SIZE = 32` | 单条命令最大字节数（当前最长 13） |
| `IN_POSITION_MARK1/2/3` | 到位帧标识 0xFD / 0x9F / 0x6B |
| `Direction` / `PositionMode` / `SystemParam` | 枚举 |
| `Frame` | 值类型帧缓冲 |
| `InPositionScanner` / `PositionScanner` / `StatusScanner` | 三类应答的滑动窗口识别器 |
| `build_multi_frame()` | MMCL 多机总帧打包 |
| 8 个命令工厂 | `zero_position` / `enable` / `velocity` / `position` / `quick_position_params` / `quick_position` / `stop` / `read` |

## `Frame`

```
写入中 --append_u8/u16/u32--> 写入中（size < 32）
写入中 --seal()--> 已封口
写入中/已封口 --越界或再写--> overflow = true（此后写入无效）
```

- `append_u16` / `append_u32` 一律大端。
- `valid() == sealed && !overflow && size > 0`；设备层发送前必须检查。
- 非法枚举（如 `read()` 收到未知 `SystemParam`）返回空 `Frame`，由设备层转 `BAD_ARG`。

**编码示例**

```cpp
emmv5::Frame f = emmv5::position(1U, emmv5::Direction::CW, 500U, 50U, 3200, emmv5::PositionMode::Absolute);
if (!f.valid()) { /* BAD_ARG：不发给总线 */ }

// 协议层不发送：交给设备层 / BspUartT
(void)uart.send(f.data, f.size, 20U, nullptr);
```

## 命令帧

| 工厂 | 帧布局 | 说明 |
| --- | --- | --- |
| `zero_position(addr)` | `addr 0A 6D 6B` | 清零位置、位置误差、脉冲数 |
| `enable(addr, on)` | `addr F3 AB on 00 6B` | `on` = 0/1；`sync` 固定 0 |
| `velocity(addr, dir, rpm, accel)` | `addr F6 dir rpmH rpmL accel 00 6B` | 速度模式 |
| `position(addr, dir, rpm, accel, pulses, mode)` | `addr FD dir rpmH rpmL accel p3 p2 p1 p0 mode 00 6B` | 位置模式 |
| `quick_position_params(addr, rpm, accel, mode)` | `addr F1 rpmH rpmL accel mode 00 6B` | 只设参数，不产生运动 |
| `quick_position(addr, pulses)` | `addr FC p3 p2 p1 p0 6B` | 用上一组快速位置参数 |
| `stop(addr)` | `addr FE 98 00 6B` | 紧急刹车，速度/位置模式都可用 |
| `read(addr, param)` | `addr code 6B` | 应答格式随参数不同 |

- 速度 0~5000 RPM；加速度 `0` = 直接按设定速度运行，`> 0` = 每提升 1 RPM 用时
  `(256 − accel) × 50 µs`（例：accel 50，0→300 RPM ≈ 3.09 s）。
- 脉冲：16 细分 3200/圈，32 细分 6400/圈；`int32_t` 带符号，写入按 32 位补码。
- `raF`：`0` = 相对上一输入目标（不是当前位置），`1` = 绝对；不依赖 Y42 的 `2`。
- `sync` 固定 0（不使用多机同步启动）。

`read()` 功能码：

| `SystemParam` | code | 应答 |
| --- | --- | --- |
| `BusVoltage` | 0x24 | mV |
| `BusCurrent` | 0x26 | mA |
| `PhaseCurrent` | 0x27 | mA |
| `EncoderRaw` | 0x29 | 编码器原始值 |
| `PulseCount` | 0x30 | 实时脉冲数 |
| `EncoderLinear` | 0x31 | 0~65535/圈 |
| `InputPulse` | 0x32 | 符号 + 32 位 |
| `TargetPosition` | 0x33 | 符号 + 32 位，65536/圈 |
| `SetPosition` | 0x34 | 符号 + 32 位 |
| `Velocity` | 0x35 | 符号 + 16 位 RPM |
| `CurrentPosition` | 0x36 | 符号 + 32 位，65536/圈 |
| `PositionError` | 0x37 | 符号 + 32 位 |
| `BatteryVoltage` | 0x38 | Y42：mV |
| `Temperature` | 0x39 | Y42：℃ |
| `StatusFlag` | 0x3A | 状态字节 |
| `OriginFlag` | 0x3B | 回零状态 |
| `StatusAndOriginFlag` | 0x3C | Y42：状态 + 回零 |
| `PinStatus` | 0x3D | Y42：引脚状态 |

位置类数值一圈 = 65536，角度 = `value × 360 / 65536`。

## 扫描器

| 扫描器 | 窗口 | 命中条件 | 命中后 |
| --- | --- | --- | --- |
| `InPositionScanner` | 4 | `addr FD 9F 6B` | 清窗，继续扫完本块 |
| `PositionScanner` | 8 | `addr 36 … 6B` | 清窗并 `return true` |
| `StatusScanner` | 4 | `addr 3A status 6B` | 清窗并 `return true` |

- 窗口满就 `memmove` 左移一格；跨块保留未凑满的后缀。
- 到位帧是事件（一块可能多条，继续扫）；位置/状态是最新值（取到即返回）。
  同一块里粘着同一台电机的两条 0x36 时，第二条被丢弃。
- 地址判断在扫描器内部完成，设备层把同一块字节喂给组内每台电机。

`PositionScanner` 数值：`magnitude = w[3..6] 大端`，`value = (w[2] == 0) ? magnitude : −magnitude`
（用 int64 取负，避开 `INT32_MIN` 溢出）。

**解析示例**

```cpp
emmv5::PositionScanner   pos_scanner;   // 每个地址一套，设备对象持有
emmv5::InPositionScanner inpos_scanner;

int32_t position = 0;
if (pos_scanner.feed(1U, buf, n, position)) { /* counts，65536/圈，带符号 */ }
if (inpos_scanner.feed(1U, buf, n))         { /* 至少一个到位帧 */ }
```

## MMCL 多机总帧

```
target(1) + 0xAA(1) + total(2, 大端) + 子命令串(N) + 0x6B(1)
total = N + 5（含头尾）；子命令 = 各自地址 + 功能码 + 数据 + 0x6B
```

- `total > 0xFFFF` 或 `total > out_capacity` → 返回 0。
- `payload` 可能与 `out + 4` 重叠，内部必须 `memmove`。
- 发送流缓冲 128 字节 → 子命令上限 123（`128 − 5`），由设备层 `MULTI_PAYLOAD_MAX` 拦截。

## 与 Device 的契约

| 方向 | Protocol | Device |
| --- | --- | --- |
| 发 | `Frame`（`valid()` 为真） | 检查 `valid()`、`uart.send(frame.data, frame.size, 20)` |
| 收 | `Scanner::feed(addr, data, size, out)` | 取字节、按地址分发、锁存结果、刷 `Online` |

- 扫描器有状态，每个地址一套。
- 协议层不 include `bsp_uart.hpp` / `device_*.hpp`。

## 新增命令 / 协议

新增命令：

1. `.hpp` 对应分区声明 `Frame xxx(...)`，写清单位与范围。
2. `.cpp` 按手册字节顺序 `append_*`，末尾 `seal()`。
3. 设备层加接口（单机 + 多机 `queue_xxx`）。
4. 需要应答的加扫描器（或扩展 `SystemParam`），设备层补 `get_*()`。

参数归属：

| 参数 | 取值 / 编码 | 校验位置 |
| --- | --- | --- |
| 地址 | 1~255；0 = 广播 | 设备层 |
| 方向 | `CW = 0` / `CCW = 1` | 设备层 |
| 速度 | 0~5000 RPM，大端 2 字节 | 设备层 |
| 加速度 | 0~255，`(256 − accel) × 50 µs` / RPM | 协议层透传 |
| 脉冲 | `int32_t`，32 位补码 | 设备层决定行程 |
| 位置模式 | `Relative = 0` / `Absolute = 1` | 枚举 |
| `sync` | 固定 0 | 工厂函数 |

- 范围校验放设备层，协议层只编码，便于主机侧按字节比对。
- 新增字段后重算帧长；超过 `FRAME_MAX_SIZE` 就同步改常量（越界会置 `overflow`）。

新增协议模块（`User/Protocol/<name>_protocol.*`）：独立 `namespace`、只 include 标准库、
编码用值类型 + 工厂函数、解析用独立扫描器；在 `User/App/test/` 用固定字节验证切帧。

## 反模式

| 写法 | 后果 |
| --- | --- |
| 协议层调 `uart.send()` | 无法复用，且依赖硬件 |
| 假设一次读到整帧 | 分块到达时帧被切歪 |
| 不检查 `Frame::valid()` | 截断帧发上总线 |
| 多个电机共用一套扫描器 | 地址判断失效 |
| 把 `SystemParam` 枚举值当功能码 | 功能码在 `.cpp` 查表里 |
| 相对模式按“当前实时位置”理解 | 基准是上一输入目标 |
