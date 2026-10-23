# EmmV5 代码剪枝计划

> 目标：把 EmmV5 相关代码收敛到「CoreXY 真正需要的部分」，降低维护面。
> 现状测量：2026-10-23，`cmake --build --preset Debug` 通过，FLASH 152256 B。

---

## 0. 现状

| 文件 | 行数 | 说明 |
| --- | --- | --- |
| `Protocol/emmv5_protocol.hpp` | 336 | 帧编码、3 个应答扫描器、MMCL 打包 |
| `Protocol/emmv5_protocol.cpp` | 496 | 各命令工厂实现 |
| `Device/device_emmv5_group.hpp` | 387 | 多电机组（MMCL + 按地址分发） |
| `Device/device_emmv5_group.cpp` | 717 | 多电机组实现 |
| **合计** | **1936** | 不含 md 文档；单电机类已删除 |

> 2026-10-23 已执行：多机同步（`start_sync` / snF）全部删除；单电机类
> `DeviceEmmv5T`、`emm_motor1`、`App/test/emmv5/emmv5_test.cpp` 删除，单台场景
> 改用「地址表单元素」的电机组。执行后 FLASH 152256 → 151256 B。

业务侧真正用到的能力（CoreXY 通路）：

```
zero_position → enable → position(相对/双电机同帧) → stop/stop_all
request_position(0x36) → get_position
request_status(0x3A)   → get_status        （仅控制台诊断）
```

---

## 1. 判定标准

| 档 | 条件 | 处理 |
| --- | --- | --- |
| **A** | 全仓 0 引用，或只在测试里出现 | 直接删 |
| **B** | 只在控制台诊断命令里用 | 先决策要不要保留诊断 |
| **C** | 结构重复（两个类各写一遍） | 合并，需重构 |

统计方法：`grep` 全仓 `User/`，排除定义处本身。

---

## 2. A 档：直接删除（0 引用）

| 项 | 位置 | 估算行数 |
| --- | --- | --- |
| `quick_position_params()` / `quick_position()` 协议工厂 | protocol hpp/cpp | ~30 |
| `set_quick_position_params()` / `set_quick_position()` / `queue_quick_*()` | group + 单电机 | ~60 |
| `start_sync()` + `position(..., sync)` 的 snF 参数 | protocol + group | ~20 |
| `queue_enable()` / `queue_velocity()` / `queue_stop()` | group | ~30 |
| 到位帧 `FD 9F` 通路：`InPositionScanner`、`get_in_position()`、`in_position()`、`_publish_in_position()` | protocol + 两个设备类 | ~90 |
| `SystemParam` 中除 `CurrentPosition` / `StatusFlag` 外的枚举与分支 | protocol | ~40 |
| **小计** | | **≈ 270 行** |

依据：

- `quick_position*`、`start_sync`、`queue_enable/velocity/stop`、`get_in_position`、`in_position`
  在 `User/` 下除定义处外引用数均为 **0**；
- 到位判定已经完全依赖 `0x36` 位置窗口 + 连续确认，到位帧是历史方案；
- 多机同步（`00 FF 66`）已被明确禁用，snF 恒为 0。

---

## 3. B 档：需先决策

### B1 单电机类 `DeviceEmmv5T`

| 事实 | 引用 |
| --- | --- |
| `device_cfg.cpp` | `DeviceEmmv5T<...> emm_motor1({bsp_uart7, 1});` + `init()` |
| `App/test/emmv5/emmv5_test.cpp` | 唯一使用者 |
| CoreXY 通路 | 不使用（用的是 group） |

**建议：删掉 `DeviceEmmv5T` 与 `emm_motor1`，测试改用 group（`{addr}` 单元素即可）。**
收益：`device_emmv5.*`（625 行）+ `device_emmv5.md` 一起消失，两个类的重复逻辑（扫描器分发、
Online 发布、发送封装）不再需要维护两份。

代价：`bsp_uart7` 若没有其他用途，可一并从 `bsp_cfg` 摘掉。

### B2 控制台诊断命令

| 命令 | 依赖的接口 | 是否保留建议 |
| --- | --- | --- |
| `dump` | `last_multi_frame()` | **保留**（排查帧格式的唯一手段） |
| `st` | `request_status` / `get_status` | 保留（看使能/到位/堵转位） |
| `t` / `a` | `queue_position` / `set_position` | 保留（单台核对接线与方向） |
| `clr` / `mode` | `clear_protection` / `set_control_mode` | 可删，需要时再临时加 |
| `v` | `set_velocity` → `velocity()` 工厂 | 可删；删了就能把速度模式整个拿掉 |

若删 `clr` / `mode` / `v`，协议层可再去掉 `velocity()`、`clear_protection()`、
`set_control_mode()` 三个工厂（≈ 60 行），设备层对应封装（≈ 40 行）。

---

## 4. C 档：结构性合并（可选，收益最大但风险高）

| 重复点 | 现状 | 合并方案 |
| --- | --- | --- |
| 应答分发 | 两个设备类各有一份 `feed_rx()` + 3 个扫描器 + 发布函数 | 抽 `device_emmv5_detail::MotorSlot`，两个类共用 |
| 发送路径 | 单帧发送、20 ms 入队超时、`_ready` 校验各写一遍 | 统一到 `_send_frame()` |
| Config | 两个 `Config` 结构高度相似 | group 的 Config 支持单元素地址表即覆盖单机场景（配合 B1） |

若 B1 执行（删单电机类），C 档基本自动解决。

---

## 5. 预期收益

| 阶段 | 代码行数 | 说明 |
| --- | --- | --- |
| 现在 | 2605 | |
| 完成 A 档 | ≈ 2335 | 只删无引用代码 |
| 完成 A + B1 | ≈ 1710 | 两个设备类变一个 |
| 完成 A + B1 + B2 | ≈ 1610 | 再去掉速度模式与两个配置命令 |

Flash 预计只下降 0.5~1 KB（这些都是小函数，且有不少本来就被优化掉），
**真正的收益是「一个概念只有一份实现」**，而不是省空间。

---

## 6. 执行顺序与验证

每步都要单独编译 + 实机验证，不要一次删完再测：

1. **A 档 ①**：删 `quick_position*`、`queue_enable/velocity/stop`、`start_sync`（协议 + 设备）
   → 编译 → `pos` / `m 50 50` 确认 MMCL 双子命令仍正常（这条链路最容易被误伤）；
2. **A 档 ②**：删到位帧通路（`InPositionScanner` 等）
   → 编译 → `m 123 21` 两段都到位；
3. **B1**：删单电机类与 `emm_motor1`，测试改 group
   → 编译 → `en 1` / `t 1 200` / `pos`；
4. **B2**：按需删 `clr` / `mode` / `v` 与其协议工厂
   → 编译 → 控制台 `?` 列表同步更新；
5. **C 档**（可选）：抽公共 `MotorSlot`。

每步验证用的控制台命令序列：

```
stop.  en 1.  pos.  spd 300 230.  m 50 50.  m 50 60.  dump.
```

判据：`done` 出现、`pos` 与 `tgt` 相差 ≤ `arrive_tol`、`dump` 里能看到两条 `FD` 子命令。

---

## 7. 明确不剪的部分

| 项 | 原因 |
| --- | --- |
| MMCL 一条帧带两台子命令 | 硬约束，实测拆成两帧同步性变差 |
| 递回总线锁 `lock_bus()` | 多任务访问半双工总线的前提 |
| `Online` 在线判定 | `is_online()` 有 21 处引用，是断线诊断的基础 |
| `0x3A` 状态查询 | 判断使能/到位/堵转的唯一来源 |
| `SEND_QUIET_MS` / `ARRIVE_CONFIRM` / `OVERRUN_MM` | 三个都是实测踩坑后加的，删了会复现故障 |

---

## 8. 执行记录

### 2026-10-23：多机同步 + 单电机类

| 项 | 动作 | 位置 |
| --- | --- | --- |
| 多机同步触发帧 `00 FF 66` | 删除 `start_sync()` | `emmv5_protocol.*`、`device_emmv5_group.*` |
| 位置命令的 snF 参数 | 删除 `sync` 形参，snF 恒写 0 | 同上 |
| 单电机类 `DeviceEmmv5T` | 删除 hpp/cpp/md | `Device/` |
| `emm_motor1` 实例 | 删除（含 `device_init()` 里的 init） | `device_cfg.*` |
| 单电机测试 | 删除 `emmv5_test.cpp` 与任务开关 | `App/test/emmv5/`、`app_test.hpp`、`api_main.cpp` |

确认保留（本次未动）：

- 到位帧扫描器 `InPositionScanner` 与 `get_in_position()` / `in_position()`；
- 快速位置 `quick_position_params()` / `quick_position()` 及其设备层封装；
- 速度模式 `velocity()` / `set_velocity()`（CoreXY 上位机协议不下发，但设备层保留）。

编译结果：`cmake --build --preset Debug` 通过，FLASH 151256 B / DTCMRAM 67256 B。
