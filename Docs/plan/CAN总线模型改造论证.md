# CAN 总线模型改造论证

> 状态：**论证已结束（2026-10-03），实施中**
> **最终决策与实施计划见 [CAN 总线改造实施计划](CAN总线改造实施计划.md)。**
> 本文保留方案对比与决策过程，作为背景与依据。
> 本文关于现状的结论均给出 `文件:行号` 依据；关于收益、开销与风险的结论均为分析推断，**未经实机验证**。
> 先导改动见 §10，已完成编译与主机测试，同样**未实机验证**。

## 1. 目的与范围

### 1.1 要回答的问题

1. 现在为什么需要 `CanRxNode` / `CanTxNode` 这两个类，去掉它们会失去什么。
2. 去掉之后用什么替代，替代物如何覆盖现有与预期的设备形状。
3. 有哪些候选方案，各自的改动面、代价与适用条件。
4. 分几步实施，每步的验证判据与回滚方式。

### 1.2 在范围内

- `User/Service/can_bus.*`、`can_rx_node.*`、`can_tx_node.*`
- `User/Bsp/bsp_can.*` 的接口边界（不改硬件逻辑与恢复算法）
- Device 层接入方式（`dji_motor.*`、`device_cfg.*`）
- 相关测试、文档与仓库记忆

### 1.3 不在范围内

- BspCan 的寄存器操作、中断流程、Bus-Off 恢复算法
- CAN FD、扩展帧、非 8 字节经典帧
- 其他外设（UART / PWM / DWT）

## 2. 术语约定

| 术语 | 含义 |
| --- | --- |
| 帧 | 一条 CAN 报文：11 位标准 ID + 8 字节数据 |
| 设备 | 挂在总线上、按某种协议收发的一个业务对象（电机、IMU、上位机） |
| 登记 | 设备把自己的收发需求告知总线的一次动作 |
| 分发 | 把收到的帧按 ID 交给对应设备 |
| 调度 | 决定哪条帧在什么时候发出去 |
| 节点 | 现状中用于登记记账的堆对象（`CanRxNode` / `CanTxNode`），是本方案要移除的对象 |
| 冻结 | 系统进入运行态后禁止再登记与注销（`CanBus::frozen`） |

## 3. 现状基线（可核查）

### 3.1 文件与规模（实测行数）

| 文件 | 行数 | 职责 |
| --- | --- | --- |
| `User/Bsp/bsp_can.hpp` / `.cpp` | 267 / 721 | 硬件收发、Bus-Off 恢复、诊断；不识别任何"节点"概念 |
| `User/Service/can_bus.hpp` / `.cpp` | 142 / 234 | 接收分发、发送调度、回退缓冲 |
| `User/Service/can_rx_node.hpp` / `.cpp` | 101 / 68 | 按"帧类型 + ID"注册一个回调 |
| `User/Service/can_tx_node.hpp` / `.cpp` | 126 / 205 | 一条帧 + 槽位 + 数据 + 锁 + 上次发送时刻 |
| `User/Service/online_check.hpp` / `.cpp` | 98 / 131 | 每设备的在线状态节点（内部静态链表） |
| `User/Device/dji_motor.hpp` / `.cpp` | 218 / 397 | DJI 电机协议解析与编码 |
| `User/App/test/can/can3_device_test.cpp` | 275 | CAN3 双电机自检 |
| `User/App/test/can/can_recovery_test.cpp` | 227 | CAN1 Bus-Off 恢复测试 |

**重构主体 = `can_bus` + `can_rx_node` + `can_tx_node` = 876 行。**

### 3.2 数据流

接收：

```text
FDCAN 中断
  → BspCan::process_fifo0_isr()  → BspCan 接收缓冲（深度 16 帧）
  → can_rx_task（1 kHz）         → CanBus::rx_poll()（每轮上限 8 帧，can_bus.cpp:88）
  → 格式校验 → 遍历 rx_head 链表匹配 ID → 设备回调
  → 未命中                      → _rx_return_buffer → CanBus::receive()
```

发送：

```text
设备 fill_data()                → CanTxNode 槽位数据 + buffer_unsend 位图
  → can_tx_task（1 kHz）         → CanBus::tx_poll()（can_bus.cpp:124）
  → 判断 ready || due（can_bus.cpp:136-137）→ CanBus::send()
  → BspCan 发送缓冲（深度 16 帧） → TX-FIFO-EMPTY 中断 → 硬件 TX FIFO
```

### 3.3 关键常量与上限

| 项 | 值 | 出处 |
| --- | --- | --- |
| 发送兜底周期 | 5 ms | `can_bus.cpp:137` |
| 每轮接收分发上限 | 8 帧 | `can_bus.hpp:112` |
| 回退缓冲深度 | 8 帧 | `can_bus.hpp:113` |
| BspCan 收发缓冲深度 | 各 16 帧 | `bsp_can.hpp:230` |
| 节点总数上限 | 50（收发各自独立计数） | `can_rx_node.cpp:33`、`can_tx_node.cpp:117` |
| 帧内槽位数 | 1 / 2 / 4 / 8 | `can_tx_node.cpp` 构造校验 |
| 分发任务栈 / 优先级 | 512 words / +8 | `api_main.cpp:72` |
| 发送任务栈 / 优先级 | 512 words / +8 | `api_main.cpp:73` |
| 系统任务栈 / 优先级 | 256 words / +7 | `api_main.cpp:69` |

### 3.4 冻结语义

`CanBus::frozen` 初值 `false`（`can_bus.cpp:12`），由两个任务在进入循环前置位（`can_bus.cpp:179`、`can_bus.cpp:205`）。登记函数在 `frozen` 或 `sys_flag_running()` 时拒绝（`can_rx_node.cpp:16`、`can_tx_node.cpp:86`）。

结论：**所有登记必须发生在调度器启动之后、运行标志置位之前**，由 `all_init()` 中的 `device_init()` 完成（`api_main.cpp:48`）。

### 3.5 现存登记调用点（全部 4 处）

| 位置 | 动作 |
| --- | --- |
| `dji_motor.cpp:220` | 注册发送槽位（division = 4） |
| `dji_motor.cpp:232` | 注册接收节点（反馈 ID） |
| `dji_motor.cpp:149` / `161` / `236` | 注销接收节点 / 注销发送槽位 / 失败回滚 |
| `can_recovery_test.cpp:206` | 测试用接收节点 |

### 3.6 内存基线（实测，2026-10-03）

| 区域 | 占用 | 占比 |
| --- | --- | --- |
| DTCMRAM | 65120 B | 49.68% |
| RAM_D1 | 3616 B | 1.10% |
| FLASH | 121576 B | 11.59% |

## 4. 现状问题清单

| 编号 | 问题 | 证据 | 影响 | 严重度 |
| --- | --- | --- | --- | --- |
| P1 | 发送节拍是全局常量 | `can_bus.cpp:136-137` | 周期不等于"槽位填满"的设备被迫接受 5 ms 兜底 | 高 |
| P2 | 发送语义单一 | `tx_poll` 只有 `ready` / `due` 两个判据 | 无法表达"变了才发"，一次性指令会被重复发 | 中高 |
| P3 | 设备承担总线记账 | `dji_motor.cpp:220/232/149/161/236` | 设备必须知道 division、槽位号、注销顺序 | 中 |
| P4 | 设备需自建静态回调 | `dji_motor.hpp:93` + `dji_motor.cpp` 的 `_rx_callback` | 每个设备重复写"静态函数 + void* 转 this" | 中 |
| P5 | 在线判定分散 | `dji_motor.hpp:87` 的 `Online _online` | 每设备一份在线节点，判定标准不统一 | 中 |
| P6 | 总线记账数据公开 | `can_bus.hpp:105-109` | 外部可绕过接口改内部结构 | 中 |
| P7 | 与编码规范冲突 | 公开成员带 `_` 前缀（`can_bus.hpp:105/109`） | 规范一致性 | 低中 |
| P8 | 节点上限是全局值 | `can_rx_node.cpp:33`、`can_tx_node.cpp:117` | 三条总线共享 50 的上限，无法按总线规划 | 低 |
| P9 | 发送任务按启动快照决定存在性 | `can_bus.cpp:209-220` | 启动时无发送节点则任务自杀，运行期无法再建发送 | 低 |
| P10 | 查找为线性 | `can_bus.cpp:88` 起的双层遍历 | 设备数与 ID 数增长后为 O(设备数 × ID数) | 低 |

### P1 展开（最重要）

`can_bus.cpp:136-137` 的判据是"槽位全部填满，或距上次发送超过 5 ms"。这一个常量同时伤害两类需求方向相反的设备：

- **周期查询型传感器**：按 100 ms 设计，实际会被 5 ms 兜底以约 20 倍频率发送。
- **电机控制帧**：需要 1 ms 周期，无法通过该判据表达，只能靠"每次把所有槽位填满"绕过兜底。

这是功能性缺陷，不是风格问题，也是本方案的最优先动机。

## 5. 设计目标与硬约束

### 5.1 目标（按优先级）

1. **可读性**：设备只描述协议，不描述总线记账。
2. **可维护性**：新增设备形状不改 Service 层。
3. **扩展性**：覆盖已知四种形状与未预见形状。
4. **实时性不退化**：接收分发与发送调度仍在 1 kHz 任务中完成。

### 5.2 硬约束（不可违反）

- 禁止 `friend`，跨类访问一律走 public 接口。
- 私有成员 `_` 前缀；2 空格缩进；Allman 花括号。
- 不新增动态分配（现状已有 `new`，目标是减少而非增加）。
- 冻结语义保留。
- `User/App/test/can/test_recovery_host.py` 必须继续通过（它从 `bsp_can.cpp` 抽取函数体）。
- 本次改动不烧录、不提交。

## 6. 候选方案对比

### 方案 A：仅把发送周期下沉为节点属性

将"发送时机"与"周期"作为 `CanTxNode` 的属性，`tx_poll` 改为按节点判断。

- 改动面：`can_tx_node.hpp/cpp` + `can_bus.cpp`，约 40 行。
- 优点：改动最小；一次消除 P1、P2；调用点零变化（新增字段带默认值）。
- 缺点：P3~P8 全部保留。
- 适用：近期必须接传感器、暂无时间做主改造。

### 方案 B：设备表 + 帧表（本文主推）

删除两个节点类；总线持有**设备表**（设备指针 + 关心的 ID 表 + 最近收帧时刻）与**帧表**（帧内容 + 发送时机 + 周期 + 槽位占用）；设备实现一个接收接口，并在 `init()` 中登记一次。

- 改动面：新增 1 个头文件；重写 `can_bus.*`；删除 `can_rx_node.*`、`can_tx_node.*`；迁移 4 个调用点。
- 优点：一次消除 P1~P8；设备侧代码净减；概念数由 4（两种节点 + 总线 + 设备）降为 3（设备、帧、总线）。
- 缺点：中等改动量，必须实机回归。
- 适用：推荐。

### 方案 C：保留节点，仅做封装收口

把 `CanBus` 的公开成员收为私有，登记函数改为成员函数，节点类继续存在。

- 优点：风险最低，可作为分两步走的中间态。
- 缺点：只解决 P6 / P7 / P8；设备侧记账负担不变。

### 方案 D：总线反向回调设备

总线的发送调度不持有数据，每拍询问设备"这一拍要发什么"。

- 优点：设备对发送时机有完全控制，不需要槽位。
- 缺点：每拍对每个设备一次虚调用；同一帧由多设备共写时需额外协调；与"数据放在总线中、由中断直接取"的零拷贝路径相悖。

### 方案 E：描述表驱动（无虚函数）

用静态描述表 + 函数指针代替虚接口。

- 优点：无 vtable 开销；表为只读数据，可放 Flash。
- 缺点：函数指针仍需 `void*` 上下文与静态转换函数，**P4 的问题原样保留**；C++11 下描述表书写冗长。

### 6.6 对比

| 维度 | A | B | C | D | E |
| --- | --- | --- | --- | --- | --- |
| 消除 P1 / P2 | 是 | 是 | 否 | 是 | 是 |
| 消除 P3~P5 | 否 | 是 | 否 | 是 | 是 |
| 消除 P6 / P7 | 否 | 是 | 是 | 是 | 是 |
| 改动量 | 小 | 中 | 小 | 中 | 中 |
| 风险 | 低 | 中 | 低 | 中 | 中 |
| 设备侧代码量 | 不变 | 减少 | 不变 | 减少 | 减少 |
| 实机回归需求 | 低 | 必须 | 低 | 必须 | 必须 |

**推荐：方案 B。** 若需降低单次风险，先用方案 A 顶住 P1 / P2，再实施 B。

## 7. 主推方案细化（方案 B）

### 7.1 三个角色与职责边界

| 角色 | 知道 | 不知道 |
| --- | --- | --- |
| 设备 | 自己协议的解析与编码 | 总线上还有谁、自己排第几个、什么时候发 |
| 帧 | 数据、发送时机、周期、槽位占用 | 谁在写、内容语义 |
| 总线 | 有哪些设备与帧、这帧该给谁 | 帧内字节的语义 |

### 7.2 目标数据流

接收：中断 → BspCan → `can_rx_task` → 总线遍历设备表 → 匹配 ID → 设备接收接口 → 未命中进回退缓冲。

发送：设备写帧数据 → `can_tx_task` → 总线遍历帧表 → **帧自述"该不该发"** → `BspCan::send`。

与现状的关键差别：**"该不该发"由帧自己回答，不再由总线用一个全局常量统一判决。**

### 7.3 接口草案（仅签名，不含实现）

```cpp
/** @brief 一条帧的发送时机 */
enum class TxMode : uint8_t
{
  SLOT_FULL, ///< 槽位填齐才发，另配兜底周期（一条帧装多个设备的协议）
  PERIODIC,  ///< 到达周期就发，不看数据是否变化（电机控制帧、周期查询）
  ON_CHANGE, ///< 数据变化才发（一次性查询、参数写指令）
};

/** @brief 总线上的一个设备 */
class CanDevice
{
public:
  /** @brief 设备关心的接收帧 */
  struct RxId
  {
    uint32_t can_id;  ///< 帧 ID
    uint32_t id_type; ///< FDCAN_STANDARD_ID / FDCAN_EXTENDED_ID
  };

  /** @brief 总线分派一帧；返回 true 表示已消费 */
  virtual bool on_can_rx(const CanRxMsg &rx) = 0;
};

/** @brief 总线上的发送帧（可被多个设备共享槽位） */
class CanFrame
{
public:
  Status write(const uint8_t *data, size_t len, uint8_t slot = 0);
  Status claim_slot(uint8_t &slot_out);
  void   release_slot(uint8_t slot);
  bool   due(TickType_t now) const;
};

/** @brief 一条总线 */
class CanBus
{
public:
  /** @brief 设备登记描述 */
  struct DeviceDesc
  {
    const CanDevice::RxId *rx_ids;    ///< 接收 ID 表（nullptr 表示只发不收）
    uint8_t                rx_num;    ///< ID 个数
    uint32_t               tx_can_id; ///< 要发的帧 ID；0 表示只收不发
  };

  Status    attach(CanDevice &dev, const DeviceDesc &desc);
  Status    detach(CanDevice &dev);
  CanFrame *frame(uint32_t can_id, uint8_t division, TxMode mode, uint16_t period_ms);
  bool      online(const CanDevice &dev, uint32_t timeout_ms) const;

  Status send(uint32_t std_id, const uint8_t *data);
  Status receive(CanRxMsg *msg, uint32_t timeout_ms = 0);
  void   rx_poll();
  void   tx_poll();
};
```

### 7.4 四种设备形状的映射

| 设备形状 | 登记内容 |
| --- | --- |
| 只上报、从不发 | 只登记接收 ID，**不建帧** |
| 周期性查询 | 登记接收 ID + 一条 `PERIODIC` 帧（周期由设备给定） |
| 查一次后转入主动上报 | 登记接收 ID + 一条 `ON_CHANGE` 帧，开机发一次配置后不再发 |
| 电机（必须恒定频率发） | 登记接收 ID + 一条 `PERIODIC` 或 `SLOT_FULL` 帧，周期 1~5 ms |

**关键结论：查询型设备不需要新增任何类型。**"周期发查询帧"是帧的一种模式，"等不到应答算掉线"是在线判定，两者都已在模型中。

### 7.5 内存与性能估算（未实测）

| 项 | 估算 | 说明 |
| --- | --- | --- |
| 设备表 | 16 × 约 12 B ≈ 192 B | 静态数组，DTCM |
| 帧表 | 16 × 约 40 B ≈ 640 B | 含帧内容、位图、锁句柄 |
| 虚接口开销 | 每设备 8 B | vptr |
| 分配位置变化 | 堆 → 静态 | 可预测性提高 |
| 接收遍历 | O(设备数 × 每设备 ID 数) | 当前 2 设备，可忽略 |

### 7.6 与现有测试的关系

| 测试 | 影响 |
| --- | --- |
| `test_recovery_host.py` | 只抽取 `bsp_can.cpp`，本方案不改 BspCan 恢复算法 → **不受影响** |
| `can3_device_test.cpp` | 依赖 `CanBus::receive()` 读回退缓冲 → 保留该接口即不受影响 |
| `can_recovery_test.cpp:206` | 登记调用需同步改为新接口 |

## 8. 逐项决策的正反论证

### D1 槽位与共帧是否保留

- 支持保留：四轮底盘把 4 个电机放在同一条控制帧上，是该协议的固有形状；去掉后共帧无法表达。
- 反对保留：当前 `m2006` 与 `gm6020` 使用不同 ID、各自 `division = 4` 却只填槽位 0，**共帧场景目前未被使用**；去掉槽位可使 `CanTxNode`（205 行）整体消失。
- 影响面：`CanFrame` 的复杂度、`tx_poll` 的判据数量。
- 当前倾向：**保留**，但把槽位做成帧的可选能力（`division = 1` 时不启用位图逻辑）。

### D2 接口形态

| 选项 | 支持理由 | 反对理由 |
| --- | --- | --- |
| 虚函数接口 | 可读性最好，设备不再写 `void*` 转换 | 每设备 8 B vptr；禁用对象复制 |
| 函数指针 | 无 vptr | 仍要静态转换函数，P4 原样保留 |
| 描述表 | 数据可放 Flash | C++11 书写冗长，回调仍需上下文转换 |

当前倾向：**虚函数接口**。RAM 余量充足（DTCM 49.68%），可读性优先。

### D3 在线判定归属

- 支持收到总线：消除每设备的 `Online` 成员与 `refresh_task()` 调用；判定标准统一；`Online` 内部的静态链表与 `sys_task` 的 10 ms `update()` 可随之简化。
- 反对收到总线：总线多了一项与"分发"弱相关的职责；丢帧与协议层"有效帧"的差别可能被忽略（当前 `DjiMotor` 只在数据确实有效时刷新在线）。
- 折中：总线只在**设备接收接口返回 true**（即设备认可该帧有效）时刷新时间戳，语义与现状一致。
- 当前倾向：**收到总线**，并以"设备接口返回 true"为刷新条件。

### D4 回退缓冲去留

- 支持保留：`can3_device_test.cpp:117` 依赖它做联调观测；调试期可观察未被认领的帧。
- 反对保留：仅一个测试在用；消掉可减少一条缓冲与一次判断。
- 当前倾向：**保留**。

### D5 登记时机与冻结

- 必须保留冻结语义：设备表与帧表在运行期被任务遍历，运行期增删会引入并发修改。
- 需明确：`detach` 在冻结后返回 `NOT_SUPPORTED`（与现状 `unregist` 一致），因此成功初始化的设备必须存活至系统停止。

### D6 表容量与溢出策略

- 现状为 50（全局）。新方案建议按总线分配固定容量（例如每总线设备 16、帧 16）。
- 溢出时记为初始化错误（`FULL`），与现状一致，不做动态扩容。

### D7 是否保留直发接口 `CanBus::send()`

- 支持保留：`can_recovery_test.cpp` 直接发送零电流帧做故障注入，不经帧表。
- 当前倾向：**保留**。

## 9. 迁移计划

| 步骤 | 内容 | 触及文件 | 验证判据 | 回滚 |
| --- | --- | --- | --- | --- |
| **S0**（已完成） | BspCan 句柄与缓冲私有化 + `is_ready()` | `bsp_can.*`、`can_bus.cpp`、`can_rx_node.cpp` | 编译通过 + 主机测试 PASS | 见 §10 |
| **S1**（可选） | 发送时机与周期下沉为节点属性，`tx_poll` 按节点判断 | `can_tx_node.*`、`can_bus.cpp` | 编译通过；实机观察周期查询帧的实际间隔 | 撤销属性与判断分支 |
| **S2**（主体） | 设备表 + 帧表；删除两个节点类与自由注册函数；迁移 4 个调用点 | `can_bus.*`、新增头文件、`dji_motor.*`、`can_recovery_test.cpp` | 编译通过 + 主机测试 PASS + **实机跑 CAN3 双电机自检** | 该步骤应在独立分支上进行 |
| **S3**（收尾） | 文档与仓库记忆同步 | `Docs/spec/分层架构.md`、`Docs/guide/项目结构.md`、`User/App/test/can/README.md` | 人工核对 | 文档可直接回退 |

**S1 可跳过。** 其产出（发送时机枚举与周期字段）在 S2 中原样保留，因此先做不浪费。

调用点只有 4 处，因此**不需要新旧并存的过渡期**，一次切换比双轨更省事。

## 10. 已实施的先导改动（2026-10-03）

| 改动 | 位置 |
| --- | --- |
| `_hfdcan`、`_tx_message_buffer`、`_rx_message_buffer` 移入 private；`diagnostics` 保持 public | `bsp_can.hpp` |
| 新增 `bool is_ready() const`（句柄有效性） | `bsp_can.hpp` / `bsp_can.cpp` |
| `CanBus::rx_poll()` 由直接读接收缓冲改为调用 `BspCan::receive()` | `can_bus.cpp` |
| 登记校验改用 `is_ready()` | `can_rx_node.cpp` |

验证：`cmake --build build/Debug` 通过（FLASH 121576 B / 11.59%，DTCMRAM 65120 B / 49.68%）；`python3 User/App/test/can/test_recovery_host.py` 输出 PASS。**实机未验证。**

## 11. 风险清单与对策

| 风险 | 影响 | 对策 |
| --- | --- | --- |
| 冻结语义被破坏 | 运行期登记导致并发修改设备表 | 登记只保留一个入口，入口内统一判冻结 |
| 析构顺序错误 | 总线中残留悬空设备指针 | 设备析构必须先 `detach`；冻结后 `detach` 返回 `NOT_SUPPORTED` 并由上层断言 |
| 帧锁竞争 | 1 kHz 发送任务与设备写入冲突 | 每帧一把锁，临界区内只做 `memcpy` |
| 共帧槽位退化 | 四电机共帧无法表达 | 见 D1，保留槽位作为帧的可选能力 |
| 回退缓冲无人消费 | 未被认领的帧堆积并计 `FULL` | 保留 `receive()`；缓冲满时已有计数 |
| 主机测试脚本失效 | 回归失去保护 | 脚本只依赖 BspCan，本方案不动恢复算法 |
| 实机回归不足 | 线上问题 | S2 后必须跑 CAN3 双电机自检与 CAN1 恢复测试 |
| 在线判定语义变化 | 无效帧被误判为在线 | 以设备接收接口返回 true 为刷新条件（见 D3） |

## 12. 待决问题

1. D1：槽位与共帧是否保留？
2. D2：设备接口用虚函数、函数指针还是描述表？
3. D3：在线判定是否收到总线？
4. D4：回退缓冲保留还是删除？
5. D6：每条总线的设备表与帧表容量各取多少？
6. 是否先执行 S1（把发送周期下沉）再实施 S2？

## 13. 附录

### A. 相关文件

| 层 | 文件 |
| --- | --- |
| Bsp | `User/Bsp/bsp_can.hpp` / `.cpp`、`bsp_cfg.hpp` / `.cpp` |
| Service | `User/Service/can_bus.*`、`can_rx_node.*`、`can_tx_node.*`、`online_check.*`、`service_cfg.*`、`status.*` |
| Device | `User/Device/dji_motor.*`、`device_cfg.*` |
| App | `User/App/api_main.cpp`、`task/sys_task.cpp` |
| Test | `User/App/test/can/can3_device_test.cpp`、`can_recovery_test.cpp`、`test_recovery_host.py` |

### B. 现状引用速查

| 内容 | 位置 |
| --- | --- |
| 发送兜底 5 ms | `can_bus.cpp:137` |
| 接收分发循环 | `can_bus.cpp:88` |
| 冻结置位 | `can_bus.cpp:179`、`can_bus.cpp:205` |
| 冻结标志定义 | `can_bus.cpp:12` |
| 发送任务无节点退出 | `can_bus.cpp:209-220` |
| 总线公开成员 | `can_bus.hpp:105-109` |
| 节点上限 50 | `can_rx_node.cpp:33`、`can_tx_node.cpp:117` |
| 电机登记与注销 | `dji_motor.cpp:220`、`232`、`149`、`161`、`236` |
| 测试用登记 | `can_recovery_test.cpp:206` |
| 回退缓冲消费 | `can3_device_test.cpp:117` |

### C. 上游约定

- `Docs/spec/分层架构.md`：Device 为单设备驱动，Service 为跨设备公共设施；`can_bus`、`can_rx_node`、`can_tx_node` 均属 Service。
- `Docs/spec/编码规范.md`：禁止 `friend`；私有成员 `_` 前缀；2 空格缩进。
- `Docs/README.md`：不得把"编译通过"写成"已验证"；严禁烧录与自动提交。
