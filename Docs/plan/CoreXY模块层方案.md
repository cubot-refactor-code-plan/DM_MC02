# CoreXY 模块层方案

> 适用：`User/Module/`（当前为空）、`User/Device/device_emmv5_group.*`、`User/App/task/`。
> 目标：把 USART2（RS485，地址 1、2）上两台 EmmV5 组合成一个世界坐标系下的平面机构：
> 给出电机转向 ↔ 世界方向的对应关系、0 位与最大矩形、内收的保护矩形，用串口输入坐标直接移动，
> 且运动只允许 45° / 90° 走向。

## 1. 结论

- 新增 `User/Module/corexy.{hpp,cpp}`：纯运动学 + 限位 + 分段规划 + 到位判定，**不读串口**；
- 新增 `User/Module/module_cfg.{hpp,cpp}`：全局实例 `corexy` 与 `module_init()`；
- 新增 `User/App/task/corexy_task.cpp`：串口控制台（读 `bsp_uart1` 命令行）+ 移动状态机；
- 比例取图的实测值（40 mm/圈 → 80 脉冲/mm、1638.4 counts/mm），限位取**理论值**：
  - 理论限位（机械矩形）：`(0,0)` ~ `(190,200)`，只用于碰撞检测；
  - 安全限位（允许运动）：`(1,1)` ~ `(189,199)`，可用行程 **188 × 198 mm**；
- **上电自动清零两台电机编码器，并把上电位置认定为世界坐标系原点 `(0,0)`**，不要求先回零；
- 运动只在「轴向」或「45° 对角」上执行：任意目标坐标先拆成一到两段（45° 段 + 轴向段）。

## 2. 已有标定数据（来自 `corexy_world_frame.png`）

### 2.1 世界系

原点在左下角，向右 `+X`，向上 `+Y`，单位 mm。
**理论矩形 `190 × 200 mm`**（用户给定）；图上实测四角为 `190.2 × 199.7 mm`，差 ≤ 0.3 mm 来自装配公差，
所以运行期只用理论值 + 安全内收，不用实测角点修正。

| 角 | (X, Y) mm | id1 counts | id2 counts |
| --- | --- | --- | --- |
| 角0（原点） | (0, 0) | **+3** | **-27** |
| 角1 | (190.2, 0) | -311591 | +311661 |
| 角2 | (190.3, 199.9) | -639277 | -15831 |
| 角3 | (0.1, 199.7) | -327341 | -327175 |

实测四角与理论矩形的差 ≤ 0.3 mm（装配公差 + 皮带张紧），属于「到不了边」的那部分，
因此运行期只用理论值 + 安全内收，不用实测角点修正。

### 2.2 换算

| 量 | 值 | 来源 |
| --- | --- | --- |
| 一圈行程 | 40 mm | 2GT 节距 2 mm × 20 齿 |
| 指令单位 | **80 脉冲/mm**（3200 脉冲/圈） | 位置命令 `0xFD` 的输入脉冲 |
| 反馈单位 | **1638.4 counts/mm**（65536 counts/圈） | `0x36` 实时位置应答 |
| 换算比 | **1 脉冲 = 20.48 counts** | 65536 / 3200 |

```text
X = -(c1 - c2) / (2 * 1638.4)      c = 0x36 读数（上电已清零，原点处 c1 = c2 = 0）
Y = -(c1 + c2) / (2 * 1638.4)

p1 = -80 * (X + Y)                 p = 0xFD 目标脉冲
p2 = +80 * (X - Y)
```

> 图上角0 的 `+3 / -27` 是**未清零**时的读数，只用于核对换算关系；
> 运行时上电即清零，原点读数恒为 0，公式里不再带偏移量。

> ⚠️ 两种单位必须分清：**发命令用脉冲（p1/p2），读位置用 counts（c1/c2）**，差 20.48 倍。
> 混用会表现为「走了 1/20 的距离」或「目标永远到不了」。

### 2.3 转向 ↔ 世界方向（用户要的对应关系）

| 世界方向 | ΔX | ΔY | 电机 1 | 电机 2 |
| --- | --- | --- | --- | --- |
| +X（右） | + | 0 | ↓ | ↑ |
| -X（左） | - | 0 | ↑ | ↓ |
| +Y（上） | 0 | + | ↓ | ↓ |
| -Y（下） | 0 | - | ↑ | ↑ |
| 右上 45° | + | + | ↓↓ | 不动 |
| 右下 45° | + | - | 不动 | ↑↑ |
| 左下 45° | - | - | ↑↑ | 不动 |
| 左上 45° | - | + | 不动 | ↓↓ |

结论：**45° 对角只动一台电机，轴向是两台同向/反向**。图里「向右推 id1↓ id2↑、向上推 id1↓ id2↓」与上表一致，可作方向符号的实测依据。

## 3. 分层与文件清单

| 文件 | 职责 |
| --- | --- |
| `User/Module/corexy.hpp` | `CorexyT<Group>`；`Config`；`Step` / `State` / `Rect` / `Point` 等值类型；公开接口 |
| `User/Module/corexy.cpp` | 正/逆运动学、限位校验、45°/90° 分段、MMCL 下发、到位轮询 |
| `User/Module/module_cfg.hpp` | `extern CorexyT<...> corexy;`、`void module_init(void);` |
| `User/Module/module_cfg.cpp` | 实例定义 + `module_init()`（`configASSERT(corexy.init() == Status::OK)`） |
| `User/Module/corexy.md` | 模块手册（换算表 + 限位模型 + 命令表 + 反模式） |
| `User/App/task/corexy_task.cpp` | 串口命令行解析 + 移动状态机 + 周期轮询 |
| `User/App/app_task.hpp` | `void corexy_task(void *);` 声明 |
| `User/App/api_main.cpp` | `module_init()` 与 `xTaskCreate(corexy_task, ...)` |

依赖方向：`App → Module → Device → Protocol`，Module 只持有 `DeviceEmmv5GroupT` 的引用，不碰 `bsp_uart`，不拼协议帧。

## 4. 模块接口

```cpp
template <typename Group>
class CorexyT
{
public:
  // 硬件决定，不作为配置项
  static constexpr float PULSES_PER_MM = 80.0f;     ///< 3200 脉冲/40 mm
  static constexpr float COUNTS_PER_MM = 1638.4f;   ///< 65536 counts/40 mm

  struct Rect  { float x0 = 0.0f; float y0 = 0.0f; float x1 = 0.0f; float y1 = 0.0f; };
  struct Point { float x = 0.0f; float y = 0.0f; };

  struct Config
  {
    Group  *group = nullptr;                 ///< emm_group
    uint8_t addr[2] = {1U, 2U};              ///< 电机 1 / 电机 2 地址
    Rect    theory = {0.0f, 0.0f, 190.0f, 200.0f}; ///< 理论限位（碰撞检测）
    Rect    safe   = {1.0f, 1.0f, 189.0f, 199.0f}; ///< 安全限位（允许运动，188 × 198）
    uint16_t rpm = 300;                      ///< 平动速度（mm/s ≈ rpm × 0.6667）
    uint8_t  acc = 230;                      ///< 加速度参数（越大越快）
    float    arrive_tol = 0.20f;             ///< 到位窗口 mm
  };

  enum class Step { X_Plus, X_Plus_Y_Plus, Y_Plus, X_Minus_Y_Plus,
                    X_Minus, X_Minus_Y_Minus, Y_Minus, X_Plus_Y_Minus }; // 0° ~ 315°，步进 45°

  enum class State { Idle, Moving, Fault };

  Status init();                       ///< 校验配置；只做赋值与校验，不发命令
  Status zero_here();                  ///< 两台编码器清零，当前位置即为 (0,0)

  Status home() { return zero_here(); } ///< 控制台 home：重新把当前位置当原点

  Status move_to(Point p);             ///< 绝对移动到 p（mm，已按安全限位校验）
  Status move_by(Point d);             ///< 相对移动
  Status step(Step s, float mm);       ///< 单步 8 方向（调试 / 方向核对用）
  Status stop();                       ///< 立即停止并清状态

  void   update();                     ///< 周期调用：轮询 0x36、判到位、越界停机
  State  state() const;
  Point  position() const;             ///< 当前世界坐标（由 0x36 读数换算）
  bool   arrived() const;
  Status last_error() const;

  // 查询
  Rect    limits_theory() const;       ///< (0,0) ~ (190,200)
  Rect    limits_safe() const;         ///< (1,1) ~ (189,199)
};
```

- `Config` 字段数 > 4，按编码规范第 6 条使用 `Config`，构造只收 `const Config &`；
- `init()` 里做 `BAD_ARG` 校验（`group` 非空、地址不同且非 0、`safe` 完全落在 `theory` 内、
  `x1 > x0` / `y1 > y0`、`arrive_tol > 0`），并确认 `group->is_ready()`；
- **上电清零不在 `init()` 里做**：`all_init()` 在调度器启动前执行，串口阻塞发送不可靠 →
  由 `corexy_task` 启动阶段调 `zero_here()`，清完再置 `Idle`；
- `zero_here()` 内部：先 `stop_all()`，再对地址 1、2 各发 `zero_position()`（间隔 10 ms），
  然后读一次 `0x36` 确认两台都接近 0（|c| < 200 counts ≈ 0.12 mm）。

## 5. 限位模型

```text
理论限位（0,0）~（190,200）     安全限位（1,1）~（189,199）
┌──────────────────────────┐ 190
│ ┌──────────────────────┐ │ 199
│ │  允许运动区 188×198   │ │
│ └──────────────────────┘ │ 1
└──────────────────────────┘ 0
  1                    189
```

- **理论限位** `Rect{0,0,190,200}`：机械行程，只用于碰撞检测，`move_to` 不按它放行；
- **安全限位** `Rect{1,1,189,199}`：`move_to` / `move_by` / `step` 的目标必须落在里面，否则 `BAD_ARG`
  并在串口打印「目标超出安全限位」+ 实际输入值；
- 运动过程中实测坐标越出**安全限位**（> 0.2 mm）→ 打印警告；
  越出**理论限位** → 立即 `group->stop_all()` 并置 `State::Fault`（防撞）；
- 矩形是凸集：45°/90° 分段的中转点由两个端点凸组合得到，端点都在安全限位内 ⇒ 中转点必然也在内，
  无需逐段再判（`assert` 级别的额外校验仍保留一次）。

## 6. 45° / 90° 运动规划

目标 `(x0,y0) → (x1,y1)`，`dx = x1-x0`，`dy = y1-y0`：

1. `dx == 0` 或 `dy == 0` 或 `|dx| == |dy|` → 单段；
2. 否则 `d = min(|dx|,|dy|)`，先走 **45° 段** 到 `(x0 + sign(dx)·d, y0 + sign(dy)·d)`，再走 **轴向段** 到目标（顺序固定：先对角后轴向，避免先贴边再斜插）；
3. 每段生成一次 MMCL 位置命令：`p1 = -80·(X+Y)`、`p2 = +80·(X-Y)`，`PositionMode::Absolute`；
4. **速度按脉冲数分摊**，保证两台同时完成：
   ```text
   maxΔp = max(|Δp1|, |Δp2|)
   rpm_i = clamp(rpm · |Δp_i| / maxΔp, 1, 5000)     // |Δp_i| == 0 的电机不参与本段 MMCL
   acc   两者相同（加速时间 ∝ rpm，比例关系自动保持）
   ```
5. 两段之间必须等第一段到位再发第二段（`State::Moving` 内部排队）。

段时长估算（用于超时）：`t ≈ maxΔp / (rpm·3200/60) + 加速时间`，例如 100 mm 走 300 rpm ≈ 0.5 s。

## 7. 到位判定与位置轮询

- 电机未开「到位帧回复」，**不用 `get_in_position()`**，改用 `0x36` 轮询；
- RS485 半双工：同一时刻只问一台，两台交替，周期 20 ms（单次往返 ≈ 0.96 ms，余量充足）；
- 到位判据：两台 `|c_i - c_target_i| ≤ arrive_tol·1638.4`；超时（估算时长 × 3 + 500 ms）→ `State::Fault` + `stop_all()`；
- `update()` 由 `corexy_task` 单任务调用，符合「串口唯一接收消费者 + 单任务串行」的既有约束；
- 目标只校验安全限位；实测越理论限位才停机（见 §5）。

## 8. 串口控制台（`bsp_uart1`，115200，同一任务内收发）

- 行缓冲 64 B，收 `\r\n` 或 `\n` 结束；`receive(..., 0, &n)` 非阻塞读，逐字符回显；
- 解析自己写（不用 `sscanf` 浮点），毫米按 **0.01 mm 整数**解析；打印同样用整数，避免 `%f` 依赖；
- 命令表：

| 命令 | 作用 | 示例 |
| --- | --- | --- |
| `?` | 帮助 + 限位 + 当前状态 | `?` |
| `pos` | 打印 X/Y（0.01 mm）、两台 counts、目标、状态 | `pos` |
| `home` | 重新清零编码器，当前位置记为 (0,0) | `home` |
| `m <x> <y>` | 绝对移动（mm，两位小数） | `m 50.00 20.00` |
| `j <dx> <dy>` | 相对移动 | `j -10 0` |
| `g <0..7> [mm]` | 单步 8 方向，默认 10 mm（核对方向表用） | `g 1 20` |
| `spd <rpm> <acc>` | 设置速度与加速度 | `spd 400 240` |
| `lim` | 打印理论限位与安全限位 | `lim` |
| `stop` | 立即停止（广播） | `stop` |
| `en <0\|1>` | 两台使能 / 失能 | `en 1` |

- 不做帧协议，纯 ASCII 文本；上位机协议（`Protocol/` 预留）以后再包。

## 9. 任务与并发

| 项目 | 取值 |
| --- | --- |
| 任务 | `corexy_task`，优先级 `tskIDLE_PRIORITY + 5`（与其它测试任务同级） |
| 周期 | 10 ms（控制台读 + 状态机 + 交替轮询） |
| 栈 | 512 words（与现有测试任务一致） |
| 串行约束 | 发送 / 轮询 / 状态机全部在这一个任务里；Module 不建任务、不加锁 |

## 10. 必须一起处理的冲突

1. `User/App/task/zdt_task.cpp` 现在每 500 ms 对地址 1、2 发 `enable(false)`，
   **会让 CoreXY 一使能就被失能** → 删除该任务（`api_main.cpp` 里的创建与 `app_task.hpp` 声明一起删），
   或在 `corexy_task` 上电后自己接管使能；本方案选前者。
2. `APP_TEST_EMMV5_POS_PROBE_ENABLED` 当前 = 1，探针会失能电机并抢占 0x36 应答 →
   改为 0（标定已完成，读数已入文档）。
3. `emm_group` 的地址表是 `{1, 2, 3}`，第 3 台是独立旋转轴，Corexy 配置只用 1、2，互不影响。
4. `device_cfg.cpp` 里注释写「地址 1、2」，实际表里还有 3 —— 顺手改正注释。

## 11. 实施步骤

| 步 | 内容 | 验收 |
| --- | --- | --- |
| 1 | 写 `corexy.hpp`：Config / 枚举 / 接口 / 常量；私有分区按 Outline region 规则 | 头文件单独编译通过 |
| 2 | 写 `corexy.cpp`：运动学、限位、分段、下发、`update()` 轮询与到位判定 | `Build` 任务通过 |
| 3 | `module_cfg.{hpp,cpp}` + `module_init()`；`api_main.cpp` 在 `device_init()` 之后调用 | 上电断言通过 |
| 4 | 写 `corexy_task.cpp`：启动即 `zero_here()`，再进控制台循环；`app_task.hpp` 声明；`all_init()` 创建任务 | `?`、`pos`、`lim` 有输出，`pos` 初始 ≈ (0.00, 0.00) |
| 5 | 删除 `zdt_task`、关掉 `APP_TEST_EMMV5_POS_PROBE_ENABLED`、修 `device_cfg` 注释 | 电机不再被周期失能 |
| 6 | 写 `User/Module/corexy.md`；更新 `Docs/spec/分层架构.md` 的 Module 行与 `Docs/plan/进度与待办.md` | 文档索引一致 |

## 12. 实机验证清单（用户烧录后执行）

1. 上电后 `pos`：两台 counts 应 ≈ 0，X/Y ≈ (0.00, 0.00)；
2. `en 1` → `g 0 10` … `g 7 10`：每个方向走 10 mm，与 §2.3 表核对（+X 右、+Y 上、45° 只动一台）；
3. `m 100 100`：观察是否先 45° 再轴向，且两段都停在目标附近；
4. 安全限位拒绝：`m 0.5 50`、`m 190 100`、`m 100 199.5` 应打印「超出安全限位」且电机不动；
5. 边界可达：`m 1 1`、`m 189 199` 应能走到（允许 0.2 mm 误差）；
6. `stop` 在移动中执行，两台立刻停；
7. 手推机器后 `home`，`pos` 重新变为 (0.00, 0.00)；再 `m 10 10` 正常。

## 13. 风险与待确认

| 项 | 说明 | 处理 |
| --- | --- | --- |
| 上电位置=原点 | 若上次停车不在左下角，上电后世界坐标整体偏移（不是真实位置） | 已知前提，由使用者保证停靠位置；`home` 可随时重定 |
| 上电清零的应答 | `zero_position` 是单向命令，无应答 | 清零后读一次 `0x36` 复核 \|c\| < 200 counts |
| 位置模式 `raF` | 只能发 1（绝对），发 2 会被当绝对运动 | 只用 `PositionMode::Absolute` |
| 两轴无插补同步 | 每条 MMCL 只保证各自到点，短段可能不完全同步 | 分段只走 45°/90°，影响可忽略 |
| 45°/90° 的解释 | 本方案 = 任意坐标可达，路径拆成 45° + 轴向两段 | **待确认**：是否要「只接受落在 45°/90° 网格上的目标」 |
| 速度默认值 | 300 rpm ≈ 200 mm/s，acc=230（0→300 rpm 约 0.47 s） | 实机按噪声/丢步调 |

## 14. 实施记录（与本计划的差异）

已按本计划落地，代码见 `User/Module/corexy.*`、`User/Module/module_cfg.*`、
`User/App/task/corexy_task.cpp`，说明见 `User/Module/corexy.md`。

实现阶段做了三处调整：

1. **几何参数进 `Config`**：`mm_per_rev` / `pulses_per_rev` / `counts_per_rev` 由实例配置给出，
   模块类内不再写死 40 mm / 3200 / 65536，便于换皮带轮与细分；
2. **`Config` 不给限位与几何默认值**：电机组、地址、两块矩形与三个几何参数必须显式传入，
   项目数值集中在 `module_cfg.cpp`；
3. **上电清零放在 `corexy_task` 启动阶段**（`all_init()` 处于调度器启动前，串口阻塞发送不可靠）；
4. **发位置命令前加 20 ms 总线静默期（`SEND_QUIET_MS`）**：`update()` 的轮询在到位判定之前，
   到位那一拍刚发出 `0x36` 查询，紧接 0 ms 就发 MMCL 会与驱动器的应答在半双工总线上撞车，
   驱动器正在发送、收不进字节，整帧被丢（现象：第一段能走、第二段完全不执行）。
   静默期内同时置 `_poll_hold`，保证不发任何查询；
5. **到位判定要连续确认（`ARRIVE_CONFIRM` = 2 拍）**，且理论限位包围盒的外扩容差改用
   `OVERRUN_MM`（3 mm）而不是 `arrive_tol`（0.2 mm）。原来判一拍就发下一段，减速段末尾
   读数已进窗口但驱动器未停稳，新的相对位置命令叠在未完成的运动上，两轴失步，
   贴近 `y=0` 的段直接被理论限位判 `BAD_ARG` 截停（实测 `m 1 1` 必现）。

当前状态：**编译通过**（FLASH 151512 B / DTCMRAM 67320 B），双子命令 MMCL 执行情况待实机确认。
验收判据见 `User/Module/corexy.md` §10。
