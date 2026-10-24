/**
 * @file corexy.hpp
 * @author Rh
 * @brief CoreXY 平面机构模块层：世界坐标运动学、双矩形限位、45°/90° 分段运动
 * @version 1.2
 * @date 2026-10-23
 *
 * @copyright Copyright (c) 2026
 *
 * @note 原理、时序图、用法示例与实测踩坑清单见 `corexy.md`；本文件只保留接口契约。
 * @note 限位、速度、到位窗口来自 Config；几何参数固定为本工程用的 ZDT 闭环步进
 *       （40 mm/圈、3200 脉冲/圈、65536 counts/圈），以 constexpr 收在私有区。
 */

#ifndef __MODULE_COREXY_HPP__
#define __MODULE_COREXY_HPP__

#include "status.hpp"

#include "FreeRTOS.h" // IWYU pragma: keep
#include "task.h"

#include <stddef.h>
#include <stdint.h>


/**
 * @brief CoreXY 平面机构（两台电机 + 双矩形限位 + 45°/90° 分段）
 *
 * @tparam Group 设备层“单串口多电机组”类型，构造只保存指针，不拥有设备。
 *
 * @note `Group` 是鸭子类型（模板编译期检查），只需满足 `_send_segment()` 用到的成员：
 *       位置清零 / 使能 / 停止 / 读位置 / 请求位置 / MMCL（清空、入队、发送）/ 快速位置入队。
 *       具体接口清单与两侧约束见 `corexy.md`。
 *
 * @note 换串口只需换 `Group` 类型；换非 ZDT 驱动器要同步改私有区的三个几何常量。
 */
template <typename Group>
class CorexyT
{
public:
  // ---------------- 常量 ----------------
  static constexpr uint8_t ZERO_CONFIRM   = 2U; ///< 连续多少次读数合格才算原点有效
  static constexpr uint8_t ARRIVE_CONFIRM = 2U; ///< 连续多少拍「两台读回的位置都正确且不变」才算到位

  /**
   * @brief 总线节拍（921600 下 13 B 帧 ≈ 0.14 ms，2 ms 留足余量）
   *
   * @note 一个常数覆盖五处间隔：帧与帧之间、发无应答命令前的静默、命令族之后
   *       先不读位置、发完 MMCL 到第一次读、相邻两次「读两台位置」。
   *       改波特率时只需要改这一个值。
   */
  static constexpr uint32_t BUS_GAP_MS = 2U;

  static constexpr uint32_t REPLY_TIMEOUT_MS = 30U;   ///< 单台位置应答的等待上限
  static constexpr uint32_t SEG_DONE_MS      = 3000U; ///< 段兜底：这么久还没判到位就按“已到位”处理（不是故障）
  static constexpr int32_t  STILL_COUNTS     = 10;    ///< 相邻两次读回相差 ≤ 该值（约 0.006 mm）视为已停稳
  static constexpr float    OVERRUN_MM       = 2.0f;  ///< 段内超出预期行程多少 mm 就停机
  static constexpr size_t   SEG_MAX          = 2U;    ///< 最多两段：45° 段 + 轴向段
  static constexpr float    ZERO_TOL_MM      = 0.50f; ///< 原点复核窗口 mm
  // ----------------
  // ---------------- 类型与配置 ----------------

  /** @brief 轴对齐矩形（mm） */
  struct Rect
  {
    /**
     * @brief 按序构造矩形
     * @param x0 左边界
     * @param y0 下边界
     * @param x1 右边界
     * @param y1 上边界
     */
    Rect(float x0 = 0.0f, float y0 = 0.0f, float x1 = 0.0f, float y1 = 0.0f) : x0(x0), y0(y0), x1(x1), y1(y1)
    {
    }

    float x0; ///< 左边界 mm
    float y0; ///< 下边界 mm
    float x1; ///< 右边界 mm
    float y1; ///< 上边界 mm
  };

  /** @brief 世界坐标点（mm） */
  struct Point
  {
    /**
     * @brief 按序构造坐标
     * @param x X 坐标 mm
     * @param y Y 坐标 mm
     */
    Point(float x = 0.0f, float y = 0.0f) : x(x), y(y)
    {
    }

    float x; ///< X 坐标 mm
    float y; ///< Y 坐标 mm
  };

  /** @brief 模块状态 */
  enum class State : uint8_t
  {
    Idle,   ///< 空闲，可接收新目标
    Moving, ///< 正在走当前分段
    Fault   ///< 越界 / 发送失败，已停机，需调用 stop() 清除
  };

  /**
   * @brief 分段下发模式
   *
   * @note 两种模式走的是驱动器不同的命令：
   *       - `Position`：每段发 `addr FD …`（13 B），方向、速度、加速度、脉冲一帧到底；
   *       - `Quick`：先用 `addr F1 …`（7 B）设一次速度/加速度/相对模式，
   *         之后每段只发 `addr FC 脉冲(4B, 带符号)`（7 B）。
   *       快速模式帧更短（两轴单帧 19 B 对 31 B），驱动器也少解析一套参数，
   *       代价是：
   *       1. 速度一变就要重发参数帧；
   *       2. `0xFC` 没有方向字节，**靠脉冲符号表方向**，换固件后需实机确认符号约定。
   */
  enum class MoveMode : uint8_t
  {
    Position, ///< 0xFD 完整位置帧（默认，最稳）
    Quick     ///< 0xF1 参数 + 0xFC 快速位置
  };

  /**
   * @brief 模块配置
   *
   * @note 参数顺序 = 字段顺序。电机组、地址与两块限位必须显式给出（由模块层实例配置负责），
   *       速度 / 加速度 / 到位窗口可省略。几何参数不在 Config 里：本模块只驱动 ZDT 闭环步进，
   *       两个比例由类内私有常量算出。
   *
   * @code{.cpp}
   *   // 最简：限位全给，速度用默认
   *   CorexyT<G> cx({&group, addr1, addr2, theory, safe});
   * @endcode
   *
   * @note **两块限位的语义区别**（很关键）：
   *       - `theory`：机械真实行程。只在**运动过程中**拿实测坐标去比，越界就判碰撞并停机；
   *       - `safe`：允许作为**运动目标**的范围。所有 `move_to` 都先过这一关。
   *       典型取法：`theory` = 机械能到的范围，`safe` 在四周各内收一点作为减速余量。
   *       `safe` 必须完全落在 `theory` 内，否则 `init()` 返回 BAD_ARG。
   */
  struct Config
  {
    /**
     * @brief 按序构造配置
     * @param group          电机组（不拥有）
     * @param addr1          电机 1 地址
     * @param addr2          电机 2 地址
     * @param theory         理论限位（机械行程，只做碰撞检测 mm）
     * @param safe           安全限位（允许运动的目标范围 mm）
     * @param rpm            基准速度 RPM（按每圈行程换算成 mm/s）
     * @param acc            加速度参数（0~255，越大加速越快，0 = 直接启动）
     * @param arrive_tol     到位窗口 mm
     * @param move_mode      分段下发模式（默认 Position）
     */
    Config(Group   *group,
           uint8_t  addr1,
           uint8_t  addr2,
           Rect     theory,
           Rect     safe,
           uint16_t rpm        = 300U,
           uint8_t  acc        = 230U,
           float    arrive_tol = 0.20f,
           MoveMode move_mode  = MoveMode::Position)

      : group(group),
        addr1(addr1),
        addr2(addr2),
        theory(theory),
        safe(safe),
        rpm(rpm),
        acc(acc),
        arrive_tol(arrive_tol),
        move_mode(move_mode)
    {
    }

    Group   *group;          ///< 电机组（不拥有）
    uint8_t  addr1;          ///< 电机 1 地址
    uint8_t  addr2;          ///< 电机 2 地址
    Rect     theory;         ///< 理论限位（机械行程）
    Rect     safe;           ///< 安全限位（允许运动的目标范围）
    uint16_t rpm;            ///< 基准速度 RPM
    uint8_t  acc;            ///< 加速度参数
    float    arrive_tol;     ///< 到位窗口 mm
    MoveMode move_mode;      ///< 分段下发模式
  };

  // ----------------
  // ---------------- 构造与析构 ----------------

  /**
   * @brief 构造函数
   * @param cfg 模块配置
   * @note 只做拷贝；参数校验在 init() 里做。
   */
  explicit CorexyT(const Config &cfg);

  // 状态机与读位置基准不可复制
  CorexyT(const CorexyT &)            = delete;
  CorexyT &operator=(const CorexyT &) = delete;
  CorexyT(CorexyT &&)                 = delete;
  CorexyT &operator=(CorexyT &&)      = delete;

  // ----------------
  // ---------------- 公有接口 ----------------

  /**
   * @brief 校验配置，复位状态机
   * @return OK / BAD_ARG（配置非法）/ NOT_INIT（设备组未 init）
   * @note 不发任何串口命令：清零编码器由 zero_here() 在任务上下文里做。
   */
  Status init();

  /**
   * @brief 两台电机使能 / 失能
   * @param on true = 使能，false = 失能
   * @return 第一条失败的命令结果（都成功则 OK）
   *
   * @note **必须先静默**：上一条 `0x36` 查询的应答可能还在回传，驱动器此刻收不进字节，
   *       直接 `send()` 会被整帧丢弃——实测就是“只有第一台失能了”。
   * @note 两帧之间、以及使能之后到开始读位置，都留 `BUS_GAP_MS`。
   */
  Status enable(bool on);

  /**
   * @brief 两台编码器清零，当前位置记为世界原点 (0,0)
   * @return OK=两条命令都已入发送缓冲；其他=发送失败
   *
   * @note 会先发一次广播停止，避免清零瞬间还有旧目标在跑。
   * @note 单向命令（无应答），清零后 `origin_verified()` 仍为 false，必须靠后续 `update()`
   *       读到的位置接近 (0,0) 才能自动放行运动。
   * @note 典型用法（上电）：`zero_here()` → 等 `origin_verified()` → 再 `move_to()`。
   */
  Status zero_here();

  /**
   * @brief 绝对移动到世界坐标 p（mm）
   * @param p 目标点
   * @return OK=已开始移动；BAD_ARG=目标超出安全限位（或处于 Fault）；BUSY=上一段还在走；
   *         NOT_INIT=未 init / 原点未复核；IO_ERROR=串口发送失败（已停机）
   *
   * @note 路径拆段规则：`dx`、`dy` 任一为 0 或 `|dx| == |dy|` 走单段；
   *       否则先走 45° 段（`d = min(|dx|,|dy|)`），再走轴向段到目标。
   *       例：`0,0 → 100,50` 先到 `50,50`，再到 `100,50`。
   * @note **每段都必须先确认到位**才发下一段：发完 MMCL 先静默 `BUS_GAP_MS`，
   *       然后依次读两台（读 1 → `BUS_GAP_MS` → 读 2）的 `0x36` 位置；
   *       两台读回的位置都落在本段目标的 `arrive_tol` 内、**而且相邻两次读回几乎不变**
   *       （≤ `STILL_COUNTS`，证明电机真的停住了），连续 `ARRIVE_CONFIRM` 拍成立才算到位；
   *       下一段同样走「2 ms → 读两台」的流程。
   *       不依赖驱动器的"到位帧"（该回报默认不开）。
   * @note **不设故障超时**：本段最晚等 `SEG_DONE_MS`，到点还没判到位就**按已经到位处理**
   *       （跳下一段 / 结束），不会进 `Fault`。跳下一段前会先停一次，避免命令叠在没跑完的运动上。
   *       真要中断请用 `stop()`。
   * @note 分段起点是当前**实测**坐标（位置反馈换算），不是上一次指令值。
   *
   * @code{.cpp}
   *   // (0,0) → (50,60)：先 45° 到 (50,50)（只动电机 1），到位后再走 +Y 10 mm
   *   if (corexy.move_to({50.0f, 60.0f}) != Status::OK)
   *   {
   *     // 失败：看 last_error()，常见 BAD_ARG = 超出安全限位
   *   }
   *   while (!corexy.arrived()) { corexy.update(); vTaskDelay(pdMS_TO_TICKS(10)); }
   * @endcode
   */
  Status move_to(Point p);

  /**
   * @brief 相对移动
   * @param d 相对位移（mm）
   */
  Status move_by(Point d);

  /**
   * @brief 立即停止并退出 Moving / Fault
   * @return 广播停止的发送结果
   *
   * @note 清故障的唯一入口：`Fault` 状态下 `move_to` 返回 BAD_ARG，必须先 `stop()`。
   * @note 停止是“逐台 + 广播”共三帧，帧间留 `BUS_GAP_MS`，共约 6 ms；
   *       整段包在总线锁里，不会被其他任务插帧。
   */
  Status stop();

  /**
   * @brief 设置基准速度与加速度
   * @param rpm 1~5000（每段实际转速会按脉冲数比例下调，保证同时到达）
   * @param acc 0~255（越大加速越快，0 = 直接启动，冲击大）
   *
   * @note 运行期可改，不影响已下发的段。
   */
  Status set_speed(uint16_t rpm, uint8_t acc);

  /**
   * @brief 切换分段下发模式（位置 / 快速位置）
   * @param mode 目标模式
   * @return OK / BUSY（正在走分段，改模式会让两台参数与指令不一致）/ NOT_INIT
   *
   * @note 切到 `Quick` 后，下一段自动补发一次 `0xF1` 参数帧再发 `0xFC`。
   */
  Status set_move_mode(MoveMode mode);

  /**
   * @brief 周期更新：收串口字节、读两台位置、推进状态机
   * @param timeout_ms 等待新字节的时间：0 = 非阻塞（当前一拍有多少处理多少），
   *                   N = 最多阻塞 N ms 等数据到（超时即继续推进状态机）
   *
   * @note 必须由**同一个任务**调用，且不能在中断里调：`_send_segment()` 里会先静默
   *       `BUS_GAP_MS` 再发帧，`_read_position()` 还会等应答（每台最多 `REPLY_TIMEOUT_MS`）。
   * @note 超时参数只作用于「等字节」这一步；状态机本身的推进不额外等。
   * @note 读位置是一问一答：先发请求、收到应答再读下一台；实际读周期受调用周期限制（≥ `BUS_GAP_MS`）。
   */
  void update(uint32_t timeout_ms = 0U);

  // ----------------
  // ---------------- 查询接口 ----------------

  ///< 当前状态
  State state() const
  {
    return _state;
  }

  ///< 最近一次错误（stop() 后清为 OK）
  Status last_error() const
  {
    return _last_error;
  }

  ///< 当前世界坐标（由位置反馈换算）
  Point position() const
  {
    return _pos;
  }

  ///< 最近一次移动的目标（未移动过时为原点）
  Point target() const
  {
    return _target;
  }

  ///< 是否已停止且没有待走的分段
  bool arrived() const
  {
    return _state == State::Idle && _seg_count == 0U;
  }

  ///< 原点是否已复核（清零后读数接近 (0,0)）；未复核时拒绝运动
  bool origin_verified() const
  {
    return _armed;
  }

  ///< 当前分段下发模式
  MoveMode move_mode() const
  {
    return _cfg.move_mode;
  }

  ///< 原始反馈 counts（0 = 电机 1，1 = 电机 2）；诊断用，没新应答就不变
  int32_t raw_counts(size_t axis) const
  {
    return (axis < 2U) ? _cur_counts[axis] : 0;
  }

  ///< 当前分段进度（0 起）与本次移动的总段数；只在诊断输出里用
  size_t segment_index() const
  {
    return _seg_index;
  }

  ///< 本次移动拆成了几段（0 = 没有待走分段）
  size_t segment_count() const
  {
    return _seg_count;
  }

  ///< 理论限位
  Rect limits_theory() const
  {
    return _cfg.theory;
  }

  ///< 安全限位
  Rect limits_safe() const
  {
    return _cfg.safe;
  }

  ///< 电机 1 / 2 最近一次 counts
  int32_t counts(size_t index) const
  {
    return (index < 2U) ? _cur_counts[index] : 0;
  }

  ///< 最近一次实际下发的增量（脉冲，带符号）；用于控制台核对
  int32_t last_delta(size_t index) const
  {
    return (index < 2U) ? _last_delta[index] : 0;
  }

  // ----------------
private:
  // ---------------- 内部常量 ----------------

  // 几何参数固定为 ZDT（张大头）闭环步进：2GT 2 mm × 20 齿 = 40 mm/圈；
  // 驱动器 16 细分 = 3200 脉冲/圈；编码器一圈 65536 counts。
  // 改驱动器型号时这三个值要跟着改，下面两个比例会自动重算。
  static constexpr float    MM_PER_REV     = 40.0f;
  static constexpr uint32_t PULSES_PER_REV = 3200U;
  static constexpr uint32_t COUNTS_PER_REV = 65536U;

  static constexpr float PULSES_PER_MM    = static_cast<float>(PULSES_PER_REV) / MM_PER_REV;      ///< 80 脉冲/mm
  static constexpr float COUNTS_PER_MM    = static_cast<float>(COUNTS_PER_REV) / MM_PER_REV;      ///< 1638.4 counts/mm
  static constexpr float COUNTS_PER_PULSE = static_cast<float>(COUNTS_PER_REV) / PULSES_PER_REV; ///< 20.48 counts/脉冲

  // ---------------- 私有实现 ----------------

  /**
   * @brief 把目标拆成 1~2 段（45° 段 + 轴向段）
   * @param goal 目标点
   * @note 起点取 `_pos`；调用前 `move_to` 已校验 goal 在安全限位内。
   *       安全限位是凸矩形，端点在内 ⇒ 中转点必然在内，无需再判。
   */
  void _build_segments(Point goal);

  /**
   * @brief 发送第 index 段（相对位置：方向字节 + 正幅值）
   * @param index 分段下标
   * @return 组帧/发送结果
   *
   * @note **一律走 MMCL 总帧**：两台都动时带两条子命令（帧长 31 B），只有一台动时带一条（18 B）。
   *       不拆成两条单机帧——同一帧里的两台才是同时起动。
   * @note `rpm_i = rpm·|Δp_i|/max|Δp|`，保证同时到达；`|Δp_i| == 0` 的电机不参与本段。
   * @note 组帧与发送由设备组的半双工总线锁串行化，其他任务插不进半帧。
   */
  Status _send_segment(size_t index);

  ///< 由 `_cur_counts` 换算 `_pos`
  void _refresh_position();

  ///< 进入 Fault 并广播停止
  void _fault(Status reason);

  ///< 总线静默：置 `_read_hold` 并真实延时（半双工发帧前的统一前置动作）
  void _quiet();

  /**
   * @brief 逐台 + 广播发送停止，帧间留 BUS_GAP_MS
   * @note RS485 半双工：连续 send() 只是入队，底层会把它连成一串，必须留真实间隔。
   */
  void _stop_motors();

  ///< 点是否在矩形内（含边界）
  static bool _inside(Point p, const Rect &r);

  ///< 单轴目标脉冲：axis 0 = 电机 1，axis 1 = 电机 2
  int32_t _pulses(Point p, size_t axis) const;

  /**
   * @brief 读一台的位置：发 `0x36` 请求 → 最多等 `REPLY_TIMEOUT_MS` 收应答
   * @param addr 电机地址
   * @param out  读到的新位置 counts
   * @return true=本次请求拿到了应答（旧应答在发请求前丢弃）
   *
   * @note 半双工总线上一次只允许一条查询在途，所以读两台必须**一次一台**、
   *       读到应答再发下一条；两条查询之间留 `BUS_GAP_MS`。
   */
  bool _read_position(uint8_t addr, int32_t &out);

  ///< 浮点合法（排除 NaN / Inf）
  static bool _finite(float v);

  // ----------------
  // ---------------- 成员变量 ----------------

  Config  _cfg;               ///< 配置（速度可在运行期改）
  bool    _inited    = false; ///< init() 是否成功
  bool    _armed     = false; ///< 原点已复核（未复核时移动返回 NOT_INIT）
  uint8_t _zero_ok   = 0U;    ///< 连续合格读数计数
  uint8_t _arrive_ok = 0U;    ///< 连续满足「两台位置都对 + 不再变化」的拍数（见 ARRIVE_CONFIRM）

  State  _state      = State::Idle; ///< 状态机
  Status _last_error = Status::OK;  ///< 最近一次错误

  Point _pos;    ///< 当前世界坐标（实测）
  Point _target; ///< 最终目标

  Point  _seg_end[SEG_MAX]; ///< 各段终点（构建时一次定下，执行中不再改）
  size_t _seg_count = 0;    ///< 本次移动的分段数
  size_t _seg_index = 0;    ///< 正在走的分段下标

  int32_t _cur_counts[2]  = {0, 0};         ///< 最近一次 0x36 读数
  int32_t _prev_counts[2] = {0, 0};         ///< 上一次成功读到的 0x36 读数（用来判“位置不再变化”）
  bool    _prev_ok[2]     = {false, false}; ///< _prev_counts 是否有效（每段开始时作废）
  int32_t _cmd_target[2]  = {0, 0};         ///< 本段到位目标（段起点实测 + 本段增量，counts）

  int32_t _seg_from[2]   = {0, 0}; ///< 本段开始时的实测 counts
  int32_t _seg_dir[2]    = {0, 0}; ///< 本段预期方向（+1 / -1 / 0 = 不动）
  int32_t _seg_travel[2] = {0, 0}; ///< 本段预期行程（|counts|）
  int32_t _last_delta[2] = {0, 0}; ///< 最近一次下发的增量（脉冲，带符号）

  TickType_t _seg_tick  = 0; ///< 本段命令发出的时刻（算 SEG_DONE_MS 兜底用）
  TickType_t _read_hold = 0; ///< 此之前不读位置：给驱动器留出处理命令 / 回完应答的时间

  // 快速位置模式：驱动器里当前生效的参数（避免每段都重发 0xF1）
  bool     _quick_valid[2] = {false, false}; ///< 该台的 0xF1 参数是否已下发
  uint16_t _quick_rpm[2]   = {0U, 0U};       ///< 已下发的转速
  uint8_t  _quick_acc[2]   = {0U, 0U};       ///< 已下发的加速度

  // ----------------
};

#endif // __MODULE_COREXY_HPP__
