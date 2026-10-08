/**
 * @file device_r9ds.hpp
 * @author Rh
 * @brief R9DS 遥控接收机设备 —— 在 BspUart 之上把 SBUS 字节流解成通道值
 * @version 0.1
 * @date 2026-10-07
 *
 * @copyright Copyright (c) 2026
 *
 * @details 只做三件事：从现在 BspUart 的接收流缓冲区里切出 25 字节的 SBUS 帧、把那
 *          16 × 11 bit 拆成 16 个通道、再把通道映射成「摇杆 / 旋钮 / 开关」的语义。
 *
 * @note SBUS 协议（来源：R9DS 使用说明）：
 *       100000 bps、8 数据位、偶校验、2 停止位（每字节 12 位）、**反相 TTL**；
 *       一帧 25 字节 = 0x0F 帧头 + 22 字节通道数据（16 通道 × 11 bit，小端）+ flags + 0x00 帧尾。
 *       串口参数在 CubeMX 里配，见 bsp_cfg.cpp 的 bsp_uart5 与 bsp_init() 里的兜底断言；
 *       本类一个字节的串口配置都不改。接收机侧要拨到 SBUS 模式（蓝灯）。
 *
 * @note flags = frame[23]：bit0/bit1 是数字通道 17/18，bit2 = 本帧丢失，bit3 = 失控保护激活。
 *       这里把 bit2/bit3 用 frame_lost() / failsafe() 暴露出来 —— 遥控链路失控时怎么处置
 *       （停车 / 切手动）是上层的事，驱动不替它决定。
 *
 * @note 两个层次的量，别混用（1 是协议侧原始值，2 是遥控器侧语义值）：
 *
 *       1) raw_channels() —— 协议侧得到的遥控器对应量，上为200 中为1000 下为1800
 *       2) `Rc` / `rc()`　—— 遥控器侧，按实物定义的通道含义连续通道换算成-100 ~ +100
 *                            开关直接给挡位。写控制逻辑用这一套。
 *
 * @note 遥控器侧的通道（1基，就是遥控器上通道显示的那个编号）：
 *
 *       | SBUS 通道  | 物理部件    | 类型 | `Rc` 里的字段          |
 *       | --------- | ---------- | ---- | -------------------- |
 *       | CH1       | 右摇杆 上下  | 连续 | `right_vertical`     |
 *       | CH2       | 右摇杆 左右  | 连续 | `right_horizontal`   |
 *       | CH3       | 左摇杆 上下  | 连续 | `left_vertical`      |
 *       | CH4       | 左摇杆 左右  | 连续 | `left_horizontal`    |
 *       | CH5       | C 三挡开关   | 三挡 | `switch_c`           |
 *       | CH6       | 正面左旋钮   | 连续 | `knob_left`          |
 *       | CH7       | 背面左推杆   | 连续 | `slider_left`        |
 *       | CH8       | 正面右旋钮   | 连续 | `knob_right`         |
 *       | CH9       | 正面 B 开关  | 两挡 | `switch_b`           |
 *       | CH10      | 正面 A 开关  | 两挡 | `switch_a`           |
 *
 *       CH11 ~ CH16 遥控器侧未显示，但原始值依然能从 raw_channels() 读到。
 *
 * @note 开关不是连续值，按「离 200 / 1000 / 1800 哪个最近」判挡（两挡开关只比 200 / 1800）。
 *       物理开关是硬换向的，实测就是这三个值；用最近值判定比划阈值区间稳。
 * @note 整数截断自带一个小死区：raw 993 ~ 1007 都算 0（不足 1 个单位就舍掉，正好 15 个原始值宽）。
 *
 * @note 一个周期的标准用法（收帧必须在同一个任务里串行做，本类不加锁）：
 *
 *       r9ds.update();                            // 把这一拍串口里攒的字节全消化掉
 *       if (r9ds.is_online() && !r9ds.failsafe())
 *       {
 *         const DeviceR9ds::Rc &rc = r9ds.rc();
 *         float fwd   = rc.left_vertical * 0.01f;                  // 左摇杆上下：-1.0 ~ +1.0
 *         bool  a_up  = (rc.switch_a == DeviceR9ds::Switch::UP);   // A 开关在上挡
 *       }
 *
 * @warning 不是线程安全的：update() 只允许由一个任务调用；查询接口返回实时引用，
 *          读取期间不得并发执行 update()。
 * @warning 在线判定**不依赖 Online 节点**，由本类自己按 tick 算（见 is_online()）：所以它只在
 *          is_online() 被调用那一刻成立，调用频率只影响分辨率、不会漏判。
 */

#ifndef __DEVICE_R9DS_HPP__
#define __DEVICE_R9DS_HPP__

#include "bsp_uart.hpp" // 被解析的串口（配成 100000 / 9B 含偶校验 / 2 停止位）
#include "status.hpp"

#include <stdint.h>


/**
 * @brief R9DS 遥控接收机设备
 *
 * @note 不拥有串口、不创建任务、不创建 RTOS 资源、也不注册到 Online 链表：只解析调用方喂进来的
 *       时间片，在线判定自己按 tick 算。
 */
class DeviceR9ds
{
public:
  // ---------------- 帧结构常量 ----------------

  static constexpr uint32_t FRAME_SIZE   = 25U;     ///< 一帧字节数：帧头 + 22 字节通道数据 + flags + 帧尾
  static constexpr uint8_t  FRAME_HEADER = 0x0FU;   ///< 帧头标识
  static constexpr uint8_t  FRAME_FOOTER = 0x00U;   ///< 帧尾标识（扩展帧另有取值，本类只认它）
  static constexpr uint32_t CHANNEL_NUM  = 16U;     ///< 通道数
  static constexpr uint16_t CHANNEL_MASK = 0x07FFU; ///< 单个通道占 11 bit

  // ----------------
  // ---------------- 协议数值常量 ----------------

  static constexpr uint16_t CHANNEL_MID = 1000U; ///< 通道中位原始值
  static constexpr uint16_t CHANNEL_MIN = 200U;  ///< 一端满（遥控器的上挡）的原始值
  static constexpr uint16_t CHANNEL_MAX = 1800U; ///< 另一端满（遥控器的下挡）的原始值

  /**
   * @brief 连续通道换算：每多少个原始值对应 1 个语义单位
   *
   * @note 两端 200 / 1800 对应 +100 / -100 ⇒ (1800 - 200) / (100 + 100) = 8。
   */
  static constexpr int32_t RAW_PER_UNIT = 8;

  // ----------------
  // ---------------- 时基常量 ----------------

  /**
   * @brief 接收机静默多久就判离线 (ms)
   *
   * @note 帧周期约 14 ms（慢速）/ 7 ms（快速），取 100 ms ≈ 7 帧。
   * @note 计时在 is_online() 里按 tick 现算，不需要任何周期任务推进。
   */
  static constexpr uint16_t ONLINE_TIMEOUT_MS = 100U;

  /**
   * @brief 连续多少帧带「本帧丢失」就判离线
   *
   * @note 实测遥控器关机时 **lost 先置 1 并保持一段时间，fs 之后才置 1**，所以拿它当提前量：
   *       3 帧 × 13.5 ms ≈ 40 ms，比等 fs 早、也比上面的 100 ms 帧超时早。
   * @note 要求“连续”是为了抗单帧偶发丢包：正常遥控时偶发一帧 lost 不应该判离线。
   * @note 调这个数的依据看 diag().lost_streak：正常遥控时它应该一直是 0；若能看到涨到 1 ~ 2，
   *       说明链路本身在零星丢帧，把它调大（5 或 8）。
   */
  static constexpr uint32_t LOST_STREAK_TO_OFFLINE = 5U;

  // ----------------
  // ---------------- 开关挡位 ----------------

  /**
   * @brief 开关挡位
   *
   * @note 枚举值刻意取 +1 / 0 / -1：乘 100 就是和连续通道同一量纲的语义值
   *       （上挡 +100、中挡 0、下挡 -100）。
   */
  enum class Switch : int8_t
  {
    UP   = 1,  ///< 上挡（原始值 200；语义值 +100）
    MID  = 0,  ///< 中挡（原始值 1000；语义值 0）
    DOWN = -1, ///< 下挡（原始值 1800；语义值 -100）
  };

  // ----------------
  // ---------------- 遥控器通道 ----------------

  /**
   * @brief 遥控器通道语义值（遥控器侧视图，**写控制逻辑用这一套**）
   *
   * @note 字段顺序就是 CH1 ~ CH10 的顺序，通道含义见文件头那张表。
   * @note 连续通道范围 -100 ~ +100（上/一端满 +100、中位 0、下/另一端满 -100）；
   *       开关字段是挡位，不是数值。
   */
  struct Rc
  {
    int16_t right_vertical;   ///< CH1 右摇杆 上下，-100 ~ +100
    int16_t right_horizontal; ///< CH2 右摇杆 左右
    int16_t left_vertical;    ///< CH3 左摇杆 上下
    int16_t left_horizontal;  ///< CH4 左摇杆 左右
    Switch  switch_c;         ///< CH5 C 三挡开关：UP / MID / DOWN
    int16_t knob_left;        ///< CH6 正面左旋钮
    int16_t slider_left;      ///< CH7 背面左推杆
    int16_t knob_right;       ///< CH8 正面右旋钮
    Switch  switch_b;         ///< CH9 正面 B 开关：只会是 UP / DOWN
    Switch  switch_a;         ///< CH10 正面 A 开关：只会是 UP / DOWN
  };

  // ----------------
  // ---------------- 接收诊断 ----------------

  /**
   * @brief 接收诊断：前四个计数只增不减，最后一个是实时值
   *
   * @note 专门用来区分两种"收不到数据"：byte_cnt 为 0 说明一个字节都没进来（查接线 / 波特率 /
   *       反相），byte_cnt 在涨而 frame_cnt 不动说明字节进来了但切不出帧（查帧格式 / 反相）。
   */
  struct Diag
  {
    uint32_t byte_cnt;       ///< 从串口读到的总字节数
    uint32_t frame_cnt;      ///< 帧头帧尾都正确、已解析的帧数
    uint32_t bad_footer_cnt; ///< 帧头对上了但帧尾不是 0x00 的帧数
    uint32_t skip_byte_cnt;  ///< 为了找帧头而丢掉的字节数（正常应远小于 byte_cnt）
    uint32_t lost_streak;    ///< 【实时值】连续多少帧带「本帧丢失」，收到一帧不带就清 0
  };

  // ----------------
  // ---------------- 构造与析构 ----------------

  /** @brief 必须绑定一路已配成 SBUS 的串口，禁用默认构造 */
  DeviceR9ds() = delete;

  /** @brief 绑定串口（只赋值，不碰硬件、不读串口） */
  explicit DeviceR9ds(BspUart<128> &uart);

  /** @brief 默认析构（不持有任何需要释放的资源） */
  ~DeviceR9ds() = default;

  // 内部持有解析状态（半截帧、通道值等），复制会让两份状态各自推进
  DeviceR9ds(const DeviceR9ds &)            = delete;
  DeviceR9ds &operator=(const DeviceR9ds &) = delete;
  DeviceR9ds(DeviceR9ds &&)                 = delete;
  DeviceR9ds &operator=(DeviceR9ds &&)      = delete;

  // ----------------
  // ---------------- 生命周期 ----------------

  /**
   * @brief 复位解析状态与诊断计数（幂等）
   * @return OK=已复位（串口由 bsp_init() 负责，本函数没有会失败的事）
   *
   * @note 清掉半截帧、清零诊断计数，并把通道值置成一组安全默认（摇杆/旋钮全回中位 0、
   *       开关全回 UP），这样"还没收到过任何帧"时上层读到的也不是野值。
   */
  Status init(void);

  /**
   * @brief 把这一拍串口里攒的字节全部消化掉（非阻塞，可能一次解出多帧）
   *
   * @note 逐字节推进「找帧头 → 凑满 25 字节 → 验帧尾」的同步状态机。之所以不先确认
   *       "够 25 字节"再读：StreamBuffer 的 receive 只保证有多少给多少，一次要 24 字节
   *       可能只拿到一部分，反而会把帧切歪；逐字节读天然不受分片影响。
   * @note 没有数据时立刻返回，不会阻塞调用它的任务。
   */
  void update(void);

  // ----------------
  // ---------------- 查询接口 ----------------

  /**
   * @brief 遥控链路是否可用
   *
   * @note 三个条件同时成立才算在线，各管一段故障、互相兜底：
   *       ① 收到过帧，且最近一帧在 ONLINE_TIMEOUT_MS 内 —— 管“接收机自己静默了”（拔线 / 掉电）；
   *       ② 最近那一帧的 flags 没置「失控保护」—— 管“遥控器失联了”；
   *       ③ 连续带「本帧丢失」的帧数没到 LOST_STREAK_TO_OFFLINE —— 管“链路正在变差”，是②的提前量。
   * @note 为什么光看帧不够：**遥控器一关机，接收机不会安静下来**，它继续按帧周期发 SBUS，
   *       只把 flags 的位置起来，所以只看“有没有帧”就会一直显示在线。
   * @note 为什么还要③：实测关机时 **lost 先置 1 并保持一段时间，fs 之后才置 1**，所以拿连续的
   *       lost 当提前量，比等 fs 早几十毫秒报出失联；要求“连续”是为抗单帧偶发丢包。
   * @note 为什么①不能省：接收机被拔掉时一个字节都不会来，flags 和 lost_streak 都冻在最后一次
   *       的值上，**只有这条能发现**。想知道它还在不在发帧，看 diag().byte_cnt / frame_cnt 涨不涨。
   * @note ① 的计时由本类自己按 tick 现算（**不走 Online 节点**），所以不需要任何周期任务喂它，
   *       也就不用关心调用频率；代价是结果只在调用那一刻成立。
   */
  bool is_online(void) const;

  /** @brief 遥控器语义值（**控制逻辑用这个**），实时引用，读期间不得并发 update() */
  const Rc &rc(void) const;

  /** @brief 最近一帧的 16 个原始通道值（协议侧：上 200 / 中 1000 / 下 1800），下标 = 通道号 - 1 */
  const uint16_t *raw_channels(void) const;

  /** @brief 最近一帧的「本帧丢失」标志（frame[23] 的 bit2） */
  bool frame_lost(void) const;

  /** @brief 最近一帧的「失控保护激活」标志（frame[23] 的 bit3） */
  bool failsafe(void) const;

  /** @brief 诊断计数快照 */
  Diag diag(void) const;

  // ----------------
private:
  // ---------------- 帧解析 ----------------

  /** @brief 解析一帧已通过帧尾校验的数据：16 通道 → 语义值 → flags → 记录帧时刻 */
  void _parse_frame(const uint8_t *frame);

  // ----------------
  // ---------------- 原始值换算 ----------------

  /** @brief 原始值 → 遥控器语义值：raw 200 → +100、1000 → 0、1800 → -100（截断取整） */
  static int16_t _to_rc_value(uint16_t raw);

  // ----------------
  // ---------------- 开关判挡 ----------------

  /** @brief 两挡开关判挡：比离 200 / 1800 哪个近 */
  static Switch _to_switch_2pos(uint16_t raw);

  /** @brief 三挡开关判挡：比离 200 / 1000 / 1800 哪个近 */
  static Switch _to_switch_3pos(uint16_t raw);

  // ----------------
  // ---------------- 成员变量 ----------------

  BspUart<128> &_uart; ///< 被解析的串口（不拥有）

  Rc       _rc {};                ///< 最近一帧的遥控器语义值
  uint16_t _raw[CHANNEL_NUM] {};  ///< 最近一帧的原始通道值（协议侧）
  uint8_t  _frame[FRAME_SIZE] {}; ///< 正在拼的帧
  uint32_t _fill = 0U;            ///< 已拼到第几个字节，0 = 还没进入帧
  bool     _frame_lost = false;   ///< 最近一帧 flags 的 bit2：本帧丢失
  bool     _failsafe   = false;   ///< 最近一帧 flags 的 bit3：失控保护激活
  Diag     _diag {};              ///< 诊断计数（frame_cnt 顺带当“收到过帧吗”的标志用）
  uint32_t _last_frame_tick = 0U; ///< 最近一帧的 tick，is_online() 拿它算“接收机静默了多久”

  // ----------------
};

#endif // __DEVICE_R9DS_HPP__
