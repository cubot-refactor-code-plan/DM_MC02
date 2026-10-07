/**
 * @file dji_motor_group.hpp
 * @author Rh
 * @brief 一条 CAN 总线上的 DJI 电机组（M3508 / M2006 / GM6020）
 * @version 0.1
 * @date 2026-10-04
 *
 * @copyright Copyright (c) 2026
 *
 * @details 一个实例绑定一条 BspCan，按反馈标识符 0x201 ~ 0x20B 固定 11 个位置，
 *          **位置推出一切**（控制帧、帧内槽位、指令限幅、电调 ID、反馈标识符）：
 *
 *      | 位置 | 反馈标识符  | 允许型号               | 控制帧        | 帧内槽位 |
 *      | 0~3  | 0x201~0x204 | M3508 / M2006          | 0x200         | 0~3      |
 *      | 4~7  | 0x205~0x208 | M3508 / M2006 / GM6020 | 0x1FF / 0x1FE | 0~3      |
 *      | 8~10 | 0x209~0x20B | 仅 GM6020              | 0x2FE         | 0~2      |
 *
 *          位置定下来之后，下面四项全部是定值，init() 算一次、运行期不再变：
 *
 *          反馈标识符 = 0x201 + 位置        ← 位置对，收帧就不用人工对表
 *          帧内槽位   = 位置 % 4
 *          控制帧     = 位置 0~3 → 0x200；4~7 → 0x1FF（GM6020 改走 0x1FE）；8~10 → 0x2FE
 *          指令限幅   = 型号决定（3508 / 6020 为 ±16384，2006 为 ±10000）
 *
 * @note 位置 4~7 是混挂区：3508/2006 归 0x1FF，GM6020 归 0x1FE。两类混在一起时会多发
 *       一条控制帧，但**反馈标识符完全相同，接收侧不受影响**。
 *
 * @note 装车前要在 RoboMaster Assistant 里给每台电调设 ID。**电调 ID 与「位置」不是同一个数**，
 *       对照关系如下（位置 = 反馈标识符 - 0x201）：
 *
 *      | 位置 | 型号       | 电调 ID | 控制帧 | 帧内槽位 |
 *      | 0~3  | M3508/2006 | 1 ~ 4   | 0x200  | 位置     |
 *      | 4~7  | M3508/2006 | 5 ~ 8   | 0x1FF  | 位置 - 4 |
 *      | 4~7  | GM6020     | 1 ~ 4   | 0x1FE  | 位置 - 4 |
 *      | 8~10 | GM6020     | 5 ~ 7   | 0x2FE  | 位置 - 8 |
 *
 * @note 本类不分发接收、也不创建任务：调用方自己从 BspCan 取帧交给 update()，并在自己的
 *       任务里周期调 poll()。成员表在构造时一次定完，运行期不增删。
 *
 * @note 常量取自 Docs/sheet 手册：M3508 ±16384（减速比 3591/187）、M2006 ±10000（36）、
 *       GM6020 ±16384（1）。GM6020 只支持电流模式（0x1FE / 0x2FE），要求固件 ≥ 1.0.11.2
 *       且已在 RoboMaster Assistant 中打开电流环开关。
 *
 * @note 一个控制周期的标准用法（本类不自己收发，所以这几步由调用方按顺序做，且必须在同一个任务里）：
 *
 * @code{.cpp}
 *       CanRxMsg rx = {};
 *       for (uint32_t i = 0U; i < 8U && bsp_can3.receive(&rx, 0U) == Status::OK; ++i)
 *         (void)can3_dji_group.update(rx);          // 1) 收：按反馈标识符自动分派到位置
 *
 *       (void)can3_dji_group.set_output(4U, 5000);  // 2) 算：写原始控制量，只落发送缓冲
 *       (void)can3_dji_group.poll();                // 3) 发：把有成员的控制帧一次性发出
 *
 *       const MotorData &md = can3_dji_group.data(4U); // 4) 读：该位置最新的输出轴数据
 * @endcode
 */

#ifndef __DJI_MOTOR_GROUP_HPP__
#define __DJI_MOTOR_GROUP_HPP__

#include "bsp_can.hpp"
#include "motor_definition.hpp"
#include "online_check.hpp"
#include "status.hpp"

#include <stdint.h>


/** @brief 参与本组的电机型号（型号决定指令限幅、默认减速比，以及在位置 4~7 时走哪条控制帧） */
enum class DjiMotorModel : uint8_t
{
  M3508 = 0, ///< C620 + M3508：限幅 ±16384，默认减速比 3591/187，控制帧 0x200 / 0x1FF
  M2006,     ///< C610 + M2006：限幅 ±10000，默认减速比 36，控制帧 0x200 / 0x1FF
  GM6020,    ///< GM6020 电流模式：限幅 ±16384，默认减速比 1，只能放位置 4~10（0x1FE / 0x2FE）
};


/**
 * @brief 一条 CAN 总线上的 DJI 电机组
 *
 * @note 必须在初始化期 init()；未 init 时所有接口安全返回，不会触碰空句柄。
 * @note **本类不是线程安全的**：约定由同一个任务依次调 update() → set_output() → poll()，
 *       所以内部不加锁。data() 返回实时引用，读取期间不得有并发的 update()。
 */
class DjiMotorGroup
{
public:
  // ---------------- 常量 ----------------

  // 位置 = 反馈标识符 - 0x201，共 11 个；按允许的型号分成三段
  static constexpr uint32_t MOTOR_NUM     = 11U;    ///< 位置数量：反馈标识符 0x201 ~ 0x20B
  static constexpr uint32_t RX_ID_BASE    = 0x201U; ///< 位置 0 对应的反馈标识符
  static constexpr uint8_t  POS_MIX_BEGIN = 4U;     ///< 位置 ≥ 4 起是 3508/2006/GM6020 混挂区（0x205~0x208）
  static constexpr uint8_t  POS_6020_ONLY = 8U;     ///< 位置 ≥ 8 起只收 GM6020（0x209~0x20B）

  // 控制帧：下标与标识符的对应关系见 TX_ID，帧内槽位 0~3
  static constexpr uint32_t FRAME_NUM       = 4U; ///< 控制帧条数
  static constexpr uint32_t SLOTS_PER_FRAME = 4U; ///< 一条控制帧最多抱 4 台电机
  static constexpr uint32_t MOTOR_REG_BYTES = 2U; ///< 每台电机在控制帧里占 2 字节（int16，大端）
  static constexpr uint8_t  FRAME_0X200     = 0U; ///< 位置 0~3 的 3508/2006
  static constexpr uint8_t  FRAME_0X1FF     = 1U; ///< 位置 4~7 的 3508/2006
  static constexpr uint8_t  FRAME_0X1FE     = 2U; ///< 位置 4~7 的 GM6020（只有电流模式）
  static constexpr uint8_t  FRAME_0X2FE     = 3U; ///< 位置 8~10 的 GM6020

  /** @brief 控制帧下标 → 标识符：0~3 固定对应 0x200 / 0x1FF / 0x1FE / 0x2FE，不要改顺序（定义在 .cpp） */
  static const uint32_t TX_ID[FRAME_NUM];

  // 编码器
  static constexpr uint16_t ECD_FULL_RANGE = 8192U; ///< 转子一圈的编码器计数，手册给的是 0 ~ 8191

  /** @brief 一个型号的协议常量（MODEL_TRAITS 的元素类型） */
  struct ModelTraits
  {
    int16_t limit;         ///< 原始指令（转矩电流）绝对值上限
    float   default_ratio; ///< 手册减速比 = 转子转速 / 输出轴转速
  };

  /** @brief 支持的型号数量（= DjiMotorModel 的枚举个数）；加型号必须同步改，数组下标就是枚举值 */
  static constexpr uint32_t MODEL_NUM = 3U;

  /** @brief 各型号的协议常量：下标 = DjiMotorModel 的值（3508=0 / 2006=1 / 6020=2），定义在 .cpp */
  static const ModelTraits MODEL_TRAITS[MODEL_NUM];

  // ----------------
  // ---------------- 类型与配置 ----------------

  /**
   * @brief 一个位置上的电机描述；该位置没有电机就不传它（传 nullptr）
   *
   * @note 实例一般写成文件级静态常量（见 device_cfg.cpp）：
   *
   * @code{.cpp}
   *       static const DjiMotorGroup::Member m3508_1 {DjiMotorModel::M3508};             // 用型号默认减速比
   *       static const DjiMotorGroup::Member m2006_2 {DjiMotorModel::M2006, 36.0f, 0.0f}; // 显式给减速比与零位
   * @endcode
   *
   * @note ratio 传 0 表示「用该型号的手册默认值」，不用去查手册码数字；
   *       只有确实要改机械零位时才用得上 offset。
   */
  struct Member
  {
    /** @brief 型号必填；减速比 / 零位偏移不填就用型号默认值（3591/187、36、1）和 0 */
    Member(DjiMotorModel model, float ratio = 0.0f, float offset = 0.0f) : model(model), ratio(ratio), offset(offset)
    {
    }

    DjiMotorModel model;  ///< 位置 0~3：M3508/M2006；位置 4~7：三种均可；位置 8~10：必须 GM6020
    float         ratio;  ///< 转子到输出轴的减速比（转子转 ratio 圈 = 输出轴转 1 圈）；传 0 用型号默认
    float         offset; ///< 机械零位偏移，单位 rad；只影响输出的角度，不影响转速/加速度
  };

  /**
   * @brief 组配置：按反馈标识符 0x201 ~ 0x20B 的顺序传 11 位，无电机的位置传 nullptr
   *
   * @note 参数按位置排好后只留一个 `member[]`，下标即位置（0 → 0x201，10 → 0x20B）。
   *
   * @note 实参顺序就是位置顺序：第 1 个实参 = 位置 0 = 0x201，第 11 个 = 位置 10 = 0x20B。
   *       **没挂电机的位置也必须写 nullptr 占位**，少写一个后面所有位置都会错一位。
   *
   * @note 位置 4~7 既能放 3508/2006 也能放 GM6020，本类按 Member::model 自己决定走 0x1FF
   *       还是 0x1FE，所以混挂时不需要做任何额外处理，按位置填即可。
   */
  struct Config
  {
    /** @brief 按序构造配置（参数顺序 = 位置顺序） */
    Config(BspCan       *can    = nullptr,
           const Member *m0x201 = nullptr,
           const Member *m0x202 = nullptr,
           const Member *m0x203 = nullptr,
           const Member *m0x204 = nullptr,
           const Member *m0x205 = nullptr,
           const Member *m0x206 = nullptr,
           const Member *m0x207 = nullptr,
           const Member *m0x208 = nullptr,
           const Member *m0x209 = nullptr,
           const Member *m0x20A = nullptr,
           const Member *m0x20B = nullptr) :
      can(can), member {m0x201, m0x202, m0x203, m0x204, m0x205, m0x206, m0x207, m0x208, m0x209, m0x20A, m0x20B}
    {
    }

    BspCan       *can;               ///< 本组绑定的 CAN 总线（须已由 bsp_init() 初始化）
    const Member *member[MOTOR_NUM]; ///< 各位置的成员描述，下标 = 位置；nullptr = 该位没有电机
  };

  // ----------------
  // ---------------- 构造与析构 ----------------

  /** @brief 只保存配置，不碰硬件、不创建 RTOS 对象（也没有需要释放的资源） */
  explicit DjiMotorGroup(const Config &cfg);

  // 禁止复制/移动：内部持有 Online 链表节点，复制会让链表指向原对象
  DjiMotorGroup(const DjiMotorGroup &)            = delete;
  DjiMotorGroup &operator=(const DjiMotorGroup &) = delete;
  DjiMotorGroup(DjiMotorGroup &&)                 = delete;
  DjiMotorGroup &operator=(DjiMotorGroup &&)      = delete;

  // ----------------
  // ---------------- 公有接口 ----------------

  /**
   * @brief 校验成员表并把配置折算进各位置（幂等）
   * @return OK=可用；NOT_INIT=总线句柄为空；BAD_ARG=型号与位置不匹配
   *
   * @note 只算不碰硬件：先校验型号是否允许放在该位置，再把限幅 / 减速比 / 零位 / 控制帧与
   *       槽位这些「每帧都要用」的量提前算进 Slot，热路径就不必反复查型号表。
   * @note 只要构造时的成员表写错（型号放错位置），这里就会返回 BAD_ARG —— 成员表错误
   *       在 init() 阶段一次性暴露，运行期就不用再怀疑配置。
   */
  Status init(void);

  /**
   * @brief 传入一帧反馈
   * @param rx 待解析的 CAN 标准帧
   * @return OK=已更新；BAD_ARG=标识符越界、该位没有电机或帧格式不符；NOT_INIT=未 init
   *
   * @note 位置由标识符直接推出：位置 = 标识符 - 0x201，不用查表。
   * @note 把本周期从 BspCan 取到的帧全部喂进来即可：不属于本类的标识符只会被判
   *       BAD_ARG 丢掉，不会影响别的位置，所以调用方不需要自己做过滤。
   */
  Status update(const CanRxMsg &rx);

  /**
   * @brief 设置某个位置的原始控制量（超限即饱和，只写发送缓冲不发送）
   * @param index 位置 0 ~ 10
   * @param raw 有符号原始指令，按该位型号限幅（M3508/GM6020 ±16384，M2006 ±10000）
   * @return OK=已写入；BAD_ARG=位置越界或该位没有电机；NOT_INIT=未 init
   *
   * @note raw 是手册里的「转矩电流原始值」，不是占空比；正负号即转向。
   * @note 本函数不做斜坡：突给突撤都是立即生效，要平滑得由调用方自己给斜坡。
   */
  Status set_output(uint8_t index, int16_t raw);

  /**
   * @brief 把四条控制帧当前的缓冲值发出去（该帧没有成员时跳过，不算失败）
   * @return OK=已处理完；NOT_INIT=未 init
   *
   * @note 四条帧依次是 0x200 / 0x1FF / 0x1FE / 0x2FE，只在位置有成员时才发，
   *       一条空帧都不会占用总线。
   * @note 单次发送失败不上报，可查 BspCan::diagnostics 的 tx_buf_full / tx_dropped。
   * @note 本函数不看在线状态：电机掉线时照样发，由上层决定要不要先 set_output(idx, 0)。
   */
  Status poll(void);

  // ----------------
  // ---------------- 查询接口 ----------------

  /**
   * @brief 某个位置的输出轴运动学数据
   * @param index 位置 0 ~ 10
   * @return 数据引用；位置越界或该位没有电机时返回内部静态全零对象
   *
   * @note 取之前不必先查 has_motor()：无效位置拿到的是全零对象，直接算也不会出问题。
   * @note `radian_data.acceleration` **本类不填，恒为 0**：角加速度是跨帧量（相邻两帧角速度
   *       之差 / 时间差），直接差分噪声很大、滤波方式取决于用途，需要就由上层自己算。
   * @warning 返回实时引用而非快照，调用方须保证读取期间不并发执行 update()。
   */
  const MotorData &data(uint8_t index) const;

  /**
   * @brief 某个位置是否在线
   * @param index 位置 0 ~ 10
   * @return true=最近 30 ms 内收到过有效反馈；位置越界或该位没有电机时为 false
   *
   * @note 阈值 30 ms 由继承来的 Online 节点决定，计时靠 sys_task 每 10 ms 推一次，
   *       所以本函数不受本任务调用频率影响。
   */
  bool is_online(uint8_t index) const;

  /**
   * @brief 某个位置是否挂了电机
   * @param index 位置 0 ~ 10；越界返回 false
   *
   * @note 只看构造时给的成员表，与在线状态无关；是一道纯查表、无副作用的判断。
   */
  bool has_motor(uint8_t index) const;

  // ----------------
private:
  // ---------------- 私有类型 ----------------

  /**
   * @brief 一个位置上电机的运行时状态
   *
   * @note 分两类：init() 时算一次就固定下来的（limit / tx_frame / tx_slot / data.param），
   *       以及每收到一帧就被刷新的（data.radian_data / total_ecd / last_ecd / online）。
   *
   * @note total_ecd 必须排在最后：它要 8 字节对齐，放前面会让每个 Slot 多占 8 字节填充。
   *       改本结构体的字段顺序前先看一下 DTCMRAM 占用是不是涨了。
   */
  struct Slot
  {
    MotorData data {};                 ///< 输出轴运动学数据：param 在 init() 里折算；radian_data 每帧刷新，但 acceleration 恒为 0
    Online    online;                  ///< 在线检查节点（默认阈值 30 ms）
    int32_t   last_ecd       = 0;      ///< 上一帧的单圈编码器计数，跨零展开用
    uint16_t  limit          = 0U;     ///< 原始指令绝对值上限（由型号决定）
    uint8_t   tx_frame       = 0U;     ///< 控制帧下标 0 ~ 3，init() 里算好，热路径不再看型号
    uint8_t   tx_slot        = 0U;     ///< 帧内槽位 0 ~ 3
    bool      feedback_ready = false;  ///< 是否已收到过至少一帧有效反馈
    int64_t   total_ecd      = 0;      ///< 跨零展开后的累计转子编码器计数；必须排最后（8 字节对齐）
  };

  /** @brief 一个位置在控制帧里的落点 */
  struct FramePos
  {
    uint32_t frame; ///< 控制帧下标 0 ~ 3，取值见 FRAME_0Xxxx
    uint32_t slot;  ///< 帧内槽位 0 ~ 3
  };

  // ----------------
  // ---------------- 私有实现 ----------------

  /**
   * @brief 把位置换算成「控制帧 + 帧内槽位」
   * @param index 位置 0 ~ 10
   * @param model 该位置的型号（位置 4~7 靠它区分 0x1FF 与 0x1FE）
   *
   * @note 举例：位置 3 → (FRAME_0X200, 槽位 3)；位置 5 的 M3508 → (FRAME_0X1FF, 槽位 1)；
   *       位置 5 的 GM6020 → (FRAME_0X1FE, 槽位 1)；位置 8 → (FRAME_0X2FE, 槽位 0)。
   */
  static FramePos frame_position(uint8_t index, DjiMotorModel model)
  {
    if (index < POS_MIX_BEGIN)
    {
      return {FRAME_0X200, index}; // 位置 0~3 → 0x200 的槽 0~3，槽位与位置同值
    }
    if (index < POS_6020_ONLY)
    {
      // 4~7 是混挂区：3508/2006 走 0x1FF，GM6020 走 0x1FE，槽位公式相同
      const uint8_t frame = (model == DjiMotorModel::GM6020) ? FRAME_0X1FE : FRAME_0X1FF;
      return {frame, static_cast<uint32_t>(index - POS_MIX_BEGIN)};
    }
    return {FRAME_0X2FE, static_cast<uint32_t>(index - POS_6020_ONLY)}; // 位置 8~10 → 0x2FE 的槽 0~2
  }

  /** @brief 该位置是否允许放这种型号（0~3 只收 3508/2006，4~7 三种均可，8~10 只收 6020） */
  static bool is_model_allowed(uint8_t index, DjiMotorModel model)
  {
    if (index < POS_MIX_BEGIN)
    {
      // 0x201 ~ 0x204 与控制帧 0x200 的槽位一一对应，只放 3508/2006
      return (model == DjiMotorModel::M3508) || (model == DjiMotorModel::M2006);
    }
    if (index < POS_6020_ONLY)
    {
      return true; // 0x205 ~ 0x208 是混挂区：3508/2006 走 0x1FF，GM6020 走 0x1FE
    }
    return (model == DjiMotorModel::GM6020); // 0x209 ~ 0x20B 只有 GM6020 占控制帧 0x2FE
  }

  /** @brief 取大端 16 位（手册：反馈帧的多字节字段一律高字节在前） */
  static uint16_t be_u16(const uint8_t *bytes)
  {
    return static_cast<uint16_t>((static_cast<uint16_t>(bytes[0]) << 8) | bytes[1]);
  }

  /** @brief 把 16 位按大端写进目标两个字节（be_u16() 的反向操作） */
  static void put_be16(uint8_t *dst, int16_t value)
  {
    const uint16_t bits = static_cast<uint16_t>(value);
    dst[0]              = static_cast<uint8_t>(bits >> 8);
    dst[1]              = static_cast<uint8_t>(bits);
  }

  /**
   * @brief 把单圈编码器计数跨零展开成累计计数
   * @param total 上一次的累计计数；last 上一次的单圈计数；now 本帧的单圈计数
   * @param range 一圈的计数总数
   * @return 本帧的累计计数
   *
   * @note 判断依据：相邻两帧的转子位移必然小于半圈。差值超过半圈即认为跨过零点，
   *       按反方向补/减一整圈。
   * @note 举例（range = 8192，half = 4096）：
   *
   *       正转跨零：last=8000，now=200 → delta = -7800 < -4096 → delta += 8192 = 392 ✓
   *       反转跨零：last=200，now=8000 → delta = +7800 > +4096 → delta -= 8192 = -392 ✓
   *       正常前进：last=1000，now=1100 → delta = 100（在半圈内，不动）✓
   */
  static int64_t unwrap_ecd(int64_t total, int32_t last, int32_t now, uint16_t range)
  {
    const int32_t half  = static_cast<int32_t>(range / 2U);
    int32_t       delta = now - last;

    if (delta > half)
    {
      delta -= range;
    }
    else if (delta < -half)
    {
      delta += range;
    }
    return total + delta;
  }

  /**
   * @brief 解析一帧已校验过的反馈，刷新该位置的运动学数据
   * @param slot 目标位置的运行时状态
   * @param data 反馈帧的 8 字节数据域
   * @return OK=已刷新；BAD_ARG=编码器值越界
   *
   * @note 只做纯计算与赋值：不校验帧格式、不查成员表（那些在 update() 里做完了）。
   * @note 字节布局与各处换算关系见 .cpp 里本函数的注释。
   */
  Status _parse_feedback(Slot &slot, const uint8_t *data);

  // ----------------
  // ---------------- 成员变量 ----------------

  Config  _cfg;                         ///< 成员表与总线，构造时整份拷入，运行期只读
  Slot    _slot[MOTOR_NUM];             ///< 各位置的运行时状态，下标 = 位置
  uint8_t _frame_data[FRAME_NUM][8] {}; ///< 各控制帧的 8 字节发送缓冲，下标 0~3 = 0x200 / 0x1FF / 0x1FE / 0x2FE
  bool    _frame_active[FRAME_NUM] {};  ///< 各控制帧是否挂了成员；没挂就整条不发
  bool    _inited = false;              ///< init() 是否跑过；未 init 时接口一律安全返回

  // ----------------
};

#endif // __DJI_MOTOR_GROUP_HPP__
