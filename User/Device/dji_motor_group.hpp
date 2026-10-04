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
 * @note 位置 4~7 是混挂区：3508/2006 归 0x1FF，GM6020 归 0x1FE。两类混在一起时会多发
 *       一条控制帧，但**反馈标识符完全相同，接收侧不受影响**。
 *
 * @note 本类不分发接收、也不创建任务：调用方自己从 BspCan 取帧交给 update()，并在自己的
 *       任务里周期调 poll()。成员表在构造时一次定完，运行期不增删。
 *
 * @note 常量取自 Docs/sheet 手册：M3508 ±16384（减速比 3591/187）、M2006 ±10000（36）、
 *       GM6020 ±16384（1）。GM6020 只支持电流模式（0x1FE / 0x2FE），要求固件 ≥ 1.0.11.2
 *       且已在 RoboMaster Assistant 中打开电流环开关。
 */

#ifndef __DJI_MOTOR_GROUP_HPP__
#define __DJI_MOTOR_GROUP_HPP__

#include "bsp_can.hpp"
#include "motor_definition.hpp"
#include "online_check.hpp"
#include "status.hpp"

#include <stdint.h>


/** @brief 参与本组的电机型号 */
enum class DjiMotorModel : uint8_t
{
  M3508 = 0, ///< C620 + M3508，指令限幅 ±16384
  M2006,     ///< C610 + M2006，指令限幅 ±10000
  GM6020,    ///< GM6020 电流模式，指令限幅 ±16384
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
  // ---------------- 类型与配置 ----------------

  static constexpr uint32_t MOTOR_NUM  = 11U;    ///< 位置数量：反馈标识符 0x201 ~ 0x20B
  static constexpr uint32_t RX_ID_BASE = 0x201U; ///< 位置 0 对应的反馈标识符
  static constexpr uint32_t FRAME_NUM  = 4U;     ///< 控制帧条数，下标对应 0x200 / 0x1FF / 0x1FE / 0x2FE

  /** @brief 一个位置上的电机描述；该位置没有电机就不传它（传 nullptr） */
  struct Member
  {
    /** @brief 型号必填；减速比 / 零位偏移不填就用型号默认值（3591/187、36、1）和 0 */
    Member(DjiMotorModel model, float ratio = 0.0f, float offset = 0.0f) : model(model), ratio(ratio), offset(offset)
    {
    }

    DjiMotorModel model;  ///< 位置 0~3：M3508/M2006；位置 4~7：三种均可；位置 8~10：必须 GM6020
    float         ratio;  ///< 转子到输出轴的减速比；传 0 用型号默认（3591/187、36、1）
    float         offset; ///< 机械零位偏移，单位 rad
  };

  /**
   * @brief 组配置：按反馈标识符 0x201 ~ 0x20B 的顺序传 11 位，无电机的位置传 nullptr
   *
   * @note 参数按位置排好后只留一个 `member[]`，下标即位置（0 → 0x201，10 → 0x20B）。
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
   */
  Status init(void);

  /**
   * @brief 喂入一帧反馈
   * @param rx 待解析的 CAN 标准帧
   * @return OK=已更新；BAD_ARG=标识符越界、该位没有电机或帧格式不符；NOT_INIT=未 init
   */
  Status update(const CanRxMsg &rx);

  /**
   * @brief 设置某个位置的原始控制量（超限即饱和，只写发送缓冲不发送）
   * @param index 位置 0 ~ 10
   * @param raw 有符号原始指令，按该位型号限幅（M3508/GM6020 ±16384，M2006 ±10000）
   * @return OK=已写入；BAD_ARG=位置越界或该位没有电机；NOT_INIT=未 init
   */
  Status set_output(uint8_t index, int16_t raw);

  /**
   * @brief 把四条控制帧当前的缓冲值发出去（该帧没有成员时跳过，不算失败）
   * @return OK=已处理完；NOT_INIT=未 init
   * @note 单次发送失败不上报，可查 BspCan::diagnostics 的 tx_buf_full / tx_dropped。
   */
  Status poll(void);

  // ----------------
  // ---------------- 查询接口 ----------------

  /**
   * @brief 某个位置的输出轴运动学数据
   * @param index 位置 0 ~ 10
   * @return 数据引用；位置越界或该位没有电机时返回内部静态全零对象
   * @warning 返回实时引用而非快照，调用方须保证读取期间不并发执行 update()。
   */
  const MotorData &data(uint8_t index) const;

  /**
   * @brief 某个位置是否在线
   * @param index 位置 0 ~ 10
   * @return true=最近 30 ms 内收到过有效反馈；位置越界或该位没有电机时为 false
   */
  bool is_online(uint8_t index) const;

  /**
   * @brief 某个位置是否挂了电机
   * @param index 位置 0 ~ 10；越界返回 false
   */
  bool has_motor(uint8_t index) const;

  // ----------------
private:
  // ---------------- 私有类型 ----------------

  /**
   * @brief 一个位置上电机的运行时状态
   *
   * @note total_ecd 必须排在最后：它要 8 字节对齐，放前面会让每个 Slot 多占 8 字节填充。
   */
  struct Slot
  {
    MotorData  data {};                 ///< 输出轴运动学数据：param 在 init() 里折算，radian_data 每帧刷新
    Online     online;                  ///< 在线检查节点（默认阈值 30 ms）
    float      last_velocity = 0.0f;    ///< 上一帧的输出轴角速度，算角加速度用
    TickType_t last_tick     = 0;       ///< 上一帧的 tick，算角加速度用
    int32_t    last_ecd      = 0;       ///< 上一帧的单圈编码器计数
    uint16_t   limit         = 0U;      ///< 原始指令绝对值上限（由型号决定）
    uint8_t    tx_frame      = 0U;      ///< 控制帧下标 0 ~ 3，init() 里算好，热路径不再看型号
    uint8_t    tx_slot       = 0U;      ///< 帧内槽位 0 ~ 3
    bool       feedback_ready = false;  ///< 是否已收到过至少一帧有效反馈
    int64_t    total_ecd      = 0;      ///< 跨零展开后的累计转子编码器计数
  };

  /** @brief 一个型号的协议常量 */
  struct ModelTraits
  {
    int16_t limit;         ///< 原始指令（转矩电流）绝对值上限
    float   default_ratio; ///< 手册减速比 = 转子转速 / 输出轴转速
  };

  /** @brief 一个位置在控制帧里的落点 */
  struct FramePos
  {
    uint32_t frame; ///< 控制帧下标 0 ~ 3（对应 0x200 / 0x1FF / 0x1FE / 0x2FE）
    uint32_t slot;  ///< 帧内槽位 0 ~ 3
  };

  // ----------------
  // ---------------- 私有常量 ----------------

  static constexpr uint16_t ECD_FULL_RANGE = 8192U; ///< 转子一圈的编码器计数，手册给的是 0 ~ 8191

  // ----------------
  // ---------------- 私有实现 ----------------

  /** @brief 控制帧下标 → 标识符 */
  static uint32_t tx_id(uint32_t frame)
  {
    static const uint32_t ID[FRAME_NUM] = {0x200U, 0x1FFU, 0x1FEU, 0x2FEU};
    return ID[frame];
  }

  /** @brief 型号 → 该型号的协议常量（取值均来自 Docs/sheet 下的手册） */
  static ModelTraits model_traits(DjiMotorModel model)
  {
    static const ModelTraits TRAITS[] = {
      /* M3508  */ {16384, 3591.0f / 187.0f},
      /* M2006  */ {10000, 36.0f},
      /* GM6020 */ {16384, 1.0f},
    };
    return TRAITS[static_cast<uint8_t>(model)];
  }

  /**
   * @brief 把位置换算成「控制帧 + 帧内槽位」
   * @param index 位置 0 ~ 10
   * @param model 该位置的型号（位置 4~7 靠它区分 0x1FF 与 0x1FE）
   */
  static FramePos frame_position(uint8_t index, DjiMotorModel model)
  {
    if (index <= 3U)
    {
      return {0U, index};
    }
    if (index <= 7U)
    {
      // 4~7 是混挂区：3508/2006 走 0x1FF，GM6020 走 0x1FE，槽位公式相同
      return {(model == DjiMotorModel::GM6020) ? 2U : 1U, static_cast<uint32_t>(index - 4U)};
    }
    return {3U, static_cast<uint32_t>(index - 8U)};
  }

  /** @brief 该位置是否允许放这种型号（0~3 只收 3508/2006，4~7 三种均可，8~10 只收 6020） */
  static bool is_model_allowed(uint8_t index, DjiMotorModel model)
  {
    if (index < 4U)
    {
      return (model == DjiMotorModel::M3508) || (model == DjiMotorModel::M2006);
    }
    if (index < 8U)
    {
      return true;
    }
    return (model == DjiMotorModel::GM6020);
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
   * @note 相邻两帧的转子位移必然小于半圈，差值超过半圈即认为跨过零点，按反方向补一圈。
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
   */
  Status _parse_feedback(Slot &slot, const uint8_t *data);

  // ----------------
  // ---------------- 成员变量 ----------------

  Config  _cfg;                         ///< 成员表与总线，构造时整份拷入
  Slot    _slot[MOTOR_NUM];             ///< 各位置的运行时状态
  uint8_t _frame_data[FRAME_NUM][8] {}; ///< 各控制帧的发送缓冲
  bool    _frame_active[FRAME_NUM] {};  ///< 各控制帧是否挂了成员；没挂就整条不发
  bool    _inited = false;              ///< init() 是否跑过；未 init 时接口一律安全返回

  // ----------------
};

#endif // __DJI_MOTOR_GROUP_HPP__
