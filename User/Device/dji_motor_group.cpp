#include "dji_motor_group.hpp"

#include <math.h>


// ---------------- 静态常量定义 ----------------

// 常量都声明在 .hpp 的「常量」分区里，这里只放需要存储的定义
// （C++11 里类内的 static const 数组只能声明，所以值写在这里）

/** @brief 控制帧下标 → 标识符；顺序必须与 .hpp 里的 FRAME_0Xxxx 一致 */
const uint32_t DjiMotorGroup::TX_ID[DjiMotorGroup::FRAME_NUM] = {0x200U, 0x1FFU, 0x1FEU, 0x2FEU};

/**
 * @brief 各型号的协议常量（取值均来自 Docs/sheet 下的手册）
 * @note 下标就是 DjiMotorModel 的值，枚举里加型号时必须同步加一行，并同步 MODEL_NUM
 */
const DjiMotorGroup::ModelTraits DjiMotorGroup::MODEL_TRAITS[DjiMotorGroup::MODEL_NUM] = {
  /* M3508  */ {16384, 3591.0f / 187.0f},
  /* M2006  */ {10000, 36.0f},
  /* GM6020 */ {16384, 1.0f},
};


// ----------------
// ---------------- 构造与析构 ----------------


/**
 * @brief 只保存配置，不碰硬件、不创建 RTOS 对象
 *
 * @note 所以可以在 main() 之前的静态初始化阶段构造（device_cfg.cpp 里的 can3_dji_group 就是这样）。
 *       真正会失败的事（校验成员表）都放在 init() 里，构造函数不返回状态。
 */
DjiMotorGroup::DjiMotorGroup(const Config &cfg) : _cfg(cfg)
{
}


// ----------------
// ---------------- 公有接口 ----------------


/**
 * @brief 校验成员表并把配置折算进各位置
 *
 * @note 本函数是「位置推出一切」的唯一落地点：把每个位置的控制帧、帧内槽位、指令限幅、
 *       减速比、零位一次算好存进 Slot，之后 update() / set_output() 只看 Slot，不再查型号表。
 * @note 失败只有两种：总线句柄没给（NOT_INIT），或型号放错位置（BAD_ARG）。任意一种
 *       都直接返回，调用方（device_init）会 configASSERT 停机，不会带着半张成员表继续跑。
 */
Status DjiMotorGroup::init(void)
{
  // 没有总线，本组没有任何意义
  if (_cfg.can == nullptr)
  {
    return Status::NOT_INIT;
  }

  // 幂等：已经算过就不再算一遍（也避免重复置位 _frame_active）
  if (_inited)
  {
    return Status::OK;
  }

  // 从左到右扫一遍 11 个位置
  for (uint8_t index = 0U; index < MOTOR_NUM; ++index)
  {
    const Member *member = _cfg.member[index];
    if (member == nullptr)
    {
      continue; // 该位没有电机：Slot 保持全零，对应控制帧也不会被它置为 active
    }

    // 型号必须与位置相容：0~3 只收 3508/2006，4~7 三种都行，8~10 只收 6020
    if (!is_model_allowed(index, member->model))
    {
      return Status::BAD_ARG;
    }

    // 查常量表：下标就是 DjiMotorModel 的值
    const ModelTraits traits = MODEL_TRAITS[static_cast<uint8_t>(member->model)];
    const FramePos    pos    = frame_position(index, member->model); // 该位置落在哪条控制帧的哪个槽
    Slot             &slot   = _slot[index];

    slot.limit             = static_cast<uint16_t>(traits.limit);
    slot.data.param.offset = member->offset;
    // ratio 传 0 表示「用型号默认值」，调用方不必去手册里抄 3591/187 这种数字
    slot.data.param.ratio  = (member->ratio > 0.0f) ? member->ratio : traits.default_ratio;
    slot.tx_frame          = static_cast<uint8_t>(pos.frame);
    slot.tx_slot           = static_cast<uint8_t>(pos.slot);

    // 这条控制帧上至少有一个成员了，poll() 才会真的把它发出去
    _frame_active[pos.frame] = true;
  }

  _inited = true;
  return Status::OK;
}

/**
 * @brief 喂入一帧反馈
 *
 * @note 两道过滤（帧格式、标识符范围）都在这里做，全部通过后才交给 _parse_feedback()
 *       做纯计算 —— 这样解析函数里不必再重复判断帧的合法性。
 * @note 位置直接由标识符相减得到，所以整组不需要任何「标识符 → 位置」的查找表。
 */
Status DjiMotorGroup::update(const CanRxMsg &rx)
{
  if (!_inited)
  {
    return Status::NOT_INIT;
  }

  // 只收「标准帧 + 经典 CAN + 数据帧 + 8 字节」：DJI 电调反馈就是这个形式，
  // 其余（扩展帧 / FD 帧 / 远程帧 / 非 8 字节）一律不当反馈，避免把别家的帧解析成电机数据
  const bool classic_8 = (rx.header.IdType == FDCAN_STANDARD_ID) && (rx.header.RxFrameType == FDCAN_DATA_FRAME) && (rx.header.FDFormat == FDCAN_CLASSIC_CAN) && (rx.header.DataLength == FDCAN_DLC_BYTES_8);
  if (!classic_8)
  {
    return Status::BAD_ARG;
  }

  // 反馈标识符连续：0x201 ~ 0x20B，减去 0x201 就是位置；
  // 范围外的（比如 0x200 / 0x1FF 这类控制帧的回显）不属于本组，丢掉即可
  if ((rx.header.Identifier < RX_ID_BASE) || (rx.header.Identifier >= (RX_ID_BASE + MOTOR_NUM)))
  {
    return Status::BAD_ARG;
  }

  const uint8_t index = static_cast<uint8_t>(rx.header.Identifier - RX_ID_BASE);
  if (_cfg.member[index] == nullptr)
  {
    return Status::BAD_ARG; // 位置合法但那儿没装电机：成员表与实车不符，同样当无效帧丢掉
  }

  return _parse_feedback(_slot[index], rx.data);
}

/**
 * @brief 写入某个位置的原始控制量（只更新发送缓冲，不发送）
 *
 * @note 本函数只做三件事：定位到该位置的发送槽 → 把 raw 限幅 → 按大端写进缓冲。
 * @note 定位用的是 init() 算好的 tx_frame / tx_slot，所以热路径上看不到型号判断。
 * @note 反复调用安全：同一个位置每周期覆盖写同一处，不累加、不排队。
 */
Status DjiMotorGroup::set_output(uint8_t index, int16_t raw)
{
  if (!_inited)
  {
    return Status::NOT_INIT;
  }
  if (index >= MOTOR_NUM)
  {
    return Status::BAD_ARG;
  }
  if (_cfg.member[index] == nullptr)
  {
    return Status::BAD_ARG; // 该位置没有挂电机
  }

  Slot &slot = _slot[index];

  // 手册给出的原始指令绝对值上限（3508/6020 是 16384，2006 是 10000）；最大 16384，int16_t 装得下
  const int16_t limit = static_cast<int16_t>(slot.limit);
  if (raw > limit)
  {
    raw = limit;
  }
  else if (raw < -limit)
  {
    raw = -limit;
  }

  // 帧与槽位在 init() 里一次算好，这里不再看型号；槽位 0~3 对应数据域偏移 0/2/4/6，
  // 手册要求每个 16 位值高字节在前
  put_be16(&_frame_data[slot.tx_frame][slot.tx_slot * MOTOR_REG_BYTES], raw);
  return Status::OK;
}

/**
 * @brief 发送四条控制帧
 *
 * @note 四条帧固定是 0x200 / 0x1FF / 0x1FE / 0x2FE（下标 0 ~ 3）。只有挂了成员的那几条
 *       才真的发，所以只有 3508/2006 的车不会白白占用 0x1FE / 0x2FE。
 * @note 帧内没被 set_output() 写过的槽位一直是 0（构造时清零），等价于「这一路 0 输出」，
 *       对没挂电机的槽位也没有副作用。
 * @note 单次发送失败不在此处上报：BspCan::diagnostics 里有 tx_buf_full / tx_dropped 计数可查。
 */
Status DjiMotorGroup::poll(void)
{
  if (!_inited)
  {
    return Status::NOT_INIT;
  }

  for (uint32_t frame = 0U; frame < FRAME_NUM; ++frame)
  {
    if (!_frame_active[frame])
    {
      continue; // 该帧一个成员都没有，整条不发（这不算失败）
    }
    (void)_cfg.can->send(TX_ID[frame], _frame_data[frame]); // 发送结果有意丢弃，见上面的 @note
  }
  return Status::OK;
}


// ----------------
// ---------------- 查询接口 ----------------


/**
 * @brief 取某个位置的输出轴数据
 *
 * @note 返回的是 slot 内部的实时引用，不是快照 —— 所以调用方读完就得用掉，
 *       不能把引用存起来跨周期用（update() 会改这块内存）。
 */
const MotorData &DjiMotorGroup::data(uint8_t index) const
{
  // 无效位置返回全零对象（静态常量初始化，无运行时开销），
  // 避免调用方拿到悬空引用，也省掉调用侧每次都要先查 has_motor()
  static const MotorData ZERO_DATA {};

  return has_motor(index) ? _slot[index].data : ZERO_DATA;
}

/**
 * @brief 某个位置是否在线
 *
 * @note 先查成员表再查 Online 节点：没装电机的位没必要看在线状态，也顺手挡掉越界。
 * @note Online 的计时由 sys_task 每 10 ms 推进一次，所以本函数与自己的调用频率无关。
 */
bool DjiMotorGroup::is_online(uint8_t index) const
{
  return has_motor(index) && (_slot[index].online.is_online() == Status::OK);
}

/**
 * @brief 某个位置是否存在电机
 *
 * @note 只看构造时给的成员表，与在线状态、是否收到过反馈都无关。
 */
bool DjiMotorGroup::has_motor(uint8_t index) const
{
  return (index < MOTOR_NUM) && (_cfg.member[index] != nullptr); // 越界一律 false
}


// ----------------
// ---------------- 私有实现 ----------------


/**
 * @brief 解析一帧已校验过的反馈，刷新该位置的运动学数据
 *
 * @note 反馈帧 8 字节数据域的布局（手册里的表，多字节一律高字节在前）：
 *
 *       DATA[0:2]  转子机械角度    0 ~ 8191，转子转一圈 = 8192 个计数
 *       DATA[2:4]  转子转速        int16，单位 rpm —— **是转子转速，不是输出轴转速**
 *       DATA[4:6]  实际转矩电流    int16（本类不用）
 *       DATA[6]    温度            M3508 / GM6020 有；M2006 此字节为 Null（本类不存）
 *       DATA[7]    保留
 *
 * @note 一帧要折算出的两个量，换算链分别是（ratio = 减速比，offset = 机械零位）：
 *
 *       角度   ：转子计数 --跨零展开--> 累计计数 --×2π/8192--> 转子角 --(-offset)/ratio--> 输出轴角
 *       角速度 ：DATA[2:4] 的 rpm / ratio × π/30                             --> 输出轴 rad/s
 *
 * @note 不在这里算角加速度：它是跨帧量（相邻两帧角速度之差 / 时间差），单帧算不出来，而且直接
 *       差分噪声很大、滤波方式取决于用途。radian_data.acceleration 保持 0，需要就由上层自己算。
 */
Status DjiMotorGroup::_parse_feedback(Slot &slot, const uint8_t *data)
{
  // 转子单圈计数；8192 是满量程，取到范围外说明这帧是脏的，不能当有效反馈
  const uint16_t ecd = be_u16(&data[0]);
  if (ecd >= ECD_FULL_RANGE)
  {
    return Status::BAD_ARG;
  }

  // 电调反馈的是转子转速：先除减速比换算到输出轴，再由 rpm 换算 rad/s（× 2π/60 = π/30）
  const float   ratio    = slot.data.param.ratio;
  const int16_t rpm      = static_cast<int16_t>(be_u16(&data[2]));
  const float   velocity = static_cast<float>(rpm) / ratio * (PI / 30.0f);
  slot.data.radian_data.velocity = velocity;

  // 单圈计数在 [0, 8191] 里循环，必须跨零展开成单调的累计计数，
  // 否则每过一圈角度就会跳变一次，就表示不了「转了好几圈」
  if (slot.feedback_ready)
  {
    slot.total_ecd = unwrap_ecd(slot.total_ecd, slot.last_ecd, ecd, ECD_FULL_RANGE);
  }
  else
  {
    slot.total_ecd = static_cast<int64_t>(ecd); // 首帧：以当前位置作累计原点，所以上电后角度从 0 附近起算
  }
  slot.last_ecd = static_cast<int32_t>(ecd);

  // 累计计数 → 转子角（rad）→ 减零位、再除减速比 = 输出轴角度。
  // 这里用 double：累计计数能到几十万，float 只有 24 位有效位，直接算会丢精度
  const double rotor_angle  = static_cast<double>(slot.total_ecd) * (2.0 * PI) / static_cast<double>(ECD_FULL_RANGE);
  const double output_angle = (rotor_angle - static_cast<double>(slot.data.param.offset)) / static_cast<double>(ratio);

  slot.data.radian_data.angle_multi_round  = output_angle; // 多圈角就是 double，不要再压回 float 丢掉精度
  slot.data.radian_data.angle_single_round = static_cast<float>(fmod(output_angle, 2.0 * PI));

  // 单圈角归一到 [0, 2π)：fmod 对负数给的是负值，补一圈即可
  if (slot.data.radian_data.angle_single_round < 0.0f)
  {
    slot.data.radian_data.angle_single_round += 2.0f * PI;
  }

  // 走到这里才算「这一帧被成功吃下」：置位后下一帧才能做跨零展开，
  // 同时给 Online 节点续一次命（真正的掉线判定由 sys_task 每 10 ms 推进）
  slot.feedback_ready = true;
  slot.online.refresh_task();
  return Status::OK;
}


// ----------------
