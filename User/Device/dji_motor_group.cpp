#include "dji_motor_group.hpp"

#include <math.h>


// ---------------- 构造与析构 ----------------


DjiMotorGroup::DjiMotorGroup(const Config &cfg) : _cfg(cfg)
{
}


// ----------------
// ---------------- 公有接口 ----------------


/** @brief 校验成员表并把配置折算进各位置；型号与位置不匹配则立刻返回 BAD_ARG */
Status DjiMotorGroup::init(void)
{
  if (_cfg.can == nullptr)
  {
    return Status::NOT_INIT;
  }
  if (_inited)
  {
    return Status::OK; // 幂等：重复初始化不重复累加
  }

  for (uint8_t index = 0U; index < MOTOR_NUM; ++index)
  {
    const Member *member = _cfg.member[index];
    if (member == nullptr)
    {
      continue; // 该位没有电机
    }
    if (!is_model_allowed(index, member->model))
    {
      return Status::BAD_ARG;
    }

    const ModelTraits traits = model_traits(member->model);
    const FramePos    pos    = frame_position(index, member->model);
    Slot             &slot   = _slot[index];

    slot.limit             = static_cast<uint16_t>(traits.limit);
    slot.data.param.offset = member->offset;
    slot.data.param.ratio  = (member->ratio > 0.0f) ? member->ratio : traits.default_ratio;
    slot.tx_frame          = static_cast<uint8_t>(pos.frame);
    slot.tx_slot           = static_cast<uint8_t>(pos.slot);

    _frame_active[pos.frame] = true;
  }

  _inited = true;
  return Status::OK;
}

/** @brief 喂入一帧反馈：校验帧与位置后交给 _parse_feedback() 解析 */
Status DjiMotorGroup::update(const CanRxMsg &rx)
{
  if (!_inited)
  {
    return Status::NOT_INIT;
  }

  // 只收标准经典 8 字节数据帧
  const bool classic_8 = (rx.header.IdType == FDCAN_STANDARD_ID) && (rx.header.RxFrameType == FDCAN_DATA_FRAME) && (rx.header.FDFormat == FDCAN_CLASSIC_CAN) && (rx.header.DataLength == FDCAN_DLC_BYTES_8);
  if (!classic_8)
  {
    return Status::BAD_ARG;
  }

  // 反馈标识符连续：0x201 ~ 0x20B 减去 0x201 就是位置
  if ((rx.header.Identifier < RX_ID_BASE) || (rx.header.Identifier >= (RX_ID_BASE + MOTOR_NUM)))
  {
    return Status::BAD_ARG;
  }

  const uint8_t index = static_cast<uint8_t>(rx.header.Identifier - RX_ID_BASE);
  if (_cfg.member[index] == nullptr)
  {
    return Status::BAD_ARG; // 该位置没有挂电机
  }

  return _parse_feedback(_slot[index], rx.data);
}

/** @brief 写入某个位置的原始控制量（只更新发送缓冲，不发送） */
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

  // 手册给出的原始指令绝对值上限，最大 16384，int16_t 可表示
  const int16_t limit = static_cast<int16_t>(slot.limit);
  if (raw > limit)
  {
    raw = limit;
  }
  else if (raw < -limit)
  {
    raw = -limit;
  }

  // 帧与槽位在 init() 里一次算好，这里不再看型号；手册要求高字节在前
  put_be16(&_frame_data[slot.tx_frame][slot.tx_slot * 2U], raw);
  return Status::OK;
}

/**
 * @brief 发送四条控制帧（该帧没有成员时整条跳过）
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
      continue; // 该帧一个成员都没有，不发
    }
    (void)_cfg.can->send(tx_id(frame), _frame_data[frame]);
  }
  return Status::OK;
}


// ----------------
// ---------------- 查询接口 ----------------


const MotorData &DjiMotorGroup::data(uint8_t index) const
{
  // 无效位置返回全零对象（常量初始化，无运行时开销），避免调用方拿到悬空引用
  static const MotorData ZERO_DATA {};

  return has_motor(index) ? _slot[index].data : ZERO_DATA;
}

bool DjiMotorGroup::is_online(uint8_t index) const
{
  return has_motor(index) && (_slot[index].online.is_online() == Status::OK);
}

bool DjiMotorGroup::has_motor(uint8_t index) const
{
  return (index < MOTOR_NUM) && (_cfg.member[index] != nullptr);
}


// ----------------
// ---------------- 私有实现 ----------------


/** @brief 解析一帧已校验过的反馈：转速 / 角加速度 → 跨零展开 → 换算输出轴角度 */
Status DjiMotorGroup::_parse_feedback(Slot &slot, const uint8_t *data)
{
  // DATA[0..1] 转子机械角度，DATA[2..3] 转速，DATA[4..5] 实际转矩电流，DATA[6] 温度
  const uint16_t ecd = be_u16(&data[0]);
  if (ecd >= ECD_FULL_RANGE)
  {
    return Status::BAD_ARG;
  }

  // 电调反馈的是转子转速：先除减速比换算到输出轴，再由 rpm 换算 rad/s
  const float   ratio    = slot.data.param.ratio;
  const int16_t rpm      = static_cast<int16_t>(be_u16(&data[2]));
  const float   velocity = static_cast<float>(rpm) / ratio * (PI / 30.0f);

  // 角加速度只能用相邻两帧差分：Δ输出轴角速度 / Δt；tick 差值除以 tick 频率就是秒
  const TickType_t now      = xTaskGetTickCount();
  const TickType_t dt_ticks = now - slot.last_tick;
  if (slot.feedback_ready && (dt_ticks != 0U))
  {
    const float dt_s                   = static_cast<float>(dt_ticks) / static_cast<float>(configTICK_RATE_HZ);
    slot.data.radian_data.acceleration = (velocity - slot.last_velocity) / dt_s;
  }
  else
  {
    slot.data.radian_data.acceleration = 0.0f; // 首帧或同一 tick 内的重复帧：没有差分依据
  }
  slot.last_velocity             = velocity;
  slot.last_tick                 = now;
  slot.data.radian_data.velocity = velocity;

  // 相邻两帧的转子位移必然小于半圈，超过半圈说明跨了零点
  if (slot.feedback_ready)
  {
    slot.total_ecd = unwrap_ecd(slot.total_ecd, slot.last_ecd, ecd, ECD_FULL_RANGE);
  }
  else
  {
    slot.total_ecd = static_cast<int64_t>(ecd); // 首帧：以当前绝对位置作为累计原点
  }
  slot.last_ecd = static_cast<int32_t>(ecd);

  // 累计计数 → 设备轴（转子）角度 → 输出轴角度：(转子角 - 零位) / 减速比
  const double rotor_angle  = static_cast<double>(slot.total_ecd) * (2.0 * PI) / static_cast<double>(ECD_FULL_RANGE);
  const double output_angle = (rotor_angle - static_cast<double>(slot.data.param.offset)) / static_cast<double>(ratio);

  slot.data.radian_data.angle_multi_round  = static_cast<float>(output_angle);
  slot.data.radian_data.angle_single_round = static_cast<float>(fmod(output_angle, 2.0 * PI));

  // 单圈角归一到 [0, 2π)
  if (slot.data.radian_data.angle_single_round < 0.0f)
  {
    slot.data.radian_data.angle_single_round += 2.0f * PI;
  }

  slot.feedback_ready = true;
  slot.online.refresh_task();
  return Status::OK;
}


// ----------------
