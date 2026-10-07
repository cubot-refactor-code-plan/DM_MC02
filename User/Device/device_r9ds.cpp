#include "device_r9ds.hpp"


// ---------------- 构造与析构 ----------------


/**
 * @brief 绑定串口
 *
 * @note 构造函数只做赋值，不碰硬件、不读串口：真正的复位在 init() 里，
 *       所以可以在 main() 之前的静态初始化阶段构造（见 device_cfg.cpp）。
 */
DeviceR9ds::DeviceR9ds(BspUart<128> &uart) : _uart(uart), _online(ONLINE_TIMEOUT_MS)
{
}


// ----------------
// ---------------- 生命周期 ----------------


/**
 * @brief 复位解析状态与诊断计数
 *
 * @note 重复调用安全：丢掉的只是"半截帧"和统计数字，下一次 update() 会重新找帧头。
 *       通道值一并置成安全默认，这样还没收到过任何帧时上层读到的不是野值。
 */
Status DeviceR9ds::init(void)
{
  // 解析状态：从"找帧头"重新开始
  _fill       = 0U;
  _frame_lost = false;
  _failsafe   = false;
  _diag       = Diag {};

  for (uint32_t i = 0U; i < CHANNEL_NUM; ++i)
  {
    _raw[i] = CHANNEL_MID; // 还没收到帧，原始通道先按中位显示
  }
  for (uint32_t i = 0U; i < FRAME_SIZE; ++i)
  {
    _frame[i] = 0U;
  }

  // 安全默认（顺序 = Rc 字段顺序）：连续通道全回中位 0，开关全回上挡
  _rc = Rc {0, 0, 0, 0, Switch::UP, 0, 0, 0, Switch::UP, Switch::UP};

  // 在线状态不用管：Online 节点初始就是离线，收到第一帧有效数据才会转在线
  return Status::OK;
}

/**
 * @brief 把这一拍串口里攒的字节全部消化掉
 *
 * @note 状态机只有三档：_fill == 0 时在找 0x0F；0 < _fill < 25 时在拼帧；
 *       拼满 25 字节后用帧尾 0x00 确认，不匹配就整帧丢掉、重新找帧头。
 * @note 帧体内部出现 0x0F 不会误判 —— 拼帧期间不看内容，只看长度和帧尾。
 */
void DeviceR9ds::update(void)
{
  for (;;)
  {
    uint8_t byte = 0U;
    size_t  got  = 0U;

    // 超时 0：没有数据立刻返回，不阻塞调用它的任务
    if ((_uart.receive(&byte, 1U, 0U, &got) != Status::OK) || (got == 0U))
    {
      return; // 这一拍的字节取完了
    }

    ++_diag.byte_cnt;

    // 还没进入帧：跳过一切非帧头的字节
    if ((_fill == 0U) && (byte != FRAME_HEADER))
    {
      ++_diag.skip_byte_cnt;
      continue;
    }

    _frame[_fill] = byte;
    ++_fill;

    if (_fill < FRAME_SIZE)
    {
      continue; // 还没拼满，等后面的字节
    }

    // 拼满 25 字节：帧尾必须是 0x00，否则整帧作废、重新找帧头
    if (_frame[FRAME_SIZE - 1U] == FRAME_FOOTER)
    {
      ++_diag.frame_cnt;
      _parse_frame(_frame);
    }
    else
    {
      ++_diag.bad_footer_cnt;
    }

    _fill = 0U;
  }
}

// ----------------
// ---------------- 查询接口 ----------------


/** @brief 在线判定交给 Online 节点（计时由 sys_task 每 10 ms 推进，与本任务频率无关） */
bool DeviceR9ds::is_online(void) const
{
  return _online.is_online() == Status::OK;
}

/** @brief 遥控器语义值（控制逻辑用这个），实时引用 */
const DeviceR9ds::Rc &DeviceR9ds::rc(void) const
{
  return _rc;
}

/** @brief 最近一帧的 16 个原始通道值（协议侧） */
const uint16_t *DeviceR9ds::raw_channels(void) const
{
  return _raw;
}

/** @brief 最近一帧的「本帧丢失」标志 */
bool DeviceR9ds::frame_lost(void) const
{
  return _frame_lost;
}

/** @brief 最近一帧的「失控保护激活」标志 */
bool DeviceR9ds::failsafe(void) const
{
  return _failsafe;
}

/** @brief 诊断计数快照 */
DeviceR9ds::Diag DeviceR9ds::diag(void) const
{
  return _diag;
}


// ----------------
// ---------------- 帧解析 ----------------


/**
 * @brief 解析一帧已通过帧尾校验的数据
 *
 * @note 位提取：16 个通道 × 11 bit = 176 bit，小端排在 frame[1] ~ frame[22] 里。
 *       每个通道都跨字节边界，所以统一取「起始字节 + 后两字节」拼成 24 位窗口，
 *       右移掉窗口内偏移后取低 11 位。算例（ch10）：
 *
 *       bit 偏移 = 10 × 11 = 110 → 起始字节 = 110 / 8 + 1 = 14、窗口内偏移 = 110 % 8 = 6
 *       ⇒ (frame[14] | frame[15] << 8 | frame[16] << 16) >> 6 & 0x7FF
 *
 * @note 用统一下标循环算，不手抄 16 条移位表达式 —— 手抄一旦有一位下标写错，现象会是
 *       某个通道只在特定范围才不对，很难查。
 */
void DeviceR9ds::_parse_frame(const uint8_t *frame)
{
  uint32_t bit_index = 0U;
  for (uint32_t i = 0U; i < CHANNEL_NUM; ++i)
  {
    const uint32_t byte_index = (bit_index >> 3U) + 1U; // +1 = 跳过帧头
    const uint32_t bit_offset = bit_index & 7U;

    const uint32_t window = static_cast<uint32_t>(frame[byte_index]) |
                            (static_cast<uint32_t>(frame[byte_index + 1U]) << 8U) |
                            (static_cast<uint32_t>(frame[byte_index + 2U]) << 16U);

    _raw[i] = static_cast<uint16_t>((window >> bit_offset) & CHANNEL_MASK);

    bit_index += 11U;
  }

  // CH1 ~ CH10，下标 = 通道号 - 1。含义见 device_r9ds.hpp 文件头那张表

  // 四个摇杆：连续值
  _rc.right_vertical   = _to_rc_value(_raw[0]); // CH1 右摇杆 上下
  _rc.right_horizontal = _to_rc_value(_raw[1]); // CH2 右摇杆 左右
  _rc.left_vertical    = _to_rc_value(_raw[2]); // CH3 左摇杆 上下
  _rc.left_horizontal  = _to_rc_value(_raw[3]); // CH4 左摇杆 左右

  // 三个旋钮/推杆：连续值
  _rc.knob_left   = _to_rc_value(_raw[5]); // CH6 正面左旋钮
  _rc.slider_left = _to_rc_value(_raw[6]); // CH7 背面左推杆
  _rc.knob_right  = _to_rc_value(_raw[7]); // CH8 正面右旋钮

  // 三个开关：挡位（CH5 是三挡，CH9/CH10 是两挡）
  _rc.switch_c = _to_switch_3pos(_raw[4]); // CH5  C 三挡开关
  _rc.switch_b = _to_switch_2pos(_raw[8]); // CH9  正面 B 开关
  _rc.switch_a = _to_switch_2pos(_raw[9]); // CH10 正面 A 开关

  // frame[23] 是 flags：bit2 = 本帧丢失，bit3 = 失控保护激活（bit0/bit1 是数字通道 17/18）
  const uint8_t flags = frame[FRAME_SIZE - 2U];
  _frame_lost         = (flags & 0x04U) != 0U;
  _failsafe           = (flags & 0x08U) != 0U;

  // 帧完整且帧尾正确 = 接收机确实在发帧，续一次在线命。
  // 注意 flags 里的失控位不影响"在线"：接收机在线、只是它和遥控器失联了，两件事分开报。
  (void)_online.refresh_task();
}


// ----------------
// ---------------- 原始值换算 ----------------


/**
 * @brief 原始值 → 遥控器语义值（连续通道）
 *
 * @note 两端 200 / 1800 对应 +100 / -100，中位 1000 对应 0，线性：
 *
 *       值 = (1000 - raw) / 8        // 8 = (1800 - 200) / (100 - (-100))
 *
 *       算例：raw 200 → +100、raw 1000 → 0、raw 1800 → -100、raw 1200 → -25。
 * @note 整数除法是截断，所以 raw 993 ~ 1007 都得到 0（不足 1 个单位就舍掉），等于自带一个
 *       15 个原始值宽的小死区。
 * @note 故意不限幅到 ±100：原始值跑出 200 ~ 1800（噪声或通道异常）时结果会跟着超出去，
 *       一眼看得见；限了幅反而会把"通道根本没接上"这种故障糊过去。
 */
int16_t DeviceR9ds::_to_rc_value(uint16_t raw)
{
  return static_cast<int16_t>((static_cast<int32_t>(CHANNEL_MID) - static_cast<int32_t>(raw)) / RAW_PER_UNIT);
}


// ----------------
// ---------------- 开关判挡 ----------------


/** @brief 两挡开关判挡：比离 200 / 1800 哪个近，分界正好是中位 1000 */
DeviceR9ds::Switch DeviceR9ds::_to_switch_2pos(uint16_t raw)
{
  return (raw < CHANNEL_MID) ? Switch::UP : Switch::DOWN;
}

/**
 * @brief 三挡开关判挡：比离 200 / 1000 / 1800 哪个近
 *
 * @note 分界取相邻两挡的中点：400 与 1400。物理开关是硬换向的，实测就落在三个标称值上，
 *       所以"就近"比"划阈值区间"稳。
 */
DeviceR9ds::Switch DeviceR9ds::_to_switch_3pos(uint16_t raw)
{
  if (raw < (CHANNEL_MIN + CHANNEL_MID) / 2U) // < 600
  {
    return Switch::UP;
  }
  if (raw < (CHANNEL_MID + CHANNEL_MAX) / 2U) // < 1400
  {
    return Switch::MID;
  }
  return Switch::DOWN;
}


// ----------------
