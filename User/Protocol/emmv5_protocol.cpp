#include "emmv5_protocol.hpp"

#include <string.h> // memmove


// EmmV5.0 协议编码 / 解码。帧格式、功能码表、加速度时间公式与 MMCL 布局见
// User/Protocol/README.md；本文件只按手册的字节顺序拼帧、按窗口切帧。


namespace
{
///< bool → 协议里的 0/1 字节
uint8_t to_u8(bool value)
{
  return value ? 1U : 0U;
}

/**
 * @brief 系统参数枚举 → 协议功能码
 *
 * @note 功能码来源：EmmV5.0 说明书 6.3.4「读取参数命令列表」。
 *       返回数据的格式随参数不同而不同：电压/电流为无符号多字节，
 *       位置/速度为“符号字节 + 数值”，角度类数值一圈为 65536。
 */
uint8_t sys_param_code(emmv5::SystemParam param)
{
  switch (param)
  {
    case emmv5::SystemParam::BusVoltage:
      return 0x24; // mV
    case emmv5::SystemParam::BusCurrent:
      return 0x26; // mA
    case emmv5::SystemParam::PhaseCurrent:
      return 0x27; // mA
    case emmv5::SystemParam::EncoderRaw:
      return 0x29; // 编码器原始值
    case emmv5::SystemParam::PulseCount:
      return 0x30; // 实时脉冲数
    case emmv5::SystemParam::EncoderLinear:
      return 0x31; // 校准后编码器值，0~65535/圈
    case emmv5::SystemParam::InputPulse:
      return 0x32; // 符号 + 32 位输入脉冲
    case emmv5::SystemParam::TargetPosition:
      return 0x33; // 符号 + 32 位目标位置，65536/圈
    case emmv5::SystemParam::SetPosition:
      return 0x34; // 符号 + 32 位实时设定位置
    case emmv5::SystemParam::Velocity:
      return 0x35; // 符号 + 16 位转速，RPM
    case emmv5::SystemParam::CurrentPosition:
      return 0x36; // 符号 + 32 位实时位置
    case emmv5::SystemParam::PositionError:
      return 0x37; // 符号 + 32 位位置误差
    case emmv5::SystemParam::BatteryVoltage:
      return 0x38; // Y42：多圈编码器电池 mV
    case emmv5::SystemParam::Temperature:
      return 0x39; // Y42：温度 ℃
    case emmv5::SystemParam::StatusFlag:
      return 0x3A; // 状态标志位
    case emmv5::SystemParam::OriginFlag:
      return 0x3B; // 回零状态标志位
    case emmv5::SystemParam::StatusAndOriginFlag:
      return 0x3C; // Y42：状态 + 回零标志
    case emmv5::SystemParam::PinStatus:
      return 0x3D; // Y42：引脚状态
  }
  return 0;
}
} // namespace


namespace emmv5
{
// ---------------- 到位帧识别 ----------------

/**
 * @brief 清空到位帧滑动窗口
 *
 * @note 到位帧格式：地址 + FD 9F 6B（说明书 8.1）。
 *       菜单 Response 必须设为 Reached / Both / Other 才会返回；
 *       默认 Receive 只回“收到命令”，不返回到位帧。
 */
void InPositionScanner::reset()
{
  length = 0;
}

/**
 * @brief 喂入一块串口字节，识别“地址 + FD 9F 6B”
 * @param addr 本电机地址（1~255）
 * @param data 数据
 * @param size 字节数
 * @return true=本块中至少出现一个完整到位帧
 *
 * @note 帧可能被串口分块切开，窗口保留最后 3 字节跨块拼接。
 *       命中后继续扫描本块剩余字节，避免漏掉粘在一起的多帧。
 */
bool InPositionScanner::feed(uint8_t addr, const uint8_t *data, size_t size)
{
  if (data == nullptr || size == 0)
  {
    return false;
  }

  bool hit = false;

  for (size_t i = 0; i < size; i++)
  {
    // 窗口满了还没命中：滚掉最旧的一个字节，再放新字节
    if (length == sizeof(window))
    {
      memmove(window, window + 1, sizeof(window) - 1U);
      length = sizeof(window) - 1U;
    }

    window[length++] = data[i];

    // 只有真正凑齐「地址 + FD 9F 6B」才算命中
    if (length == sizeof(window)
        && window[0] == addr
        && window[1] == IN_POSITION_MARK1
        && window[2] == IN_POSITION_MARK2
        && window[3] == IN_POSITION_MARK3)
    {
      length = 0;
      hit    = true;
      // 不 return：同一块里可能还有第二帧，继续扫完
    }
  }

  return hit;
}


/**
 * @brief 清空 0x36 位置扫描窗口
 */
void PositionScanner::reset()
{
  length = 0;
}

/**
 * @brief 喂入串口字节并尝试拼出 `addr 36 sign v3 v2 v1 v0 6B`
 *
 * @note 窗口固定 8 字节，满了就左移一格；这样即使一帧被串口拆成两三块，
 *       下一次调用也能把首尾接起来。
 * @note 命中后窗口立即清空，同一块里如果还粘着第二帧，循环会继续扫完。
 */
bool PositionScanner::feed(uint8_t addr, const uint8_t *data, size_t size, int32_t &position)
{
  if (data == nullptr || size == 0U)
  {
    return false;
  }

  for (size_t i = 0; i < size; i++)
  {
    if (length == sizeof(window))
    {
      memmove(window, window + 1, sizeof(window) - 1U);
      length = sizeof(window) - 1U;
    }

    window[length++] = data[i];

    if (length == sizeof(window) && window[0] == addr && window[1] == 0x36 && window[7] == CHECKSUM)
    {
      const uint32_t magnitude = (static_cast<uint32_t>(window[3]) << 24) | (static_cast<uint32_t>(window[4]) << 16)
                                 | (static_cast<uint32_t>(window[5]) << 8) | static_cast<uint32_t>(window[6]);

      // 用 int64 取负，避开 magnitude == INT32_MIN 时的溢出
      const int64_t value = (window[2] == 0U) ? static_cast<int64_t>(magnitude) : -static_cast<int64_t>(magnitude);

      position = static_cast<int32_t>(value);
      length   = 0;
      return true;
    }
  }

  return false;
}

/**
 * @brief 清空 0x3A 状态扫描窗口
 */
void StatusScanner::reset()
{
  length = 0;
}

/**
 * @brief 喂入串口字节并尝试拼出 `addr 3A status 6B`
 *
 * @note 状态位含义见说明书 6.3.4；本函数只负责把状态字节原样交出去。
 */
bool StatusScanner::feed(uint8_t addr, const uint8_t *data, size_t size, uint8_t &status)
{
  if (data == nullptr || size == 0U)
  {
    return false;
  }

  for (size_t i = 0; i < size; i++)
  {
    if (length == sizeof(window))
    {
      memmove(window, window + 1, sizeof(window) - 1U);
      length = sizeof(window) - 1U;
    }

    window[length++] = data[i];

    if (length == sizeof(window) && window[0] == addr && window[1] == 0x3A && window[3] == CHECKSUM)
    {
      status = window[2];
      length = 0;
      return true;
    }
  }

  return false;
}


// ----------------
// ---------------- MMCL 总帧编码 ----------------

/**
 * @brief 把 MMCL 子命令打包成一帧总帧
 * @param target       总帧地址（通常 0 广播）
 * @param payload      子命令串（每条都是独立命令帧）
 * @param payload_size 子命令字节数
 * @param out          输出缓冲
 * @param out_capacity 输出缓冲容量
 * @return 总帧长度；0=参数非法或容量不足
 *
 * @note 总帧布局：地址(1) + 0xAA(1) + 总长度(2, 大端) + 子命令(N) + 0x6B(1)
 *       总长度包含头尾 5 字节；子命令里的地址仍是各电机自己的地址。
 *       说明书要求总帧一次发完；发送流缓冲区 128 字节下子命令最多 123 字节。
 */
size_t build_multi_frame(uint8_t target, const uint8_t *payload, size_t payload_size, uint8_t *out, size_t out_capacity)
{
  if (payload == nullptr || payload_size == 0 || out == nullptr)
  {
    return 0;
  }

  const size_t total = payload_size + MULTI_OVERHEAD;

  // 总帧长度字段只有 2 字节；哪怕调用方给了大缓冲，也不能越过协议上限
  if (total > 0xFFFFU || total > out_capacity)
  {
    return 0;
  }

  out[0] = target;
  out[1] = MULTI_MARK;
  out[2] = static_cast<uint8_t>(total >> 8);
  out[3] = static_cast<uint8_t>(total);

  // payload 可能就在 out+4（调用方把子命令直接攒在总帧缓冲里），必须用 memmove
  memmove(out + 4U, payload, payload_size);
  out[4U + payload_size] = CHECKSUM;

  return total;
}


// ----------------
// ---------------- 触发动作命令 ----------------

/**
 * @brief 将当前位置清零
 * @param addr 电机地址
 * @return 帧：addr 0A 6D 6B
 *
 * @note 会同时清零位置角度、位置误差、脉冲数等。
 */
Frame zero_position(uint8_t addr)
{
  Frame f;
  f.append_u8(addr);
  f.append_u8(0x0A);
  f.append_u8(0x6D);
  f.seal();
  return f;
}


// ----------------
// ---------------- 运动控制命令 ----------------

/**
 * @brief 电机使能/关闭
 * @param addr 电机地址
 * @param on   true=使能，false=关闭
 * @return 帧：addr F3 AB on 00 6B
 *
 * @note sync 字节固定写 0；本工程不使用多机同步模式。
 * @note 速度/位置命令在未使能时会返回“条件不满足”状态码（E2）。
 */
Frame enable(uint8_t addr, bool on)
{
  Frame f;
  f.append_u8(addr);
  f.append_u8(0xF3);
  f.append_u8(0xAB);
  f.append_u8(to_u8(on));
  f.append_u8(0U); // sync：固定 0
  f.seal();
  return f;
}

/**
 * @brief 速度模式
 * @param addr  电机地址
 * @param dir   0=CW，1=CCW
 * @param rpm   速度，0~5000 RPM
 * @param accel 加速度档位 0~255；0=直接启动
 * @return 帧：addr F6 dir rpm_H rpm_L accel 00 6B
 *
 * @note acc>0 时每提升 1 RPM 用时 (256-acc)*50us；
 *       acc=50 时 0→300 RPM 约 3.09s，移动时间要按这个公式估。
 * @note sync 字节固定写 0；本工程不使用多机同步模式。
 */
Frame velocity(uint8_t addr, Direction dir, uint16_t rpm, uint8_t accel)
{
  Frame f;
  f.append_u8(addr);
  f.append_u8(0xF6);
  f.append_u8(static_cast<uint8_t>(dir));
  f.append_u16(rpm);
  f.append_u8(accel);
  f.append_u8(0U); // sync：固定 0
  f.seal();
  return f;
}

/**
 * @brief 位置模式
 * @param addr   电机地址
 * @param dir    0=CW，1=CCW
 * @param rpm    速度，0~5000 RPM
 * @param accel  加速度档位 0~255
 * @param pulses 带符号脉冲数；绝对模式按 32 位补码写入
 * @param mode   0=相对运动，1=绝对运动（EmmV5.0 按布尔解释 raF）
 * @return 帧：addr FD dir rpm_H rpm_L accel clk3 clk2 clk1 clk0 mode 00 6B
 *
 * @note 相对模式参考“上一输入目标位置”，不是当前实时位置；
 *       绝对模式目标为 0 时表示回位置零点，方向字节通常被忽略。
 * @note sync 字节固定写 0；本工程不使用多机同步模式。
 */
Frame position(uint8_t addr, Direction dir, uint16_t rpm, uint8_t accel, int32_t pulses, PositionMode mode)
{
  Frame f;
  f.append_u8(addr);
  f.append_u8(0xFD);
  f.append_u8(static_cast<uint8_t>(dir));
  f.append_u16(rpm);
  f.append_u8(accel);
  f.append_u32(static_cast<uint32_t>(pulses));
  f.append_u8(static_cast<uint8_t>(mode));
  f.append_u8(0U); // sync：固定 0
  f.seal();
  return f;
}

/**
 * @brief 设置快速位置模式参数
 * @param addr  电机地址
 * @param rpm   速度，0~5000 RPM
 * @param accel 加速度档位 0~255
 * @param mode  0=相对，1=绝对
 * @return 帧：addr F1 rpm_H rpm_L accel mode 00 6B
 *
 * @note 只设置参数，不产生运动；后续 quick_position() 使用这组参数。
 * @note sync 字节固定写 0；本工程不使用多机同步模式。
 */
Frame quick_position_params(uint8_t addr, uint16_t rpm, uint8_t accel, PositionMode mode)
{
  Frame f;
  f.append_u8(addr);
  f.append_u8(0xF1);
  f.append_u16(rpm);
  f.append_u8(accel);
  f.append_u8(static_cast<uint8_t>(mode));
  f.append_u8(0U); // sync：固定 0
  f.seal();
  return f;
}

/**
 * @brief 快速位置控制
 * @param addr   电机地址
 * @param pulses 带符号脉冲数；16 细分下 ±3200 = ±1 圈
 * @return 帧：addr FC clk3 clk2 clk1 clk0 6B
 *
 * @note 使用 quick_position_params() 设置的转速/加速度/相对绝对模式。
 */
Frame quick_position(uint8_t addr, int32_t pulses)
{
  Frame f;
  f.append_u8(addr);
  f.append_u8(0xFC);
  f.append_u32(static_cast<uint32_t>(pulses));
  f.seal();
  return f;
}

/**
 * @brief 立即停止
 * @param addr 电机地址
 * @return 帧：addr FE 98 00 6B
 *
 * @note 速度模式和位置模式都可用，属于紧急刹车。
 * @note sync 字节固定写 0；本工程不使用多机同步模式。
 */
Frame stop(uint8_t addr)
{
  Frame f;
  f.append_u8(addr);
  f.append_u8(0xFE);
  f.append_u8(0x98);
  f.append_u8(0U); // sync：固定 0
  f.seal();
  return f;
}

// ----------------
// ---------------- 读取系统参数命令 ----------------

/**
 * @brief 读取一次系统参数
 * @param addr  电机地址
 * @param param 参数类型（功能码见 sys_param_code）
 * @return 帧：addr code 6B
 *
 * @note 回复格式随参数不同：
 *       电压/电流/编码器：无符号多字节；
 *       位置/速度：符号字节(0=正,1=负) + 数值；
 *       位置类一圈对应 65536，角度 = value * 360 / 65536。
 */
Frame read(uint8_t addr, SystemParam param)
{
  const uint8_t code = sys_param_code(param);

  // 非法枚举不拼帧：返回空 Frame，由设备层 _send() 转为 BAD_ARG
  if (code == 0U)
  {
    return Frame {};
  }

  Frame f;
  f.append_u8(addr);
  f.append_u8(code);
  f.seal();
  return f;
}


// ----------------

} // namespace emmv5
