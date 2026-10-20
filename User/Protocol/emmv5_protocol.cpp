#include "emmv5_protocol.hpp"

#include <string.h> // memmove


// ============================================================================
// EmmV5.0 协议参数速查
// 来源：张大头《Emm_V5.0 步进闭环驱动说明书 Rev1.3》第 6、8 章
//
// 1. 帧格式
//    地址(1) + 功能码(1~2) + 数据(N) + 校验(1)
//    - 地址 1~255，0 为广播；广播命令只有地址 1 的电机回复（6.2.1）
//    - 校验默认 0x6B；本驱动按 0x6B 编码，未实现 XOR/CRC-8
//    - 多字节字段一律大端（高字节在前）
//
// 2. 方向
//    0 = CW（顺时针），1 = CCW（逆时针）
//
// 3. 速度与加速度
//    - 速度单位 RPM，通讯范围 0~5000；实际机械上限见电机铭牌
//    - 加速度 acc 0~255：
//        acc = 0：不使用曲线加减速，直接按设定速度运行
//        acc > 0：每提升 1 RPM 用时 (256 - acc) * 50us
//                 例：acc=50，0→300 RPM 需要 300*(256-50)*50us = 3.09s
//    - 曲线只影响启动/停止的平滑度，不改变匀速段速度
//
// 4. 脉冲与位置
//    - 16 细分、1.8° 电机：3200 脉冲/圈；32 细分：6400 脉冲/圈
//    - 位置模式 raF：0 = 相对运动（相对上一输入目标），1 = 绝对运动
//    - 位置到达窗口 prw：单位 0.1°（例如 8 = 0.8°）
//
// 5. 回零参数（0x4C 0xAE）
//    o_mode：0 单圈就近 / 1 单圈方向 / 2 多圈无限位碰撞 / 3 多圈有限位开关
//    o_dir ：0 CW / 1 CCW
//    o_vel ：RPM
//    o_tm  ：ms
//    sl_vel：碰撞检测转速 RPM
//    sl_ma ：碰撞检测电流 mA
//    sl_ms ：碰撞检测时间 ms
//    potF  ：上电自动回零使能
//
// 6. 单位约定
//    电流 mA（0~3000），电压 mV，温度 ℃，时间 ms，心跳周期 ms
//
// 7. 到位返回帧
//    地址 + FD 9F 6B；仅当菜单 Response 设为 Reached/Both/Other 时返回。
//    默认 Response=Receive 只回“收到命令”，不返回到位帧。
//
// 8. MMCL 多机总帧
//    地址(通常 0) + 0xAA + 总长度(2B，含头尾 5B) + 子命令串 + 0x6B
//    子命令就是普通命令帧：各自地址 + 功能码 + 数据 + 0x6B
// ============================================================================


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
    case emmv5::SystemParam::BusVoltage:          return 0x24; // mV
    case emmv5::SystemParam::BusCurrent:          return 0x26; // mA
    case emmv5::SystemParam::PhaseCurrent:        return 0x27; // mA
    case emmv5::SystemParam::EncoderRaw:          return 0x29; // 编码器原始值
    case emmv5::SystemParam::PulseCount:          return 0x30; // 实时脉冲数
    case emmv5::SystemParam::EncoderLinear:       return 0x31; // 校准后编码器值，0~65535/圈
    case emmv5::SystemParam::InputPulse:          return 0x32; // 符号 + 32 位输入脉冲
    case emmv5::SystemParam::TargetPosition:      return 0x33; // 符号 + 32 位目标位置，65536/圈
    case emmv5::SystemParam::SetPosition:         return 0x34; // 符号 + 32 位实时设定位置
    case emmv5::SystemParam::Velocity:            return 0x35; // 符号 + 16 位转速，RPM
    case emmv5::SystemParam::CurrentPosition:     return 0x36; // 符号 + 32 位实时位置
    case emmv5::SystemParam::PositionError:       return 0x37; // 符号 + 32 位位置误差
    case emmv5::SystemParam::BatteryVoltage:      return 0x38; // Y42：多圈编码器电池 mV
    case emmv5::SystemParam::Temperature:         return 0x39; // Y42：温度 ℃
    case emmv5::SystemParam::StatusFlag:          return 0x3A; // 状态标志位
    case emmv5::SystemParam::OriginFlag:          return 0x3B; // 回零状态标志位
    case emmv5::SystemParam::StatusAndOriginFlag: return 0x3C; // Y42：状态 + 回零标志
    case emmv5::SystemParam::PinStatus:           return 0x3D; // Y42：引脚状态
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
 * @brief 触发编码器校准
 * @param addr 电机地址（1~255）
 * @return 帧：addr 06 45 6B
 *
 * @note 对应 OLED 菜单 Cal；要求闭环模式、未触发堵转保护，建议空载。
 *       校准过程电机会先正转一圈、再反转一圈。
 */
Frame calibrate_encoder(uint8_t addr)
{
  Frame f;
  f.append_u8(addr);
  f.append_u8(0x06);
  f.append_u8(0x45);
  f.seal();
  return f;
}

/**
 * @brief 重启电机
 * @param addr 电机地址
 * @return 帧：addr 08 97 6B
 */
Frame reset(uint8_t addr)
{
  Frame f;
  f.append_u8(addr);
  f.append_u8(0x08);
  f.append_u8(0x97);
  f.seal();
  return f;
}

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

/**
 * @brief 解除堵转保护
 * @param addr 电机地址
 * @return 帧：addr 0E 52 6B
 *
 * @note 只在已经触发堵转保护时有效；不使能堵转保护时无意义。
 */
Frame clear_stall(uint8_t addr)
{
  Frame f;
  f.append_u8(addr);
  f.append_u8(0x0E);
  f.append_u8(0x52);
  f.seal();
  return f;
}

/**
 * @brief 恢复出厂设置
 * @param addr 电机地址
 * @return 帧：addr 0F 5F 6B
 *
 * @note 执行后需要断电重新上电，并重新空载校准编码器。
 */
Frame restore_defaults(uint8_t addr)
{
  Frame f;
  f.append_u8(addr);
  f.append_u8(0x0F);
  f.append_u8(0x5F);
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
// ---------------- 原点回零命令 ----------------

/**
 * @brief 设置单圈回零的零点位置
 * @param addr  电机地址
 * @param store true=存进电机
 * @return 帧：addr 93 88 store 6B
 *
 * @note 先把电机转到想要的位置，再发本命令；只影响单圈就近/方向回零。
 */
Frame set_home_zero(uint8_t addr, bool store)
{
  Frame f;
  f.append_u8(addr);
  f.append_u8(0x93);
  f.append_u8(0x88);
  f.append_u8(to_u8(store));
  f.seal();
  return f;
}

/**
 * @brief 触发回零
 * @param addr 电机地址
 * @param mode 0=单圈就近 / 1=单圈方向 / 2=多圈无限位碰撞 / 3=多圈有限位开关
 * @return 帧：addr 9A mode 00 6B
 *
 * @note 单圈模式要求先 set_home_zero()；碰撞/限位模式使用 HomeParams 里的阈值。
 * @note sync 字节固定写 0；本工程不使用多机同步模式。
 */
Frame home(uint8_t addr, HomingMode mode)
{
  Frame f;
  f.append_u8(addr);
  f.append_u8(0x9A);
  f.append_u8(static_cast<uint8_t>(mode));
  f.append_u8(0U); // sync：固定 0
  f.seal();
  return f;
}

/**
 * @brief 强制中断并退出回零
 * @param addr 电机地址
 * @return 帧：addr 9C 48 6B
 */
Frame abort_home(uint8_t addr)
{
  Frame f;
  f.append_u8(addr);
  f.append_u8(0x9C);
  f.append_u8(0x48);
  f.seal();
  return f;
}

/**
 * @brief 读取回零参数
 * @param addr 电机地址
 * @return 帧：addr 22 6B
 *
 * @note 回复 15 字节，顺序：mode(1) dir(1) rpm(2) timeout_ms(4)
 *       collision_rpm(2) collision_ma(2) collision_ms(2) auto_home(1)，多字节大端。
 */
Frame read_home_params(uint8_t addr)
{
  Frame f;
  f.append_u8(addr);
  f.append_u8(0x22);
  f.seal();
  return f;
}

/**
 * @brief 修改回零参数
 * @param addr   电机地址
 * @param params 回零参数（单位见 HomeParams）
 * @return 帧：addr 4C AE store mode dir rpm(2) timeout_ms(4)
 *             collision_rpm(2) collision_ma(2) collision_ms(2) auto_home 6B
 *
 * @note 总帧 20 字节；阈值的判定条件是
 *       “实际转速 < collision_rpm 且 实际电流 > collision_ma 且持续时间 > collision_ms”。
 */
Frame set_home_params(uint8_t addr, const HomeParams &params)
{
  Frame f;
  f.append_u8(addr);
  f.append_u8(0x4C);
  f.append_u8(0xAE);
  f.append_u8(to_u8(params.store));
  f.append_u8(static_cast<uint8_t>(params.mode));
  f.append_u8(static_cast<uint8_t>(params.direction));
  f.append_u16(params.rpm);
  f.append_u32(params.timeout_ms);
  f.append_u16(params.collision_rpm);
  f.append_u16(params.collision_current_ma);
  f.append_u16(params.collision_ms);
  f.append_u8(to_u8(params.auto_home));
  f.seal();
  return f;
}

/**
 * @brief 读取碰撞回零返回角度（X42S/Y42）
 * @param addr 电机地址
 * @return 帧：addr 3F 6B
 */
Frame read_home_return_angle(uint8_t addr)
{
  Frame f;
  f.append_u8(addr);
  f.append_u8(0x3F);
  f.seal();
  return f;
}

/**
 * @brief 修改碰撞回零返回角度（X42S/Y42）
 * @param addr      电机地址
 * @param store     true=存进电机
 * @param tenth_deg 返回角度，单位 0.1°
 * @return 帧：addr 5C AC store angle_H angle_L 6B
 */
Frame set_home_return_angle(uint8_t addr, bool store, uint16_t tenth_deg)
{
  Frame f;
  f.append_u8(addr);
  f.append_u8(0x5C);
  f.append_u8(0xAC);
  f.append_u8(to_u8(store));
  f.append_u16(tenth_deg);
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

/**
 * @brief 定时自动返回系统参数（Y42）
 * @param addr      电机地址
 * @param param     要返回的参数
 * @param period_ms 返回周期，单位 ms
 * @return 帧：addr 11 18 code period_H period_L 6B
 */
Frame auto_report(uint8_t addr, SystemParam param, uint16_t period_ms)
{
  const uint8_t code = sys_param_code(param);

  if (code == 0U)
  {
    return Frame {};
  }

  Frame f;
  f.append_u8(addr);
  f.append_u8(0x11);
  f.append_u8(0x18);
  f.append_u8(code);
  f.append_u16(period_ms);
  f.seal();
  return f;
}


// ----------------
// ---------------- 驱动参数命令 ----------------

/**
 * @brief 修改电机 ID 地址
 * @param addr  当前地址
 * @param store true=存进电机
 * @param id    新地址，1~255（0 为广播，不能作为本机地址）
 * @return 帧：addr AE 4B store id 6B
 */
Frame set_id(uint8_t addr, bool store, uint8_t id)
{
  Frame f;
  f.append_u8(addr);
  f.append_u8(0xAE);
  f.append_u8(0x4B);
  f.append_u8(to_u8(store));
  f.append_u8(id);
  f.seal();
  return f;
}

/**
 * @brief 修改细分值
 * @param addr      电机地址
 * @param store     true=存进电机
 * @param microstep 1~255；0 表示 256 细分
 * @return 帧：addr 84 8A store microstep 6B
 *
 * @note 16 细分、1.8° 电机：200 * 16 = 3200 脉冲/圈。
 */
Frame set_microstep(uint8_t addr, bool store, uint8_t microstep)
{
  Frame f;
  f.append_u8(addr);
  f.append_u8(0x84);
  f.append_u8(0x8A);
  f.append_u8(to_u8(store));
  f.append_u8(microstep);
  f.seal();
  return f;
}

/**
 * @brief 修改掉电标志
 * @param addr       电机地址
 * @param power_down 掉电标志
 * @return 帧：addr 50 power_down 6B
 */
Frame set_power_down_flag(uint8_t addr, bool power_down)
{
  Frame f;
  f.append_u8(addr);
  f.append_u8(0x50);
  f.append_u8(to_u8(power_down));
  f.seal();
  return f;
}

/**
 * @brief 读取选项参数状态（Y42）
 * @param addr 电机地址
 * @return 帧：addr 1A 6B
 */
Frame read_option_status(uint8_t addr)
{
  Frame f;
  f.append_u8(addr);
  f.append_u8(0x1A);
  f.seal();
  return f;
}

/**
 * @brief 修改电机类型（Y42）
 * @param addr  电机地址
 * @param store true=存进电机
 * @param type  25=1.8° 电机，50=0.9° 电机（枚举值即协议值）
 * @return 帧：addr D7 35 store type 6B
 *
 * @note 说明书 6.3.5 配置参数示例：`0x19 = 25 = 1.8°电机（0.9°电机为50）`。
 */
Frame set_motor_type(uint8_t addr, bool store, MotorType type)
{
  Frame f;
  f.append_u8(addr);
  f.append_u8(0xD7);
  f.append_u8(0x35);
  f.append_u8(to_u8(store));
  f.append_u8(static_cast<uint8_t>(type)); // 25 = 1.8°，50 = 0.9°（枚举值即协议值）
  f.seal();
  return f;
}

/**
 * @brief 修改固件类型（Y42）
 * @param addr  电机地址
 * @param store true=存进电机
 * @param type  0=X 固件，1=Emm 固件
 * @return 帧：addr D5 69 store type 6B
 */
Frame set_firmware_type(uint8_t addr, bool store, FirmwareType type)
{
  Frame f;
  f.append_u8(addr);
  f.append_u8(0xD5);
  f.append_u8(0x69);
  f.append_u8(to_u8(store));
  f.append_u8(static_cast<uint8_t>(type));
  f.seal();
  return f;
}

/**
 * @brief 修改开环/闭环控制模式（Y42）
 * @param addr  电机地址
 * @param store true=存进电机
 * @param mode  0=关闭，1=开环，2=闭环 FOC，3=脉冲方向
 * @return 帧：addr 46 69 store mode 6B
 */
Frame set_control_mode(uint8_t addr, bool store, ControlMode mode)
{
  Frame f;
  f.append_u8(addr);
  f.append_u8(0x46);
  f.append_u8(0x69);
  f.append_u8(to_u8(store));
  f.append_u8(static_cast<uint8_t>(mode));
  f.seal();
  return f;
}

/**
 * @brief 修改电机运动正方向（Y42）
 * @param addr      电机地址
 * @param store     true=存进电机
 * @param direction 0=CW，1=CCW
 * @return 帧：addr D4 60 store direction 6B
 */
Frame set_motor_direction(uint8_t addr, bool store, Direction direction)
{
  Frame f;
  f.append_u8(addr);
  f.append_u8(0xD4);
  f.append_u8(0x60);
  f.append_u8(to_u8(store));
  f.append_u8(static_cast<uint8_t>(direction));
  f.seal();
  return f;
}

/**
 * @brief 修改锁定按键功能（Y42）
 * @param addr  电机地址
 * @param store true=存进电机
 * @param lock  true=锁定按键
 * @return 帧：addr D0 B3 store lock 6B
 */
Frame set_button_lock(uint8_t addr, bool store, bool lock)
{
  Frame f;
  f.append_u8(addr);
  f.append_u8(0xD0);
  f.append_u8(0xB3);
  f.append_u8(to_u8(store));
  f.append_u8(to_u8(lock));
  f.seal();
  return f;
}

/**
 * @brief 修改通讯输入速度是否缩小 10 倍（Y42）
 * @param addr         电机地址
 * @param store        true=存进电机
 * @param enable_scale true=输入速度缩小 10 倍（可输入 0.1 RPM 精度）
 * @return 帧：addr 4F 71 store enable_scale 6B
 */
Frame set_speed_scale(uint8_t addr, bool store, bool enable_scale)
{
  Frame f;
  f.append_u8(addr);
  f.append_u8(0x4F);
  f.append_u8(0x71);
  f.append_u8(to_u8(store));
  f.append_u8(to_u8(enable_scale));
  f.seal();
  return f;
}

/**
 * @brief 修改开环模式工作电流
 * @param addr  电机地址
 * @param store true=存进电机
 * @param ma    电流，0~3000 mA（默认 1000 mA）
 * @return 帧：addr 44 33 store ma_H ma_L 6B
 */
Frame set_open_loop_current(uint8_t addr, bool store, uint16_t ma)
{
  Frame f;
  f.append_u8(addr);
  f.append_u8(0x44);
  f.append_u8(0x33);
  f.append_u8(to_u8(store));
  f.append_u16(ma);
  f.seal();
  return f;
}

/**
 * @brief 修改闭环模式堵转最大电流
 * @param addr  电机地址
 * @param store true=存进电机
 * @param ma    电流，0~3000 mA（默认 3000 mA）
 * @return 帧：addr 45 66 store ma_H ma_L 6B
 */
Frame set_closed_loop_current(uint8_t addr, bool store, uint16_t ma)
{
  Frame f;
  f.append_u8(addr);
  f.append_u8(0x45);
  f.append_u8(0x66);
  f.append_u8(to_u8(store));
  f.append_u16(ma);
  f.seal();
  return f;
}

/**
 * @brief 读取位置环 PID 参数
 * @param addr 电机地址
 * @return 帧：addr 21 6B
 *
 * @note 回复 12 字节：Kp(4) + Ki(4) + Kd(4)，均为无符号 32 位大端。
 */
Frame read_pid(uint8_t addr)
{
  Frame f;
  f.append_u8(addr);
  f.append_u8(0x21);
  f.seal();
  return f;
}

/**
 * @brief 修改位置环 PID 参数
 * @param addr   电机地址
 * @param params Kp/Ki/Kd（各 32 位）与存储标志
 * @return 帧：addr 4A C3 store kp(4) ki(4) kd(4) 6B
 *
 * @note 说明书示例：Kp=62000、Ki=100、Kd=62000。
 */
Frame set_pid(uint8_t addr, const PidParams &params)
{
  Frame f;
  f.append_u8(addr);
  f.append_u8(0x4A);
  f.append_u8(0xC3);
  f.append_u8(to_u8(params.store));
  f.append_u32(params.kp);
  f.append_u32(params.ki);
  f.append_u32(params.kd);
  f.seal();
  return f;
}

/**
 * @brief 读取 DMX512 参数（Y42）
 * @param addr 电机地址
 * @return 帧：addr 49 78 6B
 */
Frame read_dmx512(uint8_t addr)
{
  Frame f;
  f.append_u8(addr);
  f.append_u8(0x49);
  f.append_u8(0x78);
  f.seal();
  return f;
}

/**
 * @brief 修改 DMX512 参数（Y42）
 * @param addr   电机地址
 * @param params 通道数、每电机通道数、模式、速度/加速度/步长
 * @return 帧：addr D9 90 store tch(2) nch(1) mode(1) rpm(2) acc(2) vel_step(2) pos_step(4) 6B
 */
Frame set_dmx512(uint8_t addr, const Dmx512Params &params)
{
  Frame f;
  f.append_u8(addr);
  f.append_u8(0xD9);
  f.append_u8(0x90);
  f.append_u8(to_u8(params.store));
  f.append_u16(params.total_channels);
  f.append_u8(params.channels_per_motor);
  f.append_u8(static_cast<uint8_t>(params.mode));
  f.append_u16(params.rpm);
  f.append_u16(params.accel);
  f.append_u16(params.velocity_step);
  f.append_u32(params.position_step);
  f.seal();
  return f;
}

/**
 * @brief 读取位置到达窗口（Y42）
 * @param addr 电机地址
 * @return 帧：addr 41 6B
 */
Frame read_position_window(uint8_t addr)
{
  Frame f;
  f.append_u8(addr);
  f.append_u8(0x41);
  f.seal();
  return f;
}

/**
 * @brief 修改位置到达窗口（Y42）
 * @param addr   电机地址
 * @param store  true=存进电机
 * @param window 窗口值，单位 0.1°（1=0.1°，8=0.8°）
 * @return 帧：addr D1 07 store window_H window_L 6B
 *
 * @note 目标位置与实时位置之差小于窗口时置位到位标志。
 */
Frame set_position_window(uint8_t addr, bool store, uint16_t window)
{
  Frame f;
  f.append_u8(addr);
  f.append_u8(0xD1);
  f.append_u8(0x07);
  f.append_u8(to_u8(store));
  f.append_u16(window);
  f.seal();
  return f;
}

/**
 * @brief 读取过热过流保护阈值（Y42）
 * @param addr 电机地址
 * @return 帧：addr 13 6B
 */
Frame read_protection(uint8_t addr)
{
  Frame f;
  f.append_u8(addr);
  f.append_u8(0x13);
  f.seal();
  return f;
}

/**
 * @brief 修改过热过流保护阈值（Y42）
 * @param addr   电机地址
 * @param params 温度 ℃、电流 mA、检测时间 ms、存储标志
 * @return 帧：addr D3 56 store temp(2) current(2) time(2) 6B
 */
Frame set_protection(uint8_t addr, const ProtectionParams &params)
{
  Frame f;
  f.append_u8(addr);
  f.append_u8(0xD3);
  f.append_u8(0x56);
  f.append_u8(to_u8(params.store));
  f.append_u16(params.over_temp_c);
  f.append_u16(params.over_current_ma);
  f.append_u16(params.detect_ms);
  f.seal();
  return f;
}

/**
 * @brief 读取心跳保护时间（Y42）
 * @param addr 电机地址
 * @return 帧：addr 16 6B
 */
Frame read_heartbeat(uint8_t addr)
{
  Frame f;
  f.append_u8(addr);
  f.append_u8(0x16);
  f.seal();
  return f;
}

/**
 * @brief 修改心跳保护时间（Y42）
 * @param addr      电机地址
 * @param store     true=存进电机
 * @param period_ms 心跳超时时间，单位 ms
 * @return 帧：addr 68 38 store period(4) 6B
 *
 * @note 使能后，主机超过该时间不发命令，电机将停止（防止通讯断开后飞车）。
 */
Frame set_heartbeat(uint8_t addr, bool store, uint32_t period_ms)
{
  Frame f;
  f.append_u8(addr);
  f.append_u8(0x68);
  f.append_u8(0x38);
  f.append_u8(to_u8(store));
  f.append_u32(period_ms);
  f.seal();
  return f;
}

/**
 * @brief 读取积分限幅/刚性系数（Y42）
 * @param addr 电机地址
 * @return 帧：addr 23 6B
 */
Frame read_integral_limit(uint8_t addr)
{
  Frame f;
  f.append_u8(addr);
  f.append_u8(0x23);
  f.seal();
  return f;
}

/**
 * @brief 修改积分限幅/刚性系数（Y42）
 * @param addr  电机地址
 * @param store true=存进电机
 * @param limit 积分限幅值
 * @return 帧：addr 4B 57 store limit(4) 6B
 */
Frame set_integral_limit(uint8_t addr, bool store, uint32_t limit)
{
  Frame f;
  f.append_u8(addr);
  f.append_u8(0x4B);
  f.append_u8(0x57);
  f.append_u8(to_u8(store));
  f.append_u32(limit);
  f.seal();
  return f;
}

/**
 * @brief 读取系统状态参数（整包）
 * @param addr 电机地址
 * @return 帧：addr 43 7A 6B
 *
 * @note 回复 31 字节，包含电压/电流/编码器/目标位置/转速/位置/误差/状态标志。
 */
Frame read_all_state(uint8_t addr)
{
  Frame f;
  f.append_u8(addr);
  f.append_u8(0x43);
  f.append_u8(0x7A);
  f.seal();
  return f;
}

/**
 * @brief 读取驱动配置参数（整包）
 * @param addr 电机地址
 * @return 帧：addr 42 6C 6B
 *
 * @note 回复 33 字节、21 个配置参数（电机类型/细分/电流/波特率/ID 等）。
 */
Frame read_all_config(uint8_t addr)
{
  Frame f;
  f.append_u8(addr);
  f.append_u8(0x42);
  f.append_u8(0x6C);
  f.seal();
  return f;
}

// ----------------

} // namespace emmv5
