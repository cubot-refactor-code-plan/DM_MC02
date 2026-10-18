/**
 * @file emmv5_protocol.hpp
 * @author Rh
 * @brief 张大头 EmmV5.0 串口协议编码层（只拼帧，不碰串口）
 * @version 1.0
 * @date 2026-10-18
 *
 * @copyright Copyright (c) 2026
 *
 * @details 本文件只做一件事：把「语义参数」编码成协议手册里的字节序列。
 *          它不持有 BspUart、不创建任务、不读接收缓冲，因此可以被单机驱动
 *          （device_emmv5）和多机驱动（device_emmv5_group）同时复用。
 *
 * @note 帧格式（协议手册）：
 *
 *       | 字段     | 长度 | 说明                                      |
 *       | -------- | ---- | ----------------------------------------- |
 *       | 地址     | 1    | 1~255；0 为广播地址                        |
 *       | 功能码   | 1~2  | 命令码，部分命令带辅助码                    |
 *       | 数据     | 0~N  | 多字节字段一律**大端**                      |
 *       | 校验     | 1    | 固定 0x6B                                  |
 *
 * @note 用法：先让工厂函数生成 Frame，再交给设备层发送：
 *
 *       auto frame = emmv5::velocity(1, emmv5::Direction::CW, 1000, 50);
 *       motor.send(frame);
 *
 * @note 为什么把编码独立出来：单机与多机的区别只在「帧往哪里去」（直接发 / 塞进 MMCL
 *       缓冲），编码完全一样。分开之后，多机版本不需要把几十条命令再抄一遍。
 */

#ifndef __EMMV5_PROTOCOL_HPP__
#define __EMMV5_PROTOCOL_HPP__

#include <stddef.h>
#include <stdint.h>


namespace emmv5
{
// ---------------- 协议常量 ----------------

constexpr uint8_t  CHECKSUM = 0x6B; ///< 每条命令固定校验字节
constexpr uint8_t  BROADCAST = 0x00; ///< 广播地址
constexpr size_t   FRAME_MAX_SIZE = 32U; ///< 单条命令的最大字节数（最长约 20 字节）

constexpr uint8_t  IN_POSITION_MARK1 = 0xFD; ///< 到位返回帧第 2 字节
constexpr uint8_t  IN_POSITION_MARK2 = 0x9F; ///< 到位返回帧第 3 字节
constexpr uint8_t  IN_POSITION_MARK3 = 0x6B; ///< 到位返回帧第 4 字节


// ----------------
// ---------------- 枚举 ----------------

/**
 * @brief 运动方向
 *
 * @note 协议规定 0 = CW，其余值 = CCW；这里只保留两个合法值。
 */
enum class Direction : uint8_t
{
  CW  = 0, ///< 顺时针
  CCW = 1, ///< 逆时针
};

/**
 * @brief 位置运动模式（set_position / set_quick_position_params 的 raF 参数）
 *
 * @note EmmV5.0 固件按布尔值解释 raF：0=相对运动，1=绝对运动。
 *       部分 Y42 固件额外支持 2=相对当前实时位置，但本工程不依赖它，
 *       否则在不支持 2 的固件上会被当成“绝对运动”，表现为第二条命令不动。
 */
enum class PositionMode : uint8_t
{
  Relative = 0, ///< 相对运动（相对上一输入目标位置）
  Absolute = 1, ///< 绝对运动
};

/**
 * @brief 回零模式（home 的 o_mode 参数）
 */
enum class HomingMode : uint8_t
{
  Nearest     = 0, ///< 单圈就近回零
  Directional = 1, ///< 单圈方向回零
  Collision   = 2, ///< 多圈无限位碰撞回零
  LimitSwitch = 3, ///< 多圈有限位开关回零
};

/**
 * @brief 电机类型（set_motor_type 的参数）
 *
 * @note 枚举值就是协议里的编码值：1.8° → 25，0.9° → 50。
 *       说明书 6.3.5 的配置参数示例明确写了 `0x19 = 25 = 1.8°电机（0.9°电机为50）`。
 */
enum class MotorType : uint8_t
{
  Step1_8 = 25, ///< 1.8° 步进电机
  Step0_9 = 50, ///< 0.9° 步进电机
};

/**
 * @brief 固件类型（set_firmware_type 的参数）
 */
enum class FirmwareType : uint8_t
{
  X   = 0, ///< X 固件
  Emm = 1, ///< Emm 固件
};

/**
 * @brief 开环/闭环控制模式（set_control_mode 的参数）
 *
 * @note 说明书 6.3.5：`01 46 69 01 01 6B` = 开环，`01 46 69 01 02 6B` = 闭环 FOC；
 *       0 = 关闭，3 = 脉冲方向模式。
 */
enum class ControlMode : uint8_t
{
  Disabled      = 0, ///< 关闭
  OpenLoop      = 1, ///< 开环模式（最高约 200~400 RPM）
  ClosedLoopFoc = 2, ///< 闭环 FOC 模式（默认，最高 3000 RPM+）
  PulseDirection = 3, ///< 脉冲方向模式
};

/**
 * @brief DMX512 运动模式（Dmx512Params::mode）
 */
enum class DmxMode : uint8_t
{
  Relative = 0, ///< 相对位置
  Absolute = 1, ///< 绝对坐标
};

/**
 * @brief 系统参数类型（read / auto_report 的参数）
 *
 * @note 枚举值只是代码内的顺序，真正发到线上的功能码由 .cpp 里的查表决定。
 */
enum class SystemParam : uint8_t
{
  BusVoltage,          ///< 总线电压
  BusCurrent,          ///< 总线电流
  PhaseCurrent,        ///< 相电流
  EncoderRaw,          ///< 编码器原始值
  PulseCount,          ///< 实时脉冲数
  EncoderLinear,       ///< 线性化校准后的编码器值
  InputPulse,          ///< 输入脉冲数
  TargetPosition,      ///< 电机目标位置
  SetPosition,         ///< 电机实时设定的目标位置
  Velocity,            ///< 电机实时转速
  CurrentPosition,     ///< 电机实时位置
  PositionError,       ///< 电机位置误差
  BatteryVoltage,      ///< 多圈编码器电池电压（Y42）
  Temperature,         ///< 电机实时温度（Y42）
  StatusFlag,          ///< 电机状态标志位
  OriginFlag,          ///< 回零状态标志位
  StatusAndOriginFlag, ///< 状态标志位 + 回零状态标志位（Y42）
  PinStatus,           ///< 引脚状态（Y42）
};


// ----------------
// ---------------- 长参数结构 ----------------

/**
 * @brief 回零参数（set_home_params 用）
 *
 * @note 字段直接对应协议手册的 0x4C 0xAE 命令，单位写在字段注释里。
 */
struct HomeParams
{
  /** @brief 按序构造（参数顺序 = 字段顺序） */
  HomeParams(HomingMode mode = HomingMode::Nearest,
             Direction  direction = Direction::CW,
             uint16_t   rpm = 0,
             uint32_t   timeout_ms = 0,
             uint16_t   collision_rpm = 0,
             uint16_t   collision_current_ma = 0,
             uint16_t   collision_ms = 0,
             bool       auto_home = false,
             bool       store = false) :
    mode(mode),
    direction(direction),
    rpm(rpm),
    timeout_ms(timeout_ms),
    collision_rpm(collision_rpm),
    collision_current_ma(collision_current_ma),
    collision_ms(collision_ms),
    auto_home(auto_home),
    store(store)
  {
  }

  HomingMode mode;                 ///< 回零模式
  Direction  direction;            ///< 回零方向
  uint16_t   rpm;                  ///< 回零速度（RPM）
  uint32_t   timeout_ms;           ///< 回零超时时间（ms）
  uint16_t   collision_rpm;        ///< 碰撞回零检测转速（RPM）
  uint16_t   collision_current_ma; ///< 碰撞回零检测电流（mA）
  uint16_t   collision_ms;         ///< 碰撞回零检测时间（ms）
  bool       auto_home;            ///< 上电自动回零
  bool       store;                ///< true = 存进电机
};

/**
 * @brief PID 参数（set_pid 用）
 */
struct PidParams
{
  /** @brief 按序构造（参数顺序 = 字段顺序） */
  PidParams(uint32_t kp = 0, uint32_t ki = 0, uint32_t kd = 0, bool store = false) : kp(kp), ki(ki), kd(kd), store(store)
  {
  }

  uint32_t kp;    ///< 比例系数
  uint32_t ki;    ///< 积分系数
  uint32_t kd;    ///< 微分系数
  bool     store; ///< true = 存进电机
};

/**
 * @brief DMX512 参数（set_dmx512 用）
 */
struct Dmx512Params
{
  /** @brief 按序构造（参数顺序 = 字段顺序） */
  Dmx512Params(uint16_t total_channels = 0,
               uint8_t  channels_per_motor = 1,
               DmxMode  mode = DmxMode::Relative,
               uint16_t rpm = 0,
               uint16_t accel = 0,
               uint16_t velocity_step = 0,
               uint32_t position_step = 0,
               bool     store = false) :
    total_channels(total_channels),
    channels_per_motor(channels_per_motor),
    mode(mode),
    rpm(rpm),
    accel(accel),
    velocity_step(velocity_step),
    position_step(position_step),
    store(store)
  {
  }

  uint16_t total_channels;     ///< 总通道数
  uint8_t  channels_per_motor; ///< 每电机通道数（1=单通道，2=双通道）
  DmxMode  mode;               ///< 运动模式
  uint16_t rpm;                ///< 单通道运动速度（RPM）
  uint16_t accel;              ///< 加速度
  uint16_t velocity_step;      ///< 双通道速度步长
  uint32_t position_step;      ///< 双通道运动步长
  bool     store;              ///< true = 存进电机
};

/**
 * @brief 过热过流保护参数（set_protection 用）
 */
struct ProtectionParams
{
  /** @brief 按序构造（参数顺序 = 字段顺序） */
  ProtectionParams(uint16_t over_temp_c = 0, uint16_t over_current_ma = 0, uint16_t detect_ms = 0, bool store = false) : over_temp_c(over_temp_c), over_current_ma(over_current_ma), detect_ms(detect_ms), store(store)
  {
  }

  uint16_t over_temp_c;     ///< 过热保护阈值（℃）
  uint16_t over_current_ma; ///< 过流保护阈值（mA）
  uint16_t detect_ms;       ///< 检测时间（ms）
  bool     store;           ///< true = 存进电机
};


// ----------------
// ---------------- 帧 ----------------

/**
 * @brief 协议帧（值类型，可直接返回、拷贝）
 *
 * @note 只提供顺序追加接口，多字节字段按协议要求写大端。
 *       越界写入会被安静丢弃，但本文件的工厂函数最长只有约 20 字节，不会触发。
 */
struct Frame
{
  uint8_t data[FRAME_MAX_SIZE] = {}; ///< 帧内容
  uint8_t size                 = 0;  ///< 有效字节数

  ///< 追加 1 字节
  void append_u8(uint8_t value)
  {
    if (size < FRAME_MAX_SIZE)
    {
      data[size++] = value;
    }
  }

  ///< 追加 2 字节（大端）
  void append_u16(uint16_t value)
  {
    append_u8(static_cast<uint8_t>(value >> 8));
    append_u8(static_cast<uint8_t>(value));
  }

  ///< 追加 4 字节（大端）
  void append_u32(uint32_t value)
  {
    append_u8(static_cast<uint8_t>(value >> 24));
    append_u8(static_cast<uint8_t>(value >> 16));
    append_u8(static_cast<uint8_t>(value >> 8));
    append_u8(static_cast<uint8_t>(value));
  }

  ///< 追加固定校验字节
  void seal()
  {
    append_u8(CHECKSUM);
  }
};


// ----------------
// ---------------- 到位帧识别 ----------------

/**
 * @brief 到位帧识别器（每个电机地址一个）
 *
 * @details 到位返回帧是「地址 + 0xFD 0x9F 0x6B」四字节，可能被串口分块切开，
 *          所以用 4 字节滑动窗口跨块装配。识别逻辑属于协议层，单机/多机共用。
 */
struct InPositionScanner
{
  uint8_t window[4] = {}; ///< 滑动窗口
  uint8_t length    = 0;  ///< 窗口当前有效字节数

  ///< 清空窗口
  void reset();

  /**
   * @brief 喂一块字节
   * @param addr 本电机的地址
   * @param data 数据
   * @param size 字节数
   * @return true=本块中至少凑齐一个完整到位帧
   */
  bool feed(uint8_t addr, const uint8_t *data, size_t size);
};


// ----------------
// ---------------- MMCL 总帧编码 ----------------

constexpr uint8_t MULTI_MARK     = 0xAA; ///< MMCL 总帧的功能码
constexpr size_t  MULTI_OVERHEAD = 5U;   ///< 地址(1) + 功能码(1) + 长度(2) + 校验(1)

/**
 * @brief 把 MMCL 子命令打包成一帧总帧
 * @param target       总帧地址（通常用 BROADCAST）
 * @param payload      子命令串（各条子命令原样拼接）
 * @param payload_size 子命令字节数
 * @param out          输出缓冲
 * @param out_capacity 输出缓冲容量
 * @return 总帧长度；0=参数非法或输出缓冲放不下整帧
 *
 * @note 长度字段是**总帧**字节数（含头尾），大端。
 */
size_t build_multi_frame(uint8_t target, const uint8_t *payload, size_t payload_size, uint8_t *out, size_t out_capacity);


// ----------------
// ---------------- 触发动作命令 ----------------

///< 触发编码器校准
Frame calibrate_encoder(uint8_t addr);

///< 重启电机（Y42）
Frame reset(uint8_t addr);

///< 将当前位置清零
Frame zero_position(uint8_t addr);

///< 解除堵转保护
Frame clear_stall(uint8_t addr);

///< 恢复出厂设置
Frame restore_defaults(uint8_t addr);


// ----------------
// ---------------- 运动控制命令 ----------------

/**
 * @brief 使能/关闭电机
 * @param addr 电机地址
 * @param on   true=使能，false=关闭
 * @param sync 多机同步标志
 */
Frame enable(uint8_t addr, bool on, bool sync = false);

/**
 * @brief 速度模式
 * @param addr  电机地址
 * @param dir   方向
 * @param rpm   速度（0~5000 RPM）
 * @param accel 加速度（0~255，0=直接启动）
 * @param sync  多机同步标志
 */
Frame velocity(uint8_t addr, Direction dir, uint16_t rpm, uint8_t accel, bool sync = false);

/**
 * @brief 位置模式
 * @param addr   电机地址
 * @param dir    方向
 * @param rpm    速度（0~5000 RPM）
 * @param accel  加速度（0~255）
 * @param pulses 脉冲数
 * @param mode   位置模式
 * @param sync   多机同步标志
 */
Frame position(uint8_t addr, Direction dir, uint16_t rpm, uint8_t accel, uint32_t pulses, PositionMode mode, bool sync = false);

/**
 * @brief 设置快速位置模式的运动参数
 * @param addr  电机地址
 * @param rpm   速度（0~5000 RPM）
 * @param accel 加速度（0~255）
 * @param mode  位置模式
 * @param sync  多机同步标志
 */
Frame quick_position_params(uint8_t addr, uint16_t rpm, uint8_t accel, PositionMode mode, bool sync = false);

/**
 * @brief 快速位置控制（带符号脉冲数）
 * @param addr   电机地址
 * @param pulses 带符号脉冲数，16 细分下 ±3200 = ±1 圈
 */
Frame quick_position(uint8_t addr, int32_t pulses);

/**
 * @brief 立即停止
 * @param addr 电机地址
 * @param sync 多机同步标志
 */
Frame stop(uint8_t addr, bool sync = false);

///< 触发多机同步开始运动
Frame sync_start(uint8_t addr);


// ----------------
// ---------------- 原点回零命令 ----------------

/**
 * @brief 设置单圈回零的零点位置
 * @param addr  电机地址
 * @param store true = 存进电机
 */
Frame set_home_zero(uint8_t addr, bool store);

/**
 * @brief 触发回零
 * @param addr 电机地址
 * @param mode 回零模式
 * @param sync 多机同步标志
 */
Frame home(uint8_t addr, HomingMode mode, bool sync = false);

///< 强制中断并退出回零
Frame abort_home(uint8_t addr);

///< 读取回零参数
Frame read_home_params(uint8_t addr);

/**
 * @brief 修改回零参数
 * @param addr   电机地址
 * @param params 回零参数
 */
Frame set_home_params(uint8_t addr, const HomeParams &params);

///< 读取碰撞回零返回角度（X42S/Y42）
Frame read_home_return_angle(uint8_t addr);

/**
 * @brief 修改碰撞回零返回角度（X42S/Y42）
 * @param addr      电机地址
 * @param store     true = 存进电机
 * @param tenth_deg 返回角度，单位 0.1°
 */
Frame set_home_return_angle(uint8_t addr, bool store, uint16_t tenth_deg);


// ----------------
// ---------------- 读取系统参数命令 ----------------

/**
 * @brief 读取一次系统参数
 * @param addr  电机地址
 * @param param 要读取的参数
 */
Frame read(uint8_t addr, SystemParam param);

/**
 * @brief 定时自动返回系统参数（Y42）
 * @param addr      电机地址
 * @param param     要返回的参数
 * @param period_ms 返回周期（ms）
 */
Frame auto_report(uint8_t addr, SystemParam param, uint16_t period_ms);


// ----------------
// ---------------- 驱动参数命令 ----------------

/**
 * @brief 修改电机 ID
 * @param addr  当前地址
 * @param store true = 存进电机
 * @param id    新地址（1~255）
 */
Frame set_id(uint8_t addr, bool store, uint8_t id);

/**
 * @brief 修改细分值
 * @param addr     电机地址
 * @param store    true = 存进电机
 * @param microstep 细分值（1~255，0=256 细分）
 */
Frame set_microstep(uint8_t addr, bool store, uint8_t microstep);

///< 修改掉电标志
Frame set_power_down_flag(uint8_t addr, bool power_down);

///< 读取选项参数状态（Y42）
Frame read_option_status(uint8_t addr);

/**
 * @brief 修改电机类型（Y42）
 * @param addr  电机地址
 * @param store true = 存进电机
 * @param type  电机类型
 *
 * @note 协议里的编码值直接取自枚举：1.8° 电机填 25，0.9° 电机填 50
 *       （说明书 6.3.5 的配置参数示例：`0x19 = 25 = 1.8°电机`）。
 */
Frame set_motor_type(uint8_t addr, bool store, MotorType type);

/**
 * @brief 修改固件类型（Y42）
 * @param addr  电机地址
 * @param store true = 存进电机
 * @param type  固件类型
 */
Frame set_firmware_type(uint8_t addr, bool store, FirmwareType type);

/**
 * @brief 修改开环/闭环控制模式（Y42）
 * @param addr  电机地址
 * @param store true = 存进电机
 * @param mode  控制模式
 * @note 1=开环，2=闭环 FOC，3=脉冲方向，0=关闭（说明书 6.3.5）。
 */
Frame set_control_mode(uint8_t addr, bool store, ControlMode mode);

/**
 * @brief 修改电机运动正方向（Y42）
 * @param addr      电机地址
 * @param store     true = 存进电机
 * @param direction 正方向
 */
Frame set_motor_direction(uint8_t addr, bool store, Direction direction);

/**
 * @brief 修改锁定按键功能（Y42）
 * @param addr  电机地址
 * @param store true = 存进电机
 * @param lock  true = 锁定
 */
Frame set_button_lock(uint8_t addr, bool store, bool lock);

/**
 * @brief 修改命令速度值是否缩小 10 倍输入（Y42）
 * @param addr   电机地址
 * @param store  true = 存进电机
 * @param enable true = 缩小 10 倍
 */
Frame set_speed_scale(uint8_t addr, bool store, bool enable);

/**
 * @brief 修改开环模式工作电流
 * @param addr 电机地址
 * @param store true = 存进电机
 * @param ma   电流（mA）
 */
Frame set_open_loop_current(uint8_t addr, bool store, uint16_t ma);

/**
 * @brief 修改闭环模式最大电流
 * @param addr 电机地址
 * @param store true = 存进电机
 * @param ma   电流（mA）
 */
Frame set_closed_loop_current(uint8_t addr, bool store, uint16_t ma);

///< 读取 PID 参数
Frame read_pid(uint8_t addr);

/**
 * @brief 修改 PID 参数
 * @param addr   电机地址
 * @param params PID 参数
 */
Frame set_pid(uint8_t addr, const PidParams &params);

///< 读取 DMX512 参数（Y42）
Frame read_dmx512(uint8_t addr);

/**
 * @brief 修改 DMX512 参数（Y42）
 * @param addr   电机地址
 * @param params DMX512 参数
 */
Frame set_dmx512(uint8_t addr, const Dmx512Params &params);

///< 读取位置到达窗口（Y42）
Frame read_position_window(uint8_t addr);

/**
 * @brief 修改位置到达窗口（Y42）
 * @param addr   电机地址
 * @param store  true = 存进电机
 * @param window 窗口值（默认 8，即 0.8°）
 */
Frame set_position_window(uint8_t addr, bool store, uint16_t window);

///< 读取过热过流保护阈值（Y42）
Frame read_protection(uint8_t addr);

/**
 * @brief 修改过热过流保护阈值（Y42）
 * @param addr   电机地址
 * @param params 保护参数
 */
Frame set_protection(uint8_t addr, const ProtectionParams &params);

///< 读取心跳保护时间（Y42）
Frame read_heartbeat(uint8_t addr);

/**
 * @brief 修改心跳保护时间（Y42）
 * @param addr   电机地址
 * @param store  true = 存进电机
 * @param period_ms 心跳保护时间（ms）
 */
Frame set_heartbeat(uint8_t addr, bool store, uint32_t period_ms);

///< 读取积分限幅/刚性系数（Y42）
Frame read_integral_limit(uint8_t addr);

/**
 * @brief 修改积分限幅/刚性系数（Y42）
 * @param addr  电机地址
 * @param store true = 存进电机
 * @param limit 积分限幅值
 */
Frame set_integral_limit(uint8_t addr, bool store, uint32_t limit);

///< 读取系统状态参数
Frame read_all_state(uint8_t addr);

///< 读取驱动配置参数
Frame read_all_config(uint8_t addr);

// ----------------

} // namespace emmv5

#endif // __EMMV5_PROTOCOL_HPP__
