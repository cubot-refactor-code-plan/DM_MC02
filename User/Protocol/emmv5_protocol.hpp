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
 * @note 帧格式：地址(1) + 功能码(1~2) + 数据(0~N，多字节一律大端) + 校验(1，固定 0x6B)。
 *       建议阅读顺序：常量/枚举 → Frame → 三个 Scanner → build_multi_frame → 各命令工厂。
 *       功能码表、加速度换算公式、MMCL 布局与扫描器行为见同目录 README.md。
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
 * @brief 位置运动模式（set_position 的 raF 参数）
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
 * @brief 系统参数类型（read 的参数）
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
// ---------------- 帧 ----------------

/**
 * @brief 协议帧（值类型，可直接返回、拷贝）
 *
 * @note 只提供顺序追加接口，多字节字段按协议要求写大端。
 *       越界写入或封口后继续写入会置 `overflow`，`valid()` 随即为 false；
 *       设备层发送前会统一检查 `valid()`，不会把截断帧发上总线。
 */
struct Frame
{
  uint8_t data[FRAME_MAX_SIZE] = {}; ///< 帧内容
  uint8_t size                 = 0;  ///< 有效字节数
  bool    overflow             = false; ///< 写入超长或封口后继续写入
  bool    sealed               = false; ///< 是否已追加校验字节

  ///< 追加 1 字节；超出容量或已封口时置 overflow，不再写入
  void append_u8(uint8_t value)
  {
    if (sealed || size >= FRAME_MAX_SIZE)
    {
      overflow = true;
      return;
    }

    data[size++] = value;
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

  ///< 追加固定校验字节；已封口或已溢出时保持原状态
  void seal()
  {
    if (sealed || overflow)
    {
      return;
    }

    append_u8(CHECKSUM);
    sealed = true;
  }

  ///< 是否是可直接发送的完整帧
  bool valid() const
  {
    return sealed && !overflow && (size > 0U);
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

/**
 * @brief 0x36 实时位置应答扫描器
 *
 * @details 帧格式：`addr 36 sign v3 v2 v1 v0 6B`，共 8 字节。
 *          sign = 0 表示正，其余表示负；数值为大端。
 *
 *          串口可能把一帧拆成多块，所以扫描器内部保留 8 字节滑动窗口，跨块拼帧。
 *          每个电机地址一个扫描器；命中后窗口清空，可以继续处理同一块里的下一帧。
 *
 * @note 这是**被动解析器**，不发送任何命令。调用方先发 `read(addr, CurrentPosition)`，
 *       再把收到的字节喂给 `feed()`。
 */
struct PositionScanner
{
  uint8_t window[8] = {}; ///< 滑动窗口
  uint8_t length    = 0;  ///< 窗口当前有效字节数

  ///< 清空窗口
  void reset();

  /**
   * @brief 喂一块字节
   * @param addr     本电机的地址
   * @param data     数据
   * @param size     字节数
   * @param position 命中时写入带符号位置（65536 counts/圈）
   * @return true=本块中至少命中一帧位置应答
   */
  bool feed(uint8_t addr, const uint8_t *data, size_t size, int32_t &position);
};

/**
 * @brief 0x3A 状态标志应答扫描器
 *
 * @details 帧格式：`addr 3A status 6B`，共 4 字节。
 *          状态位定义以《Emm_V5.0 说明书》6.3.4 为准（使能/到位/堵转等）。
 *
 * @note 同样是跨块滑动窗口；每个地址一个实例。
 */
struct StatusScanner
{
  uint8_t window[4] = {}; ///< 滑动窗口
  uint8_t length    = 0;  ///< 窗口当前有效字节数

  ///< 清空窗口
  void reset();

  /**
   * @brief 喂一块字节
   * @param addr   本电机的地址
   * @param data   数据
   * @param size   字节数
   * @param status 命中时写入状态字节
   * @return true=本块中至少命中一帧状态应答
   */
  bool feed(uint8_t addr, const uint8_t *data, size_t size, uint8_t &status);
};


// ----------------
// ---------------- MMCL 总帧编码 ----------------

constexpr uint8_t MULTI_MARK        = 0xAA; ///< MMCL 总帧的功能码
constexpr size_t  MULTI_PREFIX_SIZE = 4U;   ///< 地址(1) + 功能码(1) + 长度(2)
constexpr size_t  MULTI_OVERHEAD    = MULTI_PREFIX_SIZE + 1U; ///< 再加校验字节

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

///< 将当前位置清零
Frame zero_position(uint8_t addr);


// ----------------
// ---------------- 运动控制命令 ----------------

///< 使能/关闭电机（sync 字节固定写 0，本工程不使用多机同步）
Frame enable(uint8_t addr, bool on);

/**
 * @brief 速度模式
 * @param addr  电机地址
 * @param dir   方向
 * @param rpm   速度（0~5000 RPM）
 * @param accel 加速度（0~255，0=直接启动）
 */
Frame velocity(uint8_t addr, Direction dir, uint16_t rpm, uint8_t accel);

/**
 * @brief 位置模式
 * @param addr   电机地址
 * @param dir    方向
 * @param rpm    速度（0~5000 RPM）
 * @param accel  加速度（0~255）
 * @param pulses 带符号脉冲数；绝对模式按 32 位补码写入
 * @param mode   位置模式
 */
Frame position(uint8_t addr, Direction dir, uint16_t rpm, uint8_t accel, int32_t pulses, PositionMode mode);

/**
 * @brief 设置快速位置模式的运动参数
 * @param addr  电机地址
 * @param rpm   速度（0~5000 RPM）
 * @param accel 加速度（0~255）
 * @param mode  位置模式
 */
Frame quick_position_params(uint8_t addr, uint16_t rpm, uint8_t accel, PositionMode mode);

/**
 * @brief 快速位置控制（带符号脉冲数）
 * @param addr   电机地址
 * @param pulses 带符号脉冲数，16 细分下 ±3200 = ±1 圈
 */
Frame quick_position(uint8_t addr, int32_t pulses);

/**
 * @brief 立即停止
 * @param addr 电机地址
 */
Frame stop(uint8_t addr);

// ----------------
// ---------------- 读取系统参数命令 ----------------

/**
 * @brief 读取一次系统参数
 * @param addr  电机地址
 * @param param 要读取的参数
 */
Frame read(uint8_t addr, SystemParam param);

// ----------------

} // namespace emmv5

#endif // __EMMV5_PROTOCOL_HPP__
