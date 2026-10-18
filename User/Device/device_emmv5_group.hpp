/**
 * @file device_emmv5_group.hpp
 * @author Rh
 * @brief 张大头 EmmV5.0 多电机组（单串口多电机 + MMCL 多机命令）
 * @version 1.1
 * @date 2026-10-18
 *
 * @copyright Copyright (c) 2026
 *
 * @details 一个实例 = 一路串口 + 若干个电机地址。它只做三件事：
 *
 *          1. 从串口读字节，按地址分发给组内各电机；
 *          2. 把多条子命令攒成 MMCL 总帧，通过广播地址一次发出；
 *          3. 汇总到位位图。
 *
 *          它不创建任务、不注册全局表、不管理信号量；周期调用 update() 即可。
 *
 * @note 适用场景：**一路串口接多台电机**。一路串口只接一台时用 DeviceEmmv5。
 *
 * @note 初始化（实例一般写在 device_cfg.cpp）：
 *
 *       DeviceEmmv5Group group({bsp_uart4, {1, 2, 3}});  // 串口 + 地址表
 *       group.init();
 *
 * @note 普通命令（直接发给某个地址）：
 *
 *       group.enable(1, true);
 *       group.set_velocity(2, emmv5::Direction::CW, 800, 50);
 *
 * @note 多机同步（MMCL）：先装子命令，再统一广播：
 *
 *       group.multi_clear();
 *       group.queue_enable(1, true, true);
 *       group.queue_enable(2, true, true);
 *       group.queue_velocity(1, emmv5::Direction::CW, 500, 50, true);
 *       group.queue_velocity(2, emmv5::Direction::CCW, 500, 50, true);
 *       group.multi_send();
 *
 * @note 周期接收（在一个任务里调用，非阻塞）：
 *
 *       uint32_t hits = group.update();              // 本次到位的电机位图
 *       uint32_t latch = group.take_in_position();   // 取走累计到位并清零
 *
 * @warning update() 只能由同一个任务调用；本类必须是这路串口唯一的接收消费者。
 * @warning MMCL 子命令总字节数最多 123（BspUart<128> 的整帧上限）；
 *          queue_* 放不下时返回 FULL，不会留下半条子命令。
 */

#ifndef __DEVICE_EMMV5_GROUP_HPP__
#define __DEVICE_EMMV5_GROUP_HPP__

#include "bsp_uart.hpp"
#include "emmv5_protocol.hpp"
#include "status.hpp"

#include <initializer_list>
#include <stddef.h>
#include <stdint.h>


/**
 * @brief 单串口多电机组
 *
 * @note 地址表在构造时一次定完，运行期不增删；地址重复或为 0 时 init() 返回 BAD_ARG。
 * @note 未 init() 时所有发送接口返回 NOT_INIT，update() 返回 0。
 */
class DeviceEmmv5Group
{
private:
  // ---------------- 内部常量 ----------------

  static constexpr size_t UART_BUFFER_SIZE  = 128U; ///< 与 bsp_cfg.cpp 里的 BspUart<128> 保持一致
  static constexpr size_t MAX_MOTORS        = 8U;   ///< 组内最大电机数
  static constexpr size_t MULTI_HEADER_SIZE = 4U;   ///< 总帧头：地址 + 0xAA + 长度(2B)

  ///< MMCL 子命令总字节上限：整帧不能超过串口发送缓冲
  static constexpr size_t MULTI_PAYLOAD_MAX = UART_BUFFER_SIZE - emmv5::MULTI_OVERHEAD;

  // ----------------
public:
  // ---------------- 类型与配置 ----------------

  using Uart = BspUart<UART_BUFFER_SIZE>;

  /**
   * @brief 组配置
   *
   * @note 地址表用初始化列表传，例如 `{bsp_uart4, {1, 2, 3}}`。
   *       超过 MAX_MOTORS 的部分不保存，init() 会返回 BAD_ARG。
   */
  struct Config
  {
    /**
     * @brief 按序构造配置
     * @param uart  本组使用的串口
     * @param addrs 电机地址表（每个地址 1~255，不能重复）
     */
    Config(Uart &uart, std::initializer_list<uint8_t> addrs) : uart(uart), count(addrs.size())
    {
      for (size_t i = 0; i < MAX_MOTORS; i++)
      {
        addr[i] = 0;
      }

      size_t i = 0;
      for (uint8_t a : addrs)
      {
        if (i < MAX_MOTORS)
        {
          addr[i++] = a;
        }
      }
    }

    Uart   &uart;             ///< bsp_uart 实例引用
    uint8_t addr[MAX_MOTORS]; ///< 地址表（前 count 个有效）
    size_t  count;            ///< 传入的地址数（可能大于 MAX_MOTORS，由 init() 拒绝）
  };

  // ----------------
  // ---------------- 构造与析构 ----------------

  /**
   * @brief 构造函数
   * @param cfg 组配置
   * @note 只做赋值；地址校验在 init() 里做。
   */
  explicit DeviceEmmv5Group(const Config &cfg);

  /** @brief 不持有需要释放的资源 */
  ~DeviceEmmv5Group() = default;

  // 内部有接收窗口与 MMCL 缓冲，复制会让两份状态互相打架
  DeviceEmmv5Group(const DeviceEmmv5Group &)            = delete;
  DeviceEmmv5Group &operator=(const DeviceEmmv5Group &) = delete;
  DeviceEmmv5Group(DeviceEmmv5Group &&)                 = delete;
  DeviceEmmv5Group &operator=(DeviceEmmv5Group &&)      = delete;

  // ----------------
  // ---------------- 生命周期与查询 ----------------

  /**
   * @brief 校验地址表并复位接收状态
   * @return OK=可用；BAD_ARG=地址表为空、超过 MAX_MOTORS、地址为 0 或地址重复
   * @note 幂等，不发送任何命令。
   */
  Status init();

  ///< 是否已成功 init
  bool is_ready() const
  {
    return _ready;
  }

  ///< 组内电机数（未 init 时返回 0）
  size_t count() const
  {
    return _ready ? _count : 0U;
  }

  // ----------------
  // ---------------- 直接命令 ----------------

  ///< 使能/关闭某个电机
  Status enable(uint8_t addr, bool on, bool sync = false);

  ///< 速度模式
  Status set_velocity(uint8_t addr, emmv5::Direction dir, uint16_t rpm, uint8_t accel, bool sync = false);

  ///< 位置模式
  Status set_position(uint8_t addr, emmv5::Direction dir, uint16_t rpm, uint8_t accel, uint32_t pulses, emmv5::PositionMode mode, bool sync = false);

  ///< 设置快速位置模式的运动参数
  Status set_quick_position_params(uint8_t addr, uint16_t rpm, uint8_t accel, emmv5::PositionMode mode, bool sync = false);

  ///< 快速位置控制
  Status set_quick_position(uint8_t addr, int32_t pulses);

  ///< 立即停止
  Status stop(uint8_t addr, bool sync = false);

  ///< 触发多机同步开始
  Status sync_start(uint8_t addr);

  ///< 广播触发所有电机的同步开始（0xFF 66）
  Status sync_start_all();

  ///< 触发回零
  Status home(uint8_t addr, emmv5::HomingMode mode, bool sync = false);

  ///< 读取一次系统参数
  Status read(uint8_t addr, emmv5::SystemParam param);

  // ----------------
  // ---------------- MMCL 多机命令 ----------------

  ///< 清空 MMCL 缓冲
  void multi_clear();

  /**
   * @brief 把 MMCL 缓冲封成一帧并发送
   * @param target 总帧地址，默认广播 0
   * @return OK=已发送并清空缓冲；NOT_INIT=未 init；TIMEOUT/IO_ERROR=发送失败（缓冲保留）
   *
   * @note 缓冲为空时直接返回 OK，不占用串口。
   */
  Status multi_send(uint8_t target = emmv5::BROADCAST);

  ///< 追加「使能」子命令
  Status queue_enable(uint8_t addr, bool on, bool sync = false);

  ///< 追加「速度模式」子命令
  Status queue_velocity(uint8_t addr, emmv5::Direction dir, uint16_t rpm, uint8_t accel, bool sync = false);

  ///< 追加「位置模式」子命令
  Status queue_position(uint8_t addr, emmv5::Direction dir, uint16_t rpm, uint8_t accel, uint32_t pulses, emmv5::PositionMode mode, bool sync = false);

  ///< 追加「快速位置参数」子命令
  Status queue_quick_position_params(uint8_t addr, uint16_t rpm, uint8_t accel, emmv5::PositionMode mode, bool sync = false);

  ///< 追加「快速位置」子命令
  Status queue_quick_position(uint8_t addr, int32_t pulses);

  ///< 追加「立即停止」子命令
  Status queue_stop(uint8_t addr, bool sync = false);

  ///< 追加「同步开始」子命令
  Status queue_sync_start(uint8_t addr);

  ///< 追加「回零」子命令
  Status queue_home(uint8_t addr, emmv5::HomingMode mode, bool sync = false);

  // ----------------
  // ---------------- 接收 ----------------

  /**
   * @brief 把串口里当前攒的字节全部读完并分发给组内电机（非阻塞）
   * @return 本次命中的电机位图：bit i = 下标 i 的电机收到到位帧
   *
   * @note 每次最多读 8 块 × 16 字节，避免数据持续到达时把调用任务卡住。
   * @note 命中结果同时锁存，可用 take_in_position() 一次性取走。
   */
  uint32_t update();

  /**
   * @brief 取走累计的到位位图并清零
   * @return bit i = 下标 i 的电机自上次取走后至少到位过一次
   */
  uint32_t take_in_position();

  /**
   * @brief 查询某个电机是否累计到位（不清零）
   * @param index 下标（0 ~ count()-1，顺序与 Config 里的地址表一致）
   * @return true=自上次 take_in_position() 后至少到位过一次
   */
  bool in_position(size_t index) const;

  // ----------------
private:
  // ---------------- 内部实现 ----------------

  /**
   * @brief 单个电机的接收状态
   */
  struct Motor
  {
    uint8_t                   addr = 0;            ///< 电机地址
    emmv5::InPositionScanner  scanner;             ///< 到位帧识别（协议层实现）
    bool                      in_position = false; ///< 累计到位标志
  };

  static constexpr size_t RX_CHUNK_SIZE = 16U; ///< 单次从流缓冲区读多少字节
  static constexpr size_t RX_MAX_CHUNKS = 8U;  ///< update() 每次最多读几块

  ///< 地址 → 下标；不在组内返回 _count
  size_t index_of(uint8_t addr) const;

  ///< 地址是否在组内
  bool _is_member(uint8_t addr) const;

  ///< 直接发送一帧（校验未初始化、地址、空帧）
  Status _send_for(uint8_t addr, const emmv5::Frame &frame);

  ///< 追加一帧到 MMCL 缓冲（校验未初始化、地址、空帧、空间）
  Status _queue_for(uint8_t addr, const emmv5::Frame &frame);

  ///< 把一块字节分发给组内所有电机
  uint32_t feed_rx(const uint8_t *data, size_t size);

  // ----------------
  // ---------------- 成员变量 ----------------

  Uart  &_uart;              ///< 被驱动的串口（不拥有）
  Motor  _motor[MAX_MOTORS]; ///< 组内电机状态
  size_t _count = 0;         ///< 组内电机数
  bool   _ready = false;     ///< init() 是否成功

  uint8_t _multi[MULTI_PAYLOAD_MAX + emmv5::MULTI_OVERHEAD] = {}; ///< MMCL 总帧缓冲
  size_t  _multi_payload = 0;                                    ///< 已装载的子命令字节数

  // ----------------
};

#endif // __DEVICE_EMMV5_GROUP_HPP__
