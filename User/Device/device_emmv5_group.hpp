/**
 * @file device_emmv5_group.hpp
 * @author Rh
 * @brief EmmV5 多电机组（单串口多电机 + MMCL 多机命令）
 * @version 1.2
 * @date 2026-10-23
 *
 * @copyright Copyright (c) 2026
 *
 * @note 一个实例 = 一路串口 + 若干个电机地址：按地址分发接收、把多条子命令攒成
 *       MMCL 总帧一次发出、汇总到位位图 / 位置 / 状态、维护每台电机的在线节点。
 *       扫描器与超时模型、MMCL 容量算术、线程与总线锁约束见 `device_emmv5_group.md`。
 *
 * @note **本类不创建任务**：字节搬运靠外部任务周期调 `update()`（建议 10 ms），
 *       与 `bsp_can` / `device_r9ds` / `dji_motor_group` 保持一致。
 *
 * @warning 同一路串口只允许一个任务调 `update()`（接收缓冲区只有一个消费者）；
 *          发送接口自带半双工总线锁，多步 MMCL 事务用 `lock_bus()` / `unlock_bus()` 包住。
 */

#ifndef __DEVICE_EMMV5_GROUP_HPP__
#define __DEVICE_EMMV5_GROUP_HPP__

#include "bsp_uart.hpp"
#include "emmv5_protocol.hpp"
#include "online_check.hpp"
#include "status.hpp"

#include "FreeRTOS.h" // IWYU pragma: keep
#include "semphr.h"   // IWYU pragma: keep（总线互斥锁）
#include "task.h"

#include <initializer_list>
#include <stddef.h>
#include <stdint.h>


/**
 * @brief 单串口多电机组
 *
 * @note 地址表在构造时一次定完，运行期不增删；地址重复或为 0 时 init() 返回 BAD_ARG。
 * @note 未 init() 时所有发送接口返回 NOT_INIT，update() 返回 0。
 */
template <typename Uart>
class DeviceEmmv5GroupT
{
private:
  // ---------------- 内部常量 ----------------

  static constexpr size_t   UART_BUFFER_SIZE  = 128U; ///< 与 bsp_cfg.cpp 的 DMA 串口发送缓冲区一致
  static constexpr size_t   MAX_MOTORS        = 8U;   ///< 组内最大电机数
  static constexpr uint16_t ONLINE_TIMEOUT_MS = 100U; ///< 超过 100 ms 无有效应答判离线

  ///< 单帧入队等待上限：921600 下 13 B ≈ 0.14 ms，20 ms 足够；缓冲卡死时返回 TIMEOUT，不无限挂住调用任务
  static constexpr uint32_t TX_TIMEOUT_MS = 20U;

  ///< MMCL 子命令总字节上限：整帧不能超过串口发送缓冲
  static constexpr size_t MULTI_PAYLOAD_MAX = UART_BUFFER_SIZE - emmv5::MULTI_OVERHEAD;

  // ----------------
public:
  // ---------------- 类型与配置 ----------------

  /**
   * @brief 组配置
   *
   * @note 地址表用初始化列表传，例如 `{uart_instance, {1, 2}}`。
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
  explicit DeviceEmmv5GroupT(const Config &cfg);

  /** @brief 释放总线锁；各电机的 Online 节点随成员析构自动注销 */
  ~DeviceEmmv5GroupT();

  // 内部有接收窗口与 MMCL 缓冲，复制会让两份状态互相打架
  DeviceEmmv5GroupT(const DeviceEmmv5GroupT &)            = delete;
  DeviceEmmv5GroupT &operator=(const DeviceEmmv5GroupT &) = delete;
  DeviceEmmv5GroupT(DeviceEmmv5GroupT &&)                 = delete;
  DeviceEmmv5GroupT &operator=(DeviceEmmv5GroupT &&)      = delete;

  // ----------------
  // ---------------- 生命周期与查询 ----------------

  /**
   * @brief 校验地址表、复位接收状态，并创建总线锁
   * @return OK=可用；BAD_ARG=地址表为空、超过 MAX_MOTORS、地址为 0 或地址重复；
   *         FULL=总线锁创建失败（堆不足）
   * @note 幂等：重复 init 不会重复创建任务。
   * @warning 应在调度器启动前调用；运行期重复 init() 会复位扫描器，可能丢掉正在接收的半帧。
   */
  Status init();

  ///< 是否已成功 init
  bool is_ready() const
  {
    return _ready;
  }

  ///< 组内电机数（未 init 时返回 0）
  size_t get_count() const
  {
    return _ready ? _count : 0U;
  }

  // ----------------
  // ---------------- 半双工总线锁 ----------------

  /**
   * @brief 占用半双工总线（递归锁）
   * @param timeout_ms 最长等待时间
   * @return OK=已持有；NOT_INIT=未 init；TIMEOUT=超时
   *
   * @note RS485 是半双工：同一时刻只允许一个“会发字节的操作”在总线上。
   *       所有发送接口内部都会自己取这把锁，所以单帧命令不需要手动加锁；
   *       只有“多条子命令拼一帧”“发完命令再等应答”这类**多步事务**需要手动包。
   * @note 递归锁：同一任务可以重复 lock_bus 而不自锁，但必须 unlock 同样次数。
   */
  Status lock_bus(uint32_t timeout_ms = 200U);

  ///< 释放总线（与 lock_bus 成对，必须同任务）
  void unlock_bus();

  // ----------------
  // ---------------- 直接命令 ----------------

  ///< 使能/关闭某个电机
  Status enable(uint8_t addr, bool on);

  ///< 速度模式（rpm 0~5000，accel 0~255）
  Status set_velocity(uint8_t addr, emmv5::Direction dir, uint16_t rpm, uint8_t accel);

  ///< 位置模式（pulses 带符号；绝对模式按 32 位补码写入）
  Status set_position(uint8_t addr, emmv5::Direction dir, uint16_t rpm, uint8_t accel, int32_t pulses, emmv5::PositionMode mode);

  ///< 取最近一次 MMCL 总帧的字节（调试用；未发送过时长度 0）
  const uint8_t *last_multi_frame(size_t &len) const;

  ///< 设置快速位置模式的运动参数
  Status set_quick_position_params(uint8_t addr, uint16_t rpm, uint8_t accel, emmv5::PositionMode mode);

  ///< 快速位置控制
  Status set_quick_position(uint8_t addr, int32_t pulses);

  ///< 立即停止某个电机
  Status stop(uint8_t addr);

  ///< 广播立即停止（地址 0）
  Status stop_all();

  ///< 将某个电机当前位置清零
  Status zero_position(uint8_t addr);

  ///< 解除某个电机的堵转保护
  Status clear_protection(uint8_t addr);

  ///< 修改控制模式（0=关闭 1=开环 2=闭环 3=脉冲/方向）
  Status set_control_mode(uint8_t addr, bool store, uint8_t mode);

  /**
   * @brief 读取一次系统参数
   * @note 本类只解析 `CurrentPosition`（0x36）与 `StatusFlag`（0x3A）；
   *       其余参数的应答会被 update() 丢弃。
   */
  Status read(uint8_t addr, emmv5::SystemParam param);

  ///< 请求一次 0x36 实时位置
  Status request_position(uint8_t addr);

  ///< 请求一次 0x3A 状态标志
  Status request_status(uint8_t addr);

  // ----------------
  // ---------------- MMCL 多机命令 ----------------

  ///< 清空 MMCL 缓冲
  void multi_clear();

  /**
   * @brief 把 MMCL 缓冲封成一帧并发送
   * @param target 总帧地址，默认广播 0
   * @return OK=已发送并清空缓冲；NOT_INIT=未 init；BAD_ARG=组帧失败；
   *         TIMEOUT/IO_ERROR=发送失败（缓冲同样清空，避免重发“半帧 + 完整帧”）
   *
   * @note 缓冲为空时直接返回 OK，不占用串口。
   */
  Status multi_send(uint8_t target = emmv5::BROADCAST);

  ///< 追加「使能」子命令
  Status queue_enable(uint8_t addr, bool on);

  ///< 追加「速度模式」子命令
  Status queue_velocity(uint8_t addr, emmv5::Direction dir, uint16_t rpm, uint8_t accel);

  ///< 追加「位置模式」子命令
  Status queue_position(uint8_t addr, emmv5::Direction dir, uint16_t rpm, uint8_t accel, int32_t pulses, emmv5::PositionMode mode);

  ///< 追加「快速位置参数」子命令
  Status queue_quick_position_params(uint8_t addr, uint16_t rpm, uint8_t accel, emmv5::PositionMode mode);

  ///< 追加「快速位置」子命令
  Status queue_quick_position(uint8_t addr, int32_t pulses);

  ///< 追加「立即停止」子命令
  Status queue_stop(uint8_t addr);

  // ----------------
  // ---------------- 接收 ----------------

  /**
   * @brief 取走累计的到位位图并清零
   * @return bit i = 下标 i 的电机自上次取走后至少到位过一次
   */
  uint32_t get_in_position();

  /**
   * @brief 查询某个电机是否累计到位（不清零）
   * @param index 下标（0 ~ count()-1，顺序与 Config 里的地址表一致）
   * @return true=自上次 get_in_position() 后至少到位过一次
   */
  bool in_position(size_t index) const;

  /**
   * @brief 取走某台电机最近一次 0x36 位置
   * @param addr 电机地址
   * @param out  位置，单位 counts（编码器一圈的计数，带符号）
   * @return true=有新数据；false=没有新数据或地址不在组内
   *
   * @note 读后清有效标志；数值与标志在短临界区内一起拷贝，允许在其他任务调用。
   */
  bool get_position(uint8_t addr, int32_t &out);

  /**
   * @brief 取走某台电机最近一次 0x3A 状态字节
   * @param addr 电机地址
   * @param out  状态字节（位定义见说明书 6.3.4）
   * @return true=有新数据；false=没有新数据或地址不在组内
   */
  bool get_status(uint8_t addr, uint8_t &out);

  /**
   * @brief 某台电机最近 100 ms 内是否收到过有效应答
   * @param addr 电机地址
   * @return true=在线；false=超时或地址不在组内
   *
   * @note 计时由 sys_task 里的 Online::update() 推进；查询周期应小于 100 ms。
   */
  bool is_online(uint8_t addr) const;

  /**
   * @brief 把串口里攒的字节读完并分发给组内电机
   * @param timeout_ms 首块等待时间：0 = 非阻塞，N = 最多等 N ms（超时即返回）
   * @return 本次命中的电机位图：bit i = 下标 i 的电机收到到位帧
   *
   * @note **必须由外部任务周期调用**，且整个组只允许一个任务调：
   *       接收缓冲区只能有一个消费者。本类不建任务、不隐藏优先级。
   * @note 只有**第一块**用 timeout_ms 等；后续块一律 0，把已到的字节一次排空。
   */
  uint32_t update(uint32_t timeout_ms = 0U);

  // ----------------
private:
  // ---------------- 内部实现 ----------------

  /**
   * @brief 单个电机的接收状态
   *
   * @note 每个 Motor 自带三个扫描器和一个 Online 节点；节点在静态构造时自动挂到
   *       Online 链表，在线计时由 sys_task 的 Online::update() 统一推进。
   */
  struct Motor
  {
    Motor() : online(ONLINE_TIMEOUT_MS)
    {
    }

    uint8_t                  addr = 0;               ///< 电机地址
    emmv5::InPositionScanner inpos_scanner;          ///< 到位帧识别
    emmv5::PositionScanner   position_scanner;       ///< 0x36 位置应答
    emmv5::StatusScanner     status_scanner;         ///< 0x3A 状态应答
    Online                   online;                 ///< 在线判定节点
    bool                     in_position    = false; ///< 累计到位标志
    int32_t                  position       = 0;     ///< 最近一次位置
    uint8_t                  status         = 0;     ///< 最近一次状态
    bool                     position_valid = false; ///< 位置是否未被取走
    bool                     status_valid   = false; ///< 状态是否未被取走
  };

  static constexpr size_t RX_CHUNK_SIZE = 16U; ///< 单次从流缓冲区读多少字节
  static constexpr size_t RX_MAX_CHUNKS = 8U;  ///< update() 每次最多读几块

  ///< rpm 合法范围（说明书：0~5000）
  static bool _valid_rpm(uint16_t rpm);

  ///< 方向枚举合法性
  static bool _valid_direction(emmv5::Direction dir);

  ///< 发布位置（短临界区）
  void _publish_position(size_t index, int32_t position);

  ///< 发布状态（短临界区）
  void _publish_status(size_t index, uint8_t status);

  ///< 设置累计到位标志（短临界区）
  void _publish_in_position(size_t index);

  ///< 地址 → 下标；不在组内返回 _count
  size_t index_of(uint8_t addr) const;

  ///< 地址是否在组内
  bool _is_member(uint8_t addr) const;

  ///< 直接发送一帧（校验未初始化、地址、帧有效性）
  Status _send_for(uint8_t addr, const emmv5::Frame &frame);

  ///< 追加一帧到 MMCL 缓冲（校验未初始化、地址、帧有效性、空间）
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
  size_t  _multi_payload                                    = 0;  ///< 已装载的子命令字节数

  uint8_t _last_frame[40] = {}; ///< 最近一次发出的 MMCL 帧（调试）
  size_t  _last_frame_len = 0;  ///< 其长度

  SemaphoreHandle_t _bus_lock = nullptr; ///< 半双工总线递归锁；init() 创建

  // ----------------
};

#endif // __DEVICE_EMMV5_GROUP_HPP__
