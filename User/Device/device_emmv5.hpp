/**
 * @file device_emmv5.hpp
 * @author Rh
 * @brief 张大头 EmmV5.0 闭环步进电机驱动（单串口单电机）
 * @version 1.0
 * @date 2026-10-18
 *
 * @copyright Copyright (c) 2026
 *
 * @details 一个实例 = 一路串口上的一个电机地址。串口由 BSP 层初始化，本类只负责：
 *
 *          1. 调 emmv5 协议层生成命令帧；
 *          2. 把帧写进 BspUartT 的发送流缓冲区；
 *          3. 周期读取串口，识别三类应答：
 *             - 到位帧：`addr FD 9F 6B`
 *             - 位置应答：`addr 36 sign v3 v2 v1 v0 6B`
 *             - 状态应答：`addr 3A status 6B`
 *          4. 用 Online 节点记录“最近是否收到过有效应答”。
 *
 * @note `init()` 会创建本实例独有的**内部 RX 任务**（10 ms，优先级 +6）：它只做
 *       “读字节 + 喂扫描器”，命中后锁存，业务层用 `get_position()` / `get_status()` 取走并清标志。
 *       Online 阈值 **100 ms**，由 `sys_task` 里的 `Online::update()` 统一推进；
 *       应用层的查询周期应小于 100 ms，否则电机只是“没被问”也会显示离线。
 *
 * @warning 本对象必须是全局/静态生命周期；不要在 RX 任务自身里销毁对象。
 * @warning 内部 RX 任务是本路串口唯一的接收消费者；业务层不要直接调 `update()`，
 *          也不要直接读 BspUart。`get_*()` 可从任意任务调用。
 * @warning 命令帧的发送失败会通过 Status 返回；上层如果只关心控制周期，可以忽略返回值，
 *          失败计数在 BspUartT::get_tx_diag() 里查。
 *
 * @note 一路串口挂多台电机（地址不同）时改用 DeviceEmmv5GroupT；完整收发链与约束见 device_emmv5.md。
 */

#ifndef __DEVICE_EMMV5_HPP__
#define __DEVICE_EMMV5_HPP__

#include "bsp_uart.hpp"
#include "emmv5_protocol.hpp"
#include "online_check.hpp"
#include "status.hpp"

#include "FreeRTOS.h" // IWYU pragma: keep
#include "task.h"

#include <stddef.h>
#include <stdint.h>


/**
 * @brief EmmV5.0 单电机驱动（一路串口一台电机）
 *
 * @note 未 init() 时所有发送接口返回 NOT_INIT，update() 返回 false。
 */
template <typename Uart>
class DeviceEmmv5T
{
public:
  // ---------------- 类型与配置 ----------------

  /**
   * @brief 电机配置（可匿名按序传入）
   */
  struct Config
  {
    /**
     * @brief 按序构造配置
     * @param uart 电机所接的串口
     * @param addr 电机地址（1~255；0 是广播地址，不能用于实例）
     */
    Config(Uart &uart, uint8_t addr) : uart(uart), addr(addr)
    {
    }

    Uart   &uart; ///< bsp_uart 实例引用
    uint8_t addr; ///< 电机地址（1~255）
  };

  // ----------------
  // ---------------- 构造与析构 ----------------

  /**
   * @brief 构造函数
   * @param cfg 电机配置
   * @note 只做赋值；地址校验在 init() 里做。
   */
  explicit DeviceEmmv5T(const Config &cfg);

  /** @brief 停止内部 RX 任务；Online 节点随成员析构自动注销 */
  ~DeviceEmmv5T();

  // 内部有接收窗口状态，复制会让两份状态各自推进
  DeviceEmmv5T(const DeviceEmmv5T &)            = delete;
  DeviceEmmv5T &operator=(const DeviceEmmv5T &) = delete;
  DeviceEmmv5T(DeviceEmmv5T &&)                 = delete;
  DeviceEmmv5T &operator=(DeviceEmmv5T &&)      = delete;

  // ----------------
  // ---------------- 生命周期 ----------------

  /**
   * @brief 校验地址、复位接收状态，并创建内部 RX 任务
   * @return OK=可用；BAD_ARG=地址为 0；FULL=内部 RX 任务创建失败（堆/TCB 不足）
   * @note 幂等：重复调用不会重复创建任务。
   * @warning 应在调度器启动前调用；运行期重复 init() 会复位扫描器，可能丢掉正在接收的半帧。
   */
  Status init();

  ///< 是否已成功 init
  bool is_ready() const
  {
    return _ready;
  }

  ///< 电机地址
  uint8_t get_addr() const
  {
    return _addr;
  }

  // ----------------
  // ---------------- 常用命令 ----------------

  /**
   * @brief 使能/关闭电机
   * @param on true=使能
   */
  Status enable(bool on);

  /**
   * @brief 速度模式
   * @param dir   方向
   * @param rpm   速度（0~5000 RPM）
   * @param accel 加速度（0~255，0=直接启动）
   */
  Status set_velocity(emmv5::Direction dir, uint16_t rpm, uint8_t accel);

  /**
   * @brief 位置模式
   * @param dir    方向
   * @param rpm    速度（0~5000 RPM）
   * @param accel  加速度（0~255）
   * @param pulses 带符号脉冲数；绝对模式按 32 位补码写入
   * @param mode   位置模式
   */
  Status set_position(emmv5::Direction dir, uint16_t rpm, uint8_t accel, int32_t pulses, emmv5::PositionMode mode);

  /**
   * @brief 设置快速位置模式的运动参数
   * @param rpm   速度（0~5000 RPM）
   * @param accel 加速度（0~255）
   * @param mode  位置模式
   */
  Status set_quick_position_params(uint16_t rpm, uint8_t accel, emmv5::PositionMode mode);

  /**
   * @brief 快速位置控制
   * @param pulses 带符号脉冲数，16 细分下 ±3200 = ±1 圈
   */
  Status set_quick_position(int32_t pulses);

  /**
   * @brief 立即停止
   */
  Status stop();

  ///< 将当前位置清零
  Status zero_position();

  /**
   * @brief 读取一次系统参数
   * @param param 要读取的参数
   *
   * @note 本类只解析两种应答：`CurrentPosition`（0x36）和 `StatusFlag`（0x3A），
   *       其余参数的应答会被 update() 丢弃；需要时在设备层再补解析器。
   * @note 非阻塞：发完立即返回，结果用 get_position() / get_status() 取。
   */
  Status read(emmv5::SystemParam param);

  ///< 请求一次 0x36 实时位置
  Status request_position();

  ///< 请求一次 0x3A 状态标志
  Status request_status();

  // ----------------
  // ---------------- 接收 ----------------

  /**
   * @brief 取走最近一次 0x36 实时位置
   * @param out 位置，单位 counts（65536/圈，带符号）
   * @return true=有新数据；false=没有新数据
   *
   * @note 读后清有效标志；数值与标志在短临界区内一起拷贝，允许在其他任务调用。
   * @note 内部 RX 任务每 10 ms update()，控制任务按自己的节奏 get_position()。
   */
  bool get_position(int32_t &out);

  /**
   * @brief 取走最近一次 0x3A 状态字节
   * @param out 状态字节（位定义见说明书 6.3.4）
   * @return true=有新数据；false=没有新数据
   */
  bool get_status(uint8_t &out);

  /**
   * @brief 最近 100 ms 内是否收到过该电机的有效应答
   * @return true=在线；false=超时
   *
   * @note 由 Online 节点判定，计时由 sys_task 里的 Online::update() 推进。
   *       应用层应保证查询周期小于 100 ms，否则空闲时也会显示离线。
   */
  bool is_online() const;

  // ----------------
private:
  // ---------------- 私有实现 ----------------

  /**
   * @brief 把串口里当前攒的字节全部读完，并喂给三个扫描器（非阻塞）
   * @return true=本次至少识别到一个到位帧
   *
   * @note 只由内部 RX 任务调用；业务层不要直接调。
   *       每次最多读 8 块 × 16 字节，数据持续到达时也会把控制权还给其他任务。
   * @note 返回值只反映“到位帧”；0x36/0x3A 的结果通过 get_position()/get_status() 取。
   */
  bool update();

  /**
   * @brief 发送一帧协议命令（内部使用）
   * @param frame 由 emmv5 协议层生成的帧
   * @return OK=整帧已入发送缓冲；NOT_INIT=未 init；BAD_ARG=空帧/无效帧；
   *         TIMEOUT=发送缓冲 20 ms 内没腾出空间；IO_ERROR=串口未启用发送
   *
   * @note 入队超时 20 ms：短帧在 115200 下最慢也就几 ms，20 ms 足够；
   *       宁可返回 TIMEOUT 让调用方重试，也不无限挂住控制任务。
   */
  Status _send(const emmv5::Frame &frame);

  /**
   * @brief 把一块字节喂给到位/位置/状态三个扫描器（内部使用）
   * @param data 数据
   * @param size 字节数
   * @return true=本次至少凑齐一个到位帧
   */
  bool feed_rx(const uint8_t *data, size_t size);

  ///< rpm 合法范围（说明书：0~5000）
  static bool _valid_rpm(uint16_t rpm);

  ///< 方向枚举合法性
  static bool _valid_direction(emmv5::Direction dir);

  ///< 发布位置（短临界区）
  void _publish_position(int32_t position);

  ///< 发布状态（短临界区）
  void _publish_status(uint8_t status);

  /**
   * @brief 内部 RX 任务入口
   * @param argument 本对象指针（this）
   *
   * @note 每个类使用自己的静态入口函数，地址与 DeviceEmmv5TGroup 的入口不同。
   *       同类多实例共享入口地址，但各自有独立的 TCB、栈和 this 参数。
   */
  static void _rx_task_entry(void *argument);

  ///< 内部接收循环：固定周期调用 update()
  void _run_rx_loop();
  // ----------------

  // ---------------- 成员变量 ----------------

  static constexpr size_t   RX_CHUNK_SIZE     = 16U; ///< 单次从流缓冲区读多少字节
  static constexpr size_t   RX_MAX_CHUNKS     = 8U;  ///< update() 每次最多读几块
  static constexpr uint16_t ONLINE_TIMEOUT_MS = 100U; ///< 超过 100 ms 无有效应答判离线

  static constexpr uint32_t    RX_TASK_PERIOD_MS   = 10U; ///< 内部 RX 任务周期
  static constexpr UBaseType_t RX_TASK_PRIORITY    = tskIDLE_PRIORITY + 6U; ///< 低于 sys_task(+7)
  static constexpr uint32_t    RX_TASK_STACK_WORDS = 384U; ///< 384 words = 1.5 KB

  Uart    &_uart; ///< 被驱动的串口（不拥有）
  uint8_t  _addr; ///< 电机地址

  bool    _ready = false;                     ///< init() 是否成功
  emmv5::InPositionScanner _inpos_scanner;    ///< 到位帧识别
  emmv5::PositionScanner   _position_scanner; ///< 0x36 位置应答
  emmv5::StatusScanner     _status_scanner;   ///< 0x3A 状态应答

  int32_t _position       = 0;     ///< 最近一次 0x36 位置
  uint8_t _status         = 0;     ///< 最近一次 0x3A 状态
  bool    _position_valid = false; ///< 位置是否未被取走
  bool    _status_valid   = false; ///< 状态是否未被取走

  Online _online; ///< 在线判定节点（默认阈值见 ONLINE_TIMEOUT_MS）

  TaskHandle_t _task_handle = nullptr; ///< 内部 RX 任务句柄；init() 创建，析构时删除

  // ----------------
};

#endif // __DEVICE_EMMV5_HPP__
