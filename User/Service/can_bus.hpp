/**
 * @file can_bus.hpp
 * @author Rh
 * @brief CAN 总线（Service 层）—— 节点注册、按 ID 分发、周期发送、回退缓冲
 * @version 0.1
 * @date 2026-10-01
 *
 * @copyright Copyright (c) 2026
 *
 * @details 一条 CanBus 绑定一个 BspCan（纯硬件驱动），在本层之上提供：
 *
 *          1. 接收分发：ISR 把帧放进 BspCan::_rx_message_buffer，
 *             can_rx_task 每 1 ms 取出来按 ID 分发给注册的 CanRxNode；
 *             没有节点收取的帧进入 _rx_return_buffer，由 receive() 取出。
 *          2. 发送调度：can_tx_task 每 1 ms 扫描本总线的 CanTxNode，
 *             把「槽位已填满」或「距上次发送超过 5 ms」的控制帧交给 BspCan::send()。
 *
 * @note 同一条总线的接收缓冲只允许一个消费者：分发任务读 _rx_message_buffer，
 *       receive() 读 _rx_return_buffer，两者互不争抢。
 *       应用层请使用 CanBus 的接口，不要绕过它去直接调 BspCan::receive()。
 *
 * @note 注册与发送槽位分配只允许在系统进入运行态之前完成（CanBus::frozen）。
 *
 * @note 使用示例：
 *
 *      CanBus bus_can1({&bsp_can1}); // 只绑句柄，回退缓冲在 init() 里创建
 *      bus_can1.send(0x200, data);
 */

#ifndef __CAN_BUS_HPP__
#define __CAN_BUS_HPP__

#include "bsp_can.hpp"
#include "can_rx_node.hpp"
#include "can_tx_node.hpp"


/**
 * @brief 一条 CAN 总线
 */
class CanBus
{
public:
  // ---------------- 配置 ----------------

  /**
   * @brief 总线配置（可匿名按序传入）
   */
  struct Config
  {
    /**
     * @brief 按序构造配置（参数顺序 = 字段顺序）
     */
    Config(BspCan *can = nullptr) : can(can)
    {
    }

    BspCan *can; ///< 本总线绑定的硬件驱动（须已由 bsp_init() 初始化）
  };

  // ---------------- 公有接口 ----------------

  /**
   * @brief 构造函数
   * @param cfg 总线配置（句柄，可匿名按序传入）
   */
  explicit CanBus(const Config &cfg);

  /** @brief 析构：释放回退缓冲 */
  ~CanBus();

  /**
   * @brief 初始化：创建回退缓冲
   * @return Status OK=成功；BAD_ARG=未绑定 BspCan；IO_ERROR=缓冲创建失败
   * @note 必须在 BspCan::init() 之后调用。
   */
  Status init();

  /**
   * @brief 发送一帧标准帧（转交给 BspCan::send）
   * @return 见 BspCan::send()
   */
  Status send(uint32_t std_id, const uint8_t *data);

  /**
   * @brief 取出一帧「没有被任何节点收取」的消息
   *
   * @param msg        接收消息结构体
   * @param timeout_ms 超时（ms，0=不等待，portMAX_DELAY=一直等）
   * @return Status OK=取到一帧；TIMEOUT=无数据；BAD_ARG=参数非法；NOT_INIT=未初始化
   * @note 依赖 can_rx_task 运行；每条总线只允许一个 receive() 消费任务。
   */
  Status receive(CanRxMsg *msg, uint32_t timeout_ms = 0);

  /** @brief 接收分发一轮（每轮最多 RX_BATCH_MAX 帧）；由 can_rx_task 调用 */
  void rx_poll();

  /** @brief 发送扫描一轮；由 can_tx_task 调用 */
  void tx_poll();

  // ----------------
  // ---------------- 公开成员 ----------------
  // 仅供 Service/App 层访问与调试观察，不要随意改动。

  BspCan    *_can;    ///< 绑定的硬件驱动
  CanRxNode *rx_head; ///< 本总线接收节点链表；运行期间只读
  CanTxNode *tx_head; ///< 本总线发送节点链表；运行期间只读

  MessageBufferHandle_t _rx_return_buffer; ///< 未被节点收取的帧，只由 receive() 读取

  static constexpr uint32_t BUS_NUM         = 3; ///< 总线数量
  static constexpr uint32_t RX_BATCH_MAX    = 8; ///< 每轮分发最多处理的帧数
  static constexpr size_t   RX_RETURN_DEPTH = 8; ///< 回退缓冲深度（帧）

  static CanBus *const buses[BUS_NUM]; ///< 全部总线，供任务遍历
  static bool          frozen;         ///< 运行期标志：置位后禁止再注册/注销节点

private:
  CanBus(const CanBus &)            = delete;
  CanBus &operator=(const CanBus &) = delete;
};


// ----------------
// ---------------- 函数声明 ----------------

/** @brief 初始化全部总线（在 bsp_init() 之后调用） */
Status can_bus_init(void);


// ----------------
// ---------------- 任务声明 ----------------

/** @brief 1 kHz 轮询各总线接收缓冲，按 ID 分发；未命中的帧进入回退缓冲 */
extern "C" void can_rx_task(void *argument);

/** @brief 1 kHz 扫描各总线发送节点；无节点时任务自行退出 */
extern "C" void can_tx_task(void *argument);


// ----------------
#endif // __CAN_BUS_HPP__
