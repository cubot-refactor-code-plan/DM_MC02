/**
 * @file bsp_can.hpp
 * @author Rh
 * @brief CAN2.0 标准帧 8 长度 收发驱动（BSP 纯硬件层）
 * @version 0.6
 * @date 2026-10-01
 *
 * @todo 1. 只支持经典 CAN（8 字节）标准帧；超长帧（FD / 非法 DLC）一律丢弃
 *       2. 滤波器为「全部标准 ID 收进 FIFO0」，未做 ID 级过滤
 *
 * @copyright Copyright (c) 2026
 *
 * @details 本层只做三件事：硬件收发、总线恢复、诊断计数。
 *          它不认识任何「节点」概念：取出来的帧直接交给上层的设备类，
 *          由设备类自己判断这帧属不属于自己。
 *
 * @note 先在 bsp_cfg.cpp 实例化并外部声明，再在 bsp_init() 中初始化：
 *
 *      BspCan bsp_can1({&hfdcan1, "CAN1"}); // 句柄 + 调试名
 *      bsp_can1.init();
 *
 * @note 收发：
 *
 *      uint8_t data[8] = {0x01, 0x02, ...};
 *      bsp_can1.send(0x101, data);   // 入发送缓冲区，由中断自动送到硬件
 *
 *      CanRxMsg rx = {};
 *      bsp_can1.receive(&rx, 0);     // 0=不等待，portMAX_DELAY=一直等
 *
 * @note 同一条总线的接收缓冲只允许存在一个消费者：上层必须保证每条总线
 *       只有一个任务调用 receive()，多个消费者会互相抢帧。
 *
 * @note 总线恢复：tx_recover() / service_recovery() 由 sys_task 周期调用（非阻塞）。
 *       运行状态用公开成员 diagnostics 查询。
 */

#ifndef __BSP_CAN_HPP__
#define __BSP_CAN_HPP__

#include "FreeRTOS.h" // IWYU pragma: keep
#include "fdcan.h"    // IWYU pragma: keep
#include "message_buffer.h"
#include "semphr.h"   // IWYU pragma: keep（_tx_lock）
#include "status.hpp" // 统一状态码
#include "task.h"     // IWYU pragma: keep（TickType_t / ISR 入口）


/**
 * @brief CAN接收消息结构体（固定8字节数据）
 */
typedef struct
{
  FDCAN_RxHeaderTypeDef header;
  uint8_t               data[8];
} CanRxMsg;

/**
 * @brief CAN发送消息结构体（固定8字节数据）
 */
typedef struct
{
  uint32_t std_id;
  uint8_t  data[8];
} CanTxMsg;


/**
 * @brief CAN驱动类（纯硬件：收发 + 恢复 + 诊断）
 *
 * @note CAN2.0标准帧，固定8字节数据
 * @note 收发使用FreeRTOS Message Buffer
 * @note 发送不等待硬件FIFO，直接放入缓冲区，由中断完成发送
 */
class BspCan
{
public:
  // ---------------- 配置 ----------------

  /**
   * @brief CAN 配置结构体（可匿名按序传入）
   */
  struct Config
  {
    /**
     * @brief 按序构造配置（参数顺序 = 字段顺序）
     */
    Config(FDCAN_HandleTypeDef *hfdcan = nullptr, const char *name = "CAN") : hfdcan(hfdcan), name(name)
    {
    }

    FDCAN_HandleTypeDef *hfdcan; ///< CAN 句柄
    const char          *name;   ///< 实例名称（调试用）
  };

  /**
   * @brief 运行诊断计数（ISR 与任务都会更新，任务侧只读）
   *
   * @note 计数只增不减；除 bus_off_events 外，正常应恒为 0。
   */
  struct Diagnostics
  {
    // 总线错误
    volatile uint32_t bus_off_events;     ///< 进入/退出 Bus-Off 的次数（IR.BO 状态变化，进出各计一次）
    volatile uint32_t err_passive;        ///< 进入错误被动的次数
    volatile uint32_t err_warning;        ///< 进入错误警告的次数
    volatile uint32_t recovery_attempts;  ///< service_recovery() 实际动作次数
    volatile uint32_t recovery_successes; ///< service_recovery() 恢复成功次数
    volatile uint32_t recovery_max_ticks; ///< 单次恢复最长耗时（ticks）
    volatile bool     recovering;         ///< 当前是否处于恢复流程中

    // 接收
    volatile uint32_t rx_dropped;  ///< 软件接收缓冲区满而丢弃的帧数
    volatile uint32_t rx_lost;     ///< 硬件 FIFO 溢出而丢弃的帧数
    volatile uint32_t rx_len_drop; ///< 数据长度超过 8 字节而被丢弃的帧数

    // 发送
    volatile uint32_t tx_dropped;       ///< 写入硬件 TX FIFO 失败次数（任务与 ISR 合计）
    volatile uint32_t tx_buf_full;      ///< 软件发送缓冲区满而丢弃的帧数
    volatile uint32_t tx_stall_recover; ///< 丢唤醒后由 tx_recover() 补发的次数
    volatile uint32_t tx_it_fail;       ///< 开关 TX-FIFO-EMPTY 中断失败（HAL_BUSY）的次数
  };

  // ----------------
  // ---------------- 公有接口 ----------------

  /**
   * @brief 构造函数
   * @param cfg CAN 配置（句柄/调试名，可匿名按序传入）
   */
  BspCan(const Config &cfg);
  ~BspCan();

  /**
   * @brief 初始化：复位外设、创建收发消息缓冲区与 TX 锁、配置滤波器并启动
   *
   * @return Status OK=初始化成功，BAD_ARG=句柄为空，IO_ERROR=资源创建/硬件启动失败
   *
   * @note 可重复调用：开头先复位硬件的软件资源，任一步失败都回滚，不留半初始化状态。
   */
  Status init();

  /**
   * @brief 硬件是否可用（构造时收到了非空句柄）
   * @return true=可以使用收发接口；false=句柄为空，所有收发都会失败
   * @note 只反映句柄有效性，不代表 init() 成功；init() 的结果看返回值。
   */
  bool is_ready() const;

  /**
   * @brief 发送一帧 CAN 标准帧（8 字节，非阻塞，入缓冲后由中断发送）
   *
   * @param std_id 标准帧 ID（11 位，0x000~0x7FF）
   * @param data   8 字节数据
   * @return Status OK=已入发送缓冲，FULL=发送缓冲满，BUSY=总线不可用（Bus-Off / 恢复中），
   *                BAD_ARG=data 为空或 std_id 超出 11 位，NOT_INIT=未初始化
   */
  Status send(uint32_t std_id, const uint8_t *data);

  /**
   * @brief 从接收缓冲取出一帧消息
   *
   * @param msg        接收消息结构体（CanRxMsg，固定 8 字节）
   * @param timeout_ms 超时时间（ms，0=不等待，portMAX_DELAY=一直等）
   * @return Status OK=成功取出一帧，TIMEOUT=超时无数据，
   *                BAD_ARG=参数非法，NOT_INIT=未初始化
   */
  Status receive(CanRxMsg *msg, uint32_t timeout_ms = portMAX_DELAY);

  /**
   * @brief TX 卡死恢复：软件缓冲有帧、硬件 FIFO 有空位，但发送中断处于关闭状态时补一次发送
   *
   * @return true=本次确实恢复了一次（原先卡死，本次成功写入硬件 FIFO）
   *
   * @note Bus-Off 期间直接返回：此时帧只能写进硬件 FIFO、发不到总线上，
   *       计入恢复次数会让诊断量虚高并堆满 FIFO；总线恢复交给 service_recovery()。
   * @note 非阻塞（取锁等待为 0），可在周期任务中调用（sys_task 10 ms）。
   */
  bool tx_recover();

  /**
   * @brief 总线恢复：Bus-Off 时按寄存器流程重启收发（非阻塞）
   *
   * @return Status OK=总线可用（或本来就好），BUSY=仍在恢复中，NOT_INIT=未初始化
   *
   * @note 不重新初始化外设、不清 message RAM，也不动软件收发缓冲：
   *       先请硬件自行恢复（清 CCCR.INIT 后等 129×11 个隐性位），
   *       同时请求取消硬件里未发出的旧帧，避免恢复瞬间把过期控制帧发出去。
   * @note 由 sys_task 周期调用，10 ms 一次。
   */
  Status service_recovery();

  // ----------------
  // ---------------- ISR 入口 ----------------

  /**
   * @brief FIFO0 中断处理：按事件位分发 + 循环排空硬件 FIFO（ISR）
   * @param its 本次触发的中断位（RF0N / RF0L 等）
   * @param pxHigherPriorityTaskWoken 中断处理后可能唤醒的高优先级任务
   */
  void process_fifo0_isr(uint32_t its, BaseType_t *pxHigherPriorityTaskWoken);

  /**
   * @brief TX FIFO 变空续传（ISR，由 TX FIFO Empty 中断调用）
   * @param pxHigherPriorityTaskWoken 需初始化为pdFALSE，若唤醒高优先级任务则置为pdTRUE
   */
  void trigger_tx_from_isr(BaseType_t *pxHigherPriorityTaskWoken);

  /**
   * @brief 错误状态中断处理（ISR，由 HAL_FDCAN_ErrorStatusCallback 调用）
   * @param its 本次错误状态中断位（BO / EP / EW，HAL 已按 IR & IE 过滤）
   */
  void process_error_isr(uint32_t its);

  // ----------------
  // ---------------- 公开成员 ----------------
  // 仅供调试观察（Live Watch / 测试），不要随意改动。

  Diagnostics diagnostics; ///< 运行诊断计数

private:
  // ----------------
  // ---------------- 私有实现 ----------------

  FDCAN_HandleTypeDef *_hfdcan; ///< FDCAN 句柄

  MessageBufferHandle_t _tx_message_buffer; ///< 发送消息缓冲区（任务写入，ISR 读出）
  MessageBufferHandle_t _rx_message_buffer; ///< 接收消息缓冲区（ISR 写入，任务读出）

  static constexpr size_t RX_QUEUE_DEPTH = 16; ///< 接收消息缓冲区深度（帧，满即丢并计数）
  static constexpr size_t TX_QUEUE_DEPTH = 16; ///< 发送消息缓冲区深度（帧，满即丢并计数）

  const char *_name; ///< 实例名（调试用）

  SemaphoreHandle_t _tx_lock; ///< TX 启动锁：串行化「判有空位 → 取帧 → 写入硬件 FIFO」

  TickType_t _recovery_started;   ///< 本次 Bus-Off 恢复开始时刻
  TickType_t _recovery_attempted; ///< 上次尝试恢复的时刻（恢复限流用）

  /** @brief 停外设并摘除本驱动用过的全部通知（幂等，未初始化时调用也安全） */
  void _reset_hardware();

  /** @brief 配置滤波器 + 启动外设 + 打开 RX/错误状态通知（须先 _reset_hardware()） */
  Status _configure_hardware();

  /** @brief 初始化失败收尾：复位外设 + 释放软件资源 */
  void _rollback_init();

  /** @brief 开关 TX-FIFO-EMPTY 中断；返回 false = HAL 被占用（HAL_BUSY），中断状态未改变 */
  bool _set_tx_empty_it(bool enable);

  /** @brief 任务上下文：判断软件缓冲是否还有帧，在临界区内开关 TX-FIFO-EMPTY 中断 */
  void _update_tx_empty_it();

  /** @brief 尝试向硬件 TX FIFO 写入一帧；wait=0 时非阻塞 */
  bool _start_transmission(TickType_t wait = portMAX_DELAY);

  /** @brief 构造 8 字节经典帧的发送头（任务与 ISR 共用） */
  static void _fill_tx_header(FDCAN_TxHeaderTypeDef &header, const CanTxMsg &msg);

  /** @brief 释放所有已创建的 FreeRTOS 资源（消息缓冲区、锁） */
  void _cleanup_resources();

  // ----------------
};

#endif // __BSP_CAN_HPP__
