/**
 * @file bsp_can.hpp
 * @author Rh
 * @brief CAN2.0 标准帧 8长度 收发驱动
 * @version 0.5
 * @date 2026-09-26
 *
 * @todo 1. 只支持经典 CAN（8 字节）标准帧；超长帧（FD / 非法 DLC）一律丢弃
 *       2. 滤波器为「全部标准 ID 收进 FIFO0」，未做 ID 级过滤
 *
 * @copyright Copyright (c) 2026
 *
 * @details 使用示例：
 *
 * @note 先在 bsp_cfg.cpp 实例化并外部声明，再在 all_init() → bsp_init() 中初始化
 *
 *      BspCan bsp_can1(&hfdcan1); // 只需句柄，不设 Config
 *      bsp_can1.init();
 *
 * @note 如何使用：（多任务并发调用 send() 安全：TX 启动由 _tx_lock 串行化）
 *
 *      uint8_t data[8] = {0x01,0x02...}; // 定义数据内容
 *      bsp_can1.send(0x101, data);       // 存入发送缓冲区中 bsp层自动处理发送
 *
 *      CanRxMsg data1 = {};              // 定义接收消息
 *      bsp_can1.receive(&data1, 0);      // 从接收缓冲区取值，0=不等待，portMAX_DELAY=一直等
 *
 * @note 总线异常恢复：tx_recover() / bus_recover() 由 sys_task 周期调用（非阻塞）。
 *       详细状态用 bus_status() / rx_diag() / tx_diag() 查询（Live Watch 可看）。
 *
 */


#ifndef __BSP_CAN_HPP__
#define __BSP_CAN_HPP__

#include "fdcan.h"    // IWYU pragma: keep
#include "FreeRTOS.h" // IWYU pragma: keep
#include "message_buffer.h"
#include "semphr.h"    // IWYU pragma: keep (TX 启动锁)
#include "status.hpp"  // 统一状态码
#include "task.h"


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
 * @brief CAN驱动类
 *
 * @note CAN2.0标准帧，固定8字节数据
 * @note 收发使用FreeRTOS Message Buffer
 * @note 发送不等待硬件FIFO，直接放入缓冲区，由中断完成发送
 */
class BspCan
{
public:
  // ---------------- 公有接口 ----------------

  /**
   * @brief 构造函数
   * @param hfdcan FDCAN 句柄
   */
  BspCan(FDCAN_HandleTypeDef *hfdcan);
  ~BspCan();

  /**
   * @brief 初始化：创建收发消息缓冲区、配置过滤器、启动硬件与接收中断
   *
   * @return Status OK=初始化成功，IO_ERROR=资源创建/硬件启动失败
   */
  Status init();

  /**
   * @brief 发送一帧 CAN 标准帧（8 字节，非阻塞，入缓冲后由中断发送）
   *
   * @param std_id 标准帧 ID（11 位，0x000~0x7FF）
   * @param data   8 字节数据
   * @return Status OK=已入发送缓冲，FULL=发送缓冲满，
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
   *       计入恢复次数会让诊断量虚高并堆满 FIFO；总线恢复交给 bus_recover()。
   * @note 非阻塞（取锁等待为 0），可在周期任务中调用（sys_task 10 ms）。
   */
  bool tx_recover();

  /**
   * @brief 总线恢复：处于 Bus-Off 时重启外设（停外设 → 重新配置 → 启动）
   *
   * @return true=本次确实重启了一次
   * @note 非阻塞，可在周期任务中调用（sys_task 10 ms）。
   * @note 限流：两次重启之间至少有最小间隔（见实现中的 BUS_RECOVER_MIN_GAP_MS）。
   *       M_CAN 在 CCCR.INIT=0 时本就会自行尝试恢复（等 128×11 个隐性位），
   *       所以不应每次巡检都重启，避免总线长期故障时反复抖动。
   */
  bool bus_recover();

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
  // ---------------- 查询接口 ----------------

  /** @brief RX 诊断计数（只增不减，正常应恒为 0） */
  struct RxDiag
  {
    uint32_t sw_drop_cnt;  ///< 软件消息缓冲区满而丢弃的帧数
    uint32_t lost_cnt;     ///< 硬件 FIFO 溢出而丢弃的帧数
    uint32_t len_drop_cnt; ///< 数据长度超过 8 字节而被丢弃的帧数
  };

  /** @brief TX 诊断计数（只增不减，正常应恒为 0） */
  struct TxDiag
  {
    uint32_t drop_cnt;          ///< 软件发送缓冲满而丢弃的帧数（调用方不检查返回值时即丢帧）
    uint32_t fifo_fail_cnt;     ///< 写入硬件 TX FIFO 失败次数（任务与 ISR 合计）
    uint32_t stall_recover_cnt; ///< 丢唤醒并成功恢复的次数：缓冲有帧 + FIFO 有空位 + 发送中断关闭
    uint32_t it_fail_cnt;       ///< 开关 TX-FIFO-EMPTY 中断失败（HAL_BUSY）的次数，由巡检补开
  };

  /** @brief 总线错误状态快照（HAL PSR/错误计数器 + 本驱动的累计次数） */
  struct BusStatus
  {
    bool     bus_off;           ///< 当前是否处于 Bus-Off（M_CAN 在 INIT=0 时会自行尝试恢复）
    bool     error_passive;     ///< 当前是否处于错误被动
    bool     error_warning;     ///< 当前是否处于错误警告
    uint32_t last_error_code;   ///< 最近一次错误类型（PSR.LEC）
    uint32_t tec;               ///< 发送错误计数（0~255）
    uint32_t rec;               ///< 接收错误计数（0~127）
    uint32_t bus_off_cnt;       ///< IR.BO 触发次数（Bus-Off 状态变化，进入/退出各计一次）
    uint32_t error_passive_cnt; ///< 进入错误被动次数
    uint32_t error_warning_cnt; ///< 进入错误警告次数
    uint32_t bus_rec_cnt;       ///< bus_recover() 实际重启外设的次数
  };

  /** @brief 读取 RX 诊断计数 */
  RxDiag rx_diag() const;

  /** @brief 读取 TX 诊断计数 */
  TxDiag tx_diag() const;

  /** @brief 读取总线错误状态快照 */
  BusStatus bus_status() const;

  // ----------------
private:
  // ---------------- 私有实现 ----------------

  // 成员变量

  static constexpr size_t RX_QUEUE_DEPTH = 16; ///< 接收消息缓冲区深度（帧，满即丢并计数）
  static constexpr size_t TX_QUEUE_DEPTH = 16; ///< 发送消息缓冲区深度（帧，满即丢并计数）

  FDCAN_HandleTypeDef *_hfdcan = nullptr; ///< FDCAN 句柄

  MessageBufferHandle_t _rx_message_buffer = nullptr; ///< 接收消息缓冲区
  MessageBufferHandle_t _tx_message_buffer = nullptr; ///< 发送消息缓冲区

  SemaphoreHandle_t _tx_lock = nullptr; ///< TX 启动锁：串行化「判有空位 → 取帧 → 写入硬件 FIFO」

  // RX 诊断量（ISR 中更新，任务中读取）
  volatile uint32_t _rx_sw_drop_cnt  = 0; ///< 软件缓冲满丢弃帧数
  volatile uint32_t _rx_lost_cnt     = 0; ///< 硬件 FIFO 溢出丢帧数
  volatile uint32_t _rx_len_drop_cnt = 0; ///< 数据长度超过 8 字节而丢弃的帧数

  // TX 诊断量（ISR 中更新，任务中读取）
  volatile uint32_t _tx_drop_cnt      = 0; ///< 软件发送缓冲满而丢掉的帧数
  volatile uint32_t _tx_fifo_fail_cnt = 0; ///< 写硬件 FIFO 失败次数
  volatile uint32_t _tx_stall_rec_cnt = 0; ///< 丢唤醒并成功恢复的次数
  volatile uint32_t _tx_it_fail_cnt   = 0; ///< 开关 TX-FIFO-EMPTY 中断失败的次数

  // 总线错误量（ISR 中更新，任务中读取）
  volatile uint32_t _bus_off_cnt     = 0; ///< IR.BO 触发次数（Bus-Off 状态变化）
  volatile uint32_t _err_passive_cnt = 0; ///< 进入错误被动次数
  volatile uint32_t _err_warning_cnt = 0; ///< 进入错误警告次数
  volatile uint32_t _bus_rec_cnt     = 0; ///< bus_recover() 实际重启外设的次数

  TickType_t _last_bus_rec_tick = 0; ///< 上次重启时刻，用于重启限流（见 bus_recover()）

  // 内部实现

  /** @brief 停外设并摘除本驱动用过的全部通知（幂等，未初始化时调用也安全） */
  void reset_hardware();

  /** @brief 配置滤波器 + 启动外设 + 打开 RX/错误状态通知（须先 reset_hardware()） */
  Status configure_hardware();

  /** @brief 初始化失败收尾：复位外设 + 释放软件资源 */
  void rollback_init();

  /** @brief 开关 TX-FIFO-EMPTY 中断；返回 false = HAL 被占用（HAL_BUSY），中断状态未改变 */
  bool set_tx_empty_it(bool enable);

  /** @brief 任务上下文：判断软件缓冲是否还有帧，在临界区内开关 TX-FIFO-EMPTY 中断 */
  void update_tx_empty_it();

  /** @brief 尝试向硬件 TX FIFO 写入一帧；wait=0 时非阻塞 */
  bool start_transmission(TickType_t wait = portMAX_DELAY);

  /** @brief 构造 8 字节经典帧的发送头（任务与 ISR 共用） */
  static void fill_tx_header(FDCAN_TxHeaderTypeDef &header, const CanTxMsg &msg);

  /** @brief 释放所有已创建的 FreeRTOS 资源（消息缓冲区、锁） */
  void cleanup_resources();

  // ----------------
};

#endif
