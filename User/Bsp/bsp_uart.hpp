/**
 * @file bsp_uart.hpp
 * @author Rh
 * @brief 实现了一个简易的串口驱动（FreeRTOS + IDLE中断 + DMA）
 * @version 0.3
 * @date 2026-09-10
 *
 * @todo 接收到的数据需要在应用层 app_message 中做分发处理
 *
 * @copyright Copyright (c) 2026
 *
 * @details 使用IDLE中断接收，TX Complete中断链式发送，全程DMA。
 *
 * @note 模板参数为缓冲区大小（uint8_t）
 *
 *   // 全局实例化模板在bsp_uart.cpp中
 *   template class BspUart<128>;
 *
 *   // 全局实例化类 在bsp_cfg.cpp中
 *   __attribute__((section(".dma_buffer")))
 *   BspUart<128> bsp_uart1({&huart1, true});  // huart句柄 + 是否启用发送
 *   bsp_uart1.init();                         // 放到bsp_init中初始化串口（需FreeRTOS调度器已启动）
 *
 *   bsp_uart1.send(buffer, 8);                // 存入发送流缓冲区，DMA自动发送
 *   bsp_uart1.printf("val=%d\r\n", 42);       // 格式化输出（非阻塞，DMA发送）
 *   bsp_uart1.receive(buffer, 8);             // 从接收流缓冲区读数据，读取后对应数据会被推出缓冲区
 *
 * @note 多任务并发写同一串口是安全的：send() 由 _tx_lock 串行化「判忙→取包→启转」，
 *       printf() 另由 _printf_lock 保护实例级的格式化缓冲，不会互相覆盖。
 *
 * @note 超时单位统一为【毫秒】：send()/receive() 的 timeout 都是 ms；
 *       传 portMAX_DELAY 表示「一直等」（哨兵，内部原样下传，不做 ms 换算）。
 */

#ifndef __BSP_UART_HPP__
#define __BSP_UART_HPP__

#include "FreeRTOS.h" // IWYU pragma: keep
#include "semphr.h"   // IWYU pragma: keep (TX 启动锁)
#include "stream_buffer.h"
#include "task.h"  // IWYU pragma: keep
#include "usart.h" // IWYU pragma: keep

#include "status.hpp" // 统一状态码


/**
 * @brief 简易串口驱动（IDLE中断 + DMA收发 + FreeRTOS流缓冲区）
 *
 * @tparam BUFFER_SIZE DMA收发缓冲区大小（uint8_t），也是流缓冲区容量
 *
 * @note RX：IDLE/TC 事件 → 投递流缓冲区 → 重新武装（HT 事件忽略，全程不碰 TX DMA）
 *       TX：写入流缓冲区 → 任务侧启动或 TX-Complete 中断续传
 */
template <size_t BUFFER_SIZE = 256>
class BspUart
{
public:
  // ---------------- 公有接口 ----------------

  /** @brief 串口配置（可匿名按序传入：{huart, transmit_enable}） */
  struct Config
  {
    /** @brief 按序构造（参数顺序 = 字段顺序） */
    Config(UART_HandleTypeDef *huart = nullptr, bool transmit_enable = true)

      : huart(huart),
        transmit_enable(transmit_enable)
    {
    }

    UART_HandleTypeDef *huart;           ///< UART 句柄
    bool                transmit_enable; ///< 是否启用发送
  };

  /**
   * @brief RX 健康状态（多点一致性校验的结果）
   *
   * @note 把「HAL 软件状态」与「硬件真实使能」三者交叉核对，
   *       避免出现"HAL 宣称在收、DMA 其实已停"却无人察觉的情况。
   */
  enum class RxHealth : uint8_t
  {
    OK = 0,       ///< 三者一致地"在收" → 正常
    STOPPED,      ///< 三者一致地"停" → 未武装/已停止（非故障）
    INCONSISTENT, ///< 自相矛盾 → RX 已停摆，需重建（配合 _rx_should_run 判故障）
  };

  ///< RX 诊断计数（只增不减，正常应恒为 0）
  struct RxDiag
  {
    uint32_t arm_fail_cnt; ///< 武装失败次数（含首次 init 与运行期重装）
    uint32_t drop_bytes;   ///< 流缓冲区满而丢弃的字节数（静默丢数据）
    uint32_t error_cnt;    ///< 任务侧 rx_recover() 实际重建 RX 通道的次数
  };

  ///< TX 诊断计数（只增不减，正常应恒为 0）
  struct TxDiag
  {
    uint32_t drop_bytes;        ///< 入队失败（缓冲满/超时）而丢弃的字节数
    uint32_t start_fail_cnt;    ///< HAL_UART_Transmit_DMA 启动失败次数（任务与 ISR 合计）
    uint32_t isr_skip_cnt;      ///< ISR 因通道不空闲而跳过续传的次数
    uint32_t stall_recover_cnt; ///< 巡检判定断链并成功恢复的次数
    uint32_t error_cnt;         ///< TX 侧走错误回调的次数（最终由 tx_recover 救回）
  };

  /** @brief 构造函数（只做赋值，FreeRTOS 资源创建推迟到 init()） */
  BspUart(const Config &cfg);

  /**
   * @brief 创建 FreeRTOS 对象并启动 IDLE 接收（须在调度器启动后调用）
   * @return Status OK=成功，IO_ERROR=资源创建或接收启动失败
   */
  Status init();

  ///< 析构函数 释放所有分配的资源
  ~BspUart();

  /**
   * @brief 发送数据：写入流缓冲区并启动 DMA（DMA 异步，但入队可能阻塞）
   * @param timeout_ms 入队等待时间，单位【毫秒】；portMAX_DELAY = 一直等到全部入队
   * @param written 实际写入的字节数（可为 nullptr）
   * @return Status OK=全部入队，TIMEOUT=部分入队，BAD_ARG=非法或超长，IO_ERROR=未初始化
   */
  Status send(const uint8_t *data, size_t size, uint32_t timeout_ms = portMAX_DELAY, size_t *written = nullptr);

  /**
   * @brief 格式化输出（printf 风格，超长自动截断；须在任务上下文调用）
   *
   * @note 内部用 _printf_lock 保护实例级格式化缓冲，多任务并发调用安全。
   * @note 不用 send() 默认的 portMAX_DELAY：入队等待固定为 10 ms，发送链卡住时
   *       不会把调用任务长时间挂住；超时未全部入队则返回 Status::TIMEOUT
   *       （已入队部分照常发出）。
   */
  Status printf(const char *fmt, ...);

  /**
   * @brief 从接收流缓冲区读取数据
   *
   * @note 【字节流语义】：返回的不保证是「一帧」，只保证有序、不重、不丢（除非流缓冲满）。
   *       帧边界请由协议层自行确定（长度前缀 / 定界符 / 校验和），
   *       驱动层只负责字节搬运，不做任何组帧。
   *
   * @param buffer 接收数据的缓冲区
   * @param size 请求读取的数据大小
   * @param timeout 超时时间，单位【毫秒】；portMAX_DELAY = 一直等
   * @param received 实际读取的字节数（可为 nullptr）
   * @return Status OK=读到数据，TIMEOUT=超时或无数据，
   *                BAD_ARG=参数非法，IO_ERROR=缓冲区未创建
   */
  Status receive(uint8_t *buffer, size_t size, uint32_t timeout = portMAX_DELAY, size_t *received = nullptr);

  /**
   * @brief TX 断链兜底：缓冲里有数据却「30 ms 还没发出去」时，强制重发
   *
   * @return true=本次确实救回一次（原先卡死且成功续发）
   *
   * @note 判据就一个：有数据待发 + 超过 30 ms 毫无推进。正常发送时
   *       _tx_last_activity_tick 每帧都会被刷新（一帧最多 11 ms），不会误判。
   * @note 恢复时先算清「在途那包已经发出多少」，只续发剩下的 —— 不重复也不丢失；
   *       在途字节已全部发完则走常规点火。
   * @note 非阻塞（取锁等待为 0），可安全地在周期任务中调用（sys_task 10 ms）。
   */
  bool tx_recover();

  /**
   * @brief RX 恢复：发现 RX 已停摆时，复位接收通道并重新武装
   *
   * @return true=本次确实重建了一次（诊断量见 rx_diag().error_cnt）
   *
   * @note 判据：_rx_should_run（期望收数）且 rx_health() != OK。
   *       三处证据一致表明还在收 → 直接返回，不打断在途数据（正常时每周期只看一眼）。
   * @note 必须放在任务侧：复位 RX 要调阻塞版 HAL_UART_AbortReceive()，ISR 里不能做。
   *       ISR 出错时只留痕，由本函数在 10 ms 内发现并修复（与 TX 侧 tx_recover() 对称）。
   */
  bool rx_recover();

  // ----------------

  // ISR 入口（仅供回调分发调用）

  /**
   * @brief IDLE/TC 接收完成处理（ISR）
   * @param size 接收到的数据大小
   * @param pxHigherPriorityTaskWoken 中断处理后可能唤醒的高优先级任务
   */
  void on_idle_isr(uint16_t size, BaseType_t *pxHigherPriorityTaskWoken);

  /**
   * @brief 发送完成/续传处理（ISR上下文，由 TX Complete 中断调用）
   * @param pxHigherPriorityTaskWoken 需初始化为pdFALSE，若唤醒高优先级任务则置为pdTRUE
   */
  void start_transmission_from_isr(BaseType_t *pxHigherPriorityTaskWoken);

  /**
   * @brief UART 错误回调处理（供 HAL_UART_ErrorCallback 调用，ISR 上下文）
   *
   * @note 【只留痕，不做恢复】：HAL 的 Abort 接口内部走阻塞版 HAL_DMA_Abort，
   *       ISR 里不能调用，也不在这里嵌套重试。
   *       RX 停摆由任务侧 rx_recover() 通过 rx_health() 发现并重建；
   *       TX 出错由任务侧 tx_recover() 复位通道并重新点火。
   */
  void on_error_isr();

  // ---------------- 查询接口 ----------------

  ///< 发送流缓冲区剩余空间
  size_t get_tx_free_space();

  ///< 接收流缓冲区可用数据量
  size_t get_rx_available_data();

  /**
   * @brief RX 健康查询（多点一致性校验）
   *
   * @note 核对三处独立证据：RxState==BUSY_RX + USART_CR3.DMAR + DMA_SxCR.EN
   *       只看 RxState 不够 —— 可识别"HAL 说在收、DMA 其实已停"。
   *
   * @note _rx_should_run==true 而返回非 OK → RX 意外停摆（故障）；false → 主动停止
   */
  RxHealth rx_health() const;

  ///< 读取 RX 诊断计数
  RxDiag rx_diag() const;

  ///< 读取 TX 诊断计数
  TxDiag tx_diag() const;

  // ----------------

private:
  // ---------------- 私有实现 ----------------

  // 成员变量

  UART_HandleTypeDef *_huart; ///< UART句柄指针，指向底层硬件接口

  StreamBufferHandle_t _rx_stream_buffer = nullptr; ///< 接收流缓冲区
  StreamBufferHandle_t _tx_stream_buffer = nullptr; ///< FreeRTOS发送流缓冲区句柄

  ///< TX 启动锁：串行化「判忙 → 取包 → 启转」，多任务并发写同一串口也安全
  SemaphoreHandle_t _tx_lock = nullptr;

  ///< printf 锁：保护实例级共享的 _printf_buffer（send() 不经过它，两者不嵌套死锁）
  SemaphoreHandle_t _printf_lock = nullptr;

  bool    _rx_should_run = true;       ///< 是否期望 RX 运行（意图；不因单次失败而闩死）
  bool    _transmit_enable;            ///< 是否启用发送
  uint8_t _rx_dma_buffer[BUFFER_SIZE]; ///< DMA接收缓冲区，用于多字节接收
  uint8_t _tx_dma_buffer[BUFFER_SIZE]; ///< DMA发送缓冲区，用于多字节发送
  char    _printf_buffer[BUFFER_SIZE]; ///< printf 格式化缓冲区（vsnprintf 输出到此处）

  ///< RX 诊断量（volatile：ISR 中更新，任务中读取）
  volatile uint32_t _rx_arm_fail_cnt = 0; ///< 武装失败次数
  volatile uint32_t _rx_drop_bytes   = 0; ///< 流缓冲满而丢弃的字节数
  volatile uint32_t _rx_error_cnt    = 0; ///< 错误回调/重装失败触发恢复的次数

  ///< TX 诊断量（volatile：ISR 中更新，任务中读取）
  volatile uint32_t _tx_drop_bytes    = 0; ///< 入队失败而丢弃的字节数
  volatile uint32_t _tx_start_fail_cnt = 0; ///< TX 启动失败次数（任务与 ISR 合计）
  volatile uint32_t _tx_isr_skip_cnt   = 0; ///< ISR 因通道不空闲而跳过续传的次数
  volatile uint32_t _tx_stall_rec_cnt  = 0; ///< 巡检判定断链并成功恢复的次数
  volatile uint32_t _tx_error_cnt      = 0; ///< TX 侧走错误回调的次数（收尾由 tx_recover 完成）

  ///< 最近一次成功启动 TX 的时刻，用于 tx_recover() 判「多久没发出去」
  volatile TickType_t _tx_last_activity_tick = 0;

  ///< 当前被 DMA 载入在途的字节数；配合 DMA 剩余计数，实现卡死后的续发（不丢不重）
  volatile size_t _tx_inflight_len = 0;

  // 内部实现
  /**
   * @brief 武装 DMA 接收（仅 RX，调用前 RxState 必须为 READY）
   * @note 先清残留的 UART 错误标志（ORE/FE/NE/PE），否则 HAL 会拒绝启动
   * @return true=成功；false=HAL 拒绝（BUSY/ERROR），已累加 arm_fail_cnt
   */
  bool arm_reception();

  ///< 中止 RX 通道并复位接收状态（仅 RX，不触碰 TX DMA）
  void abort_reception();

  ///< 释放所有已创建的 FreeRTOS 资源
  void cleanup_resources();

  ///< 主动尝试排空发送流缓冲区（取包 + 启动 DMA）；wait=0 时非阻塞
  bool start_transmission(TickType_t wait = portMAX_DELAY);

  // ----------------
};


#endif // __BSP_UART_HPP__
