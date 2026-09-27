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
 * @note 多任务并发写同一串口是安全的（TX 启动由 _tx_lock 串行化）。
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
    uint32_t error_cnt;    ///< 错误回调/重装失败触发恢复的次数
  };

  ///< TX 诊断计数（只增不减，正常应恒为 0）
  struct TxDiag
  {
    uint32_t start_fail_cnt;    ///< HAL_UART_Transmit_DMA 启动失败次数（任务与 ISR 合计）
    uint32_t isr_skip_cnt;      ///< ISR 因通道不空闲而跳过续传的次数
    uint32_t stall_recover_cnt; ///< 巡检判定断链并成功恢复的次数
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
   * @param timeout 单位 ticks（默认 portMAX_DELAY 会一直等到全部入队）
   * @param written 实际写入的字节数（可为 nullptr）
   * @return Status OK=全部入队，TIMEOUT=部分入队，BAD_ARG=非法或超长，IO_ERROR=未初始化
   */
  Status send(const uint8_t *data, size_t size, uint32_t timeout = portMAX_DELAY, size_t *written = nullptr);

  ///< 格式化输出（printf 风格，超长自动截断，用户需要做缓冲区等等处理；须在任务上下文调用）
  Status printf(const char *fmt, ...);

  /**
   * @brief 从接收流缓冲区读取数据
   *
   * @param buffer 接收数据的缓冲区
   * @param size 请求读取的数据大小
   * @param timeout 超时时间（ms）
   * @param received 实际读取的字节数（可为 nullptr）
   * @return Status OK=读到数据，TIMEOUT=超时或无数据，
   *                BAD_ARG=参数非法，IO_ERROR=缓冲区未创建
   */
  Status receive(uint8_t *buffer, size_t size, uint32_t timeout = portMAX_DELAY, size_t *received = nullptr);

  /**
   * @brief TX 断链兜底：若有数据待发而通道空闲，则通过这个检查是否存在
   *
   * @return true=本次确实救回一次（原先断链且成功启动）
   * @note 非阻塞（取锁等待为 0），可安全地在周期任务中调用。
   *       与 send()/ISR 共用同一套仲裁，不会造成双启动。
   */
  bool tx_recover();

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
   * @brief UART 错误恢复（供 HAL_UART_ErrorCallback 调用）
   *
   * @note 用【多点一致性校验】代替单点 `RxState == BUSY_RX` 判断：
   *       只有三处证据一致地表明"RX 仍在运行"时才认定为非阻塞错误、不打断；
   *       否则认为 RX 已停摆，执行「复位 → 重新武装」。全程只操作 RX。
   */
  void handle_dma_error();

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
  volatile uint32_t _tx_start_fail_cnt = 0; ///< TX 启动失败次数（任务与 ISR 合计）
  volatile uint32_t _tx_isr_skip_cnt   = 0; ///< ISR 因通道不空闲而跳过续传的次数
  volatile uint32_t _tx_stall_rec_cnt  = 0; ///< 巡检判定断链并成功恢复的次数

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
