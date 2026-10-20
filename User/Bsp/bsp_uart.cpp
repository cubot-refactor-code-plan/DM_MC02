#include "bsp_uart.hpp"
#include "bsp_cfg.hpp" // 中断回调中直接引用 bsp_usartX / bsp_uartX 全局实例
#include "FreeRTOS.h"  // IWYU pragma: keep
#include <stdarg.h>
#include <stdio.h>


// ---------------- Transport 策略 ----------------

namespace
{
/** @brief DMA 接收通道的三处状态快照 */
struct DmaRxFlags
{
  bool hal_busy;
  bool dmaren;
  bool dma_en;
};

DmaRxFlags read_dma_rx_flags(const UART_HandleTypeDef *huart)
{
  DmaRxFlags flags {};

  flags.hal_busy = (huart->RxState == HAL_UART_STATE_BUSY_RX);
  flags.dmaren   = ((huart->Instance->CR3 & USART_CR3_DMAR) != 0U);

  if (huart->hdmarx != nullptr && IS_DMA_STREAM_INSTANCE(huart->hdmarx->Instance) != 0U)
  {
    const DMA_Stream_TypeDef *stream = static_cast<DMA_Stream_TypeDef *>(huart->hdmarx->Instance);
    flags.dma_en = ((stream->CR & DMA_SxCR_EN) != 0U);
  }

  return flags;
}

/** @brief 中断接收是否已使能 RXNE 与 IDLE 中断；FIFO 模式下 HAL 改用 RXFTIE */
constexpr uint32_t IT_RX_IRQ_BITS      = USART_CR1_RXNEIE_RXFNEIE | USART_CR1_IDLEIE;
constexpr uint32_t IT_FIFO_RX_IRQ_BIT  = USART_CR3_RXFTIE;

bool it_rx_irq_enabled(const UART_HandleTypeDef *huart)
{
  // HAL 在 UART_FIFOMODE_ENABLE 且接收长度 >= 阈值时走 UART_RxISR_8BIT_FIFOEN，
  // 此时开的是 CR3.RXFTIE，不再开 CR1.RXNEIE；健康检查必须跟着切。
  // 接收长度小于阈值时 HAL 仍可能退回逐字节 RXNEIE，所以两种都认。
  if (huart->FifoMode == UART_FIFOMODE_ENABLE)
  {
    const bool fifo_irq = ((huart->Instance->CR3 & IT_FIFO_RX_IRQ_BIT) != 0U) &&
                          ((huart->Instance->CR1 & USART_CR1_IDLEIE) != 0U);
    const bool byte_irq = (huart->Instance->CR1 & IT_RX_IRQ_BITS) == IT_RX_IRQ_BITS;
    return fifo_irq || byte_irq;
  }

  return (huart->Instance->CR1 & IT_RX_IRQ_BITS) == IT_RX_IRQ_BITS;
}
}

bool uart_detail::DmaTransport::arm_rx(UART_HandleTypeDef *huart, uint8_t *buffer, uint16_t size)
{
  return HAL_UARTEx_ReceiveToIdle_DMA(huart, buffer, size) == HAL_OK;
}

bool uart_detail::DmaTransport::start_tx(UART_HandleTypeDef *huart, uint8_t *buffer, uint16_t size)
{
  return HAL_UART_Transmit_DMA(huart, buffer, size) == HAL_OK;
}

bool uart_detail::DmaTransport::rx_ok(const UART_HandleTypeDef *huart)
{
  const DmaRxFlags flags = read_dma_rx_flags(huart);
  return flags.hal_busy && flags.dmaren && flags.dma_en;
}

bool uart_detail::DmaTransport::rx_stopped(const UART_HandleTypeDef *huart)
{
  const DmaRxFlags flags = read_dma_rx_flags(huart);
  return (!flags.hal_busy) && (!flags.dmaren) && (!flags.dma_en);
}

size_t uart_detail::DmaTransport::tx_sent(const UART_HandleTypeDef *huart, size_t inflight)
{
  if (huart->hdmatx == nullptr)
  {
    return 0U;
  }

  const size_t left_cnt = static_cast<size_t>(__HAL_DMA_GET_COUNTER(huart->hdmatx));
  return (left_cnt <= inflight) ? (inflight - left_cnt) : 0U;
}

void uart_detail::DmaTransport::abort_rx(UART_HandleTypeDef *huart)
{
  HAL_UART_AbortReceive(huart);
}

void uart_detail::DmaTransport::abort_tx(UART_HandleTypeDef *huart)
{
  HAL_UART_AbortTransmit(huart);
}

bool uart_detail::ItTransport::arm_rx(UART_HandleTypeDef *huart, uint8_t *buffer, uint16_t size)
{
  return HAL_UARTEx_ReceiveToIdle_IT(huart, buffer, size) == HAL_OK;
}

bool uart_detail::ItTransport::start_tx(UART_HandleTypeDef *huart, uint8_t *buffer, uint16_t size)
{
  return HAL_UART_Transmit_IT(huart, buffer, size) == HAL_OK;
}

bool uart_detail::ItTransport::rx_ok(const UART_HandleTypeDef *huart)
{
  return (huart->RxState == HAL_UART_STATE_BUSY_RX) && it_rx_irq_enabled(huart);
}

bool uart_detail::ItTransport::rx_stopped(const UART_HandleTypeDef *huart)
{
  return (huart->RxState != HAL_UART_STATE_BUSY_RX) && (!it_rx_irq_enabled(huart));
}

size_t uart_detail::ItTransport::tx_sent(const UART_HandleTypeDef *huart, size_t inflight)
{
  if (huart->gState != HAL_UART_STATE_BUSY_TX)
  {
    return inflight;
  }

  const size_t left_cnt = static_cast<size_t>(huart->TxXferCount);
  return (left_cnt <= inflight) ? (inflight - left_cnt) : 0U;
}

void uart_detail::ItTransport::abort_rx(UART_HandleTypeDef *huart)
{
  HAL_UART_AbortReceive(huart);
}

void uart_detail::ItTransport::abort_tx(UART_HandleTypeDef *huart)
{
  HAL_UART_AbortTransmit(huart);
}

// ----------------
// ---------------- UART ----------------
// ----------------

// ---------------- 模板实例化 ----------------

// 显式实例化：按项目实际使用的串口类型生成
template class BspUartT<uart_detail::DmaTransport, 128, 128>;
template class BspUartT<uart_detail::DmaTransport, 256, 256>;
template class BspUartT<uart_detail::ItTransport, 128, 32>;
template class BspUartT<uart_detail::ItTransport, 128, 128>;
// ----------------
// ---------------- HAL 回调分发 ----------------

extern "C"
{
  /**
   * @brief IDLE串口回调函数
   * @note 按句柄分发到对应的全局实例
   */
  void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
  {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (huart == &huart1)
    {
      bsp_uart1.on_idle_isr(Size, &xHigherPriorityTaskWoken);
    }
    else if (huart == &huart2)
    {
      bsp_uart2.on_idle_isr(Size, &xHigherPriorityTaskWoken);
    }
    else if (huart == &huart3)
    {
      bsp_uart3.on_idle_isr(Size, &xHigherPriorityTaskWoken);
    }
    else if (huart == &huart4)
    {
      bsp_uart4.on_idle_isr(Size, &xHigherPriorityTaskWoken);
    }
    else if (huart == &huart5)
    {
      bsp_uart_it5.on_idle_isr(Size, &xHigherPriorityTaskWoken);
    }
    else if (huart == &huart7)
    {
      bsp_uart7.on_idle_isr(Size, &xHigherPriorityTaskWoken);
    }
    else if (huart == &huart8)
    {
      bsp_uart8.on_idle_isr(Size, &xHigherPriorityTaskWoken);
    }
    else if (huart == &huart9)
    {
      bsp_uart9.on_idle_isr(Size, &xHigherPriorityTaskWoken);
    }
    else if (huart == &huart10)
    {
      bsp_uart10.on_idle_isr(Size, &xHigherPriorityTaskWoken);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }

  /**
   * @brief UART TX Complete 回调函数
   * @note 发送完成时触发，从流缓冲区取下一批数据继续发送
   */
  void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
  {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // 注：UART5 使用中断收发，TxCplt 回调中继续从发送流缓冲区取下一包
    if (huart == &huart1)
    {
      bsp_uart1.start_transmission_from_isr(&xHigherPriorityTaskWoken);
    }
    else if (huart == &huart2)
    {
      bsp_uart2.start_transmission_from_isr(&xHigherPriorityTaskWoken);
    }
    else if (huart == &huart3)
    {
      bsp_uart3.start_transmission_from_isr(&xHigherPriorityTaskWoken);
    }
    else if (huart == &huart4)
    {
      bsp_uart4.start_transmission_from_isr(&xHigherPriorityTaskWoken);
    }
    else if (huart == &huart5)
    {
      bsp_uart_it5.start_transmission_from_isr(&xHigherPriorityTaskWoken);
    }
    else if (huart == &huart7)
    {
      bsp_uart7.start_transmission_from_isr(&xHigherPriorityTaskWoken);
    }
    else if (huart == &huart8)
    {
      bsp_uart8.start_transmission_from_isr(&xHigherPriorityTaskWoken);
    }
    else if (huart == &huart9)
    {
      bsp_uart9.start_transmission_from_isr(&xHigherPriorityTaskWoken);
    }
    else if (huart == &huart10)
    {
      bsp_uart10.start_transmission_from_isr(&xHigherPriorityTaskWoken);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }

  /**
   * @brief UART 错误回调函数
   * @note 这里只累加计数，不做恢复；复位与重建由任务侧 rx_recover()/tx_recover() 完成
   */
  void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
  {
    // 8 路都要接：这里只累加计数，恢复由任务侧 rx_recover()/tx_recover()完成 ，因为修复这件事会阻塞，中断不能写阻塞
    if (huart == &huart1)
    {
      bsp_uart1.on_error_isr();
    }
    else if (huart == &huart2)
    {
      bsp_uart2.on_error_isr();
    }
    else if (huart == &huart3)
    {
      bsp_uart3.on_error_isr();
    }
    else if (huart == &huart4)
    {
      bsp_uart4.on_error_isr();
    }
    else if (huart == &huart5)
    {
      bsp_uart_it5.on_error_isr();
    }
    else if (huart == &huart7)
    {
      bsp_uart7.on_error_isr();
    }
    else if (huart == &huart8)
    {
      bsp_uart8.on_error_isr();
    }
    else if (huart == &huart9)
    {
      bsp_uart9.on_error_isr();
    }
    else if (huart == &huart10)
    {
      bsp_uart10.on_error_isr();
    }
  }
}

/**
 * @brief BspUartT<Transport, RX_SIZE, TX_SIZE> 成员函数定义
 *
 * 接收：IDLE 中断判定帧结束，由 Transport 把这一包交给流缓冲区。
 * 发送：任务或 TX Complete 中断从流缓冲区取一包，交给 Transport 发出。
 * 线程安全：FreeRTOS 流缓冲区自带锁；任务侧启动发送由 _tx_lock 串行化，printf() 由 _printf_lock 保护格式化缓冲。
 */
// ----------------
// ---------------- 公有接口 ----------------

/** @brief 只做赋值，RTOS 资源与接收启动放在 init() */
template <typename Transport, size_t RX_SIZE, size_t TX_SIZE>
BspUartT<Transport, RX_SIZE, TX_SIZE>::BspUartT(const Config &cfg) : _huart(cfg.huart), _mode(cfg.mode), _baudrate(cfg.baudrate)
{
  // 构造函数只做赋值；运行时逻辑（FreeRTOS资源创建）推迟到 init()
}

/**
 * @brief 创建 FreeRTOS 对象并启动 IDLE 接收（须在STM32初始化之后调用）
 *
 * @return Status OK=成功；BAD_ARG=句柄为空；IO_ERROR=资源创建或接收启动失败
 *
 * @note 可重复调用：开头先 cleanup_resources() 释放上一次的资源，避免指针被覆盖造成泄漏。
 *       任一步失败都会走 abort_reception() + cleanup_resources()，不留下半初始化状态。
 */
template <typename Transport, size_t RX_SIZE, size_t TX_SIZE>
Status BspUartT<Transport, RX_SIZE, TX_SIZE>::init()
{
  // 句柄非法：后续所有 HAL 调用都不可用，提前拒绝（与 bsp_can::init() 一致）
  if (_huart == nullptr)
  {
    return Status::BAD_ARG;
  }

  // 可重复调用：先释放上一次残留的软件资源，避免二次 init() 直接覆盖指针造成泄漏
  // （与 bsp_can::init() 的可重入行为保持一致）
  cleanup_resources();

  _rx_should_run = _rx_enabled();

  if (_rx_enabled())
  {
    // 创建接收流缓冲区
    _rx_stream_buffer = xStreamBufferCreate(RX_SIZE, 1);
    if (_rx_stream_buffer == nullptr)
    {
      cleanup_resources();     // 清理已创建的资源
      return Status::IO_ERROR; // 流缓冲区创建失败
    }
  }

  if (_tx_enabled())
  {
    // 创建发送流缓冲区
    _tx_stream_buffer = xStreamBufferCreate(TX_SIZE, 1);
    if (_tx_stream_buffer == nullptr)
    {
      cleanup_resources();     // 清理已创建的资源
      return Status::IO_ERROR; // 发送流缓冲区创建失败
    }

    // TX 启动锁（任务侧串行化「判忙 → 取包 → 启动发送」）
    _tx_lock = xSemaphoreCreateMutex();
    if (_tx_lock == nullptr)
    {
      cleanup_resources();     // 清理已创建的资源
      return Status::IO_ERROR; // 互斥量创建失败
    }

    // printf 锁（保护实例级共享的 _printf_buffer，消除并发 printf 互相覆盖）
    _printf_lock = xSemaphoreCreateMutex();
    if (_printf_lock == nullptr)
    {
      cleanup_resources();
      return Status::IO_ERROR; // printf 锁创建失败
    }
  }

  // 打开 UART 硬件 FIFO。CubeMX 默认把它关掉，此时接收只有 RDR 一级缓冲：
  // 只要 Transport 晚一点来搬（DMA 抢总线时尤其明显），下一个字节到达就 ORE 丢掉，
  // 整条接收流随之错位。FIFO 有 16 字节，能吸收这段搬运抖动。
  // (void)HAL_UARTEx_EnableFifoMode(_huart);

  if (_rx_enabled())
  {
    // 启动接收（HAL 内部会清 IDLE 标志并使能 IDLE 中断；失败则释放资源并上报）
    if (!arm_reception())
    {
      abort_reception();       // 复位 RX 状态并关掉 IDLE 中断源
      cleanup_resources();     // 释放已创建的资源
      return Status::IO_ERROR; // 接收启动失败
    }
  }

  return Status::OK; // 初始化成功
}

/** @brief 停掉 RX/TX 通道并释放软件资源 */
template <typename Transport, size_t RX_SIZE, size_t TX_SIZE>
BspUartT<Transport, RX_SIZE, TX_SIZE>::~BspUartT()
{
  // 句柄非法时无可停的外设，直接释放软件资源
  if (_huart == nullptr)
  {
    cleanup_resources();
    return;
  }

  _rx_should_run = false;
  Transport::abort_rx(_huart);
  Transport::abort_tx(_huart);

  cleanup_resources();
}

/**
 * @brief 发送数据：写入发送流缓冲区并启动发送
 *
 * @param data       待发数据
 * @param size       字节数（不能超过发送流缓冲区容量 TX_SIZE，否则调用方应自行分包）
 * @param timeout_ms 入队等待上限（毫秒）；portMAX_DELAY = 一直等到全部入队
 * @param written    实际入队的字节数（可为 nullptr）
 * @return Status OK=全部入队；TIMEOUT=仅部分入队；BAD_ARG=参数非法；IO_ERROR=未启用发送
 */
template <typename Transport, size_t RX_SIZE, size_t TX_SIZE>
Status BspUartT<Transport, RX_SIZE, TX_SIZE>::send(const uint8_t *data, size_t size, uint32_t timeout_ms, size_t *written)
{
  if (data == nullptr || size == 0)
  {
    return Status::BAD_ARG; // 参数非法
  }

  // 单次发送不能超过流缓冲区容量，否则 xStreamBufferSend 会永久阻塞（死锁）
  if (size > TX_SIZE)
  {
    return Status::BAD_ARG; // 数据超长，调用方应分包发送
  }

  if (!_tx_enabled() || _tx_stream_buffer == nullptr)
  {
    return Status::IO_ERROR; // 未启用发送或发送缓冲区未初始化
  }

  // 将数据写入发送流缓冲区
  size_t bytes_written = xStreamBufferSend(_tx_stream_buffer, data, size, pdMS_TO_TICKS(timeout_ms));

  // 如果发送缓冲区中有数据，启动发送
  if (bytes_written > 0)
  {
    (void)start_transmission();
  }

  // 未全部入队说明有字节被丢弃（调用方通常只检查返回码），累加计数
  if (bytes_written < size)
  {
    _tx_drop_bytes += static_cast<uint32_t>(size - bytes_written);
  }

  if (written != nullptr)
  {
    *written = bytes_written;
  }

  return (bytes_written == size) ? Status::OK : Status::TIMEOUT;
}

/**
 * @brief 格式化输出（printf 风格，超长自动截断；仅任务上下文）
 *
 * @note 实例级的 _printf_buffer 由 _printf_lock 保护，多任务并发调用安全。
 *       入队等待固定为 10 ms，不随调用方传入，避免发送卡住时
 *       把调用任务长时间挂起；未全部入队时返回 Status::TIMEOUT（已入队部分照常发出）。
 */
template <typename Transport, size_t RX_SIZE, size_t TX_SIZE>
Status BspUartT<Transport, RX_SIZE, TX_SIZE>::printf(const char *fmt, ...)
{
  // 取锁等待上限 (ms)：避免某一个任务长期持有格式化缓冲
  constexpr uint32_t PRINTF_LOCK_TIMEOUT_MS = 10U;

  // 入队等待上限 (ms)：发送卡住时不把调用任务无限挂住
  constexpr uint32_t PRINTF_TX_TIMEOUT_MS = 10U;

  if (fmt == nullptr)
  {
    return Status::BAD_ARG; // 参数非法
  }

  if (!_tx_enabled() || _tx_stream_buffer == nullptr || _printf_lock == nullptr)
  {
    return Status::IO_ERROR; // 未启用发送 / 未初始化
  }

  // _printf_buffer 是实例级共享缓冲：必须整体互斥，
  // 否则两个任务并发 printf 会在 vsnprintf 写一半时被对方覆盖。
  if (xSemaphoreTake(_printf_lock, pdMS_TO_TICKS(PRINTF_LOCK_TIMEOUT_MS)) != pdTRUE)
  {
    return Status::IO_ERROR;
  }

  va_list args;
  va_start(args, fmt);
  int len = vsnprintf(_printf_buffer, sizeof(_printf_buffer), fmt, args);
  va_end(args);

  if (len <= 0)
  {
    xSemaphoreGive(_printf_lock);
    return Status::BAD_ARG; // 格式化失败或空输出
  }

  // vsnprintf 返回的是期望写入的完整长度，实际写入可能被截断
  if (static_cast<size_t>(len) >= sizeof(_printf_buffer))
  {
    len = static_cast<int>(sizeof(_printf_buffer)) - 1;
  }

  // 注意：send() 内部只用 _tx_lock 启动发送，两把锁不会反序嵌套，不会死锁。
  //       这里用有限超时：发送卡住时不把调用任务无限挂住。
  const Status st = send(reinterpret_cast<const uint8_t *>(_printf_buffer),
                         static_cast<size_t>(len),
                         PRINTF_TX_TIMEOUT_MS);
  xSemaphoreGive(_printf_lock);
  return st;
}

/**
 * @brief 从接收流缓冲区读取数据
 *
 * @param buffer   接收缓冲区
 * @param size     最多读取的字节数
 * @param timeout  超时时间（毫秒）；portMAX_DELAY = 一直等
 * @param received 实际读到的字节数（可为 nullptr）
 * @return Status OK=读到数据；TIMEOUT=无数据；BAD_ARG=参数非法；IO_ERROR=缓冲区未创建
 *
 * @note 字节流语义：不保证一次读到完整一帧，只保证有序、不重、不丢（除非流缓冲区满）。
 *       帧边界由协议层自行确定（长度前缀 / 定界符 / 校验和）。
 */
template <typename Transport, size_t RX_SIZE, size_t TX_SIZE>
Status BspUartT<Transport, RX_SIZE, TX_SIZE>::receive(uint8_t *buffer, size_t size, uint32_t timeout, size_t *received)
{
  if (buffer == nullptr || size == 0)
  {
    return Status::BAD_ARG; // 参数非法
  }

  if (_rx_stream_buffer == nullptr)
  {
    return Status::IO_ERROR; // 流缓冲区未创建
  }

  size_t bytes_read = xStreamBufferReceive(_rx_stream_buffer, buffer, size, pdMS_TO_TICKS(timeout));
  if (received != nullptr)
  {
    *received = bytes_read;
  }
  return (bytes_read > 0) ? Status::OK : Status::TIMEOUT;
}

/**
 * @brief TX 卡死恢复：流缓冲区有数据、但连续没有推进达到阈值时，复位通道并重发（非阻塞）
 *
 * @return true=本次确实恢复了一次
 *
 * @note 阈值按 Config::baudrate 估算为"一包发送耗时的 2 倍"，见函数内注释。
 * @note 先按 Transport 报告的在途计数算出已发出多少字节，只续发剩余部分（不重复、不丢失）；
 *       在途数据已全部发出时，改为从流缓冲区取新数据发送。
 */
template <typename Transport, size_t RX_SIZE, size_t TX_SIZE>
bool BspUartT<Transport, RX_SIZE, TX_SIZE>::tx_recover()
{
  if (_huart == nullptr || _tx_stream_buffer == nullptr || !_tx_enabled() || _baudrate == 0U)
  {
    return false; // 未初始化 / 未启用发送 / 波特率未填
  }

  // 卡死判定阈值 (ms) = 一包发送耗时的 2 倍（向上取整）。
  // 一包耗时 = TX_SIZE × 10 bit ÷ 波特率，乘 1000 换成 ms；×2 是留余量。
  const uint32_t stall_ms = (static_cast<uint32_t>(TX_SIZE) * 20U * 1000U + _baudrate - 1U) / _baudrate;

  // 没有待发数据 → 不存在卡死
  if (xStreamBufferBytesAvailable(_tx_stream_buffer) == 0U)
  {
    return false;
  }

  // 判据：有数据要发，但连续超过阈值时间没有推进
  if ((xTaskGetTickCount() - _tx_last_activity_tick) < pdMS_TO_TICKS(stall_ms))
  {
    return false; // 仍在正常发送中
  }

  // ---- 强制重发 ----
  // 在途数据可能只发了一部分：由 Transport 算出已发出多少字节，只续发剩下的。
  size_t sent = 0;
  if (_tx_inflight_len > 0)
  {
    sent = Transport::tx_sent(_huart, _tx_inflight_len);
  }

  Transport::abort_tx(_huart); // 复位 TX 通道（阻塞版 HAL，只能在任务上下文调用）

  const size_t rest = _tx_inflight_len - sent;
  _tx_inflight_len  = 0;

  // 续发漏发字节（HAL 不会重发这些字节，不续发就丢了）
  if (rest > 0 && Transport::start_tx(_huart, _tx_buffer + sent, static_cast<uint16_t>(rest)))
  {
    _tx_inflight_len       = rest;
    _tx_last_activity_tick = xTaskGetTickCount();
    _tx_stall_rec_cnt++;
    return true;
  }

  // 无数据：从流缓冲区取新数据发送
  if (!start_transmission(0))
  {
    return false; // 拿不到锁 / 仍无法启动，下轮再试
  }

  _tx_stall_rec_cnt++; // 诊断：恢复成功一次
  return true;
}

/**
 * @brief RX 恢复：RX 异常停止时复位接收通道并重新启动（阻塞）
 *
 * @return true=本次确实重建了一次
 *
 * @note 必须放在任务侧：复位 RX 要调阻塞版 HAL_UART_AbortReceive()，ISR 里不能做。
 */
template <typename Transport, size_t RX_SIZE, size_t TX_SIZE>
bool BspUartT<Transport, RX_SIZE, TX_SIZE>::rx_recover()
{
  if (_huart == nullptr || _rx_stream_buffer == nullptr)
  {
    return false; // 未初始化
  }

  // 本驱动没期望收数（主动停止 / 还没启动过）→ 不是故障
  if (!_rx_should_run)
  {
    return false;
  }

  // 三处状态一致表明 RX 正常 → 直接返回（正常时每次调用只做一次查询）
  if (rx_health() == RxHealth::OK)
  {
    return false;
  }

  // RX 已停止或三处状态不一致：复位接收通道后重新启动，只动 RX、不碰 TX
  abort_reception();
  if (!arm_reception())
  {
    return false; // 仍失败，下个周期再试（_rx_should_run 保持为 true）
  }

  _rx_error_cnt++; // 诊断：确实重建了一次
  return true;
}
// ----------------
// ---------------- ISR 入口 ----------------

/**
 * @brief IDLE/TC 接收完成：投递数据并重新启动接收（ISR）
 *
 * @param size 本次收到的字节数
 * @note HT 事件也会进本回调，但它不是帧边界，直接忽略。本函数只碰 RX，不碰 TX。
 */
template <typename Transport, size_t RX_SIZE, size_t TX_SIZE>
void BspUartT<Transport, RX_SIZE, TX_SIZE>::on_idle_isr(uint16_t size, BaseType_t *pxHigherPriorityTaskWoken)
{
  if ((_huart == nullptr) || !_rx_enabled())
  {
    return; // 句柄缺失或只发模式
  }

  // HAL 在半传输（HT）事件时也会回调本函数：HT 不是帧边界，直接忽略
  if (_huart->RxEventType == HAL_UART_RXEVENT_HT)
  {
    return;
  }

  // 到此处 RxState 已为 READY：HAL 在 IDLE/TC 事件中已停掉 RX，
  // 这里只需投递数据并重新启动接收，全程不触碰 TX（不打断正在发送的数据）
  if (_rx_stream_buffer != nullptr && size > 0)
  {
    // 流缓冲区满时 SendFromISR 只会写入一部分，剩余字节被丢弃。
    // 累加诊断计数记录丢弃的字节数（正常应恒为 0）。
    size_t pushed = xStreamBufferSendFromISR(_rx_stream_buffer, _rx_buffer, size, pxHigherPriorityTaskWoken);
    _rx_drop_bytes += static_cast<uint32_t>(size - pushed);
  }

  // 重新启动接收。若失败也不在此重试（阻塞）
  // 任务侧 rx_recover() 会在 10 ms 内通过 rx_health() 发现异常并重建。
  (void)arm_reception();
}

/**
 * @brief 发送完成后续传：从流缓冲区取下一批交给 Transport（ISR，由 TX Complete 中断调用）
 *
 * @note 这里不加 _tx_lock：gState 由 TX 完成中断在回调时触发：必然没有正在进行的 TX、
 *       也没有挂起的完成中断，任务侧只需互斥其他任务。
 */
template <typename Transport, size_t RX_SIZE, size_t TX_SIZE>
void BspUartT<Transport, RX_SIZE, TX_SIZE>::start_transmission_from_isr(BaseType_t *pxHigherPriorityTaskWoken)
{
  if (_huart == nullptr || _tx_stream_buffer == nullptr || !_tx_enabled())
  {
    return; // 未启用发送或发送缓冲区未初始化
  }

  // 通道必须空闲才能续传。正常情况下这里必然空闲：
  // 不空闲说明通道被别的传输占了，此时不取数据（取出来还得回滚），交给 tx_recover() 处理。
  const bool tx_channel_idle = (_huart->gState == HAL_UART_STATE_READY);
  if (!tx_channel_idle)
  {
    _tx_isr_skip_cnt++;
    return; // 数据仍在流缓冲区，等巡检重新启动发送（tx_recover函数）
  }

  // 从发送缓冲区获取数据准备发送（ISR级API）；无数据则直接返回
  size_t bytes_to_send = xStreamBufferReceiveFromISR(_tx_stream_buffer, _tx_buffer, TX_SIZE, pxHigherPriorityTaskWoken);
  if (bytes_to_send == 0)
  {
    _tx_inflight_len = 0; // 上一批已发完且无新数据
    return;
  }

  // 启动发送
  if (!Transport::start_tx(_huart, _tx_buffer, bytes_to_send))
  {
    // 启动失败：把刚取出的字节写回流缓冲区，避免丢数据。
    // 恢复动作由巡检任务进行（tx_recover函数）
    (void)xStreamBufferSendFromISR(_tx_stream_buffer, _tx_buffer, bytes_to_send, pxHigherPriorityTaskWoken);
    _tx_inflight_len = 0;
    _tx_start_fail_cnt++;
  }
  else
  {
    // 记录推进时刻：tx_recover() 用「多久没发出去」区分「正在发」和「卡死」
    _tx_last_activity_tick = xTaskGetTickCountFromISR();
    _tx_inflight_len       = bytes_to_send;
  }
}

/**
 * @brief UART 错误回调（ISR）：只累加计数，不做恢复
 * @note HAL 的 Abort 接口内部是阻塞版 HAL_DMA_Abort，ISR 里不能调用，
 *       恢复动作交给任务侧的 rx_recover() / tx_recover()。
 */
template <typename Transport, size_t RX_SIZE, size_t TX_SIZE>
void BspUartT<Transport, RX_SIZE, TX_SIZE>::on_error_isr()
{
  if (_huart == nullptr)
  {
    return;
  }

  // HAL 在收发出错时会停掉对应通道，gState 离开 READY → 记一次 TX 侧故障
  if (_huart->gState != HAL_UART_STATE_READY)
  {
    _tx_error_cnt++;
  }

  // 只统计接收侧的线路错误：ORE 是 921600 中断接收最需要盯的指标，
  // FE/NE/PE 则说明波特率、共地或线材有问题。DMA 错误由 TX/RX 恢复流程单独处理。
  constexpr uint32_t RX_HW_ERROR_MASK = HAL_UART_ERROR_PE | HAL_UART_ERROR_NE | HAL_UART_ERROR_FE | HAL_UART_ERROR_ORE | HAL_UART_ERROR_RTO;
  if ((_huart->ErrorCode & RX_HW_ERROR_MASK) != 0U)
  {
    _rx_hw_error_cnt++;
  }
}
// ----------------
// ---------------- 查询接口 ----------------

/** @brief 发送流缓冲区剩余空间（字节） */
template <typename Transport, size_t RX_SIZE, size_t TX_SIZE>
size_t BspUartT<Transport, RX_SIZE, TX_SIZE>::get_tx_free_space()
{
  if (_tx_stream_buffer != nullptr)
  {
    return xStreamBufferSpacesAvailable(_tx_stream_buffer);
  }
  return 0;
}

/** @brief 接收流缓冲区当前可读字节数 */
template <typename Transport, size_t RX_SIZE, size_t TX_SIZE>
size_t BspUartT<Transport, RX_SIZE, TX_SIZE>::get_rx_available_data()
{
  if (_rx_stream_buffer != nullptr)
  {
    return xStreamBufferBytesAvailable(_rx_stream_buffer);
  }
  return 0;
}
/** @brief 校验 CubeMX 写入的串口帧格式 */
template <typename Transport, size_t RX_SIZE, size_t TX_SIZE>
bool BspUartT<Transport, RX_SIZE, TX_SIZE>::check_format(uint32_t baudrate, uint32_t word_length, uint32_t parity, uint32_t stop_bits) const
{
  if (_huart == nullptr)
  {
    return false;
  }

  return (_huart->Init.BaudRate == baudrate) &&
         (_huart->Init.WordLength == word_length) &&
         (_huart->Init.Parity == parity) &&
         (_huart->Init.StopBits == stop_bits);
}
/**
 * @brief RX 健康查询：由 Transport 核对 HAL 状态与底层通道状态
 *
 * @return RxHealth OK=在收；STOPPED=已停；INCONSISTENT=状态不一致，需要重建
 */
template <typename Transport, size_t RX_SIZE, size_t TX_SIZE>
typename BspUartT<Transport, RX_SIZE, TX_SIZE>::RxHealth BspUartT<Transport, RX_SIZE, TX_SIZE>::rx_health() const
{
  if (_huart == nullptr || _huart->Instance == nullptr)
  {
    return RxHealth::STOPPED;
  }

  if (Transport::rx_ok(_huart))
  {
    return RxHealth::OK;
  }
  if (Transport::rx_stopped(_huart))
  {
    return RxHealth::STOPPED;
  }
  return RxHealth::INCONSISTENT;
}

/** @brief 读取 RX 诊断计数 */
template <typename Transport, size_t RX_SIZE, size_t TX_SIZE>
typename BspUartT<Transport, RX_SIZE, TX_SIZE>::RxDiag BspUartT<Transport, RX_SIZE, TX_SIZE>::rx_diag() const
{
  RxDiag diag;
  diag.arm_fail_cnt = _rx_arm_fail_cnt;
  diag.drop_bytes   = _rx_drop_bytes;
  diag.hw_error_cnt = _rx_hw_error_cnt;
  diag.error_cnt    = _rx_error_cnt;
  return diag;
}

/** @brief 读取 TX 诊断计数 */
template <typename Transport, size_t RX_SIZE, size_t TX_SIZE>
typename BspUartT<Transport, RX_SIZE, TX_SIZE>::TxDiag BspUartT<Transport, RX_SIZE, TX_SIZE>::tx_diag() const
{
  TxDiag diag;
  diag.drop_bytes        = _tx_drop_bytes;
  diag.start_fail_cnt    = _tx_start_fail_cnt;
  diag.isr_skip_cnt      = _tx_isr_skip_cnt;
  diag.stall_recover_cnt = _tx_stall_rec_cnt;
  diag.error_cnt         = _tx_error_cnt;
  return diag;
}
// ----------------
// ---------------- 私有实现 ----------------

/**
 * @brief 启动接收（仅 RX）
 *
 * @return true=成功；false=HAL 拒绝（BUSY/ERROR），已累加 arm_fail_cnt
 *
 * @note 调用前 RxState 必须为 READY。开头先清掉残留的 UART 错误标志（ORE/FE/NE/PE），
 *       否则 HAL 会因存在挂起错误而拒绝启动。
 */
template <typename Transport, size_t RX_SIZE, size_t TX_SIZE>
bool BspUartT<Transport, RX_SIZE, TX_SIZE>::arm_reception()
{
  if (!_rx_enabled())
  {
    return false;
  }

  // 先清掉残留的 UART 错误标志：否则 HAL 会因"启动前已有挂起的 ORE/FE/NE"而拒绝启动
  // （HAL 内部会检测到 ReceptionType 被重置，返回 HAL_ERROR）
  //
  // 注：DMA 侧的 TC/HT/TE/FE/DME 标志无需在此清除——HAL 已负责：
  //     HAL_DMA_Abort() 用 IFCR 一次清掉该 Stream 全部 6 个标志
  //     （abort_reception() 与 HAL 的 IDLE 分支都会走到），
  //     HAL_DMA_IRQHandler() 也会清 TC/HT。
  __HAL_UART_CLEAR_FLAG(_huart, UART_CLEAR_PEF | UART_CLEAR_FEF | UART_CLEAR_NEF | UART_CLEAR_OREF);

  // 启动多字节接收（IDLE 模式）；HAL 会自动清 IDLE 标志并使能 IDLE 中断
  if (!Transport::arm_rx(_huart, _rx_buffer, RX_SIZE))
  {
    _rx_arm_fail_cnt++; // 诊断：启动失败计数（正常应恒为 0）
    return false;       // 启动失败：由调用方决定重试或上报
  }
  _rx_should_run = true;
  return true;
}

/** @brief 中止 RX 通道并复位接收状态（只动 RX，不碰 TX） */
template <typename Transport, size_t RX_SIZE, size_t TX_SIZE>
void BspUartT<Transport, RX_SIZE, TX_SIZE>::abort_reception()
{
  if (!_rx_enabled())
  {
    return;
  }

  Transport::abort_rx(_huart);
  ATOMIC_CLEAR_BIT(_huart->Instance->CR1, USART_CR1_IDLEIE); // 防止残留 IDLE 中断源
}

/**
 * @brief 从发送流缓冲区取一包并启动发送（任务上下文）
 *
 * @param wait 取 _tx_lock 的等待时间（ticks）；send() 传 portMAX_DELAY，巡检传 0
 * @return true=本次确实启动了一次发送
 *
 * @note "判忙 + 取包 + 启动发送" 必须整体原子，否则两个任务可能同时写 _tx_buffer，
 *       覆盖 Transport 正在读取的数据。
 */
template <typename Transport, size_t RX_SIZE, size_t TX_SIZE>
bool BspUartT<Transport, RX_SIZE, TX_SIZE>::start_transmission(TickType_t wait)
{
  if (_huart == nullptr || _tx_stream_buffer == nullptr)
  {
    return false; // 句柄/缓冲未就绪
  }

  if (_tx_lock == nullptr)
  {
    return false; // 互斥量未创建
  }

  // 判忙 + 取包 + 启动发送必须整体原子：否则两个任务可能同时写入 _tx_buffer，
  // 覆盖 Transport 正在读取的数据（已取出的字节会被丢掉）。
  // wait=0（巡检）时拿不到锁立即返回，不阻塞周期任务。
  if (xSemaphoreTake(_tx_lock, wait) != pdTRUE)
  {
    return false;
  }

  bool started = false;

  // 必须严格用 READY 判定（而不是"不等于 BUSY_TX"）：ERROR/TIMEOUT 状态下
  // HAL 会拒绝启动发送，若此时已把字节从流缓冲区取出就会丢数据
  const bool has_tx_buffer = (_tx_stream_buffer != nullptr);
  const bool channel_idle  = (_huart->gState == HAL_UART_STATE_READY);
  if (has_tx_buffer && channel_idle)
  {
    // 从发送缓冲区获取数据准备发送（任务级API）
    size_t bytes_to_send = xStreamBufferReceive(_tx_stream_buffer, _tx_buffer, TX_SIZE, 0);
    if (bytes_to_send > 0)
    {
      if (!Transport::start_tx(_huart, _tx_buffer, bytes_to_send))
      {
        // 启动失败：字节已从流缓冲区取出，必须回滚，否则丢数据。
        // 刚取出 n 字节，流缓冲区至少有 n 字节空间，所以这次回滚必定成功。
        (void)xStreamBufferSend(_tx_stream_buffer, _tx_buffer, bytes_to_send, 0);
        _tx_inflight_len = 0; // 未启动 → 无在途数据
        _tx_start_fail_cnt++;
      }
      else
      {
        _tx_last_activity_tick = xTaskGetTickCount(); // 记录推进时刻（供 tx_recover 判卡死）
        _tx_inflight_len       = bytes_to_send;       // 记录在途长度
        started                = true;
      }
    }
    else
    {
      _tx_inflight_len = 0; // 流缓冲区已空（旧的在途数据已发完）
    }
  }

  xSemaphoreGive(_tx_lock);
  return started;
}

/** @brief 释放所有已创建的 FreeRTOS 资源 */
template <typename Transport, size_t RX_SIZE, size_t TX_SIZE>
void BspUartT<Transport, RX_SIZE, TX_SIZE>::cleanup_resources()
{
  // 释放接收流缓冲区
  if (_rx_stream_buffer != nullptr)
  {
    vStreamBufferDelete(_rx_stream_buffer);
    _rx_stream_buffer = nullptr;
  }

  // 释放发送流缓冲区
  if (_tx_stream_buffer != nullptr)
  {
    vStreamBufferDelete(_tx_stream_buffer);
    _tx_stream_buffer = nullptr;
  }

  // 释放 TX 启动锁
  if (_tx_lock != nullptr)
  {
    vSemaphoreDelete(_tx_lock);
    _tx_lock = nullptr;
  }

  // 释放 printf 锁
  if (_printf_lock != nullptr)
  {
    vSemaphoreDelete(_printf_lock);
    _printf_lock = nullptr;
  }
}
// ----------------
