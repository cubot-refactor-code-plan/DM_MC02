#include "bsp_uart.hpp"
#include "bsp_cfg.hpp" // 中断回调中直接引用 bsp_usartX / bsp_uartX 全局实例
#include "FreeRTOS.h"  // IWYU pragma: keep
#include <stdarg.h>
#include <stdio.h>

// ---------------- 模板实例化 ----------------

/**
 * @brief 模板实例化实现
 * @param 缓冲区大小（uint8_t）
 *
 */
template class BspUart<128>;


// ----------------


// ---------------- HAL 回调分发 ----------------

extern "C"
{
  /**  
   * @brief IDLE串口回调函数 
   * @note 直接 if-else 判断 UART 句柄并调用对应全局实例
   */
  void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
  {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (huart == &huart1)
    {
      bsp_uart1.on_idle_isr(Size, &xHigherPriorityTaskWoken);
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
      bsp_uart5.on_idle_isr(Size, &xHigherPriorityTaskWoken);
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
   * @note 发送完成时触发，链式续传发送缓冲区中的剩余数据
   */
  void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
  {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // 注：UART5 无发送功能（未配 TX DMA），不会触发本回调，故无 huart5 分支
    if (huart == &huart1)
    {
      bsp_uart1.start_transmission_from_isr(&xHigherPriorityTaskWoken);
    }
    else if (huart == &huart3)
    {
      bsp_uart3.start_transmission_from_isr(&xHigherPriorityTaskWoken);
    }
    else if (huart == &huart4)
    {
      bsp_uart4.start_transmission_from_isr(&xHigherPriorityTaskWoken);
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
   * @note ORE/FE/NE 等错误后复位 RX 并重新武装，避免接收无声停摆
   */
  void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
  {
    // UART5 仅接收，同样需要错误恢复
    if (huart == &huart1)
    {
      bsp_uart1.handle_dma_error();
    }
    else if (huart == &huart3)
    {
      bsp_uart3.handle_dma_error();
    }
    else if (huart == &huart4)
    {
      bsp_uart4.handle_dma_error();
    }
    else if (huart == &huart5)
    {
      bsp_uart5.handle_dma_error();
    }
    else if (huart == &huart7)
    {
      bsp_uart7.handle_dma_error();
    }
    else if (huart == &huart8)
    {
      bsp_uart8.handle_dma_error();
    }
    else if (huart == &huart9)
    {
      bsp_uart9.handle_dma_error();
    }
    else if (huart == &huart10)
    {
      bsp_uart10.handle_dma_error();
    }
  }
}

/**
 * @brief BspUart<BUFFER_SIZE> 类函数定义
 *
 * 串口驱动组件实现：只使用IDLE中断接收，DMA普通模式收发。
 * 线程安全：收发各用一条FreeRTOS流缓冲区（内置锁），
 *           除任务间抢占写外，无mutex，多任务并发写同一串口需上层自行协调。
 *
 * @note 经过测试，无任何测试问题。
 *
 * @param BUFFER_SIZE 缓冲区大小（DMA收发缓冲区与流缓冲区容量，单位uint8_t）
 * @param cfg 串口配置（huart/发送使能，可匿名按序传入）
 */

// ----------------


// ---------------- 公有接口 ----------------

template <size_t BUFFER_SIZE>
BspUart<BUFFER_SIZE>::BspUart(const Config &cfg)

  : _huart(cfg.huart),
    _transmit_enable(cfg.transmit_enable)
{
  // 构造函数只做赋值；运行时逻辑（FreeRTOS资源创建）推迟到 init()
}

template <size_t BUFFER_SIZE>
Status BspUart<BUFFER_SIZE>::init()
{
  // 创建接收流缓冲区
  _rx_stream_buffer = xStreamBufferCreate(BUFFER_SIZE, 1);
  if (_rx_stream_buffer == nullptr)
  {
    cleanup_resources();     // 清理已创建的资源
    return Status::IO_ERROR; // 流缓冲区创建失败
  }

  if (_transmit_enable)
  {
    // 创建发送流缓冲区
    _tx_stream_buffer = xStreamBufferCreate(BUFFER_SIZE, 1);
    if (_tx_stream_buffer == nullptr)
    {
      cleanup_resources();     // 清理已创建的资源
      return Status::IO_ERROR; // 发送流缓冲区创建失败
    }
  }
  else
  {
    _tx_stream_buffer = nullptr;
  }

  // TX 启动锁（任务侧串行化「判忙 → 取包 → 启转」）
  _tx_lock = xSemaphoreCreateMutex();
  if (_tx_lock == nullptr)
  {
    cleanup_resources();     // 清理已创建的资源
    return Status::IO_ERROR; // 互斥量创建失败
  }

  // 启动接收（HAL 内部会清 IDLE 标志并使能 IDLE 中断；失败则释放资源并上报）
  if (!arm_reception())
  {
    abort_reception();       // 复位 RX 状态并关掉 IDLE 中断源
    cleanup_resources();     // 释放已创建的资源
    return Status::IO_ERROR; // 接收启动失败，避免静默失能
  }

  return Status::OK; // 初始化成功
}

// 析构函数实现（终止场景：RX/TX DMA 全停后释放资源）
template <size_t BUFFER_SIZE>
BspUart<BUFFER_SIZE>::~BspUart()
{
  _rx_should_run = false;
  HAL_UART_DMAStop(_huart); // 同时中止 RX 与 TX 的 DMA

  cleanup_resources();
}

// 发送数据实现
template <size_t BUFFER_SIZE>
Status BspUart<BUFFER_SIZE>::send(const uint8_t *data, size_t size, uint32_t timeout, size_t *written)
{
  if (data == nullptr || size == 0)
  {
    return Status::BAD_ARG; // 参数非法
  }

  // 单次发送不能超过流缓冲区容量，否则 xStreamBufferSend 会永久阻塞（死锁）
  if (size > BUFFER_SIZE)
  {
    return Status::BAD_ARG; // 数据超长，调用方应分包发送
  }

  if (!_transmit_enable || _tx_stream_buffer == nullptr)
  {
    return Status::IO_ERROR; // 未启用发送或发送缓冲区未初始化
  }

  // 将数据写入发送流缓冲区
  size_t bytes_written = xStreamBufferSend(_tx_stream_buffer, data, size, timeout);

  // 如果发送缓冲区中有数据，启动发送
  if (bytes_written > 0)
  {
    (void)start_transmission();
  }

  if (written != nullptr)
  {
    *written = bytes_written;
  }

  return (bytes_written == size) ? Status::OK : Status::TIMEOUT;
}

// printf 格式化发送实现
template <size_t BUFFER_SIZE>
Status BspUart<BUFFER_SIZE>::printf(const char *fmt, ...)
{
  if (fmt == nullptr)
  {
    return Status::BAD_ARG; // 参数非法
  }

  va_list args;
  va_start(args, fmt);
  int len = vsnprintf(_printf_buffer, sizeof(_printf_buffer), fmt, args);
  va_end(args);

  if (len <= 0)
  {
    return Status::BAD_ARG; // 格式化失败或空输出
  }

  // vsnprintf 返回的是期望写入的完整长度，实际写入可能被截断
  if (static_cast<size_t>(len) >= sizeof(_printf_buffer))
  {
    len = static_cast<int>(sizeof(_printf_buffer)) - 1;
  }

  return send(reinterpret_cast<const uint8_t *>(_printf_buffer), static_cast<size_t>(len));
}

// 接收数据实现
template <size_t BUFFER_SIZE>
Status BspUart<BUFFER_SIZE>::receive(uint8_t *buffer, size_t size, uint32_t timeout, size_t *received)
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

// TX 断链兜底实现（非阻塞，供周期任务调用）
template <size_t BUFFER_SIZE>
bool BspUart<BUFFER_SIZE>::tx_recover()
{
  // 断链判据：「有数据要发」+「发送通道空闲」= 没人去点火
  const bool has_pending  = (_tx_stream_buffer != nullptr) && (xStreamBufferBytesAvailable(_tx_stream_buffer) > 0);
  const bool channel_idle = (_huart->gState == HAL_UART_STATE_READY);
  const bool is_stalled   = has_pending && channel_idle;

  if (!is_stalled)
  {
    return false; // 未断链，什么都不做
  }

  if (!start_transmission(0))
  {
    return false; // 拿不到锁或通道又忙了，下轮再试
  }

  _tx_stall_rec_cnt++; // 诊断：救回一次
  return true;
}

// ----------------


// ---------------- ISR 入口 ----------------

// IDLE/TC 接收完成处理实现（ISR上下文）
template <size_t BUFFER_SIZE>
void BspUart<BUFFER_SIZE>::on_idle_isr(uint16_t size, BaseType_t *pxHigherPriorityTaskWoken)
{
  // HAL 在半传输（HT）事件时也会回调本函数：HT 不是帧边界，直接忽略
  if (_huart->RxEventType == HAL_UART_RXEVENT_HT)
  {
    return;
  }

  // 到此处 RxState 已为 READY —— HAL 在 IDLE/TC 事件中已自行停掉 RX DMA，
  // 这里只需投递数据并重新武装，全程不触碰 TX DMA（不打断正在进行的发送）
  if (_rx_stream_buffer != nullptr && size > 0)
  {
    // 流缓冲区满时 SendFromISR 只会写入一部分，剩余的字节会被丢弃。
    // 累加诊断计数，把这种"静默丢数据"变成可观测量（正常应恒为 0）。
    size_t pushed = xStreamBufferSendFromISR(_rx_stream_buffer, _rx_dma_buffer, size, pxHigherPriorityTaskWoken);
    _rx_drop_bytes += static_cast<uint32_t>(size - pushed);
  }

  // 重新武装 RX。【动作与判断分开】：先真的去武装，再看结果。
  // IDLE/TC 事件里 HAL 已停掉 RX DMA 并把 RxState 置回 READY，
  // 且本函数开头已清过 UART 错误标志，所以正常情况下必然成功。
  if (arm_reception())
  {
    return; // 武装成功，继续收数
  }

  // 武装失败：只有「本驱动期望收数」时才算异常，需要走一次完整恢复
  if (_rx_should_run)
  {
    handle_dma_error();
  }
}

// 发送完成/续传处理实现（ISR上下文，由 TX Complete 中断调用）
//
// @note 这里不加 _tx_lock：gState 由 TX-cplt 中断自己在回调前清成 READY，
//       随后同一次中断内要么立即重启（→BUSY_TX）要么退出。故任务观察到 READY 时，
//       必然「无 TX 在飞、无挂起的 TX-cplt 中断」，任务侧只需排斥其他任务即可。
template <size_t BUFFER_SIZE>
void BspUart<BUFFER_SIZE>::start_transmission_from_isr(BaseType_t *pxHigherPriorityTaskWoken)
{
  if (_tx_stream_buffer == nullptr || !_transmit_enable)
  {
    return; // 未启用发送或发送缓冲区未初始化
  }

  // 通道必须空闲才能续传。正常情况下这里必然空闲 ——
  // UART_EndTransmit_IT 在调用本回调前刚把 gState 置回 READY。
  // 不空闲说明发送通道被别的传输占了，此时不取数据（取出来还得回滚），留给巡检兜底。
  const bool tx_channel_idle = (_huart->gState == HAL_UART_STATE_READY);
  if (!tx_channel_idle)
  {
    _tx_isr_skip_cnt++;
    return; // 数据仍留在流缓冲区，等 sys_task 巡检重新点火
  }

  // 从发送缓冲区获取数据准备发送（ISR级API）；无数据则直接返回
  size_t bytes_to_send = xStreamBufferReceiveFromISR(_tx_stream_buffer, _tx_dma_buffer, BUFFER_SIZE, pxHigherPriorityTaskWoken);
  if (bytes_to_send == 0)
  {
    return;
  }

  if (HAL_UART_Transmit_DMA(_huart, _tx_dma_buffer, bytes_to_send) != HAL_OK)
  {
    // 启动失败：把刚取出的字节塞回流缓冲区，绝不静默丢失。
    // 链在这里断掉，由巡检任务（见 sys_task）在 10ms 内重新点火。
    (void)xStreamBufferSendFromISR(_tx_stream_buffer, _tx_dma_buffer, bytes_to_send, pxHigherPriorityTaskWoken);
    _tx_start_fail_cnt++;
  }
}

// UART 错误恢复实现（供 HAL_UART_ErrorCallback 调用，也被 on_idle_isr 重装失败时调用）
template <size_t BUFFER_SIZE>
void BspUart<BUFFER_SIZE>::handle_dma_error()
{
  _rx_error_cnt++; // 诊断：进入恢复流程的次数

  // 本驱动未期望收数 → 无需恢复
  if (!_rx_should_run)
  {
    return;
  }

  // 三证据一致地表明 RX 仍在运行 → 属于非阻塞错误，不打断以免丢弃在途数据
  const bool rx_still_running = (rx_health() == RxHealth::OK);
  if (rx_still_running)
  {
    return;
  }

  // RX 已停摆：复位接收状态后重新武装（仅 RX，不触碰 TX DMA）
  abort_reception();
  // 若仍失败：保留「期望收数」意图不闩死，由 rx_health()/rx_diag() 暴露故障，等上层重建
  (void)arm_reception();
}

// ----------------


// ---------------- 查询接口 ----------------

// 获取发送缓冲区剩余空间实现
template <size_t BUFFER_SIZE>
size_t BspUart<BUFFER_SIZE>::get_tx_free_space()
{
  if (_tx_stream_buffer != nullptr)
  {
    return xStreamBufferSpacesAvailable(_tx_stream_buffer);
  }
  return 0;
}

// 获取接收缓冲区可用数据量实现
template <size_t BUFFER_SIZE>
size_t BspUart<BUFFER_SIZE>::get_rx_available_data()
{
  if (_rx_stream_buffer != nullptr)
  {
    return xStreamBufferBytesAvailable(_rx_stream_buffer);
  }
  return 0;
}

// RX 健康查询实现（多点一致性校验）
template <size_t BUFFER_SIZE>
typename BspUart<BUFFER_SIZE>::RxHealth BspUart<BUFFER_SIZE>::rx_health() const
{
  if (_huart == nullptr || _huart->Instance == nullptr)
  {
    return RxHealth::STOPPED;
  }

  // 证据 1：HAL 软件状态
  const bool hal_busy = (_huart->RxState == HAL_UART_STATE_BUSY_RX);

  // 证据 2：UART 硬件侧——是否允许产生 RX DMA 请求
  const bool dmaren = ((_huart->Instance->CR3 & USART_CR3_DMAR) != 0U);

  // 证据 3：DMA 硬件侧——通道是否真的在搬运
  bool dma_en = false;
  if (_huart->hdmarx != nullptr && IS_DMA_STREAM_INSTANCE(_huart->hdmarx->Instance) != 0U)
  {
    dma_en = ((((DMA_Stream_TypeDef *)(_huart->hdmarx->Instance))->CR & DMA_SxCR_EN) != 0U);
  }

  if (hal_busy && dmaren && dma_en)
  {
    return RxHealth::OK; // 三者一致地"在收"
  }
  if (!hal_busy && !dmaren && !dma_en)
  {
    return RxHealth::STOPPED; // 三者一致地"停"
  }
  return RxHealth::INCONSISTENT; // 自相矛盾 → 需重建
}

// 读取 RX 诊断计数实现
template <size_t BUFFER_SIZE>
typename BspUart<BUFFER_SIZE>::RxDiag BspUart<BUFFER_SIZE>::rx_diag() const
{
  RxDiag diag;
  diag.arm_fail_cnt = _rx_arm_fail_cnt;
  diag.drop_bytes   = _rx_drop_bytes;
  diag.error_cnt    = _rx_error_cnt;
  return diag;
}

// 读取 TX 诊断计数实现
template <size_t BUFFER_SIZE>
typename BspUart<BUFFER_SIZE>::TxDiag BspUart<BUFFER_SIZE>::tx_diag() const
{
  TxDiag diag;
  diag.start_fail_cnt    = _tx_start_fail_cnt;
  diag.isr_skip_cnt      = _tx_isr_skip_cnt;
  diag.stall_recover_cnt = _tx_stall_rec_cnt;
  return diag;
}

// ----------------


// ---------------- 私有实现 ----------------

// 武装 DMA 接收实现（仅 RX；调用前 RxState 必须为 READY）
template <size_t BUFFER_SIZE>
bool BspUart<BUFFER_SIZE>::arm_reception()
{
  // 先清掉残留的 UART 错误标志：否则 HAL 会因"启动前已有挂起的 ORE/FE/NE"而拒绝启动
  // （HAL_UARTEx_ReceiveToIdle_DMA 内部会检测到 ReceptionType 被重置，返回 HAL_ERROR）
  //
  // 注：DMA 侧的 TC/HT/TE/FE/DME 标志无需在此清除——HAL 已负责：
  //     HAL_DMA_Abort() 用 IFCR 一次清掉该 Stream 全部 6 个标志
  //     （abort_reception() 与 HAL 的 IDLE 分支都会走到），
  //     HAL_DMA_IRQHandler() 也会清 TC/HT。
  __HAL_UART_CLEAR_FLAG(_huart, UART_CLEAR_PEF | UART_CLEAR_FEF | UART_CLEAR_NEF | UART_CLEAR_OREF);

  // 启动多字节 DMA 接收（IDLE 模式）；HAL 会自动清 IDLE 标志并使能 IDLE 中断
  if (HAL_UARTEx_ReceiveToIdle_DMA(_huart, _rx_dma_buffer, BUFFER_SIZE) != HAL_OK)
  {
    _rx_arm_fail_cnt++; // 诊断：武装失败计数（正常应恒为 0）
    return false;       // 武装失败：由调用方决定重试或上报
  }
  _rx_should_run = true;
  return true;
}

// 中止 RX 通道实现（仅 RX：清错误标志 + 复位接收状态，不触碰 TX DMA）
template <size_t BUFFER_SIZE>
void BspUart<BUFFER_SIZE>::abort_reception()
{
  HAL_UART_AbortReceive(_huart);
  ATOMIC_CLEAR_BIT(_huart->Instance->CR1, USART_CR1_IDLEIE); // 防止残留 IDLE 中断源
}

// 主动尝试排空发送流缓冲区实现（任务上下文）
template <size_t BUFFER_SIZE>
bool BspUart<BUFFER_SIZE>::start_transmission(TickType_t wait)
{
  if (_tx_lock == nullptr)
  {
    return false; // 互斥量未创建
  }

  // 判忙 + 取包 + 启转 必须整体原子：否则两个任务可能同时写入 _tx_dma_buffer，
  // 覆盖 DMA 正在读取的数据（数据被改写 / 已取出的字节丢失）。
  // wait=0（巡检兜底）时拿不到锁立即返回，绝不阻塞周期任务。
  if (xSemaphoreTake(_tx_lock, wait) != pdTRUE)
  {
    return false;
  }

  bool started = false;

  // 必须严格用 READY 判定（而不是"不等于 BUSY_TX"）：ERROR/TIMEOUT 状态下
  // HAL_UART_Transmit_DMA 会被 HAL 拒绝，若此时已把字节从流缓冲区取出就会静默丢失
  const bool has_tx_buffer = (_tx_stream_buffer != nullptr);
  const bool channel_idle  = (_huart->gState == HAL_UART_STATE_READY);
  if (has_tx_buffer && channel_idle)
  {
    // 从发送缓冲区获取数据准备发送（任务级API）
    size_t bytes_to_send = xStreamBufferReceive(_tx_stream_buffer, _tx_dma_buffer, BUFFER_SIZE, 0);
    if (bytes_to_send > 0)
    {
      if (HAL_UART_Transmit_DMA(_huart, _tx_dma_buffer, bytes_to_send) != HAL_OK)
      {
        // 启动失败：字节已从流缓冲区取出，必须回滚，否则静默丢数据。
        // 刚取出 n 字节，流缓冲区至少有 n 字节空间，故这次回滚必定成功。
        (void)xStreamBufferSend(_tx_stream_buffer, _tx_dma_buffer, bytes_to_send, 0);
        _tx_start_fail_cnt++;
      }
      else
      {
        started = true;
      }
    }
  }

  xSemaphoreGive(_tx_lock);
  return started;
}

// 释放所有已创建的 FreeRTOS 资源
template <size_t BUFFER_SIZE>
void BspUart<BUFFER_SIZE>::cleanup_resources()
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
}

// ----------------
