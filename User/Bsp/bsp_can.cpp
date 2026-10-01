#include "bsp_can.hpp"
#include "bsp_cfg.hpp"

#include <string.h>


// ---------------- 中断回调函数 ----------------

extern "C"
{
  /**
   * @brief FDCAN 接收 FIFO0 中断回调
   *
   * @note HAL 在调用本回调前已把 RxFifo0ITs 里的标志全部写 1 清除，
   *       所以这里是这些事件唯一的处理机会。RF0N/RF0W/RF0F/RF0L 四位共用本回调。
   */
  void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
  {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (hfdcan == &hfdcan1)
    {
      bsp_can1.process_fifo0_isr(RxFifo0ITs, &xHigherPriorityTaskWoken);
    }
    else if (hfdcan == &hfdcan2)
    {
      bsp_can2.process_fifo0_isr(RxFifo0ITs, &xHigherPriorityTaskWoken);
    }
    else if (hfdcan == &hfdcan3)
    {
      bsp_can3.process_fifo0_isr(RxFifo0ITs, &xHigherPriorityTaskWoken);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }

  /**
   * @brief 硬件 TX FIFO 变空中断回调：FIFO 腾空后把缓冲里的帧继续写入
   *
   * @note 由 FDCAN_IT_TX_FIFO_EMPTY 触发（该中断按需开关，见 _set_tx_empty_it()）。
   */
  void HAL_FDCAN_TxFifoEmptyCallback(FDCAN_HandleTypeDef *hfdcan)
  {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (hfdcan == &hfdcan1)
    {
      bsp_can1.trigger_tx_from_isr(&xHigherPriorityTaskWoken);
    }
    else if (hfdcan == &hfdcan2)
    {
      bsp_can2.trigger_tx_from_isr(&xHigherPriorityTaskWoken);
    }
    else if (hfdcan == &hfdcan3)
    {
      bsp_can3.trigger_tx_from_isr(&xHigherPriorityTaskWoken);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }

  /**
   * @brief 总线错误状态回调（Bus-Off / 错误被动 / 错误警告）
   *
   * @note HAL 只在 IR 与 IE 同时置位时才进入本回调。这里只累加计数，恢复动作在任务侧。
   */
  void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
  {
    if (hfdcan == &hfdcan1)
    {
      bsp_can1.process_error_isr(ErrorStatusITs);
    }
    else if (hfdcan == &hfdcan2)
    {
      bsp_can2.process_error_isr(ErrorStatusITs);
    }
    else if (hfdcan == &hfdcan3)
    {
      bsp_can3.process_error_isr(ErrorStatusITs);
    }
  }
}


// ----------------
// ---------------- 类函数实现 ----------------

/** @brief 只绑定句柄，RTOS 资源在 init() 里创建 */
BspCan::BspCan(const Config &cfg) :
  _hfdcan(cfg.hfdcan),
  _tx_message_buffer(nullptr),
  _rx_message_buffer(nullptr),
  diagnostics {},
  _name(cfg.name),
  _tx_lock(nullptr),
  _recovery_started(0),
  _recovery_attempted(0)
{
}

/** @brief 先停外设并关中断，再释放软件资源 */
BspCan::~BspCan()
{
  _reset_hardware();
  _cleanup_resources();
}

/**
 * @brief 初始化：复位外设、创建收发消息缓冲与 TX 锁、配置滤波器并启动
 *
 * @return Status OK=成功；BAD_ARG=句柄为空；IO_ERROR=资源创建或硬件启动失败
 *
 * @note 可重复调用：开头先 _reset_hardware() + _cleanup_resources()；任一步失败都走
 *       _rollback_init()，不留下半初始化状态。
 */
Status BspCan::init()
{
  if (_hfdcan == nullptr)
  {
    return Status::BAD_ARG; // 句柄非法，后续所有 HAL 调用都不可用
  }

  // 可重复调用：先复位外设，再重建软件资源，最后重新配置/启动硬件
  _reset_hardware();
  _cleanup_resources();

  // 诊断计数清零（重复初始化 = 重新计数）
  diagnostics.bus_off_events     = 0;
  diagnostics.err_passive        = 0;
  diagnostics.err_warning        = 0;
  diagnostics.recovery_attempts  = 0;
  diagnostics.recovery_successes = 0;
  diagnostics.recovery_max_ticks = 0;
  diagnostics.recovering         = false;
  diagnostics.rx_dropped         = 0;
  diagnostics.rx_lost            = 0;
  diagnostics.rx_len_drop        = 0;
  diagnostics.tx_dropped         = 0;
  diagnostics.tx_buf_full        = 0;
  diagnostics.tx_stall_recover   = 0;
  diagnostics.tx_it_fail         = 0;
  _recovery_started              = 0;
  _recovery_attempted            = 0;

  // 创建接收消息缓冲区（深度见 RX_QUEUE_DEPTH；满即丢并计数，不做流控）
  _rx_message_buffer = xMessageBufferCreate((sizeof(CanRxMsg) + 4) * RX_QUEUE_DEPTH);
  if (_rx_message_buffer == nullptr)
  {
    _rollback_init();
    return Status::IO_ERROR;
  }

  // 创建发送消息缓冲区（深度见 TX_QUEUE_DEPTH）
  _tx_message_buffer = xMessageBufferCreate((sizeof(CanTxMsg) + 4) * TX_QUEUE_DEPTH);
  if (_tx_message_buffer == nullptr)
  {
    _rollback_init();
    return Status::IO_ERROR;
  }

  // 创建 TX 启动锁
  _tx_lock = xSemaphoreCreateMutex();
  if (_tx_lock == nullptr)
  {
    _rollback_init();
    return Status::IO_ERROR;
  }

  // 配滤波 + 启动外设 + 打开接收/错误状态通知
  if (_configure_hardware() != Status::OK)
  {
    _rollback_init();
    return Status::IO_ERROR;
  }

  return Status::OK;
}

/**
 * @brief 发送一帧标准帧（8 字节，非阻塞，入队后由中断送入硬件）
 *
 * @param std_id 标准帧 ID（11 位，0x000~0x7FF）
 * @param data   8 字节数据
 * @return Status OK=已入发送缓冲；FULL=缓冲满；BUSY=总线不可用；
 *                BAD_ARG=参数非法；NOT_INIT=未初始化
 */
Status BspCan::send(uint32_t std_id, const uint8_t *data)
{
  // 标准帧 ID 只有 11 位，超范围直接拒绝（否则会被硬件截断成另一个 ID）
  if (data == nullptr || std_id > 0x7FFU)
  {
    return Status::BAD_ARG;
  }
  if (_tx_message_buffer == nullptr || _hfdcan == nullptr)
  {
    return Status::NOT_INIT;
  }
  // Bus-Off / 恢复流程中帧发不到总线上，直接拒绝，由调用方决定是否重试
  if (!_tx_available())
  {
    return Status::BUSY;
  }

  CanTxMsg txMsg;
  txMsg.std_id = std_id;
  memcpy(txMsg.data, data, sizeof(txMsg.data));

  // 放入发送缓冲区（不阻塞）
  const size_t sent = xMessageBufferSend(_tx_message_buffer, &txMsg, sizeof(CanTxMsg), 0);

  if (sent != sizeof(CanTxMsg))
  {
    diagnostics.tx_buf_full++; // 缓冲满，这一帧被丢掉
    return Status::FULL;
  }

  // 入队成功才尝试发送（有锁，漏发由 tx_recover() 补发）
  (void)_start_transmission();
  return Status::OK;
}

/** @brief 从接收缓冲区取一帧（timeout_ms：0=不等待，portMAX_DELAY=一直等） */
Status BspCan::receive(CanRxMsg *msg, uint32_t timeout_ms)
{
  if (msg == nullptr)
  {
    return Status::BAD_ARG;
  }
  if (_hfdcan == nullptr || _rx_message_buffer == nullptr)
  {
    return Status::NOT_INIT; // 与 send() 保持一致，句柄/缓冲缺一不可
  }

  // portMAX_DELAY 不能直接丢给 pdMS_TO_TICKS（会溢出），单独处理
  const TickType_t wait = (timeout_ms == portMAX_DELAY) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);

  const size_t received = xMessageBufferReceive(_rx_message_buffer, msg, sizeof(CanRxMsg), wait);
  return (received > 0) ? Status::OK : Status::TIMEOUT;
}

/**
 * @brief TX 恢复：软件缓冲有帧、硬件 FIFO 有空位，但发送中断处于关闭状态时补一次发送
 *
 * @return true=本次确实恢复了一次
 *
 * @note 正常发送由 TX-FIFO-EMPTY 中断驱动；本函数只处理中断没触发的情况
 *       （写 FIFO 失败，或开关中断时丢了一次唤醒）。
 *       Bus-Off 期间直接返回，总线恢复交给 service_recovery()。
 */
bool BspCan::tx_recover()
{
  if (_hfdcan == nullptr)
  {
    return false;
  }

  // Bus-Off 时帧发不到总线上，只能滞留在软件缓冲里，以下检查无意义
  FDCAN_ProtocolStatusTypeDef ps = {};
  if (HAL_FDCAN_GetProtocolStatus(_hfdcan, &ps) == HAL_OK && ps.BusOff != 0U)
  {
    return false;
  }

  // 判断是否有帧待发、FIFO 是否有空位、发送中断是否处于关闭状态
  const bool has_pending   = (_tx_message_buffer != nullptr) && (!xMessageBufferIsEmpty(_tx_message_buffer));
  const bool fifo_has_room = (HAL_FDCAN_GetTxFifoFreeLevel(_hfdcan) > 0);
  const bool it_was_off    = ((_hfdcan->Instance->IE & FDCAN_IT_TX_FIFO_EMPTY) == 0U);

  // 缓冲被 ISR 搬空后中断会被关掉；若关掉发生在某次入队之后，这里重新打开。
  _update_tx_empty_it();

  if (!(has_pending && fifo_has_room))
  {
    return false; // 无帧待发或 FIFO 已满，不需要干预
  }

  if (!_start_transmission(0))
  {
    return false; // 拿不到锁或 FIFO 又满了，下轮再试
  }

  if (it_was_off)
  {
    diagnostics.tx_stall_recover++; // 诊断：中断确实处于关闭状态，记 1 次丢唤醒（正常抢占不计数）
  }
  return true;
}

/** @brief 任务上下文：总线是否可用于发送（已启动、非 Bus-Off、非恢复流程中） */
bool BspCan::_tx_available() const
{
  return _hfdcan != nullptr && _hfdcan->State == HAL_FDCAN_STATE_BUSY && !diagnostics.recovering && (_hfdcan->Instance->PSR & FDCAN_PSR_BO) == 0U && (_hfdcan->Instance->CCCR & FDCAN_CCCR_INIT) == 0U;
}

/**
 * @brief 总线恢复：Bus-Off 时按寄存器流程重启收发（非阻塞）
 *
 * @return Status OK=总线可用，BUSY=仍在恢复中，NOT_INIT=未初始化
 *
 * @note 只动硬件，不重新初始化外设、不清 message RAM、不动软件收发缓冲：
 *       进入 Bus-Off 时先把硬件里未发出的旧帧请求取消（避免恢复瞬间把过期控制帧发出去），
 *       并清掉软件缓冲里的积压帧；随后清 CCCR.INIT 让 M_CAN 自行重同步。
 * @note 限流：两次动作之间至少间隔 100 ms，避免总线长期故障时反复打断恢复序列。
 */
Status BspCan::service_recovery()
{
  if (_hfdcan == nullptr || _tx_message_buffer == nullptr || _rx_message_buffer == nullptr || _hfdcan->State != HAL_FDCAN_STATE_BUSY)
  {
    return Status::NOT_INIT;
  }

  const TickType_t now = xTaskGetTickCount();

  // 整个恢复流程在一个临界区里完成：排除发送任务/ISR 与接收任务
  taskENTER_CRITICAL();

  const bool bus_off = (_hfdcan->Instance->PSR & FDCAN_PSR_BO) != 0U;
  if (bus_off && !diagnostics.recovering)
  {
    diagnostics.recovering = true;
    diagnostics.bus_off_events++;
    _recovery_started   = now;
    _recovery_attempted = now - pdMS_TO_TICKS(100U);
    // 均为非阻塞缓冲
    xMessageBufferReset(_tx_message_buffer);
    xMessageBufferReset(_rx_message_buffer);
    // 取消硬件中旧控制帧，取消请求在重新启动后由硬件完成。
    _hfdcan->Instance->TXBCR = _hfdcan->Instance->TXBRP;
  }

  if (diagnostics.recovering)
  {
    // 只在硬件再次置 INIT 时启动恢复，避免反复打断 129x11 位恢复序列。
    if ((_hfdcan->Instance->CCCR & FDCAN_CCCR_INIT) != 0U && now - _recovery_attempted >= pdMS_TO_TICKS(100U))
    {
      _hfdcan->Instance->TXBCR = _hfdcan->Instance->TXBRP;
      CLEAR_BIT(_hfdcan->Instance->CCCR, FDCAN_CCCR_INIT);
      _recovery_attempted = now;
      diagnostics.recovery_attempts++;
    }
    // 先等旧发送请求彻底取消，才允许新的控制帧入队。
    if (!bus_off && (_hfdcan->Instance->CCCR & FDCAN_CCCR_INIT) == 0U && _hfdcan->Instance->TXBRP == 0U)
    {
      diagnostics.recovering = false;
      diagnostics.recovery_successes++;
      const TickType_t elapsed = now - _recovery_started;
      if (elapsed > diagnostics.recovery_max_ticks)
      {
        diagnostics.recovery_max_ticks = elapsed;
      }
    }
    else if (now - _recovery_attempted >= pdMS_TO_TICKS(100U))
    {
      // 尚未取消的发送请求继续请求取消，不忙等、不重新初始化外设。
      _hfdcan->Instance->TXBCR = _hfdcan->Instance->TXBRP;
    }
  }

  const bool recovering = diagnostics.recovering;
  taskEXIT_CRITICAL();

  return recovering ? Status::BUSY : Status::OK;
}

// ----------------
// ---------------- ISR 入口 ----------------

/**
 * @brief FIFO0 中断处理：按事件位分发 + 循环排空硬件 FIFO（ISR）
 *
 * @param its 本次触发的中断位（RF0N / RF0L 等，由 HAL 传入）
 * @param pxHigherPriorityTaskWoken 中断处理后可能唤醒的高优先级任务
 */
void BspCan::process_fifo0_isr(uint32_t its, BaseType_t *pxHigherPriorityTaskWoken)
{
  if (_hfdcan == nullptr || _rx_message_buffer == nullptr)
  {
    return; // 未初始化
  }

  // FIFO0 溢出丢帧
  if ((its & FDCAN_IT_RX_FIFO0_MESSAGE_LOST) != 0U)
  {
    diagnostics.rx_lost++;
  }

  // 没有新帧则返回
  if ((its & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U)
  {
    return;
  }

  uint32_t read_cnt = 0U;

  // HAL_FDCAN_GetRxMessage 按 DLC 拷贝 DLCtoBytes[] 个字节（DLC=15 时为 64 字节）
  // 而 CanRxMsg::data 只有 8 字节，故先用 64 字节缓冲接收，再把合法长度的帧拷进去。
  constexpr uint32_t CAN_RX_RAW_BYTES = 64U;
  uint8_t            raw[CAN_RX_RAW_BYTES];

  // 单次中断最多搬运的帧数，取硬件 FIFO 深度
  const uint32_t drain_max = _hfdcan->Init.RxFifo0ElmtsNbr;

  // RF0N 是"写入一帧"的事件标志，HAL 已在回调前清除：若本次只读一帧，
  // FIFO 里剩下的帧不会再触发中断，所以这里一次把 FIFO 读空。
  while (HAL_FDCAN_GetRxFifoFillLevel(_hfdcan, FDCAN_RX_FIFO0) > 0U)
  {
    if (read_cnt >= drain_max)
    {
      break; // 达到单次上限，剩余帧等下次中断再搬
    }

    CanRxMsg rxMsg;
    if (HAL_FDCAN_GetRxMessage(_hfdcan, FDCAN_RX_FIFO0, &rxMsg.header, raw) != HAL_OK)
    {
      break; // 读失败立即跳出，避免死循环
    }
    read_cnt++;

    // 只支持经典 CAN 的 8 字节帧：DLC > 8（FD 帧或非法 DLC）直接丢弃并计数
    if (rxMsg.header.DataLength > FDCAN_DLC_BYTES_8)
    {
      diagnostics.rx_len_drop++;
      continue;
    }
    memcpy(rxMsg.data, raw, sizeof(rxMsg.data));

    // 软件缓冲满时只丢弃并计数，不退出循环：硬件 FIFO 仍要读空，
    // 否则 FIFO 满后不再产生 RF0N，剩下的帧会一直留在 FIFO 里。
    if (xMessageBufferSendFromISR(_rx_message_buffer, &rxMsg, sizeof(CanRxMsg), pxHigherPriorityTaskWoken) == 0U)
    {
      diagnostics.rx_dropped++;
    }

    // 循环内不 yield，搬完统一在回调返回前让出
  }
}

/**
 * @brief TX FIFO 变空续传（ISR，由 TX FIFO Empty 中断调用）
 *
 * @note 这里不加 _tx_lock：本回调只在「硬件 TX FIFO 刚变空」时被调用，
 *       且任务不可能在中断执行期间运行；即使与任务侧的 _start_transmission()
 *       发生空位检查的 TOCTOU，写入失败也会回滚到软件缓冲（见下方），不会丢帧。
 *
 * @param pxHigherPriorityTaskWoken 需初始化为pdFALSE，若唤醒高优先级任务则置为pdTRUE
 */
void BspCan::trigger_tx_from_isr(BaseType_t *pxHigherPriorityTaskWoken)
{
  if (_tx_message_buffer == nullptr || _hfdcan == nullptr)
  {
    return; // 未初始化
  }

  if (HAL_FDCAN_GetTxFifoFreeLevel(_hfdcan) > 0)
  {
    CanTxMsg txMsg;
    size_t   len = xMessageBufferReceiveFromISR(_tx_message_buffer, &txMsg, sizeof(CanTxMsg), pxHigherPriorityTaskWoken);
    if (len > 0)
    {
      FDCAN_TxHeaderTypeDef txHeader;
      _fill_tx_header(txHeader, txMsg);

      if (HAL_FDCAN_AddMessageToTxFifoQ(_hfdcan, &txHeader, txMsg.data) != HAL_OK)
      {
        // 写入失败：把刚取出的帧写回软件缓冲，避免丢帧；
        // 恢复动作由巡检任务（sys_task）完成。
        (void)xMessageBufferSendFromISR(_tx_message_buffer, &txMsg, sizeof(CanTxMsg), pxHigherPriorityTaskWoken);
        diagnostics.tx_dropped++;
      }
    }
  }

  // 按缓冲现状开关中断：缓冲空了就关掉，避免 FIFO 每次变空都进中断。
  // ISR 里用 _set_tx_empty_it()：任务不能在中断执行期间运行，无需临界区。
  _set_tx_empty_it(!xMessageBufferIsEmpty(_tx_message_buffer));
}

/**
 * @brief 错误状态中断（ISR）：只累加计数，不做恢复动作
 * @param its 本次错误状态中断位（HAL 已按 IR & IE 过滤，只可能是 BO / EP / EW）
 */
void BspCan::process_error_isr(uint32_t its)
{
  // 注意：IR.BO 是「状态变化」事件，进入与退出 Bus-Off 各触发一次
  if ((its & FDCAN_IT_BUS_OFF) != 0U)
  {
    diagnostics.bus_off_events++;
  }
  if ((its & FDCAN_IT_ERROR_PASSIVE) != 0U)
  {
    diagnostics.err_passive++;
  }
  if ((its & FDCAN_IT_ERROR_WARNING) != 0U)
  {
    diagnostics.err_warning++;
  }
}

// ----------------
// ---------------- 私有实现 ----------------

/**
 * @brief 开关 TX-FIFO-EMPTY 中断（可在 ISR 中调用，函数内不含临界区）
 *
 * @param enable true=打开，false=关闭
 *
 * @return true=已按预期写入；false=句柄为空，或 HAL 被占用（HAL_BUSY）未写入
 *
 * @note 用途：软件缓冲里还有帧、但硬件 TX FIFO 已满，需要等 FIFO 有空位时继续发送。
 *       所以使能状态跟随软件缓冲：有帧就开，没帧就关。
 *       一直开着的问题是：FIFO 每次变空都会进中断（即使没有帧要发）；
 *       如果 IR.TFE 是电平型标志（FIFO 空就置位），还会持续触发中断。
 *
 * @note 本函数不含临界区：任务侧调用要走 _update_tx_empty_it()，
 *       ISR 侧不需要（任务不会在中断执行期间运行）。
 */
bool BspCan::_set_tx_empty_it(bool enable)
{
  if (_hfdcan == nullptr)
  {
    return false; // 未初始化
  }

  const HAL_StatusTypeDef st = enable
                                 ? HAL_FDCAN_ActivateNotification(_hfdcan, FDCAN_IT_TX_FIFO_EMPTY, 0)
                                 : HAL_FDCAN_DeactivateNotification(_hfdcan, FDCAN_IT_TX_FIFO_EMPTY);

  if (st != HAL_OK)
  {
    diagnostics.tx_it_fail++; // 诊断：中断开关未写入（等巡检补开）
    return false;
  }
  return true;
}

/**
 * @brief 任务上下文：根据软件缓冲是否还有帧，开关 TX-FIFO-EMPTY 中断（临界区保护）
 *
 * @note "判断缓冲是否为空"和"写 IE 寄存器"必须作为一个整体执行。若中间被 FDCAN 中断抢占，
 *       中断里刚使能的 TX_FIFO_EMPTY 会被任务写回的旧值覆盖，中断变成关闭状态，
 *       缓冲里的帧只能等下一次 tx_recover() 补发（不丢帧，但延迟一个周期）。
 */
void BspCan::_update_tx_empty_it()
{
  if (_hfdcan == nullptr || _tx_message_buffer == nullptr)
  {
    return;
  }

  taskENTER_CRITICAL();
  _set_tx_empty_it(!xMessageBufferIsEmpty(_tx_message_buffer));
  taskEXIT_CRITICAL();
}

/**
 * @brief 尝试向硬件 TX FIFO 写入一帧（任务上下文）
 *
 * @param wait 取锁等待时间，单位 ticks；send() 传 portMAX_DELAY，巡检传 0
 * @return true=本次确实写入了一帧
 */
bool BspCan::_start_transmission(TickType_t wait)
{
  if (_tx_lock == nullptr)
  {
    return false; // 互斥量未创建
  }

  // "检查空位 → 取帧 → 写硬件 FIFO" 三步要作为一个整体，否则多个任务（或被中断抢占）
  // 可能同时通过空位检查，后写入的会失败并丢帧。
  // wait=0（巡检）时取不到锁就返回，不阻塞周期任务。
  if (xSemaphoreTake(_tx_lock, wait) != pdTRUE)
  {
    return false;
  }

  bool started = false;

  if (_tx_message_buffer != nullptr && _hfdcan != nullptr && HAL_FDCAN_GetTxFifoFreeLevel(_hfdcan) > 0)
  {
    CanTxMsg txMsg;
    size_t   len = xMessageBufferReceive(_tx_message_buffer, &txMsg, sizeof(CanTxMsg), 0);
    if (len > 0)
    {
      FDCAN_TxHeaderTypeDef txHeader;
      _fill_tx_header(txHeader, txMsg);

      if (HAL_FDCAN_AddMessageToTxFifoQ(_hfdcan, &txHeader, txMsg.data) != HAL_OK)
      {
        // 写入失败：帧已从软件缓冲取出，要放回去，否则丢帧。
        // 刚取出 1 帧，缓冲至少有 1 帧空间，放回不会失败。
        (void)xMessageBufferSend(_tx_message_buffer, &txMsg, sizeof(CanTxMsg), 0);
        diagnostics.tx_dropped++;
      }
      else
      {
        started = true;
      }
    }
  }

  xSemaphoreGive(_tx_lock);

  // 无论本次是否写入成功，都根据缓冲现状更新中断使能：
  // 例如 FIFO 满导致本次没写成功，但缓冲里还有帧，中断要保持打开。
  _update_tx_empty_it();

  return started;
}

/** @brief 构造 8 字节经典帧的发送头（任务与 ISR 共用，避免两处不一致） */
void BspCan::_fill_tx_header(FDCAN_TxHeaderTypeDef &header, const CanTxMsg &msg)
{
  header.Identifier          = msg.std_id;
  header.IdType              = FDCAN_STANDARD_ID;
  header.TxFrameType         = FDCAN_DATA_FRAME;
  header.DataLength          = FDCAN_DLC_BYTES_8;
  header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  header.BitRateSwitch       = FDCAN_BRS_OFF;
  header.FDFormat            = FDCAN_CLASSIC_CAN;
  header.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
  header.MessageMarker       = 0;
}

/** @brief 释放所有已创建的 FreeRTOS 资源（消息缓冲区、锁） */
void BspCan::_cleanup_resources()
{
  // 释放接收消息缓冲区
  if (_rx_message_buffer != nullptr)
  {
    vMessageBufferDelete(_rx_message_buffer);
    _rx_message_buffer = nullptr;
  }

  // 释放发送消息缓冲区
  if (_tx_message_buffer != nullptr)
  {
    vMessageBufferDelete(_tx_message_buffer);
    _tx_message_buffer = nullptr;
  }

  // 释放 TX 启动锁
  if (_tx_lock != nullptr)
  {
    vSemaphoreDelete(_tx_lock);
    _tx_lock = nullptr;
  }
}

/**
 * @brief 停止外设并关闭本驱动使用的全部中断（可重复调用）
 *
 * @note 先关中断再停外设，避免关闭过程中仍有中断进来。
 *       HAL_FDCAN_Stop() 会置 CCCR.INIT/CCE 并等其生效（State 变为 READY），
 *       它不清空硬件 TX FIFO：重启后 FIFO 里残留的帧可能被重发一次
 *       （AutoRetransmission=ENABLE）；软件收发缓冲不受影响。
 *
 * @note 未初始化时（State 已是 READY）Stop 返回 HAL_ERROR，属于正常情况。
 */
void BspCan::_reset_hardware()
{
  if (_hfdcan == nullptr)
  {
    return;
  }

  (void)HAL_FDCAN_DeactivateNotification(_hfdcan,
                                         FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO0_MESSAGE_LOST | FDCAN_IT_TX_FIFO_EMPTY | FDCAN_IT_BUS_OFF | FDCAN_IT_ERROR_PASSIVE | FDCAN_IT_ERROR_WARNING);
  (void)HAL_FDCAN_Stop(_hfdcan);
}

/** @brief 初始化失败收尾：复位外设 + 释放软件资源 */
void BspCan::_rollback_init()
{
  _reset_hardware(); // 先停外设、关中断，再释放缓冲
  _cleanup_resources();
}

/**
 * @brief 配置滤波器、启动外设、打开接收与错误状态中断
 *
 * @note 必须在 _reset_hardware() 之后调用：Stop 已置 CCCR.INIT/CCE，
 *       此时才能写 message RAM（滤波器）。
 *
 * @note 不使用 TX_COMPLETE 中断：FIFO 模式下 BufferIndexes 传 0 不会产生发送完成回调，
 *       发送流程只靠 FIFO 中断驱动。
 */
Status BspCan::_configure_hardware()
{
  // 滤波器：标准帧全 ID 收进 RX FIFO0（本驱动不做 ID 级过滤）
  FDCAN_FilterTypeDef sFilterConfig = {}; // 未用到的字段也清零，避免 HAL 版本差异踩到脏值
  sFilterConfig.IdType              = FDCAN_STANDARD_ID;
  sFilterConfig.FilterIndex         = 0;
  sFilterConfig.FilterType          = FDCAN_FILTER_RANGE;
  sFilterConfig.FilterConfig        = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1           = 0x000;
  sFilterConfig.FilterID2           = 0x7FF;

  if (HAL_FDCAN_ConfigFilter(_hfdcan, &sFilterConfig) != HAL_OK)
  {
    return Status::IO_ERROR;
  }

  if (HAL_FDCAN_ConfigGlobalFilter(_hfdcan,
                                   FDCAN_ACCEPT_IN_RX_FIFO0,
                                   FDCAN_ACCEPT_IN_RX_FIFO0,
                                   FDCAN_FILTER_REMOTE,
                                   FDCAN_FILTER_REMOTE)
      != HAL_OK)
  {
    return Status::IO_ERROR;
  }

  if (HAL_FDCAN_Start(_hfdcan) != HAL_OK)
  {
    return Status::IO_ERROR;
  }

  // FIFO0 新消息 + FIFO0 溢出丢帧
  if (HAL_FDCAN_ActivateNotification(_hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO0_MESSAGE_LOST, 0) != HAL_OK)
  {
    return Status::IO_ERROR;
  }

  // 总线错误状态（Bus-Off / 错误被动 / 错误警告）：只计数，恢复动作在任务侧
  if (HAL_FDCAN_ActivateNotification(_hfdcan, FDCAN_IT_BUS_OFF | FDCAN_IT_ERROR_PASSIVE | FDCAN_IT_ERROR_WARNING, 0) != HAL_OK)
  {
    return Status::IO_ERROR;
  }

  // TX-FIFO-EMPTY 不在此处打开，由 _set_tx_empty_it()/_update_tx_empty_it() 按需开关
  return Status::OK;
}

// ----------------
