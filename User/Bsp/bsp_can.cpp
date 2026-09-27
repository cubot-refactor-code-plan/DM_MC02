#include "bsp_can.hpp"
#include "bsp_cfg.hpp"

#include <string.h>


///< 承接 HAL 按 DLC 码拷贝的最大字节数（DLCtoBytes[] 最大 64，见 HAL_FDCAN_GetRxMessage）
#define CAN_RX_RAW_BYTES 64U

///< bus_recover() 两次重启之间的最小间隔（ms），防止总线长期故障时反复抖动
#define BUS_RECOVER_MIN_GAP_MS 100U


// ---------------- 中断回调函数 ----------------

extern "C"
{
  /**
   * @brief FDCAN接收FIFO0中断回调
   *
   * @note HAL 在调用本回调之前，已把RxFifo0ITs中的标志全部写 1 清除，
   *       所以这里是这些事件唯一的处理机会。RF0N/RF0W/RF0F/RF0L四位共用本回调。
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
   * @brief FDCAN 硬件 TX FIFO 变空中断回调 （fifo变空，驱动就再往fifo里面塞数据）
   *
   * @note 打开的是 FDCAN_IT_TX_FIFO_EMPTY，
   *       HAL 在该中断里调的是 TxFifoEmptyCallback(hfdcan)（单参数）。
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
   * @note HAL 只在 IR 与 IE 同时置位时才进入本回调，且传入的 ErrorStatusITs
   *       已由 IR & (EP|EW|BO) 过滤（见 HAL_FDCAN_IRQHandler），故这里只累计次数
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
// ---------------- 公有接口 ----------------

BspCan::BspCan(const Config &cfg) : _hfdcan(cfg.hfdcan)
{
  // 句柄与锁由类内默认初始化 = nullptr，运行时资源创建推迟到 init()
}

BspCan::~BspCan()
{
  // 先摘通知并停外设，再拆软件资源：否则删缓冲后中断仍可能进来访问空句柄
  reset_hardware();
  cleanup_resources();
}

Status BspCan::init()
{
  if (_hfdcan == nullptr)
  {
    return Status::BAD_ARG; // 句柄非法，后续所有 HAL 调用都不可用
  }

  // 可重复调用：先复位外设（停外设 + 摘通知）并释放上次残留，再重建软件资源，
  // 最后重新配置/启动硬件。任一环节失败都回滚到「外设已停、通知已摘、
  // 无软件缓冲」的干净未初始化态，不留“外设还在跑但没人收”的僵尸态。
  reset_hardware();
  cleanup_resources();

  // 诊断计数清零（重复初始化 = 重新计数）
  _rx_sw_drop_cnt    = 0;
  _rx_lost_cnt       = 0;
  _rx_len_drop_cnt   = 0;
  _tx_drop_cnt       = 0;
  _tx_fifo_fail_cnt  = 0;
  _tx_stall_rec_cnt  = 0;
  _tx_it_fail_cnt    = 0;
  _bus_off_cnt       = 0;
  _err_passive_cnt   = 0;
  _err_warning_cnt   = 0;
  _bus_rec_cnt       = 0;
  _last_bus_rec_tick = 0;

  // 创建接收消息缓冲区（深度见 RX_QUEUE_DEPTH；满即丢并计数，不做流控）
  _rx_message_buffer = xMessageBufferCreate((sizeof(CanRxMsg) + 4) * RX_QUEUE_DEPTH);
  if (_rx_message_buffer == nullptr)
  {
    rollback_init();
    return Status::IO_ERROR;
  }

  // 创建发送消息缓冲区（深度见 TX_QUEUE_DEPTH）
  _tx_message_buffer = xMessageBufferCreate((sizeof(CanTxMsg) + 4) * TX_QUEUE_DEPTH);
  if (_tx_message_buffer == nullptr)
  {
    rollback_init();
    return Status::IO_ERROR;
  }

  // 创建 TX 启动锁（任务侧串行化「判有空位 → 取帧 → 写入硬件 FIFO」）
  _tx_lock = xSemaphoreCreateMutex();
  if (_tx_lock == nullptr)
  {
    rollback_init();
    return Status::IO_ERROR;
  }

  // 配滤波 + 启动外设 + 打开接收/错误状态通知（TX 接力中断按需开关）
  if (configure_hardware() != Status::OK)
  {
    rollback_init();
    return Status::IO_ERROR;
  }

  return Status::OK;
}

Status BspCan::send(uint32_t std_id, const uint8_t *data)
{
  // 标准帧 ID 只有 11 位，超范围直接拒绝（否则会被硬件截断成另一个 ID）
  if (data == nullptr || std_id > 0x7FFU)
  {
    return Status::BAD_ARG;
  }
  if (_tx_message_buffer == nullptr)
  {
    return Status::NOT_INIT;
  }

  CanTxMsg txMsg;
  txMsg.std_id = std_id;
  memcpy(txMsg.data, data, sizeof(txMsg.data));

  // 放入发送缓冲区（不阻塞）
  size_t sent = xMessageBufferSend(_tx_message_buffer, &txMsg, sizeof(CanTxMsg), 0);

  if (sent != sizeof(CanTxMsg))
  {
    _tx_drop_cnt++; // 缓冲满，这一帧被丢掉（调用方常常不看返回值，必须留痕）
    return Status::FULL;
  }

  // 入队成功才尝试送出（送不出去时由 tx_recover() 巡检兜底）
  (void)start_transmission();
  return Status::OK;
}

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

  // portMAX_DELAY 是 tick 语义的哨兵，若直接丢给 pdMS_TO_TICKS 会被换算成一个
  // 巨大的 ms 值（约 71 分钟），必须原样传下去才是真正的「一直等」。
  const TickType_t ticks = (timeout_ms == portMAX_DELAY) ? portMAX_DELAY
                                                         : pdMS_TO_TICKS(timeout_ms);

  size_t received = xMessageBufferReceive(_rx_message_buffer, msg, sizeof(CanRxMsg), ticks);
  return (received > 0) ? Status::OK : Status::TIMEOUT;
}

/**
 * @brief TX 断链兜底：若有帧待发而硬件 TX FIFO 有空位（没人去送），则踢一脚
 *
 * @note 发送靠 TX-FIFO-EMPTY 中断接力排空软件缓冲；一旦某一环失败（写 FIFO 出错、
 *       或中断没触发），软件缓冲里的帧就再也送不出去。本函数由周期任务调用兜底。
 *
 * @return true=本次确实救回一次
 */
bool BspCan::tx_recover()
{
  if (_hfdcan == nullptr)
  {
    return false;
  }

  // Bus-Off 期间直接返回：总线故障时帧只能塞进硬件 FIFO、发不到总线上，
  // 计成「救回」会让诊断量虚高并把 FIFO 堆满。总线恢复交给 bus_recover()。
  // （GetProtocolStatus 是纯寄存器读，不阻塞，10 ms 巡检调用无压力）
  FDCAN_ProtocolStatusTypeDef ps = {};
  if (HAL_FDCAN_GetProtocolStatus(_hfdcan, &ps) == HAL_OK && ps.BusOff != 0U)
  {
    return false;
  }

  // 断链判据：「软件缓冲有帧要发」+「硬件 TX FIFO 有空位」= 没人去送
  const bool has_pending   = (_tx_message_buffer != nullptr) && (!xMessageBufferIsEmpty(_tx_message_buffer));
  const bool fifo_has_room = (HAL_FDCAN_GetTxFifoFreeLevel(_hfdcan) > 0);

  // 「真丢唤醒」的证据：有帧要发，而接力中断却是关的（唯一正确的稳态是「有帧⇒中断开」）。
  // 必须先采样再 update_tx_empty_it()，否则采到的是自己刚补开的状态。
  const bool it_was_off = ((_hfdcan->Instance->IE & FDCAN_IT_TX_FIFO_EMPTY) == 0U);

  // 兜底一：接力中断的使能由「软件缓冲是否还有帧」决定。
  // ISR 把缓冲搬空后会关掉它；万一「关」发生在某次入队之后，这里把它补开。
  update_tx_empty_it();

  if (!(has_pending && fifo_has_room))
  {
    return false; // 未断链，什么都不做
  }

  if (!start_transmission(0))
  {
    return false; // 拿不到锁或 FIFO 又满了，下轮再试
  }

  if (it_was_off)
  {
    _tx_stall_rec_cnt++; // 诊断：确实补回了 1 次丢唤醒（正常抢占不计数）
  }
  return true;
}

/**
 * @brief 总线异常兜底：处于 Bus-Off 时重启外设（停外设 → 重新配置 → 上线）
 *
 * @note M_CAN 在 CCCR.INIT=0 时本就会自行尝试恢复（等 128×11 个隐性位），
 *       本函数只是「长时间卡在 Bus-Off 时踢一脚」，所以用 BUS_RECOVER_MIN_GAP_MS
 *       限流：总线真的坏了的时候，反复 Stop/Start 只会带来无意义的抖动。
 *
 * @note 只动硬件，不动软件收发缓冲 —— 缓冲里待发的帧重启后会继续发出。
 *
 * @return true=本次确实重启了一次
 */
bool BspCan::bus_recover()
{
  if (_hfdcan == nullptr)
  {
    return false;
  }

  FDCAN_ProtocolStatusTypeDef ps = {};
  if (HAL_FDCAN_GetProtocolStatus(_hfdcan, &ps) != HAL_OK)
  {
    return false;
  }
  if (ps.BusOff == 0U)
  {
    return false; // 不在 Bus-Off，无需干预
  }

  const TickType_t now = xTaskGetTickCount();
  if ((now - _last_bus_rec_tick) < pdMS_TO_TICKS(BUS_RECOVER_MIN_GAP_MS))
  {
    return false; // 距上次重启太近，等下一轮（也避免重启风暴）
  }
  _last_bus_rec_tick = now;

  // 重启：停在初始化态（协议引擎一并复位）后重新配置并上线
  reset_hardware();
  if (configure_hardware() != Status::OK)
  {
    return false;
  }

  // reset_hardware() 把 TX-FIFO-EMPTY 接力中断摘掉了，这里按缓冲现状补回来，
  // 否则重启后 TX 只能靠 tx_recover() 每 10 ms 搬一帧。
  update_tx_empty_it();

  _bus_rec_cnt++;
  return true;
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

  // 溢出留痕：FIFO 顶满时硬件丢弃新帧，这里只记数，不改变数据流。
  if ((its & FDCAN_IT_RX_FIFO0_MESSAGE_LOST) != 0U)
  {
    _rx_lost_cnt++;
  }

  if ((its & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U)
  {
    return; // 本次不是「有新消息」，无帧可搬
  }

  uint32_t read_cnt = 0U;

  // HAL_FDCAN_GetRxMessage 按 DLC 码拷贝 DLCtoBytes[] 个字节（DLC=15 时 64 字节），
  // 而 CanRxMsg::data 只有 8 字节 —— 必须先用 64 字节临时缓冲承接，
  // 再把长度合法的帧拷进 CanRxMsg，否则 ISR 栈会被写穿。
  uint8_t raw[CAN_RX_RAW_BYTES];

  // 上限 = 硬件 FIFO 深度。看似多余，其实是正确性保证：
  // 只有「搬运过程中又有新帧到达」才可能读到上限，而新帧到达必然重新置位 RF0N，
  // 于是中断会再进一次，剩下的帧下一轮继续搬 —— 所以截断不会造成丢帧。
  const uint32_t drain_max = _hfdcan->Init.RxFifo0ElmtsNbr;

  // 循环排空：RF0N 是「刚写入一帧」的【事件标志】，且 HAL 已在回调前清除它。
  // 若一次中断只搬一帧，FIFO 里剩余的帧既无标志也无中断提醒，会永久滞留
  // （每次 ISR 被延迟造成的亏空都补不回来，最终顶满 FIFO 丢帧）。
  // 这里改用【状态量】做循环条件：读一次填充量 → 搬一帧 → 再读，直到真为空。
  // 搬一帧只需几十个 CPU 周期，而一帧 CAN 最短 111 us，必定追得平。
  while (HAL_FDCAN_GetRxFifoFillLevel(_hfdcan, FDCAN_RX_FIFO0) > 0U)
  {
    if (read_cnt >= drain_max)
    {
      break; // 见上方注释：此处截断不会丢帧
    }

    CanRxMsg rxMsg;
    if (HAL_FDCAN_GetRxMessage(_hfdcan, FDCAN_RX_FIFO0, &rxMsg.header, raw) != HAL_OK)
    {
      break; // 读失败立即跳出，避免死循环
    }
    read_cnt++;

    // 本驱动只支持经典 CAN 的 8 字节帧：长度超过 8 字节（DLC 9~15、FD 帧）直接丢弃。
    // 既避免残缺数据交给上层，也保证下面的 8 字节拷贝安全。
    if (rxMsg.header.DataLength > FDCAN_DLC_BYTES_8)
    {
      _rx_len_drop_cnt++;
      continue; // 继续排空硬件 FIFO，不留帧
    }
    memcpy(rxMsg.data, raw, sizeof(rxMsg.data));

    // 软件侧积压：满了就丢并计数（对应 UART 的 _rx_drop_bytes）。
    // 这里【继续排空硬件 FIFO】而不停下：停下会让硬件 FIFO 也顶满，
    // 顶满后不再产生 RF0N 事件，等于把「永久滞留」的坑再挖一遍。
    if (xMessageBufferSendFromISR(_rx_message_buffer,
                                  &rxMsg,
                                  sizeof(CanRxMsg),
                                  pxHigherPriorityTaskWoken)
        == 0U)
    {
      _rx_sw_drop_cnt++;
    }

    // 循环内不 yield：复用同一个标志，搬完一批由 extern "C" 外壳统一让出
  }
}

/**
 * @brief TX FIFO 变空续传（ISR上下文，由 TX FIFO Empty 中断调用）
 *
 * @note 这里不加 _tx_lock：本回调只在「硬件 TX FIFO 刚变空」时被调用，
 *       且任务不可能在中断执行期间运行；即使与任务侧的 start_transmission()
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
      fill_tx_header(txHeader, txMsg);

      if (HAL_FDCAN_AddMessageToTxFifoQ(_hfdcan, &txHeader, txMsg.data) != HAL_OK)
      {
        // 写入失败：把刚取出的帧塞回软件缓冲，绝不静默丢失。
        // 链在这里断掉，由巡检任务（见 sys_task）重新点火。
        (void)xMessageBufferSendFromISR(_tx_message_buffer, &txMsg, sizeof(CanTxMsg), pxHigherPriorityTaskWoken);
        _tx_fifo_fail_cnt++;
      }
    }
  }

  // 搬完一轮后按缓冲现状校正接力中断：搬空了就关掉，避免白挨下一次中断。
  // ISR 内用 set_tx_empty_it()：任务不可能在中断执行期间插进来，无需临界区
  set_tx_empty_it(!xMessageBufferIsEmpty(_tx_message_buffer));
}

/**
 * @brief 错误状态中断处理（ISR）：只累计次数，不做恢复动作
 * @param its 本次错误状态中断位（HAL 已按 IR & IE 过滤，只可能是 BO / EP / EW）
 */
void BspCan::process_error_isr(uint32_t its)
{
  // 注意：IR.BO 是「状态变化」事件，进入与退出 Bus-Off 各触发一次
  if ((its & FDCAN_IT_BUS_OFF) != 0U)
  {
    _bus_off_cnt++;
  }
  if ((its & FDCAN_IT_ERROR_PASSIVE) != 0U)
  {
    _err_passive_cnt++;
  }
  if ((its & FDCAN_IT_ERROR_WARNING) != 0U)
  {
    _err_warning_cnt++;
  }
}

// ----------------
// ---------------- 查询接口 ----------------

/**
 * @brief 读取 RX 诊断计数
 */
BspCan::RxDiag BspCan::rx_diag() const
{
  RxDiag diag;
  diag.sw_drop_cnt  = _rx_sw_drop_cnt;
  diag.lost_cnt     = _rx_lost_cnt;
  diag.len_drop_cnt = _rx_len_drop_cnt;
  return diag;
}

/**
 * @brief 读取 TX 诊断计数
 */
BspCan::TxDiag BspCan::tx_diag() const
{
  TxDiag diag;
  diag.drop_cnt          = _tx_drop_cnt;
  diag.fifo_fail_cnt     = _tx_fifo_fail_cnt;
  diag.stall_recover_cnt = _tx_stall_rec_cnt;
  diag.it_fail_cnt       = _tx_it_fail_cnt;
  return diag;
}

/**
 * @brief 读取总线错误状态快照
 *
 * @note 前 6 项是硬件寄存器的实时值（HAL PSR / 错误计数器），后 4 项是本驱动的累计次数。
 *       注意 bus_off 只反映「此刻是否在 Bus-Off」；想知道「历史上是否去过」，看 bus_off_cnt。
 */
BspCan::BusStatus BspCan::bus_status() const
{
  BusStatus status = {};

  FDCAN_ProtocolStatusTypeDef ps = {};
  FDCAN_ErrorCountersTypeDef  ec = {};

  if (_hfdcan != nullptr)
  {
    (void)HAL_FDCAN_GetProtocolStatus(_hfdcan, &ps);
    (void)HAL_FDCAN_GetErrorCounters(_hfdcan, &ec);
  }

  status.bus_off           = (ps.BusOff != 0U);
  status.error_passive     = (ps.ErrorPassive != 0U);
  status.error_warning     = (ps.Warning != 0U);
  status.last_error_code   = ps.LastErrorCode;
  status.tec               = ec.TxErrorCnt;
  status.rec               = ec.RxErrorCnt;
  status.bus_off_cnt       = _bus_off_cnt;
  status.error_passive_cnt = _err_passive_cnt;
  status.error_warning_cnt = _err_warning_cnt;
  status.bus_rec_cnt       = _bus_rec_cnt;

  return status;
}

// ----------------
// ---------------- 私有实现 ----------------

/**
 * @brief 开关 TX-FIFO-EMPTY 接力中断（可 ISR 调用，自身不含额外临界区）
 *
 * @param enable true=打开，false=关闭
 * @return true=寄存器已按预期写好；false=句柄为空 或 HAL 被锁（HAL_BUSY），本次没写进去
 *
 * @note 该中断只为一件事存在：软件缓冲还有帧、而硬件 FIFO 之前满了，
 *       等 FIFO 腾空时把剩下的帧灌进去。所以必须「有帧才开、没帧就关」：
 *       常开会带来两类问题 —— 总线空闲时每次发送都白挨一次「FIFO 变空」中断；
 *       更糟的是若该标志是电平型（FIFO 空就置位），将形成中断风暴。
 *
 * @note 返回值必须检查：HAL_FDCAN_ActivateNotification() 内部带 __HAL_LOCK，
 *       被别的 HAL 调用占用时返回 HAL_BUSY —— 此时中断并没有开成，不能静默放过。
 *       返回 false 不算致命：缓冲里的帧由 tx_recover()（10 ms 巡检）补开并搬运，
 *       不丢帧，只是多一拍延迟；失败次数记在 tx_diag().it_fail_cnt。
 *
 * @note 本函数自身不含临界区：任务侧必须走 update_tx_empty_it()，
 *       ISR 侧无需保护（任务无法在中断执行期间插进来）。
 */
bool BspCan::set_tx_empty_it(bool enable)
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
    _tx_it_fail_cnt++; // 诊断：接力中断没写成（等巡检补开），不再静默
    return false;
  }
  return true;
}

/**
 * @brief 任务上下文：按「软件缓冲是否还有帧」校正接力中断（临界区保护）
 *
 * @note 「判空 + 写 IE」必须整体原子。若在改写 IE 时被 FDCAN 中断抢占，
 *       ISR 刚使能的 TX_FIFO_EMPTY 位会被任务写回的旧值冲掉 —— 接力中断变「关」，
 *       缓冲里的帧只能等下一轮 tx_recover() 兜底（不丢帧，但多一拍延迟）。
 */
void BspCan::update_tx_empty_it()
{
  if (_hfdcan == nullptr || _tx_message_buffer == nullptr)
  {
    return;
  }

  taskENTER_CRITICAL();
  set_tx_empty_it(!xMessageBufferIsEmpty(_tx_message_buffer));
  taskEXIT_CRITICAL();
}

/**
 * @brief 主动尝试向硬件 TX FIFO 送出一帧（任务上下文）
 *
 * @param wait 取锁等待，单位 ticks；send() 用 portMAX_DELAY，巡检用 0
 * @return true=本次真的送出了一帧
 */
bool BspCan::start_transmission(TickType_t wait)
{
  if (_tx_lock == nullptr)
  {
    return false; // 互斥量未创建
  }

  // 判有空位 + 取帧 + 写硬件 FIFO 必须整体原子：否则两个任务（或被 ISR 抢占）
  // 可能都通过空位检查，后写入的那个必然失败并把帧丢掉。
  // wait=0（巡检兜底）时拿不到锁立即返回，绝不阻塞周期任务。
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
      fill_tx_header(txHeader, txMsg);

      if (HAL_FDCAN_AddMessageToTxFifoQ(_hfdcan, &txHeader, txMsg.data) != HAL_OK)
      {
        // 写入失败：帧已从软件缓冲取出，必须回滚，否则静默丢帧。
        // 刚取出 1 帧，缓冲至少有 1 帧空间，故这次回滚必定成功。
        (void)xMessageBufferSend(_tx_message_buffer, &txMsg, sizeof(CanTxMsg), 0);
        _tx_fifo_fail_cnt++;
      }
      else
      {
        started = true;
      }
    }
  }

  xSemaphoreGive(_tx_lock);

  // 无论这次是否真的送出去，都按缓冲现状校正接力中断：
  // 例：FIFO 当时满了没送成，但缓冲里还有帧 —— 必须保持中断开着等 FIFO 腾空。
  update_tx_empty_it();

  return started;
}

/**
 * @brief 释放所有已创建的 FreeRTOS 资源
 */
void BspCan::cleanup_resources()
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
 * @brief 停外设并摘除本驱动用过的全部通知（幂等）
 *
 * @note 顺序很重要：先摘通知，ISR 才不会在拆软件资源的中途进来。
 *       HAL_FDCAN_Stop() 只置 CCCR.INIT/CCE 并等其生效（State→READY），
 *       【不清】硬件 TX FIFO —— 重启后 FIFO 里残留的帧最多被重发一次
 *       （AutoRetransmission=ENABLE）；软件收发缓冲则原样保留。
 *
 * @note 未初始化（State 已是 READY）时 Stop 返回 HAL_ERROR，属正常，忽略即可。
 */
void BspCan::reset_hardware()
{
  if (_hfdcan == nullptr)
  {
    return;
  }

  (void)HAL_FDCAN_DeactivateNotification(_hfdcan,
                                         FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO0_MESSAGE_LOST | FDCAN_IT_TX_FIFO_EMPTY | FDCAN_IT_BUS_OFF | FDCAN_IT_ERROR_PASSIVE | FDCAN_IT_ERROR_WARNING);
  (void)HAL_FDCAN_Stop(_hfdcan);
}

/**
 * @brief 初始化失败收尾：复位外设 + 释放软件资源
 */
void BspCan::rollback_init()
{
  reset_hardware(); // 先停外设、摘通知，保证拆缓冲时不会有 ISR 进来访问它
  cleanup_resources();
}

/**
 * @brief 配置滤波器 + 启动外设 + 打开 RX/错误状态通知
 *
 * @note 必须在 reset_hardware() 之后调用：Stop 已置 CCCR.INIT/CCE，
 *       此时对 message RAM（滤波器）的写入才是合法的。
 *
 * @note 不用 TX_COMPLETE 中断：FIFO 模式下 BufferIndexes 传 0 不产生发送完成回调，
 *       收发链全靠 FIFO 中断驱动。
 */
Status BspCan::configure_hardware()
{
  // 滤波器：标准帧全 ID 收进 RX FIFO0（本驱动不做 ID 级过滤）
  FDCAN_FilterTypeDef sFilterConfig = {}; // 未用到的字段也清零，避免 HAL 版本差异踩到脏值
  sFilterConfig.IdType       = FDCAN_STANDARD_ID;
  sFilterConfig.FilterIndex  = 0;
  sFilterConfig.FilterType   = FDCAN_FILTER_RANGE;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1    = 0x000;
  sFilterConfig.FilterID2    = 0x7FF;

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

  // FIFO0 新消息 + FIFO0 溢出丢帧（「帧被硬件扔掉」的最后一道留痕）
  if (HAL_FDCAN_ActivateNotification(_hfdcan,
                                     FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO0_MESSAGE_LOST,
                                     0)
      != HAL_OK)
  {
    return Status::IO_ERROR;
  }

  // 总线错误状态（Bus-Off / 错误被动 / 错误警告）：只计数，恢复动作在任务侧
  if (HAL_FDCAN_ActivateNotification(_hfdcan,
                                     FDCAN_IT_BUS_OFF | FDCAN_IT_ERROR_PASSIVE | FDCAN_IT_ERROR_WARNING,
                                     0)
      != HAL_OK)
  {
    return Status::IO_ERROR;
  }

  // 注意：TX-FIFO-EMPTY 不常开，由 set_tx_empty_it()/update_tx_empty_it() 按需开关
  return Status::OK;
}

/**
 * @brief 构造 8 字节经典帧的标准发送头（任务与 ISR 共用，避免两处不同步）
 */
void BspCan::fill_tx_header(FDCAN_TxHeaderTypeDef &header, const CanTxMsg &msg)
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

// ----------------
