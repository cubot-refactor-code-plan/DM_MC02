#include "can_bus.hpp"

#include "service_cfg.hpp" // IWYU pragma: keep (bus_can1/2/3)

#include <stdint.h>


// ---------------- 静态成员 ----------------

CanBus *const CanBus::buses[CanBus::BUS_NUM] = {&bus_can1, &bus_can2, &bus_can3};

bool CanBus::frozen = false;


// ----------------
// ---------------- 类函数实现 ----------------

CanBus::CanBus(const Config &cfg) :
  _can(cfg.can),
  rx_head(nullptr),
  tx_head(nullptr),
  _rx_return_buffer(nullptr)
{
}

CanBus::~CanBus()
{
  if (_rx_return_buffer != nullptr)
  {
    vMessageBufferDelete(_rx_return_buffer);
    _rx_return_buffer = nullptr;
  }
}

/** @brief 创建回退缓冲（必须在 BspCan::init() 之后调用） */
Status CanBus::init()
{
  if (_can == nullptr)
  {
    return Status::BAD_ARG;
  }

  // 可重复调用：先释放上一份
  if (_rx_return_buffer != nullptr)
  {
    vMessageBufferDelete(_rx_return_buffer);
    _rx_return_buffer = nullptr;
  }

  _rx_return_buffer = xMessageBufferCreate((sizeof(CanRxMsg) + 4) * RX_RETURN_DEPTH);
  return (_rx_return_buffer != nullptr) ? Status::OK : Status::IO_ERROR;
}

/** @brief 发送一帧标准帧（转交给硬件驱动） */
Status CanBus::send(uint32_t std_id, const uint8_t *data)
{
  return (_can == nullptr) ? Status::NOT_INIT : _can->send(std_id, data);
}

/** @brief 取出没有被任何节点收取的一帧（0=不等待，portMAX_DELAY=一直等） */
Status CanBus::receive(CanRxMsg *msg, uint32_t timeout_ms)
{
  if (msg == nullptr)
  {
    return Status::BAD_ARG;
  }
  if (_rx_return_buffer == nullptr)
  {
    return Status::NOT_INIT;
  }

  // portMAX_DELAY 不能直接丢给 pdMS_TO_TICKS（会溢出），单独处理
  const TickType_t wait = (timeout_ms == portMAX_DELAY) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);

  const size_t received = xMessageBufferReceive(_rx_return_buffer, msg, sizeof(CanRxMsg), wait);
  return (received > 0) ? Status::OK : Status::TIMEOUT;
}

/**
 * @brief 接收分发一轮
 *
 * @note 即使本总线没有注册任何节点也要转发原始帧，保证 receive() 仍可使用。
 *       每轮最多处理 RX_BATCH_MAX 帧，避免某条总线持续占用分发任务。
 */
void CanBus::rx_poll()
{
  if (_can == nullptr || _can->_rx_message_buffer == nullptr || _rx_return_buffer == nullptr)
  {
    return;
  }

  for (uint32_t i = 0; i < RX_BATCH_MAX; ++i)
  {
    CanRxMsg rx = {};
    if (xMessageBufferReceive(_can->_rx_message_buffer, &rx, sizeof(rx), 0U) != sizeof(rx))
    {
      break;
    }

    bool consumed = false;
    // 只支持经典 CAN 的 8 字节数据帧；格式不符的帧不匹配任何节点，直接进回退缓冲
    if ((rx.header.IdType == FDCAN_STANDARD_ID || rx.header.IdType == FDCAN_EXTENDED_ID) &&
        rx.header.RxFrameType == FDCAN_DATA_FRAME && rx.header.FDFormat == FDCAN_CLASSIC_CAN &&
        rx.header.DataLength == FDCAN_DLC_BYTES_8)
    {
      for (CanRxNode *node = rx_head; node != nullptr; node = node->next)
      {
        if (node->_can_id == rx.header.Identifier && node->_id_type == rx.header.IdType)
        {
          consumed = node->_callback(node->_context, rx) == Status::OK;
          break;
        }
      }
    }

    // 没有节点收取这帧数据，帧归还给 CanBus，可由 receive() 取出
    if (!consumed && xMessageBufferSend(_rx_return_buffer, &rx, sizeof(rx), 0U) != sizeof(rx))
    {
      // 回退消费者滞后时丢弃本帧并记录 FULL，不阻塞其他节点的解算。
      sys_flag_set(Status::FULL);
    }
  }
}

/**
 * @brief 发送扫描一轮
 *
 * @note 满足任一条件就发：槽位全部填满（ready），或距离上次发送超过 5 ms（due，保底心跳）。
 */
void CanBus::tx_poll()
{
  for (CanTxNode *node = tx_head; node != nullptr; node = node->next)
  {
    if (node->bufferMutex == nullptr)
    {
      continue;
    }
    if (xSemaphoreTake(node->bufferMutex, pdMS_TO_TICKS(1U)) != pdTRUE)
    {
      continue;
    }
    const bool ready = node->bufferRegister != 0U && node->bufferUnsend == node->bufferRegister;
    const bool due   = xTaskGetTickCount() - node->last_send_tick >= pdMS_TO_TICKS(5U);
    if ((ready || due) && send(node->txBuffer.std_id, node->txBuffer.data) == Status::OK)
    {
      node->bufferUnsend   = 0;
      node->last_send_tick = xTaskGetTickCount();
    }
    xSemaphoreGive(node->bufferMutex);
  }
}


// ----------------
// ---------------- 函数实现 ----------------

/** @brief 初始化全部总线（在 bsp_init() 之后调用） */
Status can_bus_init(void)
{
  for (uint32_t i = 0; i < CanBus::BUS_NUM; ++i)
  {
    const Status status = CanBus::buses[i]->init();
    if (status != Status::OK)
    {
      return status;
    }
  }
  return Status::OK;
}


// ----------------
// ---------------- 任务实现 ----------------

/**
 * @brief 1 kHz 轮询各总线接收缓冲，按 ID 分发
 *
 * @note 分发任务必须在初始化结束后才允许运行：进入循环前先把注册表冻结，
 *       避免运行期还有节点注册进来。
 */
extern "C" void can_rx_task(void *argument)
{
  (void)argument;
  sys_flag_wait_running();
  CanBus::frozen = true;
  static_assert(configTICK_RATE_HZ == 1000U, "can_rx_task requires a 1 kHz tick");

  TickType_t wake_time = xTaskGetTickCount();
  for (;;)
  {
    if (sys_flag_running())
    {
      for (uint32_t i = 0; i < CanBus::BUS_NUM; ++i)
      {
        CanBus::buses[i]->rx_poll();
      }
    }
    vTaskDelayUntil(&wake_time, pdMS_TO_TICKS(1U));
  }
}

/**
 * @brief 1 kHz 扫描各总线发送节点
 *
 * @note 一个节点都没有时任务直接退出，不占用 CPU 和栈。
 */
extern "C" void can_tx_task(void *argument)
{
  (void)argument;
  sys_flag_wait_running();
  CanBus::frozen = true;
  static_assert(configTICK_RATE_HZ == 1000U, "can_tx_task requires a 1 kHz tick");

  TickType_t wake_time = xTaskGetTickCount();
  bool       has_nodes = false;
  for (uint32_t i = 0; i < CanBus::BUS_NUM; ++i)
  {
    for (CanTxNode *node = CanBus::buses[i]->tx_head; node != nullptr; node = node->next)
    {
      node->last_send_tick = wake_time;
      has_nodes            = true;
    }
  }
  if (!has_nodes)
  {
    vTaskDelete(NULL);
    return;
  }

  for (;;)
  {
    for (uint32_t i = 0; i < CanBus::BUS_NUM; ++i)
    {
      CanBus::buses[i]->tx_poll();
    }
    vTaskDelayUntil(&wake_time, pdMS_TO_TICKS(1U));
  }
}

// ----------------
