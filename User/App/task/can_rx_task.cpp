#include "can_rx_node.hpp"
#include "bsp_cfg.hpp"

extern "C" void can_rx_task(void *argument)
{
  (void)argument;
  SysFlagWaitRunning();
  CanRxNode::frozen = true;
  static_assert(configTICK_RATE_HZ == 1000U, "can_rx_task requires a 1 kHz tick");
  BspCan *const buses[] = {&bsp_can1, &bsp_can2, &bsp_can3};
  TickType_t wake_time = xTaskGetTickCount();
  for (;;)
  {
    if ((osEventFlagsGet(sysEvent) & SYS_FLAG_RUNNING_BIT) != 0U)
    {
      for (BspCan *can : buses)
      {
        if (can->_rx_message_buffer == nullptr || can->_rx_return_buffer == nullptr)
        {
          continue;
        }
        // 即使没有节点也要转发原始帧，保证 receive() 仍可使用。
        // 每轮每条总线最多处理 8 帧，避免某条总线持续占用任务。
        for (uint32_t i = 0; i < 8U; ++i)
        {
          CanRxMsg rx = {};
          if (xMessageBufferReceive(can->_rx_message_buffer, &rx, sizeof(rx), 0U) != sizeof(rx))
          {
            break;
          }
          bool consumed = false;
          if ((rx.header.IdType == FDCAN_STANDARD_ID || rx.header.IdType == FDCAN_EXTENDED_ID) &&
              rx.header.RxFrameType == FDCAN_DATA_FRAME && rx.header.FDFormat == FDCAN_CLASSIC_CAN &&
              rx.header.DataLength == FDCAN_DLC_BYTES_8)
          {
            for (CanRxNode *node = can->rx_head; node != nullptr; node = node->next)
            {
              if (node->_can_id == rx.header.Identifier && node->_id_type == rx.header.IdType)
              {
                consumed = node->_callback(node->_context, rx) == Status::OK;
                break;
              }
            }
          }
          // 没有rx_node收取这帧数据，这帧数据归还回bsp_can，可由bsp_can.receive
          if (!consumed && xMessageBufferSend(can->_rx_return_buffer, &rx, sizeof(rx), 0U) != sizeof(rx))
          {
            // 回退消费者滞后时丢弃本帧并记录 FULL，不阻塞其他节点的解算。
            SysFlagSet(Status::FULL);
          }
        }
      }
    }
    vTaskDelayUntil(&wake_time, pdMS_TO_TICKS(1U));
  }
}

