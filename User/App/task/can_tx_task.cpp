#include "can_tx_node.hpp"
#include "bsp_cfg.hpp"

extern "C" void can_tx_task(void *argument)
{
  (void)argument;
  SysFlagWaitRunning();
  CanTxNode::frozen = true;
  BspCan *const buses[] = {&bsp_can1, &bsp_can2, &bsp_can3};
  TickType_t wake_time = xTaskGetTickCount();
  bool has_nodes = false;
  for (BspCan *can : buses)
  {
    for (CanTxNode *node = can->tx_head; node != nullptr; node = node->next)
    {
      node->last_send_tick = wake_time;
      has_nodes = true;
    }
  }
  if (!has_nodes)
  {
    osThreadExit();
    return;
  }
  for (;;)
  {
    for (BspCan *can : buses)
    {
      for (CanTxNode *node = can->tx_head; node != nullptr; node = node->next)
      {
        if (osMutexAcquire(node->bufferMutex, pdMS_TO_TICKS(1U)) != osOK)
          continue;
        const bool ready = node->bufferRegister != 0U && node->bufferUnsend == node->bufferRegister;
        const bool due = xTaskGetTickCount() - node->last_send_tick >= pdMS_TO_TICKS(5U);
        if ((ready || due) && can->send(node->txBuffer.std_id, node->txBuffer.data) == Status::OK)
        {
          node->bufferUnsend = 0;
          node->last_send_tick = xTaskGetTickCount();
        }
        osMutexRelease(node->bufferMutex);
      }
    }
    vTaskDelayUntil(&wake_time, pdMS_TO_TICKS(1U));
  }
}
