#include "app_task.hpp"
#include "can_bus.hpp"

#include "FreeRTOS.h" // IWYU pragma: keep
#include "online_check.hpp"
#include "task.h"

#include <stdint.h>


volatile uint32_t sys_task_loop_count = 0U;
volatile Status sys_task_online_status = Status::NOT_INIT;


extern "C" void sys_task(void *argument)
{
  (void)argument; // 任务不需要外部参数。
  static_assert(configTICK_RATE_HZ == 1000U,
                "sys_task requires a 1 kHz FreeRTOS tick");

  TickType_t wake_time = xTaskGetTickCount();
  for (;;)
  {
    // CAN 巡检：补丢唤醒的发送（tx_recover）+ Bus-Off 恢复（service_recovery）
    for (uint32_t i = 0; i < CanBus::BUS_NUM; ++i)
    {
      CanBus::buses[i]->_can->tx_recover();
      CanBus::buses[i]->_can->service_recovery();
    }
    // 在线情况更新
    sys_task_online_status = Online::update();
    ++sys_task_loop_count;
    vTaskDelayUntil(&wake_time, pdMS_TO_TICKS(1U));
  }
}
