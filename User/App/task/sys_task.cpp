#include "app_task.hpp"
#include "bsp_cfg.hpp"

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
    // can硬件bus-off自恢复
    bsp_can1.service_recovery();
    bsp_can2.service_recovery();
    bsp_can3.service_recovery();
    // 在线情况更新
    sys_task_online_status = Online::update();
    ++sys_task_loop_count;
    vTaskDelayUntil(&wake_time, pdMS_TO_TICKS(1U));
  }
}
