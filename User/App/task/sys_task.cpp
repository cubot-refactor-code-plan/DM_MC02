#include "app_task.hpp"

#include "bsp_cfg.hpp"
#include "FreeRTOS.h" // IWYU pragma: keep
#include "online_check.hpp"
#include "task.h"

#include <stdint.h>


volatile uint32_t sys_task_loop_count    = 0U;
volatile Status   sys_task_online_status = Status::NOT_INIT;

static BspCan *const       can_ports[]  = {&bsp_can1, &bsp_can2, &bsp_can3};
static BspUart<128> *const uart_ports[] = {&bsp_uart1, &bsp_uart3, &bsp_uart4, &bsp_uart5, &bsp_uart7, &bsp_uart8, &bsp_uart9, &bsp_uart10};

extern "C" void sys_task(void *argument)
{
  (void)argument; // 任务不需要外部参数。
  static_assert(configTICK_RATE_HZ == 1000U,
                "sys_task requires a 1 kHz FreeRTOS tick");

  TickType_t wake_time = xTaskGetTickCount();
  for (;;)
  {
    sys_task_online_status = Online::update();

    /* 断链兜底（全部非阻塞，正常时看一眼即返回）：逐个巡检下面两张表里的实例，新增实例只需往表里加一项。 */
    // 串口兜底
    for (BspUart<128> *u : uart_ports)
    {
      (void)u->tx_recover(); // 30ms 没发出去就强制重发
      (void)u->rx_recover(); // RX 停摆就复位重装（UART5 仅接收，其 tx_recover 立即返回）
    }
    // CAN兜底
    for (BspCan *c : can_ports)
    {
      (void)c->tx_recover();  // 发送链断了补点火
      (void)c->bus_recover(); // 卡在 Bus-Off 就重启外设
    }

    ++sys_task_loop_count;
    vTaskDelayUntil(&wake_time, pdMS_TO_TICKS(10U));
  }
}
