#include "app_task.hpp"

#include "FreeRTOS.h" // IWYU pragma: keep
#include "bsp_cfg.hpp"
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
    sys_task_online_status = Online::update();

    /* 断链兜底（全部非阻塞，正常时只看一眼就返回）：
       收发链靠「取包 → 启转」接力推进，一旦某一环的中断丢了就再没人点火，
       所以每个周期都巡检一次。注意：必须与 bsp_cfg.hpp 的实例列表保持一致。 */

    // UART：tx_recover = 30ms 没发出去就强制重发；rx_recover = RX 停摆就复位重装
    //      UART5 仅接收（transmit_enable=false），其 tx_recover 会立即返回
    (void)bsp_uart1.tx_recover();
    (void)bsp_uart1.rx_recover();
    (void)bsp_uart3.tx_recover();
    (void)bsp_uart3.rx_recover();
    (void)bsp_uart4.tx_recover();
    (void)bsp_uart4.rx_recover();
    (void)bsp_uart5.tx_recover();
    (void)bsp_uart5.rx_recover();
    (void)bsp_uart7.tx_recover();
    (void)bsp_uart7.rx_recover();
    (void)bsp_uart8.tx_recover();
    (void)bsp_uart8.rx_recover();
    (void)bsp_uart9.tx_recover();
    (void)bsp_uart9.rx_recover();
    (void)bsp_uart10.tx_recover();
    (void)bsp_uart10.rx_recover();

    (void)bsp_can1.tx_recover();
    (void)bsp_can2.tx_recover();
    (void)bsp_can3.tx_recover();

    (void)bsp_can1.bus_recover();
    (void)bsp_can2.bus_recover();
    (void)bsp_can3.bus_recover();

    ++sys_task_loop_count;
    vTaskDelayUntil(&wake_time, pdMS_TO_TICKS(10U));
  }
}
