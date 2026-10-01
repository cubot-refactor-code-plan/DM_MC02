#include "app_task.hpp"
#include "bsp_cfg.hpp"
#include "can_bus.hpp"

#include "FreeRTOS.h" // IWYU pragma: keep
#include "online_check.hpp"
#include "task.h"

#include <stdint.h>

namespace
{
/** @brief 维护任务的周期（ms）：CAN / UART 的补救巡检与在线计时都按这个节拍推进。 */
constexpr uint32_t SYS_TASK_PERIOD_MS = 10U;

/** @brief 需要巡检的串口实例；与 bsp_cfg 中的实例保持一致。 */
BspUart<128> *const patrol_uarts[] = {&bsp_uart1, &bsp_uart3, &bsp_uart4, &bsp_uart5, &bsp_uart7, &bsp_uart8, &bsp_uart9, &bsp_uart10};
} // namespace


volatile uint32_t sys_task_loop_count    = 0U;
volatile Status   sys_task_online_status = Status::NOT_INIT;


extern "C" void sys_task(void *argument)
{
  (void)argument; // 任务不需要外部参数。
  static_assert(configTICK_RATE_HZ == 1000U,
                "sys_task requires a 1 kHz FreeRTOS tick");
  static_assert((SYS_TASK_PERIOD_MS % (1000U / configTICK_RATE_HZ)) == 0U,
                "SYS_TASK_PERIOD_MS must be a multiple of the tick period");

  TickType_t wake_time = xTaskGetTickCount();
  for (;;)
  {
    // CAN 补救：正常收发由 can_rx_task / can_tx_task 以 1 kHz 负责，这里只补
    // 10 ms 级的故障 —— 丢唤醒的发送（tx_recover）与 Bus-Off 恢复（service_recovery）
    for (uint32_t i = 0; i < CanBus::BUS_NUM; ++i)
    {
      CanBus::buses[i]->_can->tx_recover();
      CanBus::buses[i]->_can->service_recovery();
    }

    // UART 补救：卡死的发送补发（tx_recover）+ 接收通道异常时重建（rx_recover）
    for (auto *uart : patrol_uarts)
    {
      uart->tx_recover();
      uart->rx_recover();
    }

    // 在线情况更新：判据是距上次刷新的毫秒数，与本次调用周期无关
    sys_task_online_status = Online::update();

    ++sys_task_loop_count;
    vTaskDelayUntil(&wake_time, pdMS_TO_TICKS(SYS_TASK_PERIOD_MS));
  }
}
