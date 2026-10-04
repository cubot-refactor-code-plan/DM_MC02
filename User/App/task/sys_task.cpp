#include "app_task.hpp"
#include "bsp_cfg.hpp"

#include "FreeRTOS.h" // IWYU pragma: keep
#include "online_check.hpp"
#include "task.h"

#include <stdint.h>


// 维护周期
constexpr uint32_t SYS_TASK_PERIOD_MS = 10U;

// 串口组
BspUart<128> *const patrol_uarts[] = {&bsp_uart1, &bsp_uart3, &bsp_uart4, &bsp_uart5, &bsp_uart7, &bsp_uart8, &bsp_uart9, &bsp_uart10};

// CAN 组（收发由各设备任务负责，这里只做总线级补救）
BspCan *const patrol_cans[] = {&bsp_can1, &bsp_can2, &bsp_can3};


extern "C" void sys_task(void *argument)
{
  TickType_t wake_time = xTaskGetTickCount();

  // 降低打印次数的临时变量
  uint8_t count = 0;

  for (;;)
  {
    count++;

    // 本轮计时基准（无状态，首次结果丢弃）
    uint32_t mark = 0U;
    bsp_dwt.delta_s(&mark);

    // CAN 补救：日常收发由各设备任务负责，这里只补丢唤醒与 Bus-Off
    for (auto *can : patrol_cans)
    {
      can->tx_recover();
      can->service_recovery();
    }

    // UART 补救：卡死的发送补发 + 接收通道重建（ISR 只计数，恢复只能在任务侧做）
    for (auto *uart : patrol_uarts)
    {
      uart->tx_recover();
      uart->rx_recover();
    }

    // 在线状态更新
    Online::update();

    // 打印占用时间降低打印次数
    if(count == 200)
    {
      // 本轮耗时（µs）：从循环开头到这里，经 USART1 打印
      bsp_uart1.printf("[sys] cycle=%uus\r\n",static_cast<unsigned>(bsp_dwt.delta_s(&mark) * 1000000.0));
      count = 0;
    }

    vTaskDelayUntil(&wake_time, pdMS_TO_TICKS(SYS_TASK_PERIOD_MS));
  }
}
