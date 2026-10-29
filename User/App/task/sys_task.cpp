#include "app_task.hpp"
#include "bsp_cfg.hpp"

#include "FreeRTOS.h" // IWYU pragma: keep
#include "online_check.hpp"
#include "task.h"

#include <stdint.h>


// 维护周期
constexpr uint32_t SYS_TASK_PERIOD_MS = 10U;

// 串口组（DMA）
BspUartT<uart_detail::DmaTransport, 128, 128> *const patrol_uarts[] = {&bsp_uart1, &bsp_uart2, &bsp_uart3, &bsp_uart4, &bsp_uart7, &bsp_uart9, &bsp_uart10};

// CAN 组（收发由各设备任务负责，这里只做总线级补救）
BspCanT<16, 16> *const patrol_cans[] = {&bsp_can1, &bsp_can2, &bsp_can3};


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
    bsp_dwt.get_delta_s(&mark);

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

    // UART5 = SBUS，UART8 = 回环测试口：中断收发，单独巡检
    bsp_uart_it5.tx_recover();
    bsp_uart_it5.rx_recover();
    bsp_uart8.tx_recover();
    bsp_uart8.rx_recover();

    // 在线状态更新（emmv5 节点由消费该串口的任务在 update() 里刷新）
    Online::update();

    // 打印占用时间降低打印次数
    if (count == 200)
    {
      // 绝对时间：每次打印都读一次，顺带满足 DWT 绝对时间轴 < 7.809 s 的心跳要求
      const double uptime_s = bsp_dwt.get_time_s();

      // 本轮耗时（µs）：从循环开头到这里，经 USART1 打印
      // ⚠ 两个参数都必须先显式转成 unsigned 再交给 %u：vsnprintf 按格式串取变参，
      //   把 double 直接配 %d/%u 会读错槽位，打出来的是 double 的位模式（全是乱数，还会为负）
      bsp_uart1.printf("[sys] time=%us cycle=%uus\r\n",
                       static_cast<unsigned>(uptime_s),
                       static_cast<unsigned>(bsp_dwt.get_delta_s(&mark) * 1000000.0));
      count = 0;
    }

    vTaskDelayUntil(&wake_time, pdMS_TO_TICKS(SYS_TASK_PERIOD_MS));
  }
}
