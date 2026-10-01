#include "app_test.hpp"

#if APP_TEST_UART_TRANSPORT_ENABLED

#include "FreeRTOS.h" // IWYU pragma: keep
#include "task.h"     // IWYU pragma: keep

#include <stdint.h>

#include "bsp_cfg.hpp" // bsp_uart1


// ---------------- 测试参数 ----------------

///< 心跳周期（ms）；设为 0 可关闭周期心跳，只保留回显
#define UART_TEST_HEARTBEAT_MS 2000U

///< 单次读取的最大字节数（须小于 BspUart 的流缓冲区容量 BUFFER_SIZE=128）
#define UART_TEST_RX_CHUNK 32U


// ----------------
// ---------------- 测试统计量（Live Watch 可观察） ----------------

volatile uint32_t uart1_test_rx_bytes  = 0; ///< 累计接收字节数
volatile uint32_t uart1_test_tx_bytes  = 0; ///< 累计回发字节数
volatile uint8_t  uart1_test_last_byte = 0; ///< 最近一次收到的字节


// ----------------
// ---------------- 测试步骤 ----------------

extern "C" void uart_transport_test_step(void)
{
  static uint8_t rx_buf[UART_TEST_RX_CHUNK];

  // 1. 非阻塞收：超时 0，没有数据立即返回
  size_t rx_len = 0;
  if (bsp_uart1.receive(rx_buf, sizeof(rx_buf), 0, &rx_len) == Status::OK && rx_len > 0)
  {
    uart1_test_rx_bytes += rx_len;
    uart1_test_last_byte = rx_buf[rx_len - 1];

    // 2. 原样回显：PC 发什么就回什么
    size_t tx_len = 0;
    if (bsp_uart1.send(rx_buf, rx_len, 0, &tx_len) == Status::OK)
    {
      uart1_test_tx_bytes += tx_len;
    }
    return; // 有收发时不发心跳，避免打断回显节奏
  }

#if (UART_TEST_HEARTBEAT_MS > 0U)
  // 3. 周期心跳：验证发送通路（附累计收发字节数）
  static TickType_t last_tick = 0;
  const TickType_t  now       = xTaskGetTickCount();

  if (last_tick == 0U)
  {
    last_tick = now; // 首次只对齐时间基准，不发送
  }
  else if ((now - last_tick) >= pdMS_TO_TICKS(UART_TEST_HEARTBEAT_MS))
  {
    last_tick = now;
    (void)bsp_uart1.printf("UART1 test heartbeat: rx=%lu tx=%lu\r\n",
                           static_cast<unsigned long>(uart1_test_rx_bytes),
                           static_cast<unsigned long>(uart1_test_tx_bytes));
  }

// ----------------
#endif
}

#endif
