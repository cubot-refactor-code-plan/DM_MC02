/**
 * @file uart_loopback_test.cpp
 * @author Rh
 * @brief UART8(IT) 与 UART9(DMA) 921600 双向回环自检
 * @date 2026-10-20
 *
 * 硬件：PE1→PD14、PD15→PE0、共地；UART8=IT，UART9=DMA，921600 8N1。
 * 打印：[UART] DMA9->IT8 ... / [UART] PASS frames=... bytes=...
 * 失败：打印后停机，检查 g_uart_test_error 与两路 rx_diag/tx_diag。
 */

#include "app_test.hpp"

#if APP_TEST_UART_LOOPBACK_ENABLED

#  include "FreeRTOS.h" // IWYU pragma: keep
#  include "bsp_cfg.hpp"
#  include "service_cfg.hpp"
#  include "task.h"

#  include <stddef.h>
#  include <stdint.h>


static_assert(configTICK_RATE_HZ == 1000U, "本测试的毫秒常量按 1 kHz tick 写");

namespace
{
// ---------------- 参数 ----------------

constexpr uint32_t BAUD            = 921600U; ///< 必须与 CubeMX 里的 UART8/9 一致
constexpr uint32_t FRAMES_PER_LEN  = 1000U;   ///< 每种长度 1000 帧，两个方向共 10000 帧
constexpr uint32_t RX_TIMEOUT_MS   = 100U;    ///< 128 B @921600 约 1.4 ms，100 ms 足够
constexpr size_t   PAYLOAD_LENS[]  = {1U, 8U, 32U, 64U, 121U}; ///< 121+7=128，正好压满缓冲区
constexpr uint8_t  HEADER0         = 0xA5U;
constexpr uint8_t  HEADER1         = 0x5AU;

static_assert(PAYLOAD_LENS[4] + 7U <= 128U, "帧长不能超过 128 字节暂存区");


// ----------------
// ---------------- 统计 ----------------

struct Stats
{
  uint32_t frames;
  uint32_t bytes;
  uint32_t errors;
  uint32_t max_latency_us;
};

Stats g_it_to_dma;
Stats g_dma_to_it;

volatile uint32_t g_uart_test_error = 0U; ///< 调试器直接看：1=发送失败 2=接收超时 3=帧头/序号 4=数据 5=CRC 10=格式 20=诊断脏


// ----------------
// ---------------- 辅助 ----------------

/** @brief 失败就停机，等调试器接手 */
void halt()
{
  for (;;)
  {
    vTaskDelay(portMAX_DELAY);
  }
}

/** @brief 打印一帧的前 8 字节，用来判断是丢头、错位还是噪声 */
void print_head(const char *tag, const uint8_t *data)
{
  bsp_uart1.printf("[UART] %s=%02X %02X %02X %02X %02X %02X %02X %02X\r\n", tag,
                   data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
}

/** @brief 打印两路 HAL 错误码与 RX 硬件错误计数 */
void print_error_state()
{
  const auto rx8 = bsp_uart8.get_rx_diag(); // 内嵌类型，auto 省掉模板名
  const auto rx9 = bsp_uart9.get_rx_diag();
  bsp_uart1.printf("[UART] err8=%08lX err9=%08lX rx8hw=%lu rx9hw=%lu\r\n",
                   static_cast<unsigned long>(huart8.ErrorCode), static_cast<unsigned long>(huart9.ErrorCode),
                   static_cast<unsigned long>(rx8.hw_error_cnt), static_cast<unsigned long>(rx9.hw_error_cnt));
}

/** @brief 把某一路接收流缓冲区里已有的字节全部丢掉 */
void drain_rx(bool it_rx)
{
  uint8_t junk[32];
  while ((it_rx ? bsp_uart8.receive(junk, sizeof(junk), 0U)
                : bsp_uart9.receive(junk, sizeof(junk), 0U)) == Status::OK)
  {
  }
}

/** @brief CRC-16/CCITT-FALSE：初值 0xFFFF，多项式 0x1021，不反转 */
uint16_t crc16(const uint8_t *data, size_t len)
{
  uint16_t crc = 0xFFFFU;

  for (size_t i = 0; i < len; i++)
  {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (uint8_t bit = 0; bit < 8U; bit++)
    {
      crc = ((crc & 0x8000U) != 0U) ? static_cast<uint16_t>((crc << 1) ^ 0x1021U)
                                    : static_cast<uint16_t>(crc << 1);
    }
  }

  return crc;
}

/**
 * @brief 跑完一个方向的所有长度
 * @param it_tx true=UART8(IT) 发、UART9(DMA) 收；false=反过来
 * @param seq   全局序号，跨方向继续累加
 */
void run_direction(bool it_tx, uint16_t &seq)
{
  Stats      &stats = it_tx ? g_it_to_dma : g_dma_to_it;
  const char *name  = it_tx ? "IT8->DMA9" : "DMA9->IT8";

  drain_rx(!it_tx); // 丢弃上一个方向或启动阶段残留的字节

  for (size_t payload_len : PAYLOAD_LENS)
  {
    const size_t frame_len = payload_len + 7U;

    for (uint32_t i = 0; i < FRAMES_PER_LEN; i++)
    {
      uint8_t tx[128];
      uint8_t rx[128];

      tx[0] = HEADER0;
      tx[1] = HEADER1;
      tx[2] = static_cast<uint8_t>(seq & 0xFFU);
      tx[3] = static_cast<uint8_t>(seq >> 8);
      tx[4] = static_cast<uint8_t>(payload_len);
      for (size_t j = 0; j < payload_len; j++)
      {
        tx[5U + j] = static_cast<uint8_t>(seq + j);
      }

      const uint16_t crc = crc16(tx + 2U, payload_len + 3U);
      tx[5U + payload_len] = static_cast<uint8_t>(crc & 0xFFU);
      tx[6U + payload_len] = static_cast<uint8_t>(crc >> 8);

      const double t0 = bsp_dwt.get_time_s();

      Status st      = Status::OK;
      size_t written = 0U;
      if (it_tx)
      {
        st = bsp_uart8.send(tx, frame_len, RX_TIMEOUT_MS, &written);
      }
      else
      {
        st = bsp_uart9.send(tx, frame_len, RX_TIMEOUT_MS, &written);
      }

      if (st != Status::OK || written != frame_len)
      {
        g_uart_test_error = 1U;
        bsp_uart1.printf("[UART] %s send fail seq=%u len=%u st=%d wr=%u\r\n", name,
                         static_cast<unsigned>(seq), static_cast<unsigned>(frame_len),
                         static_cast<int>(st), static_cast<unsigned>(written));
        halt();
      }

      size_t got = 0U;
      while (got < frame_len)
      {
        size_t n = 0U;
        if (it_tx)
        {
          st = bsp_uart9.receive(rx + got, frame_len - got, RX_TIMEOUT_MS, &n);
        }
        else
        {
          st = bsp_uart8.receive(rx + got, frame_len - got, RX_TIMEOUT_MS, &n);
        }

        if (st != Status::OK || n == 0U)
        {
          g_uart_test_error = 2U;
          bsp_uart1.printf("[UART] %s recv timeout seq=%u len=%u got=%u st=%d\r\n", name,
                           static_cast<unsigned>(seq), static_cast<unsigned>(frame_len),
                           static_cast<unsigned>(got), static_cast<int>(st));
          print_error_state();
          halt();
        }
        got += n;
      }

      bool frame_ok = true;
      if (rx[0] != HEADER0 || rx[1] != HEADER1 || rx[2] != tx[2] || rx[3] != tx[3] ||
          rx[4] != static_cast<uint8_t>(payload_len))
      {
        g_uart_test_error = 3U;
        if (stats.errors < 5U)
        {
          bsp_uart1.printf("[UART] %s header mismatch seq=%u len=%u\r\n", name,
                           static_cast<unsigned>(seq), static_cast<unsigned>(frame_len));
          print_head("tx", tx);
          print_head("rx", rx);
          print_error_state();
        }
        frame_ok = false;
      }

      for (size_t j = 0; frame_ok && j < payload_len; j++)
      {
        if (rx[5U + j] != static_cast<uint8_t>(seq + j))
        {
          g_uart_test_error = 4U;
          if (stats.errors < 5U)
          {
            bsp_uart1.printf("[UART] %s payload mismatch seq=%u byte=%u\r\n", name,
                             static_cast<unsigned>(seq), static_cast<unsigned>(j));
            print_head("rx", rx);
          }
          frame_ok = false;
        }
      }

      if (frame_ok)
      {
        const uint16_t crc_rx = static_cast<uint16_t>(rx[5U + payload_len]) |
                                static_cast<uint16_t>(rx[6U + payload_len] << 8);
        if (crc_rx != crc16(rx + 2U, payload_len + 3U))
        {
          g_uart_test_error = 5U;
          if (stats.errors < 5U)
          {
            bsp_uart1.printf("[UART] %s crc mismatch seq=%u len=%u\r\n", name,
                             static_cast<unsigned>(seq), static_cast<unsigned>(frame_len));
            print_head("rx", rx);
          }
          frame_ok = false;
        }
      }

      if (!frame_ok)
      {
        stats.errors++;
        seq++;
        drain_rx(!it_tx); // 错位后清掉剩余字节，让下一帧从帧头重新开始
        continue;
      }

      const uint32_t latency_us = static_cast<uint32_t>((bsp_dwt.get_time_s() - t0) * 1000000.0);
      if (latency_us > stats.max_latency_us)
      {
        stats.max_latency_us = latency_us;
      }
      stats.frames++;
      stats.bytes += static_cast<uint32_t>(frame_len);
      seq++;
    }
  }

  bsp_uart1.printf("[UART] %s frames=%lu bytes=%lu errors=%lu max=%luus\r\n", name,
                   static_cast<unsigned long>(stats.frames), static_cast<unsigned long>(stats.bytes),
                   static_cast<unsigned long>(stats.errors), static_cast<unsigned long>(stats.max_latency_us));
}

// ----------------
} // namespace


extern "C" void uart_loopback_test_task(void *argument)
{
  (void)argument;
  sys_state.wait_running();

  // Config 里的 baudrate 只用于 tx_recover 阈值，真正波特率必须由 CubeMX 写进 huart
  const bool fmt_ok = bsp_uart8.check_format(BAUD, UART_WORDLENGTH_8B, UART_PARITY_NONE, UART_STOPBITS_1) &&
                      bsp_uart9.check_format(BAUD, UART_WORDLENGTH_8B, UART_PARITY_NONE, UART_STOPBITS_1);
  if (!fmt_ok)
  {
    g_uart_test_error = 10U;
    bsp_uart1.printf("[UART] format mismatch: u8=%lu u9=%lu err8=%lu err9=%lu\r\n",
                     static_cast<unsigned long>(huart8.Init.BaudRate),
                     static_cast<unsigned long>(huart9.Init.BaudRate),
                     static_cast<unsigned long>(huart8.ErrorCode),
                     static_cast<unsigned long>(huart9.ErrorCode));
    configASSERT(false);
  }

  bsp_uart1.printf("[UART] loopback 921600, UART8=IT, UART9=DMA\r\n");

  // 清掉启动阶段可能残留的噪声，避免第一帧被误判
  drain_rx(true);
  drain_rx(false);

  // 首次启动 UART9 DMA 发送时，UART8 可能先收到一个 0x00 前导；
  // 先各发一帧热身并丢弃，避免它把第一帧整体顶偏。
  // 热身帧内容不重要，只要触发一次真实发送即可。
  const uint8_t warm[8] = {HEADER0, HEADER1, 0x00U, 0x00U, 0x01U, 0x00U, 0x00U, 0x00U};
  (void)bsp_uart9.send(warm, sizeof(warm), RX_TIMEOUT_MS);
  vTaskDelay(pdMS_TO_TICKS(5U));
  drain_rx(true);
  (void)bsp_uart8.send(warm, sizeof(warm), RX_TIMEOUT_MS);
  vTaskDelay(pdMS_TO_TICKS(5U));
  drain_rx(false);

  uint16_t seq = 0U;
  run_direction(false, seq); // DMA9 -> IT8
  run_direction(true, seq);  // IT8 -> DMA9

  // 诊断量是 BspUartT<...> 的内嵌类型，auto 省掉一长串模板名
  const auto rx8 = bsp_uart8.get_rx_diag();
  const auto tx8 = bsp_uart8.get_tx_diag();
  const auto rx9 = bsp_uart9.get_rx_diag();
  const auto tx9 = bsp_uart9.get_tx_diag();

  bsp_uart1.printf("[UART] diag8 rx arm=%lu drop=%lu hw=%lu rec=%lu\r\n",
                   static_cast<unsigned long>(rx8.arm_fail_cnt), static_cast<unsigned long>(rx8.drop_bytes),
                   static_cast<unsigned long>(rx8.hw_error_cnt), static_cast<unsigned long>(rx8.error_cnt));
  bsp_uart1.printf("[UART] diag8 tx drop=%lu start=%lu skip=%lu stall=%lu err=%lu\r\n",
                   static_cast<unsigned long>(tx8.drop_bytes), static_cast<unsigned long>(tx8.start_fail_cnt),
                   static_cast<unsigned long>(tx8.isr_skip_cnt), static_cast<unsigned long>(tx8.stall_recover_cnt),
                   static_cast<unsigned long>(tx8.error_cnt));
  bsp_uart1.printf("[UART] diag9 rx arm=%lu drop=%lu hw=%lu rec=%lu\r\n",
                   static_cast<unsigned long>(rx9.arm_fail_cnt), static_cast<unsigned long>(rx9.drop_bytes),
                   static_cast<unsigned long>(rx9.hw_error_cnt), static_cast<unsigned long>(rx9.error_cnt));
  bsp_uart1.printf("[UART] diag9 tx drop=%lu start=%lu skip=%lu stall=%lu err=%lu\r\n",
                   static_cast<unsigned long>(tx9.drop_bytes), static_cast<unsigned long>(tx9.start_fail_cnt),
                   static_cast<unsigned long>(tx9.isr_skip_cnt), static_cast<unsigned long>(tx9.stall_recover_cnt),
                   static_cast<unsigned long>(tx9.error_cnt));

  const bool clean = rx8.arm_fail_cnt == 0U && rx8.drop_bytes == 0U && rx8.hw_error_cnt == 0U &&
                     rx8.error_cnt == 0U && tx8.drop_bytes == 0U && tx8.start_fail_cnt == 0U &&
                     tx8.isr_skip_cnt == 0U && tx8.stall_recover_cnt == 0U && tx8.error_cnt == 0U &&
                     rx9.arm_fail_cnt == 0U && rx9.drop_bytes == 0U && rx9.hw_error_cnt == 0U &&
                     rx9.error_cnt == 0U && tx9.drop_bytes == 0U && tx9.start_fail_cnt == 0U &&
                     tx9.isr_skip_cnt == 0U && tx9.stall_recover_cnt == 0U && tx9.error_cnt == 0U &&
                     g_it_to_dma.errors == 0U && g_dma_to_it.errors == 0U;
  if (!clean)
  {
    g_uart_test_error = 20U;
    bsp_uart1.printf("[UART] FAIL errors=%lu (diag 见上)\r\n",
                     static_cast<unsigned long>(g_it_to_dma.errors + g_dma_to_it.errors));
    halt();
  }

  bsp_uart1.printf("[UART] PASS frames=%lu bytes=%lu\r\n",
                   static_cast<unsigned long>(g_it_to_dma.frames + g_dma_to_it.frames),
                   static_cast<unsigned long>(g_it_to_dma.bytes + g_dma_to_it.bytes));
  halt();
}

#endif // APP_TEST_UART_LOOPBACK_ENABLED
