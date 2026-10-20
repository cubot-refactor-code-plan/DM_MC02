/**
 * @file uart_stress_test.cpp
 * @author Rh
 * @brief UART8(IT) / UART9(DMA) 921600 全双工满速压力测试
 * @date 2026-10-20
 *
 * 硬件：PE1→PD14、PD15→PE0、共地；UART8=IT，UART9=DMA，均为 TxRx。
 * 负载：双向同时发 128 B 帧（121 B payload），默认 60 s；改 STRESS_SECONDS 可加长。
 * 打印：[UART] stress ... / [UART] PASS ...；失败后停机，g_stress_error 供调试器查看。
 */

#include "app_test.hpp"

#if APP_TEST_UART_STRESS_ENABLED

#  include "FreeRTOS.h" // IWYU pragma: keep
#  include "bsp_cfg.hpp"
#  include "service_cfg.hpp"
#  include "task.h"

#  include <stddef.h>
#  include <stdint.h>
#  include <string.h>


static_assert(configTICK_RATE_HZ == 1000U, "本测试的毫秒常量按 1 kHz tick 写");

namespace
{
// ---------------- 参数 ----------------

constexpr uint32_t BAUD           = 921600U; ///< 必须与 CubeMX 里的 UART8/9 一致
constexpr uint32_t STRESS_SECONDS = 300U; ///< 5 分钟长跑；改 600 可跑 10 分钟
constexpr size_t   PAYLOAD_LEN    = 121U;    ///< 121+7=128，正好压满 128 B 缓冲区
constexpr size_t   FRAME_LEN      = PAYLOAD_LEN + 7U;
constexpr uint32_t TX_TIMEOUT_MS  = 50U;     ///< 满速时正常只需 ~1.4 ms 就能入队
constexpr uint32_t RX_TIMEOUT_MS  = 20U;     ///< 收不到数据时最多等 20 ms
constexpr uint32_t TASK_STACK     = 512U;    ///< 每个收发任务 512 words
constexpr UBaseType_t TASK_PRIO   = tskIDLE_PRIORITY + 4U; ///< 低于 sys_task(+7)，高于空闲
constexpr uint8_t HEADER0         = 0xA5U;
constexpr uint8_t HEADER1         = 0x5AU;

static_assert(FRAME_LEN == 128U, "帧长必须与 128 B 缓冲区一致");


// ----------------
// ---------------- 状态 ----------------

struct StressDir
{
  bool         it_tx; ///< true=UART8(IT) 发、UART9(DMA) 收
  const char  *name;

  volatile uint32_t tx_frames;
  volatile uint32_t tx_bytes;
  volatile uint32_t tx_errors;

  volatile uint32_t rx_frames;
  volatile uint32_t rx_bytes;
  volatile uint32_t rx_errors;
};

StressDir g_dma_to_it {false, "DMA9->IT8"};
StressDir g_it_to_dma {true, "IT8->DMA9"};

volatile bool     g_stress_tx_run  = false;
volatile bool     g_stress_rx_run  = false;
volatile uint32_t g_stress_tx_done = 0U;
volatile uint32_t g_stress_rx_done = 0U;
volatile uint32_t g_stress_error   = 0U; ///< 1=任务未退出 3=帧错误或诊断脏


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

/** @brief 把某一路接收流缓冲区里已有的字节全部丢掉 */
void drain_rx(bool it_rx)
{
  uint8_t junk[32];
  while ((it_rx ? bsp_uart8.receive(junk, sizeof(junk), 0U)
                : bsp_uart9.receive(junk, sizeof(junk), 0U)) == Status::OK)
  {
  }
}

/** @brief 任务退出：临界区累加，避免两个任务同时 ++ 丢计数 */
void mark_stress_done(bool tx_task)
{
  taskENTER_CRITICAL();
  if (tx_task)
  {
    g_stress_tx_done++;
  }
  else
  {
    g_stress_rx_done++;
  }
  taskEXIT_CRITICAL();
  vTaskDelete(NULL);
}

/** @brief 只发不等的满速发送任务 */
void stress_tx_task(void *argument)
{
  StressDir *dir = static_cast<StressDir *>(argument);
  uint8_t    frame[FRAME_LEN];
  uint16_t   seq = 0U;

  while (g_stress_tx_run)
  {
    frame[0] = HEADER0;
    frame[1] = HEADER1;
    frame[2] = static_cast<uint8_t>(seq & 0xFFU);
    frame[3] = static_cast<uint8_t>(seq >> 8);
    frame[4] = static_cast<uint8_t>(PAYLOAD_LEN);
    for (size_t i = 0; i < PAYLOAD_LEN; i++)
    {
      frame[5U + i] = static_cast<uint8_t>(seq + i);
    }

    const uint16_t crc = crc16(frame + 2U, PAYLOAD_LEN + 3U);
    frame[5U + PAYLOAD_LEN] = static_cast<uint8_t>(crc & 0xFFU);
    frame[6U + PAYLOAD_LEN] = static_cast<uint8_t>(crc >> 8);

    Status st;
    if (dir->it_tx)
    {
      st = bsp_uart8.send(frame, FRAME_LEN, TX_TIMEOUT_MS);
    }
    else
    {
      st = bsp_uart9.send(frame, FRAME_LEN, TX_TIMEOUT_MS);
    }

    if (st == Status::OK)
    {
      dir->tx_frames++;
      dir->tx_bytes += static_cast<uint32_t>(FRAME_LEN);
      seq++;
    }
    else if (g_stress_tx_run)
    {
      dir->tx_errors++;
      vTaskDelay(1); // 发送持续失败时让出 CPU，避免空转
    }
  }

  mark_stress_done(true);
}

/** @brief 持续读并解析 128 B 帧；遇到噪声/错位时向后滑动重同步 */
void stress_rx_task(void *argument)
{
  StressDir *dir = static_cast<StressDir *>(argument);
  uint8_t    buffer[256];
  size_t     fill     = 0U;
  uint16_t   expected = 0U;

  while (g_stress_rx_run)
  {
    if (fill == sizeof(buffer))
    {
      memmove(buffer, buffer + 1U, --fill); // 缓冲满了还没找到帧头，丢最旧字节
      dir->rx_errors++;
    }

    size_t n  = 0U;
    Status st = Status::OK;
    if (dir->it_tx)
    {
      st = bsp_uart9.receive(buffer + fill, sizeof(buffer) - fill, RX_TIMEOUT_MS, &n);
    }
    else
    {
      st = bsp_uart8.receive(buffer + fill, sizeof(buffer) - fill, RX_TIMEOUT_MS, &n);
    }

    if (st == Status::OK)
    {
      fill += n;
    }

    while (fill >= 7U)
    {
      if (buffer[0] != HEADER0 || buffer[1] != HEADER1)
      {
        memmove(buffer, buffer + 1U, --fill);
        dir->rx_errors++;
        continue;
      }

      const size_t payload_len = buffer[4];
      if (payload_len > PAYLOAD_LEN)
      {
        memmove(buffer, buffer + 1U, --fill);
        dir->rx_errors++;
        continue;
      }

      const size_t frame_len = payload_len + 7U;
      if (fill < frame_len)
      {
        break;
      }

      const uint16_t seq = static_cast<uint16_t>(buffer[2]) |
                           static_cast<uint16_t>(buffer[3] << 8);
      bool ok = true;
      for (size_t i = 0; i < payload_len; i++)
      {
        if (buffer[5U + i] != static_cast<uint8_t>(seq + i))
        {
          ok = false;
          break;
        }
      }

      const uint16_t crc_rx = static_cast<uint16_t>(buffer[5U + payload_len]) |
                              static_cast<uint16_t>(buffer[6U + payload_len] << 8);
      if (crc_rx != crc16(buffer + 2U, payload_len + 3U))
      {
        ok = false;
      }

      if (ok)
      {
        if (seq != expected)
        {
          dir->rx_errors++;
        }
        expected = static_cast<uint16_t>(seq + 1U);
        dir->rx_frames++;
        dir->rx_bytes += static_cast<uint32_t>(frame_len);
      }
      else
      {
        dir->rx_errors++;
      }

      memmove(buffer, buffer + frame_len, fill - frame_len);
      fill -= frame_len;
    }
  }

  mark_stress_done(false);
}

/** @brief 双向同时跑 STRESS_SECONDS 秒，返回后统计已汇总到全局 StressDir */
void run_stress()
{
  g_stress_tx_run  = true;
  g_stress_rx_run  = true;
  g_stress_tx_done = 0U;
  g_stress_rx_done = 0U;

  configASSERT(xTaskCreate(stress_tx_task, "u9tx", TASK_STACK, &g_dma_to_it, TASK_PRIO, NULL) == pdPASS);
  configASSERT(xTaskCreate(stress_rx_task, "u8rx", TASK_STACK, &g_dma_to_it, TASK_PRIO, NULL) == pdPASS);
  configASSERT(xTaskCreate(stress_tx_task, "u8tx", TASK_STACK, &g_it_to_dma, TASK_PRIO, NULL) == pdPASS);
  configASSERT(xTaskCreate(stress_rx_task, "u9rx", TASK_STACK, &g_it_to_dma, TASK_PRIO, NULL) == pdPASS);

  const TickType_t t0 = xTaskGetTickCount();
  vTaskDelay(pdMS_TO_TICKS(STRESS_SECONDS * 1000U));

  // 先停发送：发送任务可能还卡在 send() 的 50 ms 超时里，给它留 200 ms
  g_stress_tx_run = false;
  TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(200U);
  while ((g_stress_tx_done < 2U) && (xTaskGetTickCount() < deadline))
  {
    vTaskDelay(1);
  }

  // 发送停后再给接收 30 ms，把在途的最后一两帧解析完，避免关闭瞬间 drop
  vTaskDelay(pdMS_TO_TICKS(30U));
  g_stress_rx_run = false;
  deadline = xTaskGetTickCount() + pdMS_TO_TICKS(500U);
  while ((g_stress_rx_done < 2U) && (xTaskGetTickCount() < deadline))
  {
    vTaskDelay(1);
  }

  const double elapsed = (double)(xTaskGetTickCount() - t0) / (double)configTICK_RATE_HZ; // DWT 长时间不调用会少补回绕，总时长用 tick
  if ((g_stress_tx_done < 2U) || (g_stress_rx_done < 2U))
  {
    g_stress_error = 1U;
    bsp_uart1.printf("[UART] stress task exit timeout tx=%lu rx=%lu\r\n",
                     static_cast<unsigned long>(g_stress_tx_done), static_cast<unsigned long>(g_stress_rx_done));
    halt();
  }

  StressDir *const dirs[] = {&g_dma_to_it, &g_it_to_dma};
  for (StressDir *dir : dirs)
  {
    bsp_uart1.printf("[UART] stress %s tx=%lu fr/%lu B rx=%lu fr/%lu B err=%lu\r\n", dir->name,
                     static_cast<unsigned long>(dir->tx_frames), static_cast<unsigned long>(dir->tx_bytes),
                     static_cast<unsigned long>(dir->rx_frames), static_cast<unsigned long>(dir->rx_bytes),
                     static_cast<unsigned long>(dir->tx_errors + dir->rx_errors));
    bsp_uart1.printf("[UART] stress %s tx=%luB/s rx=%luB/s\r\n", dir->name,
                     static_cast<unsigned long>(dir->tx_bytes / elapsed),
                     static_cast<unsigned long>(dir->rx_bytes / elapsed));
  }
}

// ----------------
} // namespace


extern "C" void uart_stress_test_task(void *argument)
{
  (void)argument;
  sys_state.wait_running();

  const bool fmt_ok = bsp_uart8.check_format(BAUD, UART_WORDLENGTH_8B, UART_PARITY_NONE, UART_STOPBITS_1) &&
                      bsp_uart9.check_format(BAUD, UART_WORDLENGTH_8B, UART_PARITY_NONE, UART_STOPBITS_1);
  if (!fmt_ok)
  {
    g_stress_error = 3U;
    bsp_uart1.printf("[UART] format mismatch: u8=%lu u9=%lu\r\n",
                     static_cast<unsigned long>(huart8.Init.BaudRate),
                     static_cast<unsigned long>(huart9.Init.BaudRate));
    configASSERT(false);
  }

  bsp_uart1.printf("[UART] stress start: 921600, UART8=IT, UART9=DMA, %lus\r\n",
                   static_cast<unsigned long>(STRESS_SECONDS));

  // 首帧前可能有一个启动 0x00，先各发一帧热身并丢弃
  const uint8_t warm[8] = {HEADER0, HEADER1, 0x00U, 0x00U, 0x01U, 0x00U, 0x00U, 0x00U};
  drain_rx(true);
  drain_rx(false);
  (void)bsp_uart9.send(warm, sizeof(warm), TX_TIMEOUT_MS);
  vTaskDelay(pdMS_TO_TICKS(5U));
  drain_rx(true);
  (void)bsp_uart8.send(warm, sizeof(warm), TX_TIMEOUT_MS);
  vTaskDelay(pdMS_TO_TICKS(5U));
  drain_rx(false);

  run_stress();

  const auto rx8 = bsp_uart8.rx_diag(); // 内嵌类型，auto 省掉模板名
  const auto tx8 = bsp_uart8.tx_diag();
  const auto rx9 = bsp_uart9.rx_diag();
  const auto tx9 = bsp_uart9.tx_diag();

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

  // 停止时可能还有 1~2 帧在途，允许 tx 比 rx 多 4 帧
  const bool counts_ok = g_dma_to_it.tx_frames >= g_dma_to_it.rx_frames &&
                         g_dma_to_it.tx_frames - g_dma_to_it.rx_frames <= 4U &&
                         g_it_to_dma.tx_frames >= g_it_to_dma.rx_frames &&
                         g_it_to_dma.tx_frames - g_it_to_dma.rx_frames <= 4U;
  const bool clean = rx8.arm_fail_cnt == 0U && rx8.drop_bytes == 0U && rx8.hw_error_cnt == 0U &&
                     rx8.error_cnt == 0U && tx8.drop_bytes == 0U && tx8.start_fail_cnt == 0U &&
                     tx8.isr_skip_cnt == 0U && tx8.stall_recover_cnt == 0U && tx8.error_cnt == 0U &&
                     rx9.arm_fail_cnt == 0U && rx9.drop_bytes == 0U && rx9.hw_error_cnt == 0U &&
                     rx9.error_cnt == 0U && tx9.drop_bytes == 0U && tx9.start_fail_cnt == 0U &&
                     tx9.isr_skip_cnt == 0U && tx9.stall_recover_cnt == 0U && tx9.error_cnt == 0U &&
                     g_dma_to_it.tx_errors == 0U && g_dma_to_it.rx_errors == 0U &&
                     g_it_to_dma.tx_errors == 0U && g_it_to_dma.rx_errors == 0U && counts_ok;
  if (!clean)
  {
    g_stress_error = 3U;
    bsp_uart1.printf("[UART] stress FAIL errors=%lu (diag 见上)\r\n",
                     static_cast<unsigned long>(g_dma_to_it.tx_errors + g_dma_to_it.rx_errors +
                                                g_it_to_dma.tx_errors + g_it_to_dma.rx_errors));
    halt();
  }

  bsp_uart1.printf("[UART] stress PASS tx=%lu fr / rx=%lu fr\r\n",
                   static_cast<unsigned long>(g_dma_to_it.tx_frames + g_it_to_dma.tx_frames),
                   static_cast<unsigned long>(g_dma_to_it.rx_frames + g_it_to_dma.rx_frames));
  halt();
}

#endif // APP_TEST_UART_STRESS_ENABLED
