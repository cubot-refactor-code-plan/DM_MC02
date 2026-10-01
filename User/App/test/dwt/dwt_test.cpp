#include "app_test.hpp"

#if APP_TEST_DWT_ENABLED

#  include "FreeRTOS.h" // IWYU pragma: keep
#  include "bsp_cfg.hpp"
#  include "task.h"

extern "C" void dwt_test_task(void *argument)
{
  (void)argument;

  // init() 由 bsp_init() 用 configASSERT 保证成功，这里直接开始测
  uint32_t last = 0;
  (void)bsp_dwt.delta_s(&last); // 第一次结果无效，丢掉

  for (;;)
  {
    // 标定：这 1 s 里 DWT 也应该走 ≈1.000 s（若显示 ≈0.500 就是 CPU 频率算错）
    const double t0 = bsp_dwt.time_s();
    vTaskDelay(pdMS_TO_TICKS(1000U));
    const double t1 = bsp_dwt.time_s();

    // 忙等延时自测：delay_ms(1) 实测应 ≈0.001 s
    (void)bsp_dwt.delta_s(&last);
    bsp_dwt.delay_ms(1);
    const double d1 = bsp_dwt.delta_s(&last);

    bsp_uart1.printf("[DWT] t=%.3f s | 1s=%+.4f | 1ms=%+.5f\r\n", bsp_dwt.time_s(), t1 - t0, d1);
  }
}

#endif // APP_TEST_DWT_ENABLED
