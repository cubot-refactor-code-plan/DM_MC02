#include "app_test.hpp"

#include "bsp_cfg.hpp"
#include "bsp_key.hpp" // IWYU pragma: keep (BspKey::Event)
#include "FreeRTOS.h"  // IWYU pragma: keep
#include "task.h"

volatile uint32_t key_press_cnt = 0U; ///< 按下次数（Live Watch 可看）
volatile uint32_t key_short_cnt = 0U; ///< 短按次数
volatile uint32_t key_long_cnt  = 0U; ///< 长按次数

extern "C" void key_test_task(void *argument)
{
  (void)argument;

  for (;;)
  {
    // 200ms 轮询（对应 key_user 的 debounce 1 / long_press 5 ⇒ 200ms 消抖 / 1s 长按）
    switch (key_user.poll())
    {
      case BspKey::Event::PRESS: // 3kHz/40ms：按下轻嘀
        key_press_cnt++;
        bsp_buzzer.beep(3000, 40);
        break;
      case BspKey::Event::SHORT: // 4kHz/100ms：短按 高而短
        key_short_cnt++;
        bsp_uart1.printf("[KEY] SHORT\r\n");
        bsp_buzzer.beep(4000, 100);
        break;
      case BspKey::Event::LONG: // 2kHz/500ms：长按 低而长
        key_long_cnt++;
        bsp_uart1.printf("[KEY] LONG\r\n");
        bsp_buzzer.beep(2000, 500);
        break;
      default:
        break;
    }

    vTaskDelay(200); // beep 是阻塞的，周期会多出一个响铃时长，测试无所谓
  }
}
