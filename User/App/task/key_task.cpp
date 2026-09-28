#include "bsp_cfg.hpp"
#include "bsp_key.hpp" // IWYU pragma: keep (BspKey::Event)
#include "FreeRTOS.h"  // IWYU pragma: keep
#include "task.h"

extern "C" void key_task(void *argument)
{
  (void)argument;

  for (;;)
  {
    // 200ms 轮询（对应 key_user 的 debounce 1 / long_press 5 ⇒ 200ms 消抖 / 1s 长按）
    switch (key_user.poll())
    {
      case BspKey::Event::PRESS: // 3kHz/40ms：按下轻嘀
        bsp_buzzer.beep(3000, 40);
        break;
      case BspKey::Event::SHORT: // 4kHz/100ms：短按 高而短
        bsp_buzzer.beep(4000, 100);
        break;
      case BspKey::Event::LONG: // 2kHz/500ms：长按 低而长
        bsp_buzzer.beep(2000, 500);
        break;
      default:
        break;
    }

    vTaskDelay(200); // beep 是阻塞的，周期会多出一个响铃时长，测试无所谓
  }
}
