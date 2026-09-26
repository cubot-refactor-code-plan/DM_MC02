#include "app_test.hpp"

#include "FreeRTOS.h" // IWYU pragma: keep
#include "online_check.hpp"
#include "task.h"

#include <stdint.h>


#if APP_TEST_ONLINE_CHECK_ENABLED

volatile uint32_t online_check_test_stage = 0U;
volatile uint32_t online_check_test_passed = 0U;
volatile Status online_check_test_initial_status = Status::NOT_INIT;
volatile Status online_check_test_refresh_status = Status::NOT_INIT;
volatile Status online_check_test_before_timeout_status = Status::NOT_INIT;
volatile Status online_check_test_after_timeout_status = Status::NOT_INIT;


extern "C" void online_check_test_task(void *argument)
{
  (void)argument; // 测试任务不需要外部参数。

  {
    // 阈值 50 ms，明显大于 update() 的调用周期（sys_task 为 10 ms）：
    // 等 10 ms 只推进约 1 次 update → 仍在线；再等 60 ms 累计约 70 ms → 离线。
    // （阈值若接近或小于 update() 周期，结果会随调用相位抖动，测试将不稳定。）
    Online probe(50U);
    online_check_test_stage = 1U;
    online_check_test_initial_status = probe.isOnline();

    online_check_test_refresh_status = probe.refresh_task();
    online_check_test_stage = 2U;

    vTaskDelay(pdMS_TO_TICKS(10U));
    online_check_test_before_timeout_status = probe.isOnline();
    online_check_test_stage = 3U;

    vTaskDelay(pdMS_TO_TICKS(60U));
    online_check_test_after_timeout_status = probe.isOnline();
    online_check_test_stage = 4U;

    online_check_test_passed =
      ((online_check_test_initial_status == Status::TIMEOUT) &&
       (online_check_test_refresh_status == Status::OK) &&
       (online_check_test_before_timeout_status == Status::OK) &&
       (online_check_test_after_timeout_status == Status::TIMEOUT))
        ? 1U
        : 0U;
  }

  online_check_test_stage = 5U;
  vTaskDelete(nullptr);
}

#endif // APP_TEST_ONLINE_CHECK_ENABLED
