#include "app_test.hpp"

#include "FreeRTOS.h" // IWYU pragma: keep
#include "online_check.hpp"
#include "task.h"

#include <stdint.h>


#if APP_TEST_ONLINE_CHECK_ENABLED

volatile uint32_t online_check_test_stage                 = 0U;
volatile uint32_t online_check_test_passed                = 0U;
volatile Status   online_check_test_initial_status        = Status::NOT_INIT;
volatile Status   online_check_test_refresh_status        = Status::NOT_INIT;
volatile Status   online_check_test_before_timeout_status = Status::NOT_INIT;
volatile Status   online_check_test_after_timeout_status  = Status::NOT_INIT;


extern "C" void online_check_test_task(void *argument)
{
  (void)argument; // 测试任务不需要外部参数。

  {
    Online probe(4U);
    online_check_test_stage          = 1U;
    online_check_test_initial_status = probe.is_online();

    online_check_test_refresh_status = probe.refresh_task();
    online_check_test_stage          = 2U;

    vTaskDelay(pdMS_TO_TICKS(1U));
    online_check_test_before_timeout_status = probe.is_online();
    online_check_test_stage                 = 3U;

    vTaskDelay(pdMS_TO_TICKS(5U));
    online_check_test_after_timeout_status = probe.is_online();
    online_check_test_stage                = 4U;

    online_check_test_passed =
      ((online_check_test_initial_status == Status::TIMEOUT) && (online_check_test_refresh_status == Status::OK) && (online_check_test_before_timeout_status == Status::OK) && (online_check_test_after_timeout_status == Status::TIMEOUT))
        ? 1U
        : 0U;
  }

  online_check_test_stage = 5U;
  vTaskDelete(nullptr);
}

#endif // APP_TEST_ONLINE_CHECK_ENABLED
