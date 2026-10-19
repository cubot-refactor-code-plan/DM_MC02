/**
 * @file online_check_test.cpp
 * @author Rh
 * @brief Online 状态机自检：阈值 20 ms，验“初始超时 / refresh 置在线 / 未超时仍在线 / 超时转离线”
 * @date 2026-10-03
 *
 * 结论写进 volatile 全局：stage 走 1→5，passed=1 表示四步全对，供调试器直接看。
 * 状态由 sys_task 的 Online::update()（10 ms 一拍）锁存，因此等待时间取“阈值 + 2 个任务周期”。
 */

#include "app_test.hpp"

#if APP_TEST_ONLINE_CHECK_ENABLED

#include "FreeRTOS.h" // IWYU pragma: keep
#include "online_check.hpp"
#include "task.h"

#include <stdint.h>

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
    // 阈值 20 ms。状态由 sys_task（10 ms 一拍）锁存，所以等待时间取
    // "阈值 + 2 个任务周期"，保证至少有一次 update() 能覆盖到超时时刻。
    Online probe(20U);
    online_check_test_stage          = 1U;
    online_check_test_initial_status = probe.is_online();

    online_check_test_refresh_status = probe.refresh_task();
    online_check_test_stage          = 2U;

    vTaskDelay(pdMS_TO_TICKS(5U)); // 5 ms < 20 ms，仍在线
    online_check_test_before_timeout_status = probe.is_online();
    online_check_test_stage                 = 3U;

    vTaskDelay(pdMS_TO_TICKS(40U)); // 累计 45 ms > 20 ms，且跨越多次 update()
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
