/**
 * @file emmv5_test.cpp
 * @author Rh
 * @brief EmmV5 单电机点动：正转 2 圈 → 停 → 反转 2 圈 → 停，循环
 * @date 2026-10-18
 *
 * 硬件：UART7（bsp_uart7）921600，电机地址 1，电机通信波特率必须也是 921600
 * 参数：3200 脉冲/圈（1.8° 电机 16 细分）、300 RPM、acc=0；用绝对位置模式，先清零再去 +6400 再回 0
 *
 * 打印：[EMMV5] CW to +2 turns, wait 2000ms / [EMMV5] CCW back to 0, wait 2000ms ...
 *
 * ⚠ 上电即动，测试前确认电机悬空或负载安全，手不要靠近联轴器。
 */

#include "app_test.hpp"


#if APP_TEST_EMMV5_ENABLED

#  include "bsp_cfg.hpp"     // bsp_uart1（打印）、huart7（读波特率）
#  include "device_cfg.hpp"  // emm_motor1
#  include "service_cfg.hpp" // sys_state
#  include "task.h"


// ---------------- 参数 ----------------

constexpr uint32_t PULSES_PER_REV = 3200U; ///< 200 整步 × 16 细分；换细分要改这里
constexpr uint32_t TURNS          = 2U;
constexpr uint16_t RPM            = 300U;
constexpr uint8_t  ACCEL          = 0U; ///< 0 = 无曲线加减速，直接按设定速度跑

/**
 * @brief 每段等多久 (ms)
 * @note acc=0 时 2 圈 @300 RPM 只要 0.4 s，2 s 足够。改回 acc=50 时 0→300 RPM 要 3.09 s
 *       （说明书：每提升 1 RPM 用 (256-acc)*50us），等待时间必须跟着加长。
 * @note 用固定时间等，不等到位帧：部分 EmmV5 固件默认不返回 FD 9F 6B。
 */
constexpr uint32_t MOVE_MS = 2000U;

constexpr uint32_t HOLD_MS = 500U; ///< 两段之间停留


// ----------------
// ---------------- 测试任务 ----------------

extern "C" void emmv5_test_task(void *argument)
{
  (void)argument;
  sys_state.wait_running();
  static_assert(configTICK_RATE_HZ == 1000U, "本文件的周期常量按 1 ms tick 写");

  bsp_uart1.printf("[EMMV5] addr=1, UART7, baud=%lu\r\n", static_cast<unsigned long>(huart7.Init.BaudRate));

  // 把当前点定义成 0，后面的绝对位置才有确定基准
  (void)emm_motor1.enable(true);
  vTaskDelay(pdMS_TO_TICKS(200U));
  (void)emm_motor1.zero_position();
  vTaskDelay(pdMS_TO_TICKS(200U));

  constexpr int32_t TARGET = static_cast<int32_t>(TURNS * PULSES_PER_REV);

  for (;;)
  {
    (void)emm_motor1.set_position(emmv5::Direction::CW, RPM, ACCEL, TARGET, emmv5::PositionMode::Absolute);
    bsp_uart1.printf("[EMMV5] CW to +2 turns, wait %lums\r\n", static_cast<unsigned long>(MOVE_MS));
    vTaskDelay(pdMS_TO_TICKS(MOVE_MS + HOLD_MS));

    (void)emm_motor1.set_position(emmv5::Direction::CCW, RPM, ACCEL, 0, emmv5::PositionMode::Absolute);
    bsp_uart1.printf("[EMMV5] CCW back to 0, wait %lums\r\n", static_cast<unsigned long>(MOVE_MS));
    vTaskDelay(pdMS_TO_TICKS(MOVE_MS + HOLD_MS));
  }
}

// ----------------

#endif // APP_TEST_EMMV5_ENABLED
