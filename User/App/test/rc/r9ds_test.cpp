/**
 * @file r9ds_test.cpp
 * @author Rh
 * @brief R9DS 遥控接收机自检：每 200 ms 打一次 CH1 ~ CH10，拨哪个部件就哪个数在动
 * @date 2026-10-07
 *
 * @note 期望看到（接收机拨到 SBUS 模式、蓝灯）：
 *
 *       [R9DS] CH1..CH5   +100    +0    +0    +0  +100
 *       [R9DS] CH6..CH10    +0    +0    +0  -100  +100
 *
 *       拨右摇杆上下 → 只有第 1 个数在动；拨 C 三挡开关 → 只有第 5 个数在 +100/0/-100 之间跳。
 *       通道含义见 device_r9ds.hpp 文件头那张表。开关也换算到同一量纲：上 +100、中 0、下 -100。
 *
 * @note 每秒钟还会打一行收帧统计，用来区分两种“收不到”：
 *       byte=0 —— UART5 一个字节都没进来（查接线 / 波特率 / 反相）；
 *       byte 在涨而 frame 不涨 —— 字节进来了但切不出帧（查帧格式 / 反相）。
 */

#include "app_test.hpp"


#if APP_TEST_R9DS_ENABLED

// 依赖只在测试启用时有意义：放进 #if，避免关闭时整个 TU 为空、被 include-cleaner 判成多余
#  include "bsp_cfg.hpp"     // bsp_uart1（打印）
#  include "device_cfg.hpp"  // r9ds
#  include "service_cfg.hpp" // sys_state
#  include "task.h"

#  include <stdint.h>


/** @brief 语义值打印周期 (ms) */
constexpr uint32_t VALUE_PERIOD_MS = 200U;

/** @brief 收帧统计打印周期 (ms) */
constexpr uint32_t STATS_PERIOD_MS = 1000U;

extern "C" void r9ds_test_task(void *argument)
{
  (void)argument;
  sys_state.wait_running();
  static_assert(configTICK_RATE_HZ == 1000U, "本文件的周期常量按 1 ms tick 写");

  bsp_uart1.printf("[R9DS] SBUS on UART5; CH1..CH10, -100..+100 (switch up/mid/down = +100/0/-100)\r\n");

  TickType_t wake_time = xTaskGetTickCount();
  uint32_t   tick      = 0U;

  for (;;)
  {
    r9ds.update(); // 收帧：把这一拍串口里攒的字节全消化掉（非阻塞，可能解出 0 ~ n 帧）

    // 打语义值：拨哪个部件，对应的那个数就会动
    if ((tick % VALUE_PERIOD_MS) == 0U)
    {
      const DeviceR9ds::Rc &rc = r9ds.rc();

      bsp_uart1.printf("[R9DS] CH1..CH5  %+4d %+4d %+4d %+4d %+4d\r\n",
                       rc.right_vertical, rc.right_horizontal, rc.left_vertical, rc.left_horizontal,
                       static_cast<int>(rc.switch_c) * 100);

      bsp_uart1.printf("[R9DS] CH6..CH10 %+4d %+4d %+4d %+4d %+4d\r\n",
                       rc.knob_left, rc.slider_left, rc.knob_right,
                       static_cast<int>(rc.switch_b) * 100, static_cast<int>(rc.switch_a) * 100);
    }

    // 打收帧统计：byte=0 是串口没数据，byte 在涨而 frame 不涨是切不出帧
    if ((tick % STATS_PERIOD_MS) == 0U)
    {
      const DeviceR9ds::Diag d = r9ds.diag();

      bsp_uart1.printf("[R9DS] on=%u byte=%lu frame=%lu badtail=%lu\r\n",
                       r9ds.is_online() ? 1U : 0U,
                       static_cast<unsigned long>(d.byte_cnt),
                       static_cast<unsigned long>(d.frame_cnt),
                       static_cast<unsigned long>(d.bad_footer_cnt));
    }

    ++tick;
    vTaskDelayUntil(&wake_time, pdMS_TO_TICKS(1U));
  }
}
#endif



