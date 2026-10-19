/**
 * @file r9ds_test.cpp
 * @author Rh
 * @brief R9DS 接收机自检：打印 10 个通道的语义值，或打印在线状态
 * @date 2026-10-07
 *
 * 两种打印互斥，由下面的 R9DS_TEST_PRINT_CHANNELS 选：
 *
 *   1 = 通道值，200 ms 一次。拨右摇杆上下 → 第 1 个数动；C 三挡开关 → 第 5 个数动
 *       [R9DS] CH1..CH5   +100    +0    +0    +0  +100
 *       [R9DS] CH6..CH10    +0    +0    +0  -100  +100
 *       开关换算成同一量纲：上 +100、中 0、下 -100。通道含义见 device_r9ds.hpp 的表。
 *
 *   0 = 在线状态，20 ms 一次，盯遥控开关机时 on / fs 的跳变
 *       [R9DS] on=0 fs=1 lost=1 ls=12 byte=63250 frame=2530 badtail=0
 *       on   = is_online()：有帧 + 没失控 + 没连续丢帧
 *       fs   = 失控保护位（遥控器确认失联后才置）
 *       lost = 本帧丢失（链路一变差就置，比 fs 早）
 *       ls   = 连续丢帧数，收到干净帧就清 0
 *
 * 收不到时按计数判断：byte 不涨 = UART5 没数据（接线/波特率/反相）；
 * byte 涨而 frame 不涨 = 切不出帧（帧格式/反相）；两者都涨而 on=0 = 遥控器失联。
 * 注意遥控器关机后接收机仍在发帧，只把 fs 置起来，所以 frame 一直涨而 on 变 0。
 */

#include "app_test.hpp"


#if APP_TEST_R9DS_ENABLED

// 依赖只在测试启用时有意义：放进 #if，避免关闭时整个 TU 为空、被 include-cleaner 判成多余
#  include "bsp_cfg.hpp"     // bsp_uart1（打印）
#  include "device_cfg.hpp"  // r9ds
#  include "service_cfg.hpp" // sys_state
#  include "task.h"

#  include <stdint.h>


/**
 * @brief 打印开关：1 = 通道值（200 ms）；0 = 在线状态（20 ms）
 * @note 二选一不叠加：同时打会把串口刷满，跳变时刻反而看不清。
 */
#define R9DS_TEST_PRINT_CHANNELS 0

#if R9DS_TEST_PRINT_CHANNELS

/** @brief 通道值打印周期 (ms)：5 Hz，够看清手动拨杆的动作 */
constexpr uint32_t PRINT_PERIOD_MS = 200U;

#else

/** @brief 在线状态打印周期 (ms)：50 Hz，够盯住遥控开关机时 on / fs / lost 的跳变时刻 */
constexpr uint32_t PRINT_PERIOD_MS = 20U;

#endif

extern "C" void r9ds_test_task(void *argument)
{
  (void)argument;
  sys_state.wait_running();
  static_assert(configTICK_RATE_HZ == 1000U, "本文件的周期常量按 1 ms tick 写");

#if R9DS_TEST_PRINT_CHANNELS
  bsp_uart1.printf("[R9DS] SBUS on UART5; CH1..CH10, -100..+100 (switch up/mid/down = +100/0/-100)\r\n");
#else
  bsp_uart1.printf("[R9DS] SBUS on UART5; online only (on=1 在线 / fs=1 遥控器失联)\r\n");
#endif

  TickType_t wake_time = xTaskGetTickCount();
  uint32_t   tick      = 0U;

  for (;;)
  {
    r9ds.update(); // 收帧：把串口里攒的字节消化完（非阻塞）

    if ((tick % PRINT_PERIOD_MS) == 0U)
    {
#if R9DS_TEST_PRINT_CHANNELS
      // 拨哪个部件，对应的那个数就会动
      const DeviceR9ds::Rc &rc = r9ds.rc();

      bsp_uart1.printf("[R9DS] CH1..CH5  %+4d %+4d %+4d %+4d %+4d\r\n",
                       rc.right_vertical, rc.right_horizontal, rc.left_vertical, rc.left_horizontal,
                       static_cast<int>(rc.switch_c) * 100);

      bsp_uart1.printf("[R9DS] CH6..CH10 %+4d %+4d %+4d %+4d %+4d\r\n",
                       rc.knob_left, rc.slider_left, rc.knob_right,
                       static_cast<int>(rc.switch_b) * 100, static_cast<int>(rc.switch_a) * 100);
#else
      // 带上计数：免得好久没变其实是根本没数据进来
      const DeviceR9ds::Diag d = r9ds.diag();

      bsp_uart1.printf("[R9DS] on=%u fs=%u lost=%u ls=%lu byte=%.0f frame=%.0f badtail=%lu\r\n",
                       r9ds.is_online() ? 1U : 0U,
                       r9ds.failsafe() ? 1U : 0U,
                       r9ds.frame_lost() ? 1U : 0U,
                       static_cast<unsigned long>(d.lost_streak),
                       d.byte_cnt,
                       d.frame_cnt,
                       static_cast<unsigned long>(d.bad_footer_cnt));
#endif
    }

    ++tick;
    vTaskDelayUntil(&wake_time, pdMS_TO_TICKS(1U));
  }
}
#endif



