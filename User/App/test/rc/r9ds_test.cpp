/**
 * @file r9ds_test.cpp
 * @author Rh
 * @brief R9DS 遥控接收机自检：打印 10 个通道的语义值，或打印在线状态（由 R9DS_TEST_PRINT_CHANNELS 二选一）
 * @date 2026-10-07
 *
 * @note **两种打印互斥**，由下面的 R9DS_TEST_PRINT_CHANNELS 选：
 *
 *       1（默认）—— 只打 10 个通道的语义值，200 ms 一次，确认“拨哪个部件、哪个数在动”。
 *                    期望看到（接收机拨到 SBUS 模式、蓝灯）：
 *
 *                    [R9DS] CH1..CH5   +100    +0    +0    +0  +100
 *                    [R9DS] CH6..CH10    +0    +0    +0  -100  +100
 *
 *                    拨右摇杆上下 → 只有第 1 个数在动；拨 C 三挡开关 → 只有第 5 个数在
 *                    +100/0/-100 之间跳。通道含义见 device_r9ds.hpp 文件头那张表。
 *                    开关也换算到同一量纲：上 +100、中 0、下 -100。
 *
 *       0        —— 只打在线状态，20 ms 一次（比上一种快 10 倍），盯遥控开关机时 on / fs 的跳变：
 *
 *                    [R9DS] on=0 fs=1 lost=1 ls=12 byte=63250 frame=2530 badtail=0
 *
 *                    on   = is_online()：帧到 + 没失控 + 没在连续丢帧，三个都满足才是 1
 *                    fs   = flags 的失控保护（bit3，电平量）—— 遥控器确认失联后才置起来
 *                    lost = flags 的本帧丢失（bit2，瞬时量）—— 链路一变差就置，比 fs 早
 *                    ls   = 连续丢帧数（lost_streak），收到一帧干净的帧就清 0
 *
 *                    实测的跳变顺序（关机 → 再开机）：
 *                      遥控器开着            on=1 fs=0 lost=0 ls=0
 *                      刚关掉、射频在断        on=0 fs=0 lost=1 ls≥3   ← 靠 ls 提前判离线
 *                      确认失联              on=0 fs=1 lost=1 ls 继续涨
 *                      遥控器重新开机          on=1 fs=0 lost=0 ls=0
 *
 *                    计数用来区分三种“收不到”：byte 不涨 = UART5 没数据（查接线 / 波特率 /
 *                    反相）；byte 涨而 frame 不涨 = 切不出帧（查帧格式 / 反相）；byte 和 frame
 *                    都在涨而 on=0 = 接收机在发帧，但遥控器与它失联（关机 / 超距）。
 *
 * @note 遥控器关机后接收机不会安静下来：它继续按帧周期发 SBUS，只把 fs 置起来，
 *       所以 frame 一直涨而 on 变 0 —— 这就是“遥控器失联”，判据见 device_r9ds.hpp 的 is_online()。
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
 * @brief 打印内容开关：1 = 只打 10 个通道的语义值；0 = 只打在线状态（频率更高）
 *
 * @note 做成二选一而不是两路叠加：同时打会把串口 1 刷满，跳变时刻反而看不清是哪条引起的。
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
    r9ds.update(); // 收帧：把这一拍串口里攒的字节全消化掉（非阻塞，可能解出 0 ~ n 帧）

    if ((tick % PRINT_PERIOD_MS) == 0U)
    {
#if R9DS_TEST_PRINT_CHANNELS
      // 打语义值：拨哪个部件，对应的那个数就会动
      const DeviceR9ds::Rc &rc = r9ds.rc();

      bsp_uart1.printf("[R9DS] CH1..CH5  %+4d %+4d %+4d %+4d %+4d\r\n",
                       rc.right_vertical, rc.right_horizontal, rc.left_vertical, rc.left_horizontal,
                       static_cast<int>(rc.switch_c) * 100);

      bsp_uart1.printf("[R9DS] CH6..CH10 %+4d %+4d %+4d %+4d %+4d\r\n",
                       rc.knob_left, rc.slider_left, rc.knob_right,
                       static_cast<int>(rc.switch_b) * 100, static_cast<int>(rc.switch_a) * 100);
#else
      // 打在线状态：带上计数，免得“值没变”其实是“根本没数据进来”
      const DeviceR9ds::Diag d = r9ds.diag();

      bsp_uart1.printf("[R9DS] on=%u fs=%u lost=%u ls=%lu byte=%lu frame=%lu badtail=%lu\r\n",
                       r9ds.is_online() ? 1U : 0U,
                       r9ds.failsafe() ? 1U : 0U,
                       r9ds.frame_lost() ? 1U : 0U,
                       static_cast<unsigned long>(d.lost_streak),
                       static_cast<unsigned long>(d.byte_cnt),
                       static_cast<unsigned long>(d.frame_cnt),
                       static_cast<unsigned long>(d.bad_footer_cnt));
#endif
    }

    ++tick;
    vTaskDelayUntil(&wake_time, pdMS_TO_TICKS(1U));
  }
}
#endif



