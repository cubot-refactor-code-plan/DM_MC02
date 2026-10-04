#include "app_test.hpp"

#if APP_TEST_CAN3_IMU_ENABLED
#include "bsp_cfg.hpp"     // bsp_uart1
#include "device_cfg.hpp"  // dm_imu
#include "service_cfg.hpp" // bus_can3
#include "status.hpp"

#include "FreeRTOS.h" // IWYU pragma: keep
#include "task.h"

// CAN3 达妙 DM-IMU-L1 主动上报打印（CAN_ID 0x58 / MST_ID 0x59）
//
//   上电后模块按固定间隔主动上报，本任务**每收到一帧新数据就打印一行**，
//   不带任何统计 / 诊断输出。
//   串口输入任意字符：暂停打印，再输入恢复（帧号 #N 可用来估算真实帧率）。
//
// 打印样例：
//   [IMU] #1234 ypr=+1.23/-0.45/+0.67 T=32.1 a=+0.12/-0.03/+9.81 gz=+0.001
//
// 串口：USART1，115200 8N1，纯 ASCII。

namespace
{
// ---------------- 测试参数 ----------------

constexpr uint32_t MONITOR_MS       = 2U;    ///< 新数据检查周期 (ms)
constexpr uint32_t PRINT_MIN_GAP_MS = 20U;   ///< 两次打印的最小间隔 (ms)：数据率远高于串口带宽，必须抽样
constexpr uint32_t PRINT_GAP_MS     = 2U;    ///< 每次打印后留出的排空时间 (ms)
constexpr uint32_t PROBE_WAIT_MS    = 50U;   ///< 开机对照探测后等待应答的时间 (ms)
constexpr uint32_t REBOOT_WAIT_MS   = 3000U; ///< 等模块重启完成的时间 (ms)：配置期发了 reboot
constexpr uint32_t STATUS_CHECK_MS  = 1000U; ///< 收不到数据帧时的告警检查周期 (ms)

/** @brief 打印后留出排空时间（串口 TX 流缓冲小，连续快速 printf 会挤爆缓冲） */
void print_gap()
{
  vTaskDelay(pdMS_TO_TICKS(PRINT_GAP_MS));
}

/** @brief 取走回退缓冲里的帧并计数（分发未命中的帧会落到这里） */
uint32_t drain_fallback()
{
  CanRxMsg rx    = {};
  uint32_t count = 0U;

  for (uint32_t i = 0U; i < 8U && bus_can3.receive(&rx, 0) == Status::OK; ++i)
    ++count;

  return count;
}

/**
 * @brief 打印一帧最新数据
 *
 * @param frames 当前数据帧计数（打印进帧号，可直接看出真实帧率）
 */
void print_new_data(uint32_t frames)
{
  const ImuData d = dm_imu.get_imu_data();

  bsp_uart1.printf("[IMU] #%u ypr=%+.2f/%+.2f/%+.2f T=%.1f a=%+.2f/%+.2f/%+.2f gz=%+.3f\r\n", static_cast<unsigned>(frames),
                   static_cast<double>(d.yaw), static_cast<double>(d.pitch), static_cast<double>(d.roll),
                   static_cast<double>(d.cur_temp), static_cast<double>(d.accel[0]), static_cast<double>(d.accel[1]),
                   static_cast<double>(d.accel[2]), static_cast<double>(d.gyro[2]));
  print_gap();
}
} // namespace

// ---------------- 任务 ----------------

extern "C" void can3_imu_test_task(void *argument)
{
  (void)argument;
  static_assert(configTICK_RATE_HZ == 1000U, "CAN3 IMU test requires 1 kHz tick");

  sys_flag_wait_running();

  // 开机自检（每项只打一次，正常收到数据后不再输出任何诊断）：
  // init() 依次发了「设间隔 / 切主动 / 保存 / 重启」，这里回显最后一条有应答的（应为 reg=0xFE = 保存参数）
  bsp_uart1.printf("\r\n[IMU] init ack: id=0x%02X reg=0x%02X code=0x%02X\r\n", static_cast<unsigned>(dm_imu.last_ack_id()),
                   static_cast<unsigned>(dm_imu.last_ack_reg()), static_cast<unsigned>(dm_imu.last_ack_code()));
  print_gap();

  // 配置期发了 reboot，等模块重新起来再探测
  vTaskDelay(pdMS_TO_TICKS(REBOOT_WAIT_MS));

  // 对照：再发一次读请求。能应答 → 模块还在应答模式（主动上报没生效）；不应答 → 已经切到主动模式
  (void)dm_imu.request_euler();
  vTaskDelay(pdMS_TO_TICKS(PROBE_WAIT_MS));
  bsp_uart1.printf("[IMU] probe ack: id=0x%02X reg=0x%02X code=0x%02X frames=%u\r\n", static_cast<unsigned>(dm_imu.last_ack_id()),
                   static_cast<unsigned>(dm_imu.last_ack_reg()), static_cast<unsigned>(dm_imu.last_ack_code()),
                   static_cast<unsigned>(dm_imu.data_frames()));
  print_gap();

  bsp_uart1.printf("[IMU] active report, any char = pause / resume\r\n");
  print_gap();

  uint8_t    ch          = 0U;
  size_t     n           = 0U;
  bool       printing    = true;
  uint32_t   printed_at  = dm_imu.data_frames();
  uint32_t   fb_total    = 0U;
  uint32_t   seen_frames = dm_imu.data_frames();
  TickType_t last_print  = xTaskGetTickCount();
  TickType_t last_check  = xTaskGetTickCount();

  for (;;)
  {
    const uint32_t frames = dm_imu.data_frames();

    // 有新数据才打印；数据率高于串口带宽，所以两次打印之间留一个最小间隔（抽样）
    if (printing && (frames != printed_at) && ((xTaskGetTickCount() - last_print) >= pdMS_TO_TICKS(PRINT_MIN_GAP_MS)))
    {
      print_new_data(frames);
      printed_at = frames;
      last_print = xTaskGetTickCount();
    }

    fb_total += drain_fallback();

    // 每秒自检：一直收不到数据帧就告警一次（收得到数据时这行永远不会出现）
    if ((xTaskGetTickCount() - last_check) >= pdMS_TO_TICKS(STATUS_CHECK_MS))
    {
      last_check = xTaskGetTickCount();

      if (frames == seen_frames)
      {
        bsp_uart1.printf("[IMU] no data frame, fb=%u RXd=%u RXl=%u\r\n", static_cast<unsigned>(fb_total),
                         static_cast<unsigned>(bsp_can3.diagnostics.rx_dropped),
                         static_cast<unsigned>(bsp_can3.diagnostics.rx_lost));
        print_gap();
      }

      seen_frames = frames;
    }

    if (bsp_uart1.receive(&ch, 1U, 0U, &n) == Status::OK && n > 0U)
    {
      printing = !printing;
      bsp_uart1.printf("[IMU] print %s\r\n", printing ? "on" : "off");
      print_gap();
    }

    vTaskDelay(pdMS_TO_TICKS(MONITOR_MS));
  }
}
#endif
