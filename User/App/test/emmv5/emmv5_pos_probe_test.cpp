/**
 * @file emmv5_pos_probe_test.cpp
 * @author Rh
 * @brief EmmV5 双电机位置采集探针：电机失能，手推 CoreXY 到四个边缘，打印 id1/id2 的 0x36 位置
 * @date 2026-10-22
 *
 * 硬件：USART2 RS485 115200 8N1，PD4 硬件 DE；电机地址 1、2
 * 流程：上电发一次失能 → 逐台轮询 0x36 → 位置有变化就打印一行
 * 打印：[POS] id1=+0012345 (+067.83 deg)  id2=-0023456 (-128.85 deg)
 *
 * 用法：手推滑台走到四个极限角，停下后最后一行就是该角的稳定读数。
 *       counts 一圈 = 65536；deg 一列是 counts 换算的角度（0.01° 精度）。
 */

#include "app_test.hpp"

#if APP_TEST_EMMV5_POS_PROBE_ENABLED

#  include "FreeRTOS.h" // IWYU pragma: keep
#  include "bsp_cfg.hpp"  // bsp_uart1
#  include "device_cfg.hpp" // emm_group
#  include "service_cfg.hpp" // sys_state
#  include "task.h"

#  include <stddef.h>
#  include <stdint.h>


namespace
{
constexpr uint8_t ADDR[2] = {1U, 2U}; ///< CoreXY 两台
constexpr size_t  MOTORS  = 2U;

constexpr uint32_t READ_TIMEOUT_MS = 30U;  ///< 单台等应答上限
constexpr uint32_t REQUEST_GAP_MS  = 10U;  ///< 两台请求之间留的空档
constexpr uint32_t CYCLE_GAP_MS    = 100U; ///< 一轮采完到下一轮

/**
 * @brief 读一台的 0x36 实时位置
 * @return false = 超时没收到应答
 *
 * @note 必须一台一台问：RS485 半双工，两台同时请求会让它们的回帧在总线上撞车，
 *       两帧都变成废帧，扫描器一帧也锁不到。
 */
bool read_one(uint8_t addr, int32_t &out)
{
  int32_t stale = 0;
  (void)emm_group.get_position(addr, stale); // 丢掉上一轮留下的旧值

  if (emm_group.request_position(addr) != Status::OK)
  {
    return false;
  }

  const uint32_t begin = xTaskGetTickCount();
  while ((xTaskGetTickCount() - begin) < pdMS_TO_TICKS(READ_TIMEOUT_MS))
  {
    if (emm_group.get_position(addr, out))
    {
      return true;
    }

    vTaskDelay(1);
  }

  return false;
}

/** @brief counts → 角度，单位 0.01°（1 圈 = 65536 counts） */
int32_t centideg_of(int32_t counts)
{
  return static_cast<int32_t>(static_cast<int64_t>(counts) * 36000 / 65536);
}

} // namespace


extern "C" void emmv5_pos_probe_test_task(void *argument)
{
  (void)argument;
  sys_state.wait_running();

  vTaskDelay(pdMS_TO_TICKS(2000));
  bsp_uart1.printf("[POS] probe start: disable id1/id2, move CoreXY by hand\r\n");
  vTaskDelay(pdMS_TO_TICKS(CYCLE_GAP_MS));
  (void)emm_group.enable(ADDR[0], false);
  vTaskDelay(pdMS_TO_TICKS(CYCLE_GAP_MS));
  (void)emm_group.enable(ADDR[1], false);
  vTaskDelay(pdMS_TO_TICKS(CYCLE_GAP_MS));

  int32_t pos[MOTORS]  = {};
  int32_t last[MOTORS] = {};
  bool    has_last     = false;

  for (;;)
  {
    const bool ok1 = read_one(ADDR[0], pos[0]);
    vTaskDelay(pdMS_TO_TICKS(REQUEST_GAP_MS));
    const bool ok2 = read_one(ADDR[1], pos[1]);

    if (!ok1 || !ok2)
    {
      bsp_uart1.printf("[POS] no reply: id1=%s id2=%s\r\n", ok1 ? "ok" : "--", ok2 ? "ok" : "--");
      vTaskDelay(pdMS_TO_TICKS(CYCLE_GAP_MS));
      continue;
    }

    // 没动就不刷屏，手停下后屏幕上最后一行就是该角的读数
    if (has_last && pos[0] == last[0] && pos[1] == last[1])
    {
      vTaskDelay(pdMS_TO_TICKS(CYCLE_GAP_MS));
      continue;
    }

    const int32_t d1 = centideg_of(pos[0]);
    const int32_t d2 = centideg_of(pos[1]);
    const int32_t a1 = (d1 < 0) ? -d1 : d1;
    const int32_t a2 = (d2 < 0) ? -d2 : d2;

    bsp_uart1.printf("[POS] id1=%+08ld (%s%ld.%02ld deg)  id2=%+08ld (%s%ld.%02ld deg)\r\n",
                     static_cast<long>(pos[0]),
                     (d1 < 0) ? "-" : "+",
                     static_cast<long>(a1 / 100),
                     static_cast<long>(a1 % 100),
                     static_cast<long>(pos[1]),
                     (d2 < 0) ? "-" : "+",
                     static_cast<long>(a2 / 100),
                     static_cast<long>(a2 % 100));

    last[0]  = pos[0];
    last[1]  = pos[1];
    has_last = true;

    vTaskDelay(pdMS_TO_TICKS(CYCLE_GAP_MS));
  }
}

#endif // APP_TEST_EMMV5_POS_PROBE_ENABLED
