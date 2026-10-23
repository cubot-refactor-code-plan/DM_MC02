/**
 * @file corexy_task.cpp
 * @author Rh
 * @brief 上位机协议收包任务：读串口 → 解帧 → 投递命令
 * @version 3.0
 * @date 2026-10-23
 *
 * @copyright Copyright (c) 2026
 *
 * @note 本任务只搬字节，不碰电机；协议细节在 `corexy_protocol.*`。
 */

#include "app_task.hpp"

#include "bsp_cfg.hpp" // bsp_uart1
#include "corexy_protocol.hpp"
#include "service_cfg.hpp"

#include "FreeRTOS.h" // IWYU pragma: keep
#include "task.h"

#include <stddef.h>
#include <stdint.h>


extern "C" void corexy_task(void *argument)
{
  (void)argument;
  sys_state.wait_running();

  uint8_t buf[32];

  for (;;)
  {
    size_t n = 0U;

    // 最多等 10 ms：有数据提前醒，没数据就空转一圈
    if ((bsp_uart1.receive(buf, sizeof(buf), 10U, &n) == Status::OK) && (n > 0U))
    {
      cxy::feed(buf, n);
    }
  }
}
