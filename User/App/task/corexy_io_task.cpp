/**
 * @file corexy_io_task.cpp
 * @author Rh
 * @brief CoreXY 执行任务：独占电机总线，执行队列命令并回帧
 * @version 3.1
 * @date 2026-10-23
 *
 * @copyright Copyright (c) 2026
 *
 * @note 上电：清零 → 使能 → 复核原点 → 上报 BOOT；之后取队列命令执行，再 10 ms 推一拍状态机。
 *       上报方式跟随 `APP_COREXY_PROTOCOL`：1 = 二进制应答/事件帧，0 = 终端打印一行。
 *       帧格式与命令表见 `corexy_protocol.hpp`、`corexy.md` §6。
 */

#include "app_task.hpp"

#include "bsp_cfg.hpp" // bsp_uart1（命令码模式打印）
#include "corexy_protocol.hpp"
#include "module_cfg.hpp"
#include "service_cfg.hpp"

#include "FreeRTOS.h" // IWYU pragma: keep
#include "task.h"

#include <stdint.h>


namespace
{
///< 上报执行结果：协议模式发应答帧，命令码模式打印一行
void ans(const CorexyCmd &cmd, Status st)
{
#if APP_COREXY_PROTOCOL
  const CorexyModule::Point p = corexy.position();
  cxy::reply(static_cast<uint8_t>(cmd.type), st, p.x, p.y);
#else
  bsp_uart1.printf("[CXY] cmd 0x%02X -> st=%u\r\n", static_cast<unsigned>(cmd.type), static_cast<unsigned>(st));
#endif
}


///< 上报事件：协议模式发事件帧，命令码模式打印一行
void evt(uint8_t code, Status st)
{
  const CorexyModule::Point p = corexy.position();

#if APP_COREXY_PROTOCOL
  cxy::event(code, st, p.x, p.y);
#else
  bsp_uart1.printf("[CXY] %s st=%u pos=(%.2f, %.2f)\r\n",
                   (code == cxy::BOOT) ? "boot" : ((code == cxy::DONE) ? "done" : "fault"),
                   static_cast<unsigned>(st), p.x, p.y);
#endif
}


///< 执行一条命令
void run(const CorexyCmd &cmd)
{
  Status st = Status::OK;

  switch (cmd.type)
  {
    case CorexyCmdType::Enable:
      st = corexy.enable(cmd.p0 != 0); // 静默 + 两台 + 帧间隔都在模块里
      break;

    case CorexyCmdType::MoveAbs:
      st = corexy.move_to(CorexyModule::Point(cmd.a, cmd.b));
      break;

    case CorexyCmdType::MoveRel:
      st = corexy.move_by(CorexyModule::Point(cmd.a, cmd.b));
      break;

    case CorexyCmdType::Zero:
      st = corexy.zero_here();
      break;

    case CorexyCmdType::Stop:
      st = corexy.stop();
      break;

    case CorexyCmdType::SetSpeed:
      st = corexy.set_speed(static_cast<uint16_t>(cmd.p0), static_cast<uint8_t>(cmd.p1));
      break;

    case CorexyCmdType::QuickParam:
      // 单电机诊断帧：直接发（模块不再对外提供 quiet()，只在空闲时用它们）
      st = emm_group.set_quick_position_params(cmd.addr, static_cast<uint16_t>(cmd.p0), static_cast<uint8_t>(cmd.p1), (cmd.mode != 0U) ? emmv5::PositionMode::Absolute : emmv5::PositionMode::Relative);
      break;

    case CorexyCmdType::QuickPos:
      st = emm_group.set_quick_position(cmd.addr, cmd.p0);
      break;

    case CorexyCmdType::MoveMode:
      st = corexy.set_move_mode((cmd.p0 == 0) ? CorexyModule::MoveMode::Position
                                              : CorexyModule::MoveMode::Quick);
      break;

    case CorexyCmdType::Limits:
    {
      const CorexyModule::Rect th = corexy.limits_theory();
      const CorexyModule::Rect sf = corexy.limits_safe();

#if APP_COREXY_PROTOCOL
      const float t[4] = {th.x0, th.y0, th.x1, th.y1};
      const float s[4] = {sf.x0, sf.y0, sf.x1, sf.y1};
      cxy::reply_limits(Status::OK, t, s);
#else
      bsp_uart1.printf("[CXY] theory=(%.2f,%.2f)-(%.2f,%.2f) safe=(%.2f,%.2f)-(%.2f,%.2f)\r\n",
                       th.x0, th.y0, th.x1, th.y1, sf.x0, sf.y0, sf.x1, sf.y1);
#endif
      return;
    }
  }

  ans(cmd, st);
}

} // namespace


extern "C" void corexy_io_task(void *argument)
{
  (void)argument;
  sys_state.wait_running();
  vTaskDelay(pdMS_TO_TICKS(500U));

  // 上电：清零 → 使能 → 等几个轮询周期复核原点
  (void)corexy.zero_here();
  (void)corexy.enable(true);

  for (uint32_t i = 0U; i < 30U; i++)
  {
    corexy.update(10U);
  }

  {
    const CorexyModule::Point p = corexy.position();

#if APP_COREXY_PROTOCOL
    cxy::event(cxy::BOOT, corexy.origin_verified() ? Status::OK : Status::NOT_INIT, p.x, p.y);
#else
    bsp_uart1.printf("[CXY] boot st=%u pos=(%.2f, %.2f)\r\n",
                     static_cast<unsigned>(corexy.origin_verified() ? Status::OK : Status::NOT_INIT), p.x, p.y);
#endif
  }

  CorexyModule::State last = corexy.state();

  for (;;)
  {
    CorexyCmd cmd;

    while (xQueueReceive(corexy_cmd_queue, &cmd, 0) == pdPASS)
    {
      run(cmd);
    }

    corexy.update(10U); // 收字节 + 推进状态机（内部等最多 10 ms）

    const CorexyModule::State now = corexy.state();

    if (now != last)
    {
      if (now == CorexyModule::State::Fault)
      {
        evt(cxy::FAULT, corexy.last_error());
      }
      else if ((now == CorexyModule::State::Idle) && (last == CorexyModule::State::Moving))
      {
        evt(cxy::DONE, Status::OK);
      }

      last = now;
    }
  }
}
