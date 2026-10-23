/**
 * @file emmv5_group_test.cpp
 * @author Rh
 * @brief EmmV5 双电机 CoreXY Y 轴测试：两台反向各 1 圈，再反向回位，循环
 * @date 2026-10-19
 *
 * 硬件：USART2 RS485 115200 8N1，PD4 硬件 DE；电机地址 1、2
 * 流程：Y+ = id1 正转 / id2 反转，Y− = 反过来；一条 MMCL 广播两台
 * 参数：60 RPM、acc=50，单段实测约 2.24 s
 *
 * 打印：[EMM5] Y+ 1t / Y+ 1t ok 2238ms / Y- 1t / Y- 1t ok 2236ms ...
 */

#include "app_test.hpp"

#if APP_TEST_EMMV5_GROUP_ENABLED

#  include "FreeRTOS.h" // IWYU pragma: keep
#  include "bsp_cfg.hpp"  // bsp_uart1
#  include "device_cfg.hpp" // emm_group
#  include "emmv5_protocol.hpp"
#  include "service_cfg.hpp" // sys_state
#  include "task.h"

#  include <stddef.h>
#  include <stdint.h>


namespace
{
// ---------------- 参数 ----------------

constexpr uint8_t ADDR[2] = {1U, 2U}; ///< 组内地址
constexpr size_t  MOTORS  = 2U;

constexpr uint32_t PULSES_PER_REV = 3200U; ///< 1.8° 电机 200 整步 × 16 细分
constexpr int32_t  TRAVEL_PULSES  = 3200;  ///< 每段 1 圈
constexpr uint16_t RPM            = 60U;
constexpr uint8_t  ACCEL          = 50U;

constexpr uint32_t HOLD_MS         = 500U;  ///< 两段之间停留
constexpr uint32_t ZERO_WAIT_MS    = 300U;  ///< 清零后等待
constexpr uint32_t POLL_MS         = 10U;   ///< 两台交替查询，单台 20 ms
constexpr uint32_t READ_TIMEOUT_MS = 50U;   ///< 读段起始位置超时
constexpr uint32_t DEADLINE_MS     = 2400U; ///< 单段实测 2.24 s + 余量

constexpr int32_t WINDOW_COUNTS = 55; ///< 到位窗口 0.3°（1 圈 = 65536 counts）
constexpr int32_t STOP_COUNTS   = 9;  ///< 相邻两次采样位移 ≤9（0.05°）算已停
constexpr uint8_t ARRIVE_SAMPLES = 2U; ///< 连续满足几次才算到位
constexpr uint8_t RETRY_MAX     = 2U;

using Direction    = emmv5::Direction;
using PositionMode = emmv5::PositionMode;


// ----------------
// ---------------- 辅助 ----------------

/** @brief 出错就停机，等调试器接手 */
void halt()
{
  for (;;)
  {
    vTaskDelay(portMAX_DELAY);
  }
}

/** @brief 脉冲 → 0x36 报的 counts（3200 脉冲 = 65536 counts = 1 圈） */
int32_t counts_of(int32_t pulses)
{
  return static_cast<int32_t>(static_cast<int64_t>(pulses) * 65536 / static_cast<int64_t>(PULSES_PER_REV));
}

/** @brief counts → 脉冲（重发剩余位移时用） */
int32_t pulses_of(int32_t counts)
{
  return static_cast<int32_t>(static_cast<int64_t>(counts) * static_cast<int64_t>(PULSES_PER_REV) / 65536);
}

/**
 * @brief 读两台当前位置
 * @return false = 某一台 50 ms 内没回帧
 *
 * 接收由本任务调 emm_group.update() 推进，这里只发 0x36 请求再轮询 get_position()。
 *
 * @note 必须**一台一台**问：RS485 是半双工共享总线，同时发出两台请求时两台电机会
 *       几乎同一时刻开始回帧，两帧在总线上叠加后都变成废帧，扫描器一帧也锁不到。
 *       逐台“请求 → 等应答”才能保证同一时刻只有一个发送方。
 */
bool read_both(int32_t pos[MOTORS])
{
  for (size_t i = 0; i < MOTORS; i++)
  {
    int32_t stale = 0;
    (void)emm_group.get_position(ADDR[i], stale); // 丢掉上一段留下的旧值
    (void)emm_group.request_position(ADDR[i]);

    const uint32_t begin = xTaskGetTickCount();
    bool           got   = false;

    while ((xTaskGetTickCount() - begin) < pdMS_TO_TICKS(READ_TIMEOUT_MS))
    {
      if (emm_group.get_position(ADDR[i], pos[i]))
      {
        got = true;
        break;
      }

      vTaskDelay(1);
    }

    if (!got)
    {
      return false;
    }
  }

  return true;
}

/** @brief 一条 MMCL 广播两台相对位移；正 = CW，负 = CCW */
bool send_relative(const int32_t pulses[MOTORS])
{
  emm_group.multi_clear();

  for (size_t i = 0; i < MOTORS; i++)
  {
    const int32_t magnitude = (pulses[i] < 0) ? -pulses[i] : pulses[i];
    if (magnitude == 0)
    {
      continue;
    }

    const Direction dir = (pulses[i] > 0) ? Direction::CW : Direction::CCW;
    if (emm_group.queue_position(ADDR[i], dir, RPM, ACCEL, magnitude, PositionMode::Relative) != Status::OK)
    {
      return false;
    }
  }

  return emm_group.multi_send(emmv5::BROADCAST) == Status::OK;
}

// ----------------
// ---------------- 单段运动 ----------------

/**
 * @brief 两台同时走一段相对位移，轮询到双双停下
 * @param pulses 每台位移，带符号（正 = CW，负 = CCW）
 * @param tag    打印标签
 *
 * 到位 = 本段位移进入 ±0.3°，且相邻两次采样位移 ≤0.05°（说明真的停了）。
 * 超时按剩余位移重发，最多 2 次；还不行就广播停止并停机。
 */
void move_both(const int32_t pulses[MOTORS], const char *tag)
{
  bsp_uart1.printf("[EMM5] %s\r\n", tag);

  int32_t base[MOTORS]   = {}; // 本段起点
  int32_t target[MOTORS] = {}; // 本段目标位移（counts）
  for (size_t i = 0; i < MOTORS; i++)
  {
    target[i] = counts_of(pulses[i]);
  }

  if (!read_both(base))
  {
    bsp_uart1.printf("[EMM5] %s read start fail -> stop\r\n", tag);
    for (size_t i = 0; i < MOTORS; i++)
    {
      bsp_uart1.printf("[EMM5]   id=%u online=%u\r\n",
                       static_cast<unsigned>(ADDR[i]),
                       emm_group.is_online(ADDR[i]) ? 1U : 0U);
    }
    (void)emm_group.stop_all();
    halt();
  }

  if (!send_relative(pulses))
  {
    bsp_uart1.printf("[EMM5] %s send fail -> stop\r\n", tag);
    (void)emm_group.stop_all();
    halt();
  }

  int32_t now_pos[MOTORS]      = {}; // 最近一次采样
  int32_t prev[MOTORS]         = {}; // 上一次采样
  uint8_t stop_samples[MOTORS] = {}; // 连续“在窗口内且已停”的次数
  bool    arrived[MOTORS]      = {};

  for (size_t i = 0; i < MOTORS; i++)
  {
    now_pos[i] = base[i];
    prev[i]    = base[i]; // 第一次采样就拿起点当参考
  }

  const uint32_t start     = xTaskGetTickCount();
  uint32_t       deadline  = start + pdMS_TO_TICKS(DEADLINE_MS);
  uint32_t       last_poll = start - pdMS_TO_TICKS(POLL_MS);
  uint8_t        retries   = 0;
  size_t         turn      = 0;

  for (;;)
  {
    const uint32_t now = xTaskGetTickCount();

    // 两台交替查询 0x36，单台 20 ms 刷新一次
    if ((now - last_poll) >= pdMS_TO_TICKS(POLL_MS))
    {
      last_poll = now;
      (void)emm_group.read(ADDR[turn], emmv5::SystemParam::CurrentPosition);
      turn = (turn + 1U) % MOTORS;
    }

    bool all_arrived = true;
    for (size_t i = 0; i < MOTORS; i++)
    {
      int32_t pos = 0;
      if (emm_group.get_position(ADDR[i], pos) && !arrived[i])
      {
        const int32_t off  = (pos - base[i]) - target[i]; // 离目标还差多少
        const int32_t step = pos - prev[i];               // 这两次采样动了多少

        const bool in_window = (off <= WINDOW_COUNTS) && (off >= -WINDOW_COUNTS);
        const bool stopped   = (step <= STOP_COUNTS) && (step >= -STOP_COUNTS);

        stop_samples[i] = (in_window && stopped) ? static_cast<uint8_t>(stop_samples[i] + 1U) : 0U;
        arrived[i]      = (stop_samples[i] >= ARRIVE_SAMPLES);
        now_pos[i]      = pos;
        prev[i]         = pos;
      }

      all_arrived = all_arrived && arrived[i];
    }

    if (all_arrived)
    {
      bsp_uart1.printf("[EMM5] %s ok %lums\r\n", tag, static_cast<unsigned long>(now - start));
      return;
    }

    if (now < deadline)
    {
      vTaskDelay(1);
      continue;
    }

    if (retries >= RETRY_MAX)
    {
      bsp_uart1.printf("[EMM5] %s timeout -> stop\r\n", tag);
      (void)emm_group.stop_all();
      halt();
    }

    ++retries;
    bsp_uart1.printf("[EMM5] %s retry %u\r\n", tag, static_cast<unsigned>(retries));

    // 按剩余位移重发，并把基准挪到当前位置
    int32_t remain[MOTORS] = {};
    for (size_t i = 0; i < MOTORS; i++)
    {
      remain[i] = pulses_of(target[i] - (now_pos[i] - base[i]));

      base[i]         = now_pos[i];
      target[i]       = counts_of(remain[i]);
      prev[i]         = now_pos[i];
      stop_samples[i] = 0;
      arrived[i]      = false;
    }

    if (!send_relative(remain))
    {
      bsp_uart1.printf("[EMM5] %s retry fail -> stop\r\n", tag);
      (void)emm_group.stop_all();
      halt();
    }

    deadline = now + pdMS_TO_TICKS(DEADLINE_MS);
  }
}

// ----------------

} // namespace


// ---------------- 测试任务 ----------------

extern "C" void emmv5_group_test_task(void *argument)
{
  vTaskDelay(2000);
  (void)argument;
  sys_state.wait_running();
  static_assert(configTICK_RATE_HZ == 1000U, "本文件的周期常量按 1 ms tick 写");

  bsp_uart1.printf("[EMM5] group mmcl test start, addr 1/2, RS485 115200\r\n");

  // 两台一起使能并清零
  for (size_t i = 0; i < MOTORS; i++)
  {
    (void)emm_group.enable(ADDR[i], true);
    (void)emm_group.zero_position(ADDR[i]);
  }
  vTaskDelay(pdMS_TO_TICKS(ZERO_WAIT_MS));

  // CoreXY：Y 轴 = 两台反向
  const int32_t y_plus[MOTORS]  = {TRAVEL_PULSES, -TRAVEL_PULSES};
  const int32_t y_minus[MOTORS] = {-TRAVEL_PULSES, TRAVEL_PULSES};

  for (;;)
  {
    move_both(y_plus, "Y+ 1t");
    vTaskDelay(pdMS_TO_TICKS(HOLD_MS));

    move_both(y_minus, "Y- 1t");
    vTaskDelay(pdMS_TO_TICKS(HOLD_MS));
  }
}

// ----------------

#endif // APP_TEST_EMMV5_GROUP_ENABLED
