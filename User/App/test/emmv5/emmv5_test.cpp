/**
 * @file emmv5_test.cpp
 * @author Rh
 * @brief 张大头 EmmV5.0 单电机点动测试：正转 2 圈 → 停 → 反转 2 圈 → 停，循环
 * @date 2026-10-18
 *
 * @note 驱动对象是 device_cfg.cpp 里的 emm_motor1：
 *
 *       UART7（bsp_uart7）+ 电机地址 1
 *
 * @note 运动参数：
 *
 *       - 脉冲当量按 1.8° 电机、16 细分计算：200 整步/圈 × 16 = 3200 脉冲/圈；
 *         如果电机实际细分不是 16，把 EMMV5_PULSES_PER_REV 改成 200 × 实际细分。
 *       - 2 圈 = 6400 脉冲；速度 300 RPM，加速度 50。
 *       - 使用**绝对位置模式**：先把当前位置清零，再去 +6400，最后回 0。
 *         这样第二条命令的目标确实在当前点后方，不依赖 dir 字节是否被固件采用。
 *       - 运动完成后用固定时间等待（不依赖到位返回帧）：
 *         2 圈 @300 RPM 的匀速时间是 0.4 s，实测取 2 s 留足加速与安全余量。
 *
 * @note 期望现象（串口 1 打印）：
 *
 *       [EMMV5] addr=1, UART7, baud=921600; forward 2 turns / reverse 2 turns
 *       [EMMV5] CW to +2 turns: target=6400 pulses
 *       [EMMV5] CW to +2 turns done
 *       [EMMV5] CCW back to 0: target=0 pulses
 *       [EMMV5] CCW back to 0 done
 *
 *       每段运动按固定时间等待，不再依赖到位帧。
 *
 * @warning UART7 在 CubeMX 里配置为 **921600 bps**，电机的通信波特率必须也是 921600；
 *          如果电机还是出厂 115200，会表现为「命令发得出去，但永远等不到到位帧」。
 *          要么用上位机把电机改成 921600，要么在 CubeMX 里把 UART7 改成 115200 后重新生成。
 *
 * @warning 上电后电机会立即开始运动。测试前确认电机悬空或负载安全，手不要靠近联轴器。
 */

#include "app_test.hpp"


#if APP_TEST_EMMV5_ENABLED

#  include "bsp_cfg.hpp"     // bsp_uart1（打印）、huart7（读波特率）
#  include "device_cfg.hpp"  // emm_motor1
#  include "service_cfg.hpp" // sys_state
#  include "task.h"


// ---------------- 测试参数 ----------------

/** @brief 每圈脉冲数：1.8° 电机 200 整步/圈 × 16 细分 */
constexpr uint32_t EMMV5_PULSES_PER_REV = 3200U;

/** @brief 每次运动圈数 */
constexpr uint32_t EMMV5_TEST_TURNS = 2U;

/** @brief 运动速度（RPM） */
constexpr uint16_t EMMV5_TEST_RPM = 300U;

/** @brief 加速度档位：0 = 不使用曲线加减速，直接按设定速度运行 */
constexpr uint8_t EMMV5_TEST_ACCEL = 0U;

/**
 * @brief 单段运动等待时间（ms）
 *
 * @note 2 圈 @300 RPM = 2 / (300/60) = 0.4 s；本测试用 acc=0（无曲线加减速），
 *       所以 2 s 足够。若把 acc 改回 50，按说明书公式每提升 1 RPM 用时
 *       (256-acc)*50us，0→300 RPM 就要 3.09 s，等待时间必须相应加长。
 */
constexpr uint32_t EMMV5_MOVE_TIME_MS = 2000U;

/** @brief 两段运动之间停留（ms） */
constexpr uint32_t EMMV5_HOLD_MS = 500U;


using Direction    = emmv5::Direction;
using PositionMode = emmv5::PositionMode;


// ----------------
// ---------------- 测试辅助 ----------------

/**
 * @brief 等待指定时间，同时把串口接收缓冲排空
 * @param ms 等待时间（ms）
 *
 * @note 这里刻意不依赖到位帧：部分 EmmV5.0 固件默认不返回 `FD 9F 6B`，
 *       用固定时间可以保证测试一定继续往下走。
 */
static void wait_and_drain(uint32_t ms)
{
  const TickType_t start = xTaskGetTickCount();

  while ((xTaskGetTickCount() - start) < pdMS_TO_TICKS(ms))
  {
    (void)emm_motor1.update(); // 仅为排空缓冲，不使用返回值
    vTaskDelay(pdMS_TO_TICKS(10U));
  }
}

/**
 * @brief 绝对位置运动到指定脉冲数，并等待固定时间
 * @param pulses 目标绝对位置（脉冲）
 * @param dir    方向（绝对模式下固件可能忽略，但填对更稳）
 * @param name   打印用的名字
 * @return true=命令已发送；false=发送失败
 */
static bool move_to(uint32_t pulses, Direction dir, const char *name)
{
  // 丢掉上一段运动可能残留的字节
  (void)emm_motor1.update();

  bsp_uart1.printf("[EMMV5] %s: target=%lu pulses, %u RPM\r\n",
                   name,
                   static_cast<unsigned long>(pulses),
                   static_cast<unsigned>(EMMV5_TEST_RPM));

  const Status status = emm_motor1.set_position(dir, EMMV5_TEST_RPM, EMMV5_TEST_ACCEL, pulses, PositionMode::Absolute);
  if (status != Status::OK)
  {
    bsp_uart1.printf("[EMMV5] send failed, status=%u\r\n", static_cast<unsigned>(status));
    return false;
  }

  wait_and_drain(EMMV5_MOVE_TIME_MS);

  bsp_uart1.printf("[EMMV5] %s done (waited %lums)\r\n",
                   name,
                   static_cast<unsigned long>(EMMV5_MOVE_TIME_MS));
  return true;
}


// ----------------
// ---------------- 测试任务 ----------------

extern "C" void emmv5_test_task(void *argument)
{
  (void)argument;
  sys_state.wait_running();
  static_assert(configTICK_RATE_HZ == 1000U, "本文件的周期常量按 1 ms tick 写");

  bsp_uart1.printf("[EMMV5] addr=1, UART7, baud=%lu; forward 2 turns / reverse 2 turns\r\n",
                   static_cast<unsigned long>(huart7.Init.BaudRate));

  // 使能后把当前位置定义为 0，后面的绝对位置就有一个确定基准
  (void)emm_motor1.enable(true);
  vTaskDelay(pdMS_TO_TICKS(200U));
  (void)emm_motor1.zero_position();
  wait_and_drain(200U);

  for (;;)
  {
    if (!move_to(EMMV5_TEST_TURNS * EMMV5_PULSES_PER_REV, Direction::CW, "CW to +2 turns"))
    {
      vTaskDelay(pdMS_TO_TICKS(2000U));
      continue;
    }

    vTaskDelay(pdMS_TO_TICKS(EMMV5_HOLD_MS));

    if (!move_to(0U, Direction::CCW, "CCW back to 0"))
    {
      vTaskDelay(pdMS_TO_TICKS(2000U));
      continue;
    }

    vTaskDelay(pdMS_TO_TICKS(EMMV5_HOLD_MS));
  }
}

// ----------------

#endif // APP_TEST_EMMV5_ENABLED
