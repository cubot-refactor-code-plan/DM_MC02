/**
 * @file servo_test.cpp
 * @author Rh
 * @brief 舵机自检：1 kHz 推进限速轨迹，200 ms 打印目标角度 / 实际角度 / 脉宽
 * @date 2026-10-08
 *
 * @note 驱动对象是 device_cfg.cpp 里的 servo1（绑 bsp_pwm1 = PE13 / TIM1_CH3）。
 *       两种模式由下面的 SERVO_TEST_MODE_CALIBRATE 选：
 *
 *       0（默认）扫角 —— 去一端 → 到位后停 1 s → 换另一端，如此循环（按到位触发，不按固定周期）：
 *
 *       [SERVO] tgt= 270.0 deg=  72.3 pulse= 1035.6us   ← 目标已跳、实际还在爬
 *       [SERVO] tgt= 270.0 deg= 270.0 pulse= 2500.0us   ← 到位，从这里开始停 1 s 再换向
 *
 *       1  标定 —— 直接一步步写脉宽（不过角度换算、不过限速），用来量出真实端点：
 *
 *       [SERVO] CAL step=0  pulse=1500.0us    ← 每 800 ms 一步，先 1500 → 600
 *       [SERVO] CAL step=10 pulse= 500.0us    ← 到最低，再从 500 一路升到 2500
 *       [SERVO] CAL step=31 pulse=1500.0us    ← 走完回中位停住
 *
 *       判读：**位置不再变化的那个脉宽就是机械端点**，再往外加脉宽只是堵转（嗡响、发热）。
 *       记下两端脉宽，写回 device_cfg.cpp 的 servo1（脉宽端点与行程一起改），再把开关改回 0。
 *
 * @note 角度标尺按标称 270° 走：实测 500 → 2500 µs（跨度 2000 µs）实物只转约 200°
 *       （约 0.1 °/µs），但这点偏差不影响使用（用户 2026-10-08 决定），Config 行程仍填 270°。
 *       代价是指令的 200° ~ 270° 那一段不再有额外行程，脉宽只是继续往 2500 µs 走。
 *       要精确的话跑模式 1 量出真实端点，再把脉宽端点与行程一起改。
 *
 * @note 期望现象（舵机接 PE13、单独供电、与板子共地）：舵机在两端之间来回摆动，单程约 1.1 s、
 *       两端各停 1 s（停留从"到位"那一刻开始算，不是固定周期）；实测只转约 200°，看起来比
 *       270° 小一截是已知偏差，不是驱动的问题。
 *       若打印在变而舵机不动，按顺序查三件事：信号线是否真在 PE13、舵机电源与板子是否共地、
 *       脉宽端点是否与实物不符（跑模式 1 量）。
 *
 * @note 这是**开环**测试：PWM 舵机没有反馈线，打印出来的角度是"我们让它去的位置"，
 *       不是"它报回来的位置"。堵转 / 掉力都会显示成正常，别拿它当位置校验。
 */

#include "app_test.hpp"


#if APP_TEST_SERVO_ENABLED

// 依赖只在测试启用时有意义：放进 #if，避免关闭时整个 TU 为空、被 include-cleaner 判成多余
#  include "bsp_cfg.hpp"     // bsp_uart1（打印）
#  include "device_cfg.hpp"  // servo1
#  include "service_cfg.hpp" // sys_state
#  include "task.h"

#  include <math.h> // fabsf（判到位）


/** @brief 测试模式：1 = 脉宽步进标定（量真实端点）；0 = 来回扫角（看限速效果） */
#define SERVO_TEST_MODE_CALIBRATE 0

#if SERVO_TEST_MODE_CALIBRATE

// ---------------- 标定模式常量 ----------------

/** @brief 标定从哪个脉宽起步 (µs)：从中位出发，两侧都看得到 */
constexpr float CAL_START_US = 1500.0f;

/** @brief 标定扫描范围 (µs)：与 device_cfg.cpp 里 servo1 的脉宽端点一致 */
constexpr float CAL_MIN_US = 500.0f;
constexpr float CAL_MAX_US = 2500.0f;

/** @brief 步长 (µs)：100 µs 在 0.1 °/µs 的舵机上约 10°，够细 */
constexpr float CAL_STEP_US = 100.0f;

/** @brief 每步停留 (ms)：够看清这一步到底动没动 */
constexpr uint32_t CAL_HOLD_MS = 800U;

/** @brief 第一段步数：从起步降到扫描下限的步数（1500 → 600 共 10 步） */
constexpr uint32_t CAL_DOWN_STEPS = static_cast<uint32_t>((CAL_START_US - CAL_MIN_US) / CAL_STEP_US);

/** @brief 第二段步数：从扫描下限升到上限的步数（500 → 2500，含两端共 21 步） */
constexpr uint32_t CAL_UP_STEPS = static_cast<uint32_t>((CAL_MAX_US - CAL_MIN_US) / CAL_STEP_US) + 1U;

/** @brief 总步数：降段 + 升段 + 最后一步回中位 */
constexpr uint32_t CAL_TOTAL_STEPS = CAL_DOWN_STEPS + CAL_UP_STEPS + 1U;

/**
 * @brief 第 step 步要写的脉宽 (µs)
 *
 * @note 轨迹：1500 →（每步 −100）→ 600 → 500 →（每步 +100）→ 2500 → 回 1500。
 *       刻意先向下再向上，是为了让两端各只经过一次，方便观察"哪一步之后位置不再变化"。
 */
constexpr float cal_pulse_of_step(uint32_t step)
{
  return (step < CAL_DOWN_STEPS)
             ? (CAL_START_US - CAL_STEP_US * static_cast<float>(step))
             : ((step < CAL_DOWN_STEPS + CAL_UP_STEPS)
                    ? (CAL_MIN_US + CAL_STEP_US * static_cast<float>(step - CAL_DOWN_STEPS))
                    : CAL_START_US);
}

#else

// ---------------- 扫角模式常量 ----------------

/** @brief 打印周期 (ms)：5 Hz，够看清限速爬坡的过程 */
constexpr uint32_t SERVO_TEST_PRINT_MS = 200U;

/** @brief 到达一端后停留多久再换向 (ms) */
constexpr uint32_t SERVO_TEST_HOLD_MS = 1000U;

/**
 * @brief 判"到位"的角度容差 (度)
 *
 * @note update() 落位时会把实际角度**直接赋成**目标值，所以两者本来就严格相等；
 *       留 0.5° 只是不想让浮点相等成为隐式依赖。
 */
constexpr float SERVO_TEST_ARRIVE_EPS_DEG = 0.5f;

/** @brief 两个端点角度 (度)：与 device_cfg.cpp 里 servo1 的行程保持一致 */
constexpr float SERVO_TEST_LOW_DEG  = 0.0f;
constexpr float SERVO_TEST_HIGH_DEG = 270.0f;

#endif

extern "C" void servo_test_task(void *argument)
{
  (void)argument;
  sys_state.wait_running();
  static_assert(configTICK_RATE_HZ == 1000U, "本文件的周期常量按 1 ms tick 写");

#if SERVO_TEST_MODE_CALIBRATE
  bsp_uart1.printf("[SERVO] CAL on servo1 @PE13; 1500 -> 500 -> 2500 -> 1500us, %lums/step\r\n",
                   static_cast<unsigned long>(CAL_HOLD_MS));
  bsp_uart1.printf("[SERVO] CAL: 位置不再变化的那个脉宽就是机械端点（再往外只是堵转）\r\n");
#else
  bsp_uart1.printf("[SERVO] servo1 @PE13 TIM1_CH3; sweep 0 <-> 270 deg, slew limited\r\n");
#endif

  TickType_t wake_time = xTaskGetTickCount();
  uint32_t   tick      = 0U;

#if SERVO_TEST_MODE_CALIBRATE
  uint32_t applied = 0xFFFFFFFFU; // 已下发的步号（0xFFFFFFFF = 还没下发过任何一步）

  for (;;)
  {
    const uint32_t step = tick / CAL_HOLD_MS; // 每 CAL_HOLD_MS 走一步

    if (step != applied)
    {
      applied = step;
      if (step < CAL_TOTAL_STEPS)
      {
        servo1.set_pulse_us(cal_pulse_of_step(step));
        bsp_uart1.printf("[SERVO] CAL step=%lu pulse=%6.1fus\r\n",
                         static_cast<unsigned long>(step), servo1.get_pulse_us());
      }
      // 走完表就停在最后一步（回中位 1500 µs），不再变化
    }

    ++tick;
    vTaskDelayUntil(&wake_time, pdMS_TO_TICKS(1U));
  }
#else
  bool     at_high     = false; // 当前目标端：false = 低端（0°），true = 高端（270°）
  bool     arrived     = false; // 是否已到达当前目标端；到达后开始计停留时间
  uint32_t arrive_tick = 0U;    // 到达当前目标端的 tick

  servo1.set_angle(SERVO_TEST_LOW_DEG); // 上电先给低端，让舵机一开始就有脉冲

  for (;;)
  {
    servo1.update(); // 按两次调用之间流逝的时间推进实际角度，并写脉宽

    // 到位判据：落位时 angle_deg() 就等于 target_deg()，两者相差在容差内即算到位
    const bool at_target = (fabsf(servo1.get_angle_deg() - servo1.get_target_deg()) < SERVO_TEST_ARRIVE_EPS_DEG);

    if (!arrived && at_target)
    {
      arrived     = true;
      arrive_tick = tick; // 从到位的这一刻开始计停留
    }

    // 到位后停 SERVO_TEST_HOLD_MS 再换向；按到位触发而非固定周期，
    // 这样改了限速 / 行程，停留时间也不会跟着变。
    //
    // ⚠ 这段停留不是装饰：驱动是开环的，get_angle_deg() 只是"脉宽被写到了哪儿"，不是舵机的位置。
    //    脉宽还在斜坡上走的时候，舵机是在**追一个移动的目标**，自己的速度跟不上就永远差一截
    //    （速度环跟随误差）；按固定周期换向时（旧版 1.5 s），扣掉约 1.1 s 的斜坡只剩约 0.4 s
    //    停在端点上，实物还没来得及追上就被反向命令带走了，现象就是"到不了两端"。
    //    换成"到位 + 静止 1 s"之后，脉宽在端点上不动，舵机才有时间把最后那一段走完并停住。
    if (arrived && ((tick - arrive_tick) >= SERVO_TEST_HOLD_MS))
    {
      arrived = false;
      at_high = !at_high;
      servo1.set_angle(at_high ? SERVO_TEST_HIGH_DEG : SERVO_TEST_LOW_DEG);
    }

    if ((tick % SERVO_TEST_PRINT_MS) == 0U)
    {
      bsp_uart1.printf("[SERVO] tgt=%6.1f deg=%6.1f pulse=%7.1fus\r\n",
                       servo1.get_target_deg(), servo1.get_angle_deg(), servo1.get_pulse_us());
    }

    ++tick;
    vTaskDelayUntil(&wake_time, pdMS_TO_TICKS(1U));
  }
#endif
}
#endif
