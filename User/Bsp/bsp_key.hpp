/**
 * @file bsp_key.hpp
 * @author Rh
 * @brief 按键驱动：纯软件轮询消抖（无 ISR、无 FreeRTOS 依赖）
 * @version 0.2
 * @date 2026-07-25
 *
 * @copyright Copyright (c) 2026
 *
 * @details poll() 需在 FreeRTOS 任务中周期性调用（如每 50ms）。
 *
 * @note 时间换算（T = 轮询周期，即 poll() 的调用间隔）:
 *       消抖时间 = debounce_cnt × T
 *       长按时间 = long_press_cnt × T      ← 从"电平跳变那一刻"起算
 *
 * @note 有 4 种事件:
 *       - NONE:  无事件（常态，多数轮询返回此值）
 *       - PRESS: 按下（消抖确认那一刻触发一次）
 *       - SHORT: 短按触发（按下后在长按阈值内松手 → 触发一次）
 *       - LONG:  长按触发（按住达到 long_press_cnt × T → 触发一次）
 *
 * @note 使用约束:
 *       1. poll() 必须在【同一个任务】中周期性调用，本类非线程安全；
 *       2. init() 以当前电平作为初始稳定状态：上电时若按键已被按住，
 *          不会产生 PRESS（防误触发），直接从"已按下"状态开始计时。
 *       3. 未 init()（或 init() 失败）时，poll() / read_raw() 安全返回，不会崩溃。
 *
 * @note 使用示例 —— 简易按键扫描 + 测试：
 *
 *   // ── 1. 配置：低有效(上拉)，2×50ms = 100ms 消抖，20×50ms = 1s 长按 ──
 *   BspKey key;
 *   key.init({KEY_GPIO_Port, KEY_Pin, true, 2, 20});
 *
 *   // ── 2. 扫描任务：每 50ms 轮询一次 ──
 *   void key_test_task()
 *   {
 *     for (;;)
 *     {
 *       switch (key.poll())                // 无事件时返回 NONE
 *       {
 *         case BspKey::Event::PRESS:       // 按下（消抖确认）
 *           buzzer.beep(3000, 50);         // 鸣叫 50 ms
 *           break;
 *         case BspKey::Event::SHORT:       // 短按：阈值内松手
 *           buzzer.beep(2000, 50);
 *           break;
 *         case BspKey::Event::LONG:        // 长按：按住 1s，仅触发一次
 *           buzzer.beep(4000, 500);        // 鸣叫 500 ms
 *           break;                         // 注意：此后的松手不再产生 SHORT
 *         case BspKey::Event::NONE:
 *         default:
 *           break;
 *       }
 *       vTaskDelay(pdMS_TO_TICKS(50));   // 50ms 轮询周期
 *     }
 *   }
 *
 *   // ── 3. 创建任务（须在 FreeRTOS 内核启动后，如 bsp_init() 或默认任务中）──
 *   xTaskCreate(key_test_task, "key_test", 256, nullptr, 2, nullptr);
 */

#ifndef __BSP_KEY_HPP__
#define __BSP_KEY_HPP__

#include "main.h" // IWYU pragma: keep (GPIO_TypeDef, GPIO pins, HAL_GPIO_ReadPin)
#include <stdint.h>

#include "status.hpp" // 统一状态码


/**
 * @brief 按键驱动类
 *
 * @note 纯软件轮询消抖（无 ISR、无 FreeRTOS 依赖）；
 *       poll() 需在任务中周期性调用，消抖/长按时间 = 计数阈值 × 轮询周期。
 */
class BspKey
{
public:
  // ---------------- 公有接口 ----------------

  /** @brief 按键事件类型（一次"按下→松手"只会得到 PRESS+SHORT 或 PRESS+LONG） */
  enum class Event
  {
    NONE  = 0, ///< 无事件（常态）
    PRESS = 1, ///< 按下（消抖确认后触发一次）
    SHORT = 2, ///< 短按（按下后在长按阈值内松手 → 触发一次）
    LONG  = 3, ///< 长按（按住持续 long_press_cnt 次轮询 → 仅触发一次）
  };

  /**
   * @brief 按键配置结构体
   *
   * @note 消抖/长按时间取决于轮询周期:
   * @code
   *   实际消抖时间 = debounce_cnt × 轮询周期
   *   实际长按时间 = long_press_cnt × 轮询周期
   * @endcode
   * 例: 50ms 轮询 + debounce_cnt=2 → 100ms 消抖
   *
   * @note debounce_cnt / long_press_cnt 均必须 >= 1，否则 init() 返回 BAD_ARG。
   */
  struct Config
  {
    /**
     * @brief 按序构造配置（参数顺序 = 字段顺序）
     */
    Config(GPIO_TypeDef *port = nullptr, uint16_t pin = 0U, bool active_low = true, uint8_t debounce_cnt = 3U, uint16_t long_press_cnt = 200U) : port(port), pin(pin), active_low(active_low), debounce_cnt(debounce_cnt), long_press_cnt(long_press_cnt)
    {
    }

    GPIO_TypeDef *port;           ///< GPIO 端口 (GPIOA / GPIOB / ...)
    uint16_t      pin;            ///< 引脚掩码 (GPIO_PIN_x)
    bool          active_low;     ///< 有效电平: true=低有效(上拉), false=高有效(下拉)
    uint8_t       debounce_cnt;   ///< 消抖计数 (连续相同电平次数)
    uint16_t      long_press_cnt; ///< 长按计数阈值 (按住持续次数)
  };

  /** @brief 默认构造（未绑定引脚，须再调 init()） */
  BspKey() = default;

  /**
   * @brief 绑定引脚并初始化消抖状态机
   * @param cfg 引脚 + 消抖/长按参数（可匿名按序传入）
   *
   * @note 硬件引脚方向/上下拉由 CubeMX 的 MX_GPIO_Init() 配置，
   *       本函数仅保存参数并读取初始电平作为稳定状态。
   *
   * @return Status OK=绑定成功；
   *         BAD_ARG=端口/引脚非法，或 debounce_cnt / long_press_cnt 为 0
   */
  Status init(const Config &cfg);

  /**
   * @brief 轮询消抖（每周期调用一次，须在单任务中调用）
   *
   * 消抖算法:
   *   1. 读取当前 GPIO 电平
   *   2. 若与上次稳定状态相同 → 清零抖动计数
   *   3. 若不同 → 累加计数，达到 debounce_cnt 则确认状态变化
   *   4. 确认按下时，以已累计的消抖次数作为保持计数起点
   *      （即长按从"电平跳变那一刻"起算），此后每次轮询累加，
   *      达到 long_press_cnt 触发 LONG（每次按下仅一次）
   *
   * @note 未 init()（_port == nullptr）时安全返回 NONE，不会崩溃。
   *
   * @return 本次触发的事件:
   *         - NONE:  无事件
   *         - PRESS: 刚按下
   *         - SHORT: 短按（已松手，且本次按下未触发过 LONG）
   *         - LONG:  长按触发
   */
  Event poll();

  // ----------------
private:
  // ---------------- 私有实现 ----------------

  GPIO_TypeDef *_port           = nullptr; ///< GPIO 端口指针
  uint16_t      _pin            = 0U;      ///< 引脚掩码
  bool          _active_low     = true;    ///< 有效电平极性
  uint8_t       _debounce_cnt   = 3U;      ///< 消抖确认次数
  uint16_t      _long_press_cnt = 200U;    ///< 长按计数阈值

  bool     _last_stable = false; ///< 上一次确认的稳定状态 (true=按下)
  uint8_t  _cnt         = 0U;    ///< 当前连续不一致计数
  uint16_t _hold_cnt    = 0U;    ///< 按下保持计数
  bool     _long_fired  = false; ///< 本次按下周期内长按是否已触发

  // ----------------
};

#endif // __BSP_KEY_HPP__
