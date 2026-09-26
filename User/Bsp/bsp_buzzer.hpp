/**
 * @file bsp_buzzer.hpp
 * @author Rh
 * @brief 蜂鸣器驱动 —— PWM 无源蜂鸣器（默认 TIM12 CH2 PB15）
 * @version 0.4
 * @date 2026-09-26
 *
 * @copyright Copyright (c) 2026
 *
 * @details 定时器/通道/基频/音长参数全部通过 Config 传入（默认 TIM12 CH2 PB15）。
 *          TIM12: PSC=23，通过动态改写 ARR 改变发声频率；
 *          计数基频 base_clk = 定时器主频 / (PSC + 1)，TIM12 ≈ 6 MHz。
 *
 * @note 无 init()，构造时传入 Config 即可；默认构造 = 默认配置。
 *
 * @note 使用示例：
 *
 *       bsp_buzzer.beep(3000, 500);     // 3kHz 响 500ms 后自动关闭
 *       bsp_buzzer.beep(3000, 500, 80); // 3kHz 响 500ms，80% 音量后自动关闭
 *       bsp_buzzer.beep();              // 全默认（3kHz + 短鸣 + 50% 音量）
 *
 *       bsp_buzzer.off();               // 立即关闭
 */

#ifndef __BSP_BUZZER_HPP__
#define __BSP_BUZZER_HPP__

#include "tim.h" // IWYU pragma: keep

#include <stdint.h>


/**
 * @brief 蜂鸣器驱动类
 *
 * @note 只封装定时器 PWM 的「设频 → 起振 → 延时 → 停振」，
 *       不占用任何 RTOS 资源（无队列、无软件定时器）。
 */
class BspBuzzer
{
public:
  // ---------------- 公有接口 ----------------

  /**
   * @brief 蜂鸣器配置（全部带默认值：TIM12 CH2 PB15，可匿名按序传入）
   *
   * @note 换用其他定时器时只需改 htim / channel / base_clk 三项。
   */
  struct Config
  {
    /**
     * @brief 按序构造配置（参数顺序 = 字段顺序，可匿名传入）
     */
    Config(TIM_HandleTypeDef *htim = &htim12, uint32_t channel = TIM_CHANNEL_2, uint32_t base_clk = 6000000UL, uint32_t default_freq = 3000UL, uint32_t freq_min = 50, uint32_t freq_max = 20000, uint32_t short_ms = 80)
      : htim(htim),
        channel(channel),
        base_clk(base_clk),
        default_freq(default_freq),
        freq_min(freq_min),
        freq_max(freq_max),
        short_ms(short_ms)
    {
    }

    TIM_HandleTypeDef *htim;         ///< PWM 定时器句柄
    uint32_t           channel;      ///< PWM 通道
    uint32_t           base_clk;     ///< 计数基频 (Hz) = 定时器主频 / (PSC+1)
    uint32_t           default_freq; ///< 默认鸣叫频率 (Hz)
    uint32_t           freq_min;     ///< 频率下限 (Hz)
    uint32_t           freq_max;     ///< 频率上限 (Hz)
    uint32_t           short_ms;     ///< 短鸣时长 (ms)
  };

  ///< 默认构造：使用 Config 的全部默认值（TIM12 CH2 PB15）
  BspBuzzer() = default;

  ///< 用配置结构体构造（定时器/通道/基频/音长等）
  BspBuzzer(const Config &cfg);

  /**
   * @brief 鸣叫指定时长后自动关闭（阻塞）
   *
   * @note 三个参数均有默认值，可只传部分参数：
   *         beep();                  // 默认频率 + 短鸣时长 + 50% 音量
   *         beep(3000);              // 3kHz + 短鸣时长 + 50% 音量
   *         beep(3000, 500);         // 3kHz + 500ms + 50% 音量
   *         beep(3000, 500, 100);    // 3kHz + 500ms + 100% 音量
   *
   * @param freq_hz     频率 (Hz)，传 0 表示使用 Config::default_freq
   * @param duration_ms 时长 (ms)，传 0 表示使用 Config::short_ms
   * @param volume_pct  音量/占空比 (%)，0~100，默认 50
   */
  void beep(uint32_t freq_hz = 0, uint32_t duration_ms = 0, uint32_t volume_pct = 50);

  ///< 停止 PWM
  void off();

  // ----------------

private:
  // ---------------- 私有实现 ----------------

  // 成员变量

  Config _config; ///< 蜂鸣器配置（定时器/通道/基频/音长参数）

  // 内部实现

  /**
   * @brief 以指定频率和音量启动 PWM
   * @param freq_hz    频率 (Hz)，自动限幅到 Config::freq_min ~ freq_max
   * @param volume_pct 占空比 (%)，0~100
   */
  void tone(uint32_t freq_hz, uint32_t volume_pct);

  // ----------------
};

#endif // __BSP_BUZZER_HPP__
