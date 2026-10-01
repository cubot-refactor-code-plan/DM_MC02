/**
 * @file bsp_buzzer.hpp
 * @author Rh
 * @brief 蜂鸣器驱动 —— PWM 无源蜂鸣器（默认 TIM12 CH2 PB15）
 * @version 0.3
 * @date 2026-09-10
 *
 * @copyright Copyright (c) 2026
 *
 * @details 使用定时器 PWM 驱动无源蜂鸣器，定时器/通道/基频/提示音参数
 *          全部通过 Config 传入（默认 TIM12 CH2）。
 *          TIM12: PSC=23, ARR 动态调整以改变频率。
 *          计数基频 base_clk = 定时器主频 / (PSC + 1)，TIM12 ≈ 6 MHz。
 *
 * @note 没有 init()：构造时传入 Config，默认值即本板参数（TIM12 CH2 / PB15）
 *
 * @note beep 使用示例（内部用 vTaskDelay 阻塞等待，必须在任务上下文调用；传 0 表示用 Config 里的默认值）：
 *       
 *       bsp_buzzer.beep(2000, 100);      // 2 kHz 响 100 ms 后自动关闭
 *       bsp_buzzer.beep(2000, 100, 30);  // 同上，音量（占空比）30%
 *       bsp_buzzer.beep(2000, 0);        // 只给频率：时长用 Config::short_ms（默认 80 ms）
 *       bsp_buzzer.beep(0, 500);         // 只给时长：频率用 Config::default_freq（默认 3 kHz）
 *       bsp_buzzer.beep(0, 0);           // 全默认：3 kHz 响 80 ms
 *       bsp_buzzer.beep(20, 1000);       // 低频长鸣 1 s（20 Hz 低于 Config::freq_min，被限幅到 50 Hz）
 */

#ifndef __BSP_BUZZER_HPP__
#define __BSP_BUZZER_HPP__

#include <stdint.h>

#include "tim.h" // IWYU pragma: keep（TIM_HandleTypeDef）


/**
 * @brief 蜂鸣器驱动类
 *
 * @note 对外只有一个发声接口 beep()（阻塞式），起振/停振都在内部完成。
 *       不持有任何 RTOS 资源（无队列、无定时器），纯硬件 PWM 控制。
 */
class BspBuzzer
{
public:
  // ---------------- 配置 ----------------

  /**
   * @brief 蜂鸣器配置结构体（全部带默认值：TIM12 CH2 PB15）
   *
   * @note 换用其他定时器时只需改 htim/channel/base_clk 三项。
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
    uint32_t           short_ms;     ///< 默认鸣叫时长 (ms)，beep 的 duration_ms=0 时使用
  };

  // ----------------
  // ---------------- 公共接口 ----------------

  /** @brief 默认构造：全部使用 Config 的默认值（TIM12 CH2 / PB15，基频 6 MHz） */
  BspBuzzer()  = default;
  /** @brief 默认析构 */
  ~BspBuzzer() = default;

  /**
   * @brief 用配置结构体构造
   * @param cfg 定时器/通道/音长等配置
   */
  BspBuzzer(const Config &cfg);

  /**
   * @brief 鸣叫指定时长后自动关闭（任务级阻塞）
   *
   * @note 唯一的对外发声入口：等价于 _tone + 延时 + _off，只把延时做成阻塞的。
   *
   * @param freq_hz     频率 (Hz)，0 = 使用 Config::default_freq
   * @param duration_ms 时长 (ms)，0 = 使用 Config::short_ms
   * @param volume_pct  音量/占空比 (%)，默认 50
   */
  void beep(uint32_t freq_hz, uint32_t duration_ms, uint32_t volume_pct = 50);

  // ----------------
private:
  // ---------------- 私有方法 ----------------

  /**
   * @brief 以指定频率和音量启动 PWM（非阻塞，仅起振）
   * @param freq_hz    频率 (Hz)，自动限幅到 Config::freq_min ~ freq_max
   * @param volume_pct 占空比 (%)，0~100
   * @note 不对外暴露：请用 beep()，时长与关闭由调用方负责容易忘关。
   */
  void _tone(uint32_t freq_hz, uint32_t volume_pct = 50);

  /** @brief 停止 PWM 输出 */
  void _off();

  // ----------------
  // ---------------- 成员变量 ----------------

  Config _config; ///< 蜂鸣器配置（定时器/通道/基频/音长参数）

  // ----------------
};

#endif // __BSP_BUZZER_HPP__
