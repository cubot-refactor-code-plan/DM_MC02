/**
 * @file device_buzzer.hpp
 * @author Rh
 * @brief 蜂鸣器设备 —— 在 BspPwm 之上实现「音调 / 音量 / 鸣叫时长」语义
 * @version 0.1
 * @date 2026-10-02
 *
 * @copyright Copyright (c) 2026
 *
 * @details 硬件只是一路 PWM 通道（BspPwm），"发声"属于器件语义，因此放在设备层：
 *          频率限幅、默认音长、默认音量（占空比）、阻塞式鸣叫都在这里。
 *          换蜂鸣器或换 PWM 通道只需改实例化处传入的 BspPwm 引用，本类不用改。
 *
 * @note 用法（实例见 device_cfg.cpp，绑定 bsp_pwm_buzzer）：
 *
 *       buzzer.beep(3000, 40);        // 3 kHz 响 40 ms 后自动关闭（阻塞）
 *       buzzer.beep(2000, 100, 30.0f);// 2 kHz 响 100 ms，音量 30%
 *       buzzer.beep(0, 0);            // 全默认：default_freq 响 short_ms
 *
 *       buzzer.tone(2000);            // 只起振（不自动关），需自己调 off()
 *       buzzer.off();
 *
 * @warning beep() 内部用 vTaskDelay() 阻塞等待，只能在任务上下文调用。
 */

#ifndef __DEVICE_BUZZER_HPP__
#define __DEVICE_BUZZER_HPP__

#include "bsp_pwm.hpp" // 被驱动的 PWM 通道

#include <stdint.h>

/**
 * @brief 蜂鸣器设备类
 *
 * @note 不拥有 PWM 通道，只持有引用（通道由 BSP 层实例化并初始化）。
 *       不创建任何 RTOS 资源。
 */
class DeviceBuzzer
{
public:
  // ---------------- 配置 ----------------

  /**
   * @brief 蜂鸣器配置（音域与默认值；全部可手动传入）
   */
  struct Config
  {
    /**
     * @brief 按序构造配置（参数顺序 = 字段顺序，可匿名传入）
     */
    Config(uint32_t default_freq = 3000UL, uint32_t freq_min = 50UL, uint32_t freq_max = 20000UL, uint32_t short_ms = 80UL, float volume_pct = 50.0f)

      : default_freq(default_freq),
        freq_min(freq_min),
        freq_max(freq_max),
        short_ms(short_ms),
        volume_pct(volume_pct)
    {
    }

    uint32_t default_freq; ///< 默认频率 (Hz)：tone()/beep() 传 0 时使用
    uint32_t freq_min;     ///< 频率下限 (Hz)
    uint32_t freq_max;     ///< 频率上限 (Hz)
    uint32_t short_ms;     ///< 默认鸣叫时长 (ms)：beep() 传 0 时使用
    float    volume_pct;   ///< 默认音量（占空比 %）：tone()/beep() 传 ≤0 时使用
  };

  // ----------------
  // ---------------- 公共接口 ----------------

  /** @brief 必须绑定一路 PWM 通道，故禁用默认构造 */
  DeviceBuzzer() = delete;

  /**
   * @brief 绑定 PWM 通道，使用默认配置（本板参数）
   * @param pwm 已由 BSP 层初始化好的 PWM 通道引用
   */
  explicit DeviceBuzzer(BspPwm &pwm);

  /**
   * @brief 绑定 PWM 通道并传入配置
   * @param pwm 已由 BSP 层初始化好的 PWM 通道引用
   * @param cfg 音域/默认值配置
   */
  DeviceBuzzer(BspPwm &pwm, const Config &cfg);

  /** @brief 默认析构 */
  ~DeviceBuzzer() = default;

  /**
   * @brief 起振（非阻塞，不负责关闭）
   *
   * @param freq_hz    频率 (Hz)，0 = 用 Config::default_freq，自动限幅到 freq_min ~ freq_max
   * @param volume_pct 音量（占空比 %），≤0 = 用 Config::volume_pct
   */
  void tone(uint32_t freq_hz = 0U, float volume_pct = 0.0f);

  /** @brief 停振（占空比 0，PWM 仍运行） */
  void off();

  /**
   * @brief 鸣叫指定时长后自动关闭（阻塞，任务上下文）
   *
   * @param freq_hz     频率 (Hz)，0 = Config::default_freq
   * @param duration_ms 时长 (ms)，0 = Config::short_ms
   * @param volume_pct  音量（占空比 %），≤0 = Config::volume_pct
   */
  void beep(uint32_t freq_hz = 0U, uint32_t duration_ms = 0U, float volume_pct = 0.0f);

  // ----------------
private:
  // ---------------- 成员变量 ----------------

  BspPwm &_pwm;    ///< 被驱动的 PWM 通道（不拥有）
  Config  _config; ///< 音域与默认值

  // ----------------
};

#endif // __DEVICE_BUZZER_HPP__
