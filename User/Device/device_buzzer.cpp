#include "FreeRTOS.h" // IWYU pragma: keep (vTaskDelay)
#include "device_buzzer.hpp"
#include "task.h" // IWYU pragma: keep (pdMS_TO_TICKS)


// ---------------- 构造 ----------------


/** @brief 用默认配置绑定 PWM 通道 */
DeviceBuzzer::DeviceBuzzer(BspPwm &pwm) : DeviceBuzzer(pwm, Config{})
{
}

/** @brief 只保存引用与配置，不碰硬件（PWM 由 BSP 层 init()） */
DeviceBuzzer::DeviceBuzzer(BspPwm &pwm, const Config &cfg) : _pwm(pwm), _config(cfg)
{
}


// ----------------
// ---------------- 公共接口 ----------------


/** @brief 起振：限幅频率后设频率 + 设音量（占空比） */
void DeviceBuzzer::tone(uint32_t freq_hz, float volume_pct)
{
  // 传 0 时回落到配置默认值
  if (freq_hz == 0U)
    freq_hz = _config.default_freq;

  // 频率限幅到可听/器件允许范围
  if (freq_hz < _config.freq_min)
    freq_hz = _config.freq_min;
  if (freq_hz > _config.freq_max)
    freq_hz = _config.freq_max;

  // 音量（占空比）传 ≤0 时用默认值
  if (!(volume_pct > 0.0f))
    volume_pct = _config.volume_pct;

  _pwm.set_freq(freq_hz);
  _pwm.set_duty(volume_pct);
}

/** @brief 停振（占空比 0，无源蜂鸣器不再发声） */
void DeviceBuzzer::off()
{
  _pwm.off();
}

/** @brief 鸣叫指定时长：tone + 阻塞延时 + off */
void DeviceBuzzer::beep(uint32_t freq_hz, uint32_t duration_ms, float volume_pct)
{
  // 传 0 时回落到配置默认值
  if (duration_ms == 0U)
    duration_ms = _config.short_ms;

  tone(freq_hz, volume_pct);
  vTaskDelay(pdMS_TO_TICKS(duration_ms));
  off();
}

// ----------------
