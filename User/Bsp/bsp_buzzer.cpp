#include "bsp_buzzer.hpp"

#include "FreeRTOS.h" // IWYU pragma: keep (vTaskDelay)
#include "task.h"     // IWYU pragma: keep (pdMS_TO_TICKS)


// ---------------- 构造 ----------------


BspBuzzer::BspBuzzer(const Config &cfg) : _config(cfg)
{
}


// ----------------
// ---------------- 公共接口 ----------------


/**
 * @brief 鸣叫指定时长后自动关闭（任务级阻塞）
 *
 * @note freq_hz / duration_ms 传 0 时使用 Config 中的默认值，
 *       volume_pct 为音量（占空比），默认 50%。
 *
 * @param freq_hz     频率 (Hz)，0 = 使用 Config::default_freq
 * @param duration_ms 时长 (ms)，0 = 使用 Config::short_ms
 * @param volume_pct  音量/占空比 (%)，默认 50
 */
void BspBuzzer::beep(uint32_t freq_hz, uint32_t duration_ms, uint32_t volume_pct)
{
  // 声音选项：频率/时长缺省时回落到配置默认值
  if (freq_hz == 0)
    freq_hz = _config.default_freq;
  if (duration_ms == 0)
    duration_ms = _config.short_ms;

  tone(freq_hz, volume_pct);
  vTaskDelay(pdMS_TO_TICKS(duration_ms));
  off();
}

/**
 * @brief 停止 PWM
 */
void BspBuzzer::off()
{
  HAL_TIM_PWM_Stop(_config.htim, _config.channel);
}


// ----------------
// ---------------- 私有方法 ----------------


/**
 * @brief 以指定频率和音量启动 PWM
 *
 * @note 频率公式：freq = base_clk / (ARR + 1)
 *       → ARR = base_clk / freq - 1
 *
 * @param freq_hz    频率 (Hz)，自动限幅到 Config::freq_min ~ freq_max
 * @param volume_pct 占空比 (%)
 */
void BspBuzzer::tone(uint32_t freq_hz, uint32_t volume_pct)
{
  // 频率限制
  if (freq_hz < _config.freq_min)
    freq_hz = _config.freq_min;
  if (freq_hz > _config.freq_max)
    freq_hz = _config.freq_max;

  // 占空比 0-100 限幅
  if (volume_pct > 100)
    volume_pct = 100;

  // 计算 ARR 并设置 CCR
  uint32_t arr = (_config.base_clk / freq_hz) - 1;
  if (arr < 10)
    arr = 10; // 防 0/过小（避免产生超出可听范围的怪声）
  if (arr > 65535)
    arr = 65535; // 16 位 ARR 寄存器硬件上限

  // 设置 ARR 和 CCR 并启动 PWM
  __HAL_TIM_SET_AUTORELOAD(_config.htim, arr);
  __HAL_TIM_SET_COMPARE(_config.htim, _config.channel, arr * volume_pct / 100);
  HAL_TIM_PWM_Start(_config.htim, _config.channel);
}

// ----------------
