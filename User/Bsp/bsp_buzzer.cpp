#include "bsp_buzzer.hpp"

#include "FreeRTOS.h" // IWYU pragma: keep (vTaskDelay)
#include "task.h"     // IWYU pragma: keep (pdMS_TO_TICKS)


// ---------------- 构造 ----------------


/** @brief 只保存配置，硬件操作在 tone()/off() 里做 */
BspBuzzer::BspBuzzer(const Config &cfg) : _config(cfg)
{
}


// ----------------
// ---------------- 公共接口 ----------------


/**
 * @brief 鸣叫指定时长后自动关闭（阻塞）
 *
 * @param freq_hz     频率 (Hz)，0 = 用 Config::default_freq
 * @param duration_ms 时长 (ms)，0 = 用 Config::short_ms
 * @param volume_pct  占空比 (%)
 *
 * @note 唯一的阻塞点是 vTaskDelay()，必须在任务上下文调用。
 */
void BspBuzzer::beep(uint32_t freq_hz, uint32_t duration_ms, uint32_t volume_pct)
{
  // 传 0 时回落到配置默认值
  if (freq_hz == 0)
    freq_hz = _config.default_freq;
  if (duration_ms == 0)
    duration_ms = _config.short_ms;

  tone(freq_hz, volume_pct);
  vTaskDelay(pdMS_TO_TICKS(duration_ms));
  off();
}


// ----------------
// ---------------- 私有方法 ----------------


/**
 * @brief 起振：设置 ARR/CCR 并启动 PWM（非阻塞，不负责关闭）
 *
 * @param freq_hz    频率 (Hz)，自动限幅到 Config::freq_min ~ freq_max
 * @param volume_pct 占空比 (%)，上限 100
 *
 * @note 频率由 ARR 决定：freq = base_clk / (ARR + 1)。
 */
void BspBuzzer::tone(uint32_t freq_hz, uint32_t volume_pct)
{
  // 频率限幅
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
    arr = 10; // 避免 ARR 为 0 或过小，使输出频率超出可听范围
  if (arr > 65535)
    arr = 65535; // 16 位 ARR 寄存器上限

  // 设置 ARR 和 CCR 并启动 PWM
  __HAL_TIM_SET_AUTORELOAD(_config.htim, arr);
  __HAL_TIM_SET_COMPARE(_config.htim, _config.channel, arr * volume_pct / 100);
  HAL_TIM_PWM_Start(_config.htim, _config.channel);
}

/** @brief 停止 PWM 输出 */
void BspBuzzer::off()
{
  HAL_TIM_PWM_Stop(_config.htim, _config.channel);
}

// ----------------
