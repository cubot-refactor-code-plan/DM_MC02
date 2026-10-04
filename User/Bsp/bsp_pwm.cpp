#include "bsp_pwm.hpp"


// ---------------- 构造 ----------------


/** @brief 只保存配置，硬件操作在 init() 里做 */
BspPwm::BspPwm(const Config &cfg) : _config(cfg)
{
}


// ----------------
// ---------------- 公共接口 ----------------


/**
 * @brief 启动 PWM：PSC 不动（由 MX_TIMx_Init 配置），同步 ARR，先清零 CCR 再启动
 */
Status BspPwm::init()
{
  // 配置合法性：句柄/通道/时钟必须给出，PSC 是 16 位寄存器
  if (_config.htim == nullptr || _config.channel == 0U || _config.timer_clk_hz == 0U || _config.prescaler > 0xFFFFU)
    return Status::BAD_ARG;

  // 计数器时钟 = 定时器时钟 / (PSC + 1)
  _counter_clk_hz = _config.timer_clk_hz / (_config.prescaler + 1U);

  // 与 CubeMX 配置对齐一次 ARR，之后由 set_freq() 动态改
  _period = _config.period;
  __HAL_TIM_SET_AUTORELOAD(_config.htim, _period);

  // 关键：先把 CCR 清 0，保证启动瞬间是 0% 占空比（CubeMX 里 Pulse 可能非 0）
  _duty_pct = 0.0f;
  __HAL_TIM_SET_COMPARE(_config.htim, _config.channel, 0U);

  if (HAL_TIM_PWM_Start(_config.htim, _config.channel) != HAL_OK)
    return Status::IO_ERROR;

  _initialized = true;

  // 配置里若显式给了非 0 上电占空比，按它输出（默认仍是 0%）
  set_duty(_config.duty_pct);

  return Status::OK;
}

/** @brief 设置占空比（0~100 浮点，自动限幅） */
void BspPwm::set_duty(float duty_pct)
{
  if (!_initialized)
    return;

  _duty_pct = duty_pct;
  _apply_duty();
}

/** @brief 按脉宽设置占空比：占空比 = 脉宽 / 周期 */
void BspPwm::set_pulse_us(float width_us)
{
  if (!_initialized || _counter_clk_hz == 0U)
    return;

  // 周期时长 (us) = 计数个数 / 计数器时钟
  const float period_us = static_cast<float>(_period + 1U) * 1000000.0f / static_cast<float>(_counter_clk_hz);
  if (period_us <= 0.0f)
    return;

  set_duty(width_us * 100.0f / period_us);
}

/** @brief 动态改频率：ARR = 计数器时钟 / 频率 - 1，PSC 保持不变 */
void BspPwm::set_freq(uint32_t freq_hz)
{
  if (!_initialized || freq_hz == 0U || _counter_clk_hz == 0U)
    return;

  uint32_t arr = _counter_clk_hz / freq_hz;
  if (arr == 0U)
    arr = 1U; // 频率高于计数器时钟时取最小周期
  arr -= 1U;

  // 上限取决于计数器位宽：TIM2 为 32 位，其余为 16 位
  const uint32_t arr_max = IS_TIM_32B_COUNTER_INSTANCE(_config.htim->Instance) ? 0xFFFFFFFFUL : 0xFFFFUL;
  if (arr > arr_max)
    arr = arr_max;

  _period = arr;
  __HAL_TIM_SET_AUTORELOAD(_config.htim, arr);

  // 按新周期重算 CCR，保持占空比不变
  _apply_duty();
}

/** @brief 关断输出（CCR=0，引脚保持低电平） */
void BspPwm::off()
{
  set_duty(0.0f);
}


// ----------------
// ---------------- 私有方法 ----------------


/** @brief 占空比 → CCR（含限幅，并把实际生效值写回 _duty_pct） */
void BspPwm::_apply_duty()
{
  // 限幅到 0~100；顺手挡住 NaN（!(x > 0) 对 NaN 成立）
  float pct = _duty_pct;
  if (!(pct > 0.0f))
    pct = 0.0f;
  if (pct > 100.0f)
    pct = 100.0f;

  const uint32_t period_cnt = _period + 1U;
  uint32_t       ccr        = static_cast<uint32_t>(pct * static_cast<float>(period_cnt) / 100.0f + 0.5f);
  if (ccr > period_cnt)
    ccr = period_cnt;

  _duty_pct = pct;
  __HAL_TIM_SET_COMPARE(_config.htim, _config.channel, ccr);
}


// ----------------
// ---------------- 查询接口 ----------------


/** @brief 当前生效占空比 (%) */
float BspPwm::duty_pct() const
{
  return _duty_pct;
}

/** @brief 计数器时钟 (Hz) */
uint32_t BspPwm::counter_clk_hz() const
{
  return _counter_clk_hz;
}

/** @brief 当前周期计数个数（ARR + 1） */
uint32_t BspPwm::period_cnt() const
{
  return _period + 1U;
}

/** @brief 当前频率 (Hz) */
uint32_t BspPwm::freq_hz() const
{
  if (_counter_clk_hz == 0U)
    return 0U;

  return _counter_clk_hz / (_period + 1U);
}

// ----------------
