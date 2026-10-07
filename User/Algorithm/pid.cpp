#include "pid.hpp"

#include <math.h>


// ---------------- 构造 ----------------

Pid::Pid(Config config) : _cfg(config), _error(0.0f), _last_error(0.0f), _last_target(0.0f), _delta_target(0.0f), _delta_error(0.0f), _i_term(0.0f), _f_term(0.0f)
{
  // kd 为 0 时微分项恒为 0，算它没有意义：直接关掉。
  // 其余情况沿用配置里的方式（默认 TARGET = 微分先行）
  if (_cfg.gain.kd == 0.0f)
    _cfg.diff_mode = PidDiffCalcMode::DISABLE;
}


// ----------------
// ---------------- 公有接口 ----------------

float Pid::calc(float target, float feedback)
{
  _calc_input(target, feedback);

  return _calc_output();
}

float Pid::calc(float target, float feedback, float df_dt)
{
  _calc_input(target, feedback);

  // 外部直接给的一阶导数优先：比内部再差分一次噪声小（如位移环直接填速度）。
  // 导数作用到哪个量由 diff_mode 决定，DISABLE 时不使用 df_dt
  if (_cfg.diff_mode == PidDiffCalcMode::TARGET)
    _delta_target = df_dt;
  else if (_cfg.diff_mode == PidDiffCalcMode::ERROR)
    _delta_error = df_dt;

  return _calc_output();
}

float Pid::feed_forward(float feedforward)
{
  _f_term += feedforward;
  return _f_term;
}


// ----------------
// ---------------- 私有实现 ----------------

void Pid::_calc_input(float target, float feedback)
{
  const float error = target - feedback;

  // 一阶差分：默认给微分项用；三参数 calc 会用外部传入的导数覆盖这两个量
  _delta_target = target - _last_target;
  _delta_error  = error - _last_error;

  // 更新历史量，供下一拍差分使用
  _error       = error;
  _last_target = target;
  _last_error  = error;
}

float Pid::_calc_output()
{
  const Gain  &gain  = _cfg.gain;  // 局部别名：下面公式里少写一层 _cfg
  const Limit &limit = _cfg.limit;

  // 比例项
  float p_term = gain.kp * _error;

  // 积分项：积分分离阈值 > 0 且误差超出阈值时清零（抑制大误差下的超调），否则累加
  const bool integral_enabled = limit.i_separation <= 0.0f || fabsf(_error) < limit.i_separation;
  _i_term = integral_enabled ? _i_term + gain.ki * _error : 0.0f;

  // 微分项：按 diff_mode 取目标差分或误差差分；DISABLE 时保持 0
  float d_term = 0.0f;
  if (_cfg.diff_mode == PidDiffCalcMode::TARGET)
    d_term = gain.kd * _delta_target;
  else if (_cfg.diff_mode == PidDiffCalcMode::ERROR)
    d_term = gain.kd * _delta_error;

  // 逐项限幅：对应 Limit 的 p_max / i_max / d_max / f_max，填 0 表示不限制
  p_term  = _clamp_sym(p_term, limit.p_max);
  _i_term = _clamp_sym(_i_term, limit.i_max);
  d_term  = _clamp_sym(d_term, limit.d_max);
  _f_term = _clamp_sym(_f_term, limit.f_max);

  // 求和后总限幅，即为本拍输出
  const float output = _clamp_sym(p_term + _i_term + d_term + _f_term, limit.out_max);

  // 前馈取用后清零：等下一拍 feed_forward() 重新累加（前馈不跨拍残留）
  _f_term = 0.0f;

  return output;
}

/** @brief 对称限幅；limit <= 0 表示不限制（与 Limit 的「0 = 不限制」约定一致） */
float Pid::_clamp_sym(float value, float limit)
{
  if (limit <= 0.0f)
    return value;
  if (value > limit)
    return limit;
  if (value < -limit)
    return -limit;
  return value;
}


// ----------------
