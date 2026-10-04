#include "pid.hpp"

#include <math.h>

// @Choose-B 这俩是 hyw，最好别用 cpp 的库，嵌入式遭不住
// #include <functional>
// #include <numeric>

namespace
{
/**
 * @brief 对称限幅
 *
 * @param limit 限幅值，写 0 表示不开启该项限制
 */
float clamp_sym(float value, float limit)
{
  if (limit <= 0.0f)
    return value;
  if (value > limit)
    return limit;
  if (value < -limit)
    return -limit;
  return value;
}
} // namespace


// ----------------
// ---------------- 系数、限幅与计算值 ----------------

PidParam::PidParam() : kp(0), ki(0), kd(0)
{
}

PidParam::PidParam(float kp, float ki, float kd) : kp(kp), ki(ki), kd(kd)
{
}

PidLimitation::PidLimitation() : out_max(0), p_max(0), i_max(0), d_max(0), f_max(0), i_separation(0)
{
}

PidLimitation::PidLimitation(float out_max, float p_max, float i_max, float d_max, float f_max, float i_separation) : out_max(out_max), p_max(p_max), i_max(i_max), d_max(d_max), f_max(f_max), i_separation(i_separation)
{
  // 积分限幅写 0 表示跟随总输出限幅
  this->i_max = i_max == 0 ? out_max : i_max;
}

PidTerm::PidTerm() : p_term(0), i_term(0), d_term(0), f_term(0)
{
}


// ----------------
// ---------------- 构造 ----------------

/** @brief 全参数构造：系数 + 限幅；kd 为 0 时没必要算微分项，直接关掉 */
Pid::Pid(PidParam param, PidLimitation limitation) : _param(param), _lim(limitation), _term(), _diff_calc_mode(param.kd == 0.0f ? PidDiffCalcMode::DISABLE : PidDiffCalcMode::TARGET)
{
}


// ----------------
// ---------------- 公有接口 ----------------

void Pid::switch_mode_diff_calc(PidDiffCalcMode mode)
{
  _diff_calc_mode = mode;
}

float Pid::feed_forward(float feedforward)
{
  _term.f_term += feedforward;
  return _term.f_term;
}

float Pid::calc(float target, float feedback)
{
  calc_input(target, feedback);

  return _calc_output();
}

float Pid::calc(float target, float feedback, float df_dt)
{
  calc_input(target, feedback);

  // 外部直接给的一阶导数优先：比再差分一次噪声小（位移环填速度用）
  if (_diff_calc_mode == PidDiffCalcMode::TARGET)
    input.delta_target = df_dt;
  else if (_diff_calc_mode == PidDiffCalcMode::ERROR)
    input.delta_error = df_dt;

  return _calc_output();
}

void Pid::print()
{
  // 暂未实现
  return;
}


// ----------------
// ---------------- 私有实现 ----------------

void Pid::calc_input(float target, float feedback)
{
  input.last_target = input.target;
  input.last_error  = input.error;

  input.target   = target;
  input.feedback = feedback;
  input.error    = target - feedback;

  // 一阶差分：默认给微分项用，3 参数 calc() 会用外部传入的导数覆盖
  input.delta_target = input.target - input.last_target;
  input.delta_error  = input.error - input.last_error;
}

/** @brief 算一次输出：比例 + 积分 + 微分 + 前馈，逐项限幅后再总限幅 */
float Pid::_calc_output()
{
  // 比例项
  _term.p_term = _param.kp * input.error;

  // 积分项：积分分离阈值大于 0 时，误差超出阈值就不积分（清零抑制超调）
  const bool integral_enabled = _lim.i_separation <= 0.0f || fabsf(input.error) < _lim.i_separation;
  _term.i_term = integral_enabled ? _term.i_term + _param.ki * input.error : 0.0f;

  // 微分项
  switch (_diff_calc_mode)
  {
    case PidDiffCalcMode::TARGET:
    {
      _term.d_term = _param.kd * input.delta_target; // 微分先行
      break;
    }
    case PidDiffCalcMode::ERROR:
    {
      _term.d_term = _param.kd * input.delta_error; // 常规微分
      break;
    }
    case PidDiffCalcMode::DISABLE:
    default:
    {
      _term.d_term = 0.0f;
      break;
    }
  }

  // 逐项限幅（写 0 表示不限制）
  _term.p_term = clamp_sym(_term.p_term, _lim.p_max);
  _term.i_term = clamp_sym(_term.i_term, _lim.i_max);
  _term.d_term = clamp_sym(_term.d_term, _lim.d_max);
  _term.f_term = clamp_sym(_term.f_term, _lim.f_max);

  // 总输出限幅
  output = clamp_sym(_term.p_term + _term.i_term + _term.d_term + _term.f_term, _lim.out_max);

  // 前馈项取用后清零，等下一次 feed_forward() 重新累加
  _term.f_term = 0.0f;

  return output;
}
