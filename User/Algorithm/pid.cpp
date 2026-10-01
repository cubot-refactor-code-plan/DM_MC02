#include "pid.hpp"
#include <math.h>

#include "arm_math.h" // IWYU pragma: keep

PID::PID(const Config& cfg) : _config(cfg)
{
  // 隐藏逻辑显式化：i_max 未设置（为0）时跟随 out_max
  if (_config.i_max == 0.0f)
  {
    _config.i_max = _config.out_max;
  }

  // 微分模式沿用配置值
  _diff_mode = _config.diff_mode;
}

void PID::calc_input(float target, float feedback)
{
  _input.last_target  = _input.target;
  _input.last_error   = _input.error;
  _input.target       = target;
  _input.feedback     = feedback;
  _input.error        = _input.target - _input.feedback;
  _input.delta_target = _input.target - _input.last_target;
  _input.delta_error  = _input.error - _input.last_error;
}

void PID::_calc_d_term()
{
  switch (_diff_mode)
  {
    case DiffMode::DIFF_TARGET:
      _term.d_term = _config.kd * _input.delta_target; // 微分先行
      break;
    case DiffMode::DIFF_ERROR:
      _term.d_term = _config.kd * _input.delta_error; // 常规微分
      break;
    case DiffMode::DISABLE_DIFF:
    default:
      _term.d_term = 0.0f;
      break;
  }
}

float PID::_apply_limits_and_output()
{
  // 对称限幅辅助 lambda：limit 为 0 表示不限制
  auto clamp_symmetric = [](float value, float limit)
  {
    if (limit == 0.0f)
    {
      return value;
    }
    if (value > limit)
    {
      return limit;
    }
    if (value < -limit)
    {
      return -limit;
    }
    return value;
  };

  _term.p_term = _config.kp * _input.error;

  // 积分分离：误差小于阈值（或阈值未设置）时累加，否则清零（注意：是清零而非冻结）
  _term.i_term = (fabsf(_input.error) < _config.i_sep || _config.i_sep == 0.0f)
                   ? _term.i_term + _config.ki * _input.error
                   : 0.0f;

  // 积分项累加后立即钳位，防止积分无限累积（即使后续总输出有限幅）
  _term.i_term = clamp_symmetric(_term.i_term, _config.i_max);

  _calc_d_term();

  // 各项独立限幅
  _term.p_term = clamp_symmetric(_term.p_term, _config.p_max);
  _term.d_term = clamp_symmetric(_term.d_term, _config.d_max);
  _term.f_term = clamp_symmetric(_term.f_term, _config.f_max);

  // 总输出限幅
  _output = _term.p_term + _term.i_term + _term.d_term + _term.f_term;
  _output = clamp_symmetric(_output, _config.out_max);

  // 前馈项每周期清零，需要每周期重新喂
  _term.f_term = 0.0f;

  return _output;
}

float PID::feed_forward(float feedforward)
{
  _term.f_term += feedforward;
  return _term.f_term;
}

float PID::calc(float target, float feedback)
{
  calc_input(target, feedback);
  return _apply_limits_and_output();
}

float PID::calc(float target, float feedback, float df_dt)
{
  calc_input(target, feedback);

  // 外部导入的目标值一阶导数直接作为微分输入
  if (_diff_mode == DiffMode::DIFF_TARGET)
  {
    _input.delta_target = df_dt;
  }
  else if (_diff_mode == DiffMode::DIFF_ERROR)
  {
    _input.delta_error = df_dt;
  }

  return _apply_limits_and_output();
}

void PID::print()
{
  // 暂未实现
  return;
}
