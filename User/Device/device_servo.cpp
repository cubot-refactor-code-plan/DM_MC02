#include "device_servo.hpp"

#include "FreeRTOS.h" // IWYU pragma: keep (configTICK_RATE_HZ)
#include "task.h"     // xTaskGetTickCount

#include <math.h> // fabsf / sqrtf


// ---------------- 构造与析构 ----------------


/**
 * @brief 只保存配置
 *
 * @note 不碰硬件：PWM 通道由 bsp_init() 启动（上电 0% 占空比 = 无脉冲），
 *       本类要写脉宽得等上层第一次 set_angle() / set_pulse_us()。
 */
DeviceServo::DeviceServo(const Config &cfg) : _cfg(cfg)
{
}


// ----------------
// ---------------- 生命周期 ----------------


/**
 * @brief 校验配置、复位运动状态
 *
 * @note 故障后果都写清楚了，改配置前先对一眼：
 *       - pulse_max 不大于 pulse_min —— 换算斜率非正，角度越大脉宽越小（反了）；
 *       - pulse_max 超过一个 PWM 周期 —— BspPwm 会把它限到 100% 占空比，引脚恒高，
 *         舵机收到的是一个没有下降沿的信号，行为未定义（常见于把 2000 µs 写成 20000 µs）；
 *       - range_deg 非正 —— 除法得不到行程比例。
 * @note 本函数不写脉宽，因此不会让舵机动起来；这正是"上电不输出"的落点。
 */
Status DeviceServo::init(void)
{
  // 配置合法性（顺序 = 字段重要性，报错只报第一个，够定位就行）
  if (_cfg.pwm == nullptr)
    return Status::BAD_ARG;
  if (!(_cfg.pulse_min_us > 0.0f)) // 顺带挡 NaN
    return Status::BAD_ARG;
  if (!(_cfg.pulse_max_us > _cfg.pulse_min_us))
    return Status::BAD_ARG;
  if (!(_cfg.range_deg > 0.0f))
    return Status::BAD_ARG;
  if ((_cfg.max_speed_dps < 0.0f) || (_cfg.max_accel_dps2 < 0.0f))
    return Status::BAD_ARG;

  // 脉宽必须能塞进一个 PWM 周期。counter_clk_hz() 为 0 说明 PWM 还没 init（正常流程里
  // bsp_init() 在前面，不会走到这），此时跳过检查而不是误判成 BAD_ARG。
  const uint32_t clk = _cfg.pwm->counter_clk_hz();
  if (clk > 0U)
  {
    const float period_us = static_cast<float>(_cfg.pwm->period_cnt()) * 1000000.0f / static_cast<float>(clk);
    if ((_cfg.pulse_max_us + _cfg.trim_us) > period_us)
      return Status::BAD_ARG;
  }

  // 复位运动状态：输出回到"未开始"，角度归 0（此刻并没有真的发给舵机）
  _target_deg  = 0.0f;
  _angle_deg   = 0.0f;
  _speed_dps   = 0.0f;
  _pulse_us    = 0.0f;
  _enabled     = false;
  _enabled_req = false; // 不置位 → 下次 set_angle() 才会打开输出
  _last_tick   = xTaskGetTickCount();
  _inited      = true;

  return Status::OK;
}

/**
 * @brief 推进实际角度、执行开关输出（**唯一的 PWM 写入者**，固定单任务周期调用）
 *
 * @note 每拍的顺序：① 没有输出请求就关断 → ② 首次生效只登记、不积分 → ③ 不限速则一拍落位
 *       → ④ 否则梯形规划。第 ② 步是为了避免长时间未更新导致跳变
 * @note 制动那一步的判据：v² ≤ 2·a·s（s = 剩余角度），等价于 |v| ≤ sqrt(2·a·|s|)。
 *       它保证"现在开始匀减速，恰好在目标点停下"，是梯形轨迹的减速段起点。
 */
void DeviceServo::update(void)
{
  if (!_inited)
    return;

  const uint32_t now = xTaskGetTickCount();
  float          dt  = static_cast<float>(now - _last_tick) / static_cast<float>(configTICK_RATE_HZ);
  _last_tick = now;

  // ① 没有"要输出"的请求（上电初值 / off() 置回）：关断并保持关闭。
  //    只在 _enabled 还亮着时写一次寄存器，之后每拍只多一次判断
  if (!_enabled_req)
  {
    if (_enabled)
    {
      _cfg.pwm->off();
      _enabled   = false;
      _speed_dps = 0.0f;
      _pulse_us  = 0.0f;
    }
    return;
  }

  // ② 首次生效：本拍只登记"开始"，不积分
  if (!_enabled)
  {
    _enabled = true;
    return;
  }

  if (dt <= 0.0f)
    return; // 同一个 tick 内重复调用，没有可积分的时间
  if (dt > MAX_DT_S)
    dt = MAX_DT_S;

  // ③ 不限速：一拍落到目标，与 BspPwm 的瞬时语义一致（仍由本函数独占写入）
  if (_cfg.max_speed_dps <= 0.0f)
  {
    _angle_deg = _target_deg;
    _speed_dps = 0.0f;
    _write_pulse();
    return;
  }

  const float v_max = _cfg.max_speed_dps;
  const float a     = _cfg.max_accel_dps2;
  const float rem   = _target_deg - _angle_deg;

  // 目标速度只由"剩余距离的方向"决定：还没到就朝目标满速，到了就 0
  float v_cmd = 0.0f;
  if (rem > 0.0f)
    v_cmd = v_max;
  else if (rem < 0.0f)
    v_cmd = -v_max;

  if (a > 0.0f)
  {
    // 第一步：速度变化量受加速度限制（a·dt 是这一拍能动的最多速度）
    float       delta = v_cmd - _speed_dps;
    const float dv    = a * dt;
    if (delta > dv)
      delta = dv;
    if (delta < -dv)
      delta = -dv;
    _speed_dps += delta;

    // 第二步：受制动距离限制，避免冲过头再来回修
    const float v_stop = sqrtf(2.0f * a * fabsf(rem));
    if (_speed_dps > v_stop)
      _speed_dps = v_stop;
    if (_speed_dps < -v_stop)
      _speed_dps = -v_stop;
  }
  else
  {
    _speed_dps = v_cmd; // 不限制加速度 = 速度瞬变到限速值（矩形速度曲线）
  }

  const float step = _speed_dps * dt;
  if (fabsf(step) >= fabsf(rem))
  {
    // 这一拍就能走到终点：直接落位并把速度清零（留个非零速度会让下一拍反着抖）
    _angle_deg = _target_deg;
    _speed_dps = 0.0f;
  }
  else
  {
    _angle_deg += step;
  }

  _write_pulse();
}


// ----------------
// ---------------- 控制接口 ----------------


/**
 * @brief 设定目标角度（**可从任意任务调用**）
 *
 * @note 本函数只写两个跨任务的 32 位变量（目标角度 + 要输出），别的一律不碰：
 *       落位、限速与脉宽写入全部由 update() 完成，所以跨任务调用不需要加锁。
 * @note `_last_tick` 不再在这里刷新：此前刷新是为了避开首拍一段陈旧的 dt，
 *       现在由 update() 在"首次生效"那一拍自己登记（见 update() 第 ② 步）。
 */
void DeviceServo::set_angle(float deg)
{
  if (!_inited)
    return;

  _target_deg  = _clamp_angle(deg); // 跨任务写点 1：单条 32 位存储，天然原子
  _enabled_req = true;              // 跨任务写点 2
}

/**
 * @brief 直接写脉宽（绕过换算与限速；须与 update() 同任务）
 *
 * @note 用途是标定：手册给的 500 ~ 2500 µs 未必就是手上这只舵机的实际端点，
 *       用它一点点试、找到真正的两端，再把结果写进 Config。
 * @note 写完之后内部角度同步成等价值、限速状态清零，避免下次 update() 从旧角度猛追。
 * @note 它是唯一会绕过 update() 直接写 PWM 的接口，所以不能跨任务用（见 hpp 的线程模型）。
 */
void DeviceServo::set_pulse_us(float width_us)
{
  if (!_inited)
    return;

  const float lo = _cfg.pulse_min_us + _cfg.trim_us;
  const float hi = _cfg.pulse_max_us + _cfg.trim_us;
  if (!(width_us > lo)) // 顺带把 NaN 归到最小端
    width_us = lo;
  if (width_us > hi)
    width_us = hi;

  _angle_deg   = _deg_of_pulse(width_us);
  _target_deg  = _angle_deg; // 就地到位：别让 update() 再把它拉回旧目标
  _speed_dps   = 0.0f;
  _pulse_us    = width_us;
  _enabled     = true;
  _enabled_req = true; // 否则下一拍 update() 会把它当"关断请求"给关掉

  _cfg.pwm->set_pulse_us(width_us);
}

/**
 * @brief 请求停止输出（**可从任意任务调用**）：脉宽 0、引脚恒低，舵机因此失力
 *
 * @note 这里只清"要输出"请求，真正的关断由 update() 执行 —— 它是唯一的 PWM 写入者。
 *       代价是最多晚一个 update 周期生效（1 kHz 调用时 ≤1 ms），换来跨任务不用加锁。
 * @note 角度状态保留（便于重新接管）；下次 set_angle() 会让输出重新打开。
 * @note 单独把 off() 做成请求式而不是直接写寄存器，主要是为了不让它和 update() 抢 PWM。
 */
void DeviceServo::off(void)
{
  if (!_inited)
    return;

  _enabled_req = false; // 跨任务写点：update() 下一拍看到它就把脉宽归零
}


// ----------------
// ---------------- 开环查询接口 ----------------


/** @brief 目标角度 (度)，即 set_angle() 最后被限幅后的值 */
float DeviceServo::get_target_deg(void) const
{
  return _target_deg;
}

/** @brief 当前实际输出角度 (度)：限速过程中会滞后于 target_deg() */
float DeviceServo::get_angle_deg(void) const
{
  return _angle_deg;
}

/** @brief 最近一次写出去的脉宽 (µs)；off() 之后为 0 */
float DeviceServo::get_pulse_us(void) const
{
  return _pulse_us;
}


// ----------------
// ---------------- 私有实现 ----------------


/** @brief 角度限幅到 [0, range_deg]；NaN / 负数一律按 0（!(x > 0) 对 NaN 成立） */
float DeviceServo::_clamp_angle(float deg) const
{
  if (!(deg > 0.0f))
    return 0.0f;
  if (deg > _cfg.range_deg)
    return _cfg.range_deg;

  return deg;
}

/** @brief 脉宽 → 角度：_write_pulse() 的逆运算（先去 trim、再去 reversed） */
float DeviceServo::_deg_of_pulse(float width_us) const
{
  const float span = _cfg.pulse_max_us - _cfg.pulse_min_us; // > 0，init() 已校验
  float       t    = (width_us - _cfg.trim_us - _cfg.pulse_min_us) / span;

  if (_cfg.reversed)
    t = 1.0f - t;

  return _clamp_angle(t * _cfg.range_deg);
}

/**
 * @brief 角度 → 脉宽 → 写 PWM（本类唯一的下行出口，用来保证 _pulse_us 与实际输出一致）
 */
void DeviceServo::_write_pulse(void)
{
  float t = _angle_deg / _cfg.range_deg; // range_deg > 0，init() 已校验
  if (_cfg.reversed)
    t = 1.0f - t;

  const float pulse = _cfg.pulse_min_us + t * (_cfg.pulse_max_us - _cfg.pulse_min_us) + _cfg.trim_us;

  _pulse_us = pulse;
  _cfg.pwm->set_pulse_us(pulse);
}

// ----------------
