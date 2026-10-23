#include "corexy.hpp"

#include "device_emmv5_group.hpp" // IWYU pragma: keep（显式实例化需要具体电机组类型）
#include "emmv5_protocol.hpp"

#include <cmath>
#include <stdlib.h>

// ---------------- 模板实例化 ----------------

// 显式实例化：必须覆盖 module_cfg.cpp 里用到的电机组类型，否则链接时找不到成员函数
template class CorexyT<DeviceEmmv5GroupT<BspUartT<uart_detail::DmaTransport, 128, 128>>>;

// ----------------
// ---------------- 构造与析构 ----------------

template <typename Group>
CorexyT<Group>::CorexyT(const Config &cfg)
  : _cfg(cfg)
{
}

// ----------------
// ---------------- 公有接口 ----------------

template <typename Group>
Status CorexyT<Group>::init()
{
  if (_cfg.group == nullptr)
  {
    return Status::BAD_ARG;
  }

  if (_cfg.addr1 == 0U || _cfg.addr2 == 0U || _cfg.addr1 == _cfg.addr2)
  {
    return Status::BAD_ARG;
  }

  if (!_cfg.group->is_ready())
  {
    return Status::NOT_INIT;
  }

  // 限位：两块矩形自身合法，且安全限位必须完全落在理论限位内（含边界）
  const Rect &th = _cfg.theory;
  const Rect &sf = _cfg.safe;

  if (!_finite(th.x0) || !_finite(th.y0) || !_finite(th.x1) || !_finite(th.y1) || th.x1 <= th.x0 || th.y1 <= th.y0)
  {
    return Status::BAD_ARG;
  }

  if (!_finite(sf.x0) || !_finite(sf.y0) || !_finite(sf.x1) || !_finite(sf.y1) || sf.x1 <= sf.x0 || sf.y1 <= sf.y0)
  {
    return Status::BAD_ARG;
  }

  if (sf.x0 < th.x0 || sf.y0 < th.y0 || sf.x1 > th.x1 || sf.y1 > th.y1)
  {
    return Status::BAD_ARG;
  }

  if (_cfg.rpm == 0U || _cfg.rpm > 5000U || !_finite(_cfg.arrive_tol) || _cfg.arrive_tol <= 0.0f)
  {
    return Status::BAD_ARG;
  }

  // 几何参数必须为正，否则下面折算出来的比例会变成除零 / 负数
  if (!_finite(_cfg.mm_per_rev) || _cfg.mm_per_rev <= 0.0f || _cfg.pulses_per_rev == 0U || _cfg.counts_per_rev == 0U)
  {
    return Status::BAD_ARG;
  }

  _pulses_per_mm    = static_cast<float>(_cfg.pulses_per_rev) / _cfg.mm_per_rev;
  _counts_per_mm    = static_cast<float>(_cfg.counts_per_rev) / _cfg.mm_per_rev;
  _counts_per_pulse = static_cast<float>(_cfg.counts_per_rev) / static_cast<float>(_cfg.pulses_per_rev);

  _state      = State::Idle;
  _last_error = Status::OK;
  _pos        = Point(0.0f, 0.0f);
  _target     = Point(0.0f, 0.0f);
  _seg_count  = 0U;
  _seg_index  = 0U;
  _cur_counts[0] = 0;
  _cur_counts[1] = 0;

  _cmd_target[0] = 0;
  _cmd_target[1] = 0;
  _armed         = false;
  _zero_ok       = 0U;

  _quick_valid[0] = false;
  _quick_valid[1] = false;
  _read_hold      = 0;
  _inited         = true;

  return Status::OK;
}

template <typename Group>
void CorexyT<Group>::quiet()
{
  if (!_inited)
  {
    return;
  }

  _quiet();
}


template <typename Group>
void CorexyT<Group>::_quiet()
{
  // 半双工：先挂起读位置，再真延时，保证驱动器把上一条应答发完、自己回到接收态
  _read_hold = xTaskGetTickCount() + pdMS_TO_TICKS(SEND_QUIET_MS);
  vTaskDelay(pdMS_TO_TICKS(SEND_QUIET_MS));
}


template <typename Group>
Status CorexyT<Group>::enable(bool on)
{
  if (!_inited)
  {
    return Status::NOT_INIT;
  }

  // 失能命令无应答，只能靠“不执行”反推：不静默的话很可能被在途的 0x36 应答吃掉
  _quiet();

  const Status st1 = _cfg.group->enable(_cfg.addr1, on);
  vTaskDelay(pdMS_TO_TICKS(ENABLE_GAP_MS));
  const Status st2 = _cfg.group->enable(_cfg.addr2, on);

  _read_hold = xTaskGetTickCount() + pdMS_TO_TICKS(30U);

  return (st1 != Status::OK) ? st1 : st2;
}


template <typename Group>
Status CorexyT<Group>::zero_here()
{
  if (!_inited)
  {
    return Status::NOT_INIT;
  }

  // 先停掉旧目标，避免清零瞬间还有运动在跑；帧间留间隔
  _stop_motors();

  // 清零命令同样无应答，发出前必须静默
  _quiet();

  const Status st1 = _cfg.group->zero_position(_cfg.addr1);
  vTaskDelay(pdMS_TO_TICKS(SEND_GAP_MS));
  const Status st2 = _cfg.group->zero_position(_cfg.addr2);

  _read_hold = xTaskGetTickCount() + pdMS_TO_TICKS(30U);

  if (st1 != Status::OK)
  {
    return st1;
  }

  if (st2 != Status::OK)
  {
    return st2;
  }

  // 清零命令没有应答，先按“位置已为 0”记账；复核由调用方读 position() 完成
  _cur_counts[0] = 0;
  _cur_counts[1] = 0;
  _cmd_target[0] = 0;
  _cmd_target[1] = 0;
  _arrive_ok     = 0U;
  _pos           = Point(0.0f, 0.0f);
  _target        = Point(0.0f, 0.0f);
  _seg_count     = 0U;
  _seg_index     = 0U;
  _state         = State::Idle;
  _last_error    = Status::OK;

  // 清零命令没有应答，必须靠后续读数复核；复核前禁止运动
  _armed   = false;
  _zero_ok = 0U;

  return Status::OK;
}

template <typename Group>
Status CorexyT<Group>::move_to(Point p)
{
  if (!_inited)
  {
    return Status::NOT_INIT;
  }

  if (!_finite(p.x) || !_finite(p.y))
  {
    return Status::BAD_ARG;
  }

  // 原点未复核时坐标含义未知：绝对目标可能落在离当前很远的位置，必须先 home
  if (!_armed)
  {
    return Status::NOT_INIT;
  }

  // 目标只按安全限位放行：这里到不了理论矩形的边上，也就不会撞
  if (!_inside(p, _cfg.safe))
  {
    return Status::BAD_ARG;
  }

  if (_state == State::Moving)
  {
    return Status::BUSY;
  }

  if (_state == State::Fault)
  {
    return Status::BAD_ARG; // 必须先 stop() 清故障
  }

  _build_segments(p);

  if (_seg_count == 0U)
  {
    return Status::OK; // 目标与当前重合
  }

  _target    = p;
  _seg_index = 0U;

  const Status st = _send_segment(0U);
  if (st != Status::OK)
  {
    _fault(st);
    return st;
  }

  _state = State::Moving;
  return Status::OK;
}

template <typename Group>
Status CorexyT<Group>::move_by(Point d)
{
  return move_to(Point(_pos.x + d.x, _pos.y + d.y));
}

template <typename Group>
Status CorexyT<Group>::step(Step s, float mm)
{
  if (!_finite(mm) || mm <= 0.0f)
  {
    return Status::BAD_ARG;
  }

  // 对角档按真实距离分摊到两轴
  static const float kDir[8][2] = {
    {1.0f, 0.0f},           // 0°  +X
    {0.70710678f, 0.70710678f},  // 45° +X +Y
    {0.0f, 1.0f},           // 90° +Y
    {-0.70710678f, 0.70710678f}, // 135° -X +Y
    {-1.0f, 0.0f},          // 180° -X
    {-0.70710678f, -0.70710678f},// 225° -X -Y
    {0.0f, -1.0f},          // 270° -Y
    {0.70710678f, -0.70710678f}  // 315° +X -Y
  };

  const size_t index = static_cast<size_t>(s);
  if (index >= 8U)
  {
    return Status::BAD_ARG;
  }

  return move_by(Point(kDir[index][0] * mm, kDir[index][1] * mm));
}

template <typename Group>
Status CorexyT<Group>::stop()
{
  if (!_inited)
  {
    return Status::NOT_INIT;
  }

  _stop_motors();

  _state      = State::Idle;
  _seg_count  = 0U;
  _seg_index  = 0U;
  _last_error = Status::OK;
  _arrive_ok  = 0U;

  return Status::OK;
}

template <typename Group>
Status CorexyT<Group>::set_speed(uint16_t rpm, uint8_t acc)
{
  if (!_inited)
  {
    return Status::NOT_INIT;
  }

  if (rpm == 0U || rpm > 5000U)
  {
    return Status::BAD_ARG;
  }

  _cfg.rpm = rpm;
  _cfg.acc = acc;
  return Status::OK;
}

template <typename Group>
Status CorexyT<Group>::set_move_mode(MoveMode mode)
{
  if (!_inited)
  {
    return Status::NOT_INIT;
  }

  // 分段进行中改模式：两台的参数与指令会不一致，先让它走完
  if (_state == State::Moving)
  {
    return Status::BUSY;
  }

  if (mode == _cfg.move_mode)
  {
    return Status::OK;
  }

  _cfg.move_mode = mode;

  // 换模式后驱动器里的 0xF1 参数不一定还可靠，下一段重新下发
  _quick_valid[0] = false;
  _quick_valid[1] = false;

  return Status::OK;
}

template <typename Group>
void CorexyT<Group>::update(uint32_t timeout_ms)
{
  if (!_inited)
  {
    return;
  }

  // 0. 先把串口里攒的字节解析成扫描器结果（timeout_ms = 0 则纯非阻塞）
  (void)_cfg.group->update(timeout_ms);

  const TickType_t now = xTaskGetTickCount();

  // 1. 读两台位置：**一次一台**，读到应答再发下一条查询。
  //    - MMCL / 使能 / 清零 / 停止这类无应答命令发出后，必须隔 READ_GAP_MS 才能开始读；
  //    - 两条查询之间同样留 SEND_GAP_MS，半双工下不能背靠背发；
  //    - 不动的电机也要读：它的“停在原地”本身就是本段执行成功的证据。
  bool read_ok = false;

  if (static_cast<int32_t>(now - _read_hold) >= 0)
  {
    int32_t c1 = _cur_counts[0];
    int32_t c2 = _cur_counts[1];

    const bool ok1 = _read_position(_cfg.addr1, c1);
    vTaskDelay(pdMS_TO_TICKS(SEND_GAP_MS));
    const bool ok2 = _read_position(_cfg.addr2, c2);

    if (ok1)
    {
      _cur_counts[0] = c1;
    }

    if (ok2)
    {
      _cur_counts[1] = c2;
    }

    if (ok1 || ok2)
    {
      _refresh_position();
    }

    read_ok    = ok1 && ok2;
    _read_hold = xTaskGetTickCount() + pdMS_TO_TICKS(READ_PERIOD_MS);
  }

  // 原点复核：清零命令无应答，只能靠读数确认。连续 ZERO_CONFIRM 拍接近 (0,0) 才放行运动
  if (!_armed)
  {
    const bool near_zero = (_pos.x > -ZERO_TOL_MM) && (_pos.x < ZERO_TOL_MM) &&
                           (_pos.y > -ZERO_TOL_MM) && (_pos.y < ZERO_TOL_MM);

    if (near_zero)
    {
      if (_zero_ok < 0xFFU)
      {
        _zero_ok++;
      }

      if (_zero_ok >= ZERO_CONFIRM)
      {
        _armed = true;
      }
    }
    else
    {
      _zero_ok = 0U;
    }
  }

  // 2. 只有移动中才需要推进状态机
  if (_state != State::Moving)
  {
    return;
  }

  // 3.1 防撞：实测坐标越出理论限位就停机（安全限位只是目标校验，不在这里停）
  // 容差用 OVERRUN_MM：正常跟随误差（减速过冲、两台微小不同步）不应触发，
  // 只有真跑飞才拦。包围盒太紧时，靠近 y=0 的段会被 1 mm 级跟随误差误杀。
  const float margin = OVERRUN_MM;
  const Rect  guard(_cfg.theory.x0 - margin, _cfg.theory.y0 - margin, _cfg.theory.x1 + margin, _cfg.theory.y1 + margin);

  if (!_inside(_pos, guard))
  {
    _fault(Status::BAD_ARG);
    return;
  }

  // 3.1b 轴级保护：偏离本段预期行程（含反向）超过 OVERRUN_MM 立即停机。
  // 能盖住“目标被误解析成极大值 / 单台电机跑飞 / 某轴反向”的情况。
  {
    const int32_t overrun = static_cast<int32_t>(OVERRUN_MM * _counts_per_mm) + 64;

    for (size_t i = 0U; i < 2U; i++)
    {
      if (_seg_dir[i] == 0)
      {
        // 本段不应动：只要有明显位移就说明被别的帧带走了
        const int32_t stray = _cur_counts[i] - _seg_from[i];
        if (stray > overrun || stray < -overrun)
        {
          _fault(Status::IO_ERROR);
          return;
        }
        continue;
      }

      const int32_t traveled = (_cur_counts[i] - _seg_from[i]) * _seg_dir[i];

      if (traveled < -overrun || traveled > (_seg_travel[i] + overrun))
      {
        _fault(Status::IO_ERROR);
        return;
      }
    }
  }

  // 3.2 到位判定：这一拍两台都读到了位置（read_ok），且两台读回的值都在本段目标窗口内。
  // 45° 段不动的那台同样要占一票，否则单台丢应答也能判通过。
  const int32_t tol_counts = static_cast<int32_t>(_cfg.arrive_tol * _counts_per_mm);

  const int32_t err1 = _cur_counts[0] - _cmd_target[0];
  const int32_t err2 = _cur_counts[1] - _cmd_target[1];

  const bool in_window = (err1 <= tol_counts) && (err1 >= -tol_counts) &&
                         (err2 <= tol_counts) && (err2 >= -tol_counts);

  const bool arrived = read_ok && in_window;

  // 连续 ARRIVE_CONFIRM 拍都成立才放行：单拍可能正好落在减速段末尾，
  // 读数刚进窗口、驱动器还没停稳，此刻接着发下一条 MMCL 会叠在未完成的运动上。
  if (arrived)
  {
    if (_arrive_ok < ARRIVE_CONFIRM)
    {
      _arrive_ok++;
    }
  }
  else
  {
    _arrive_ok = 0U;
  }

  if (_arrive_ok >= ARRIVE_CONFIRM)
  {
    // 最后一段：两台位置都已确认正确，直接结束
    if ((_seg_index + 1U) >= _seg_count)
    {
      _state     = State::Idle;
      _seg_count = 0U;
      _seg_index = 0U;
      return;
    }

    // 下一段：_send_segment 内部先静默 SEND_QUIET_MS 再发 MMCL，
    // 发完把 _read_hold 推到 +READ_GAP_MS，之后才允许读位置。
    _seg_index++;

    const Status st = _send_segment(_seg_index);
    if (st != Status::OK)
    {
      _fault(st);
    }

    return;
  }

  // 3.3 超时兜底：堵转、掉线、目标被限幅都会走到这里
  if ((xTaskGetTickCount() - _seg_tick) > _seg_timeout)
  {
    _fault(Status::TIMEOUT);
  }
}

// ----------------
// ---------------- 私有实现 ----------------

template <typename Group>
void CorexyT<Group>::_build_segments(Point goal)
{
  _seg_count = 0U;

  const float dx  = goal.x - _pos.x;
  const float dy  = goal.y - _pos.y;
  const float adx = std::fabs(dx);
  const float ady = std::fabs(dy);

  if (adx < 1.0e-3f && ady < 1.0e-3f)
  {
    return; // 与当前重合，不发命令
  }

  // 纯轴向或纯 45°：一段走完
  const bool axis_only = (adx < 1.0e-3f) || (ady < 1.0e-3f);
  const bool diag_only = std::fabs(adx - ady) < 1.0e-3f;

  if (axis_only || diag_only)
  {
    _seg_end[0] = goal;
    _seg_count  = 1U;
    return;
  }

  // 其余情况：先 45°，再轴向。安全限位是凸矩形，两个端点在内 ⇒ 中转点必在内
  const float d = (adx < ady) ? adx : ady;

  _seg_end[0] = Point(_pos.x + ((dx > 0.0f) ? d : -d), _pos.y + ((dy > 0.0f) ? d : -d));
  _seg_end[1] = goal;
  _seg_count  = 2U;
}

template <typename Group>
Status CorexyT<Group>::_send_segment(size_t index)
{
  if (index >= _seg_count)
  {
    return Status::BAD_ARG;
  }

  const Point b = _seg_end[index];

  const int32_t end_pulse[2] = {_pulses(b, 0U), _pulses(b, 1U)};

  // 先把还没取走的新应答收掉：增量基准必须是**最新实测值**。
  // 段起点在上一段到位 + 段间停顿之后，两台都已静止，此时读数可靠；
  // 拿不到新应答则沿用上一拍读数（最多 20 ms 前）。
  int32_t latest = 0;
  if (_cfg.group->get_position(_cfg.addr1, latest))
  {
    _cur_counts[0] = latest;
  }
  if (_cfg.group->get_position(_cfg.addr2, latest))
  {
    _cur_counts[1] = latest;
  }

  // 相对模式以“上一输入目标”为基准，但基准必须用实测值：
  // 指令目标与实际位置一旦脱节（故障/停止/外部推动），继续用书签值会让下一段增量为 0。
  const int32_t now_pulse[2] = {
    static_cast<int32_t>(std::lround(static_cast<double>(_cur_counts[0]) / static_cast<double>(_counts_per_pulse))),
    static_cast<int32_t>(std::lround(static_cast<double>(_cur_counts[1]) / static_cast<double>(_counts_per_pulse)))};

  // 协议字段是「正幅值 + 方向字节」：用相对模式下发增量，脉冲字段永远为正
  const int32_t delta1 = end_pulse[0] - now_pulse[0];
  const int32_t delta2 = end_pulse[1] - now_pulse[1];

  const int32_t mag1 = (delta1 < 0) ? -delta1 : delta1;
  const int32_t mag2 = (delta2 < 0) ? -delta2 : delta2;
  const int32_t max_mag = (mag1 > mag2) ? mag1 : mag2;

  if (max_mag == 0)
  {
    // 目标与当前脉冲相同（不足 1 个脉冲）：没有命令可发，但段计时必须复位，
    // 否则会拿上一条段的时间戳去比，可能立刻被判成超时。
    _seg_tick    = xTaskGetTickCount();
    _seg_timeout = pdMS_TO_TICKS(SEG_TIMEOUT_MS);
    return Status::OK;
  }

  const uint8_t addr[2] = {_cfg.addr1, _cfg.addr2};

  // 半双工总线：从“静默等待”到“整帧发出”中间不能插任何查询
  // RS485 半双工：上一条 0x36 查询的应答可能还在回传。若紧接着发 MMCL，
  // 驱动器正处于“自己发送、收不进字节”的状态，整帧会被丢掉（表现为命令不执行）。
  // 所以发命令前先静默一段，静默期内也不发任何查询。
  _quiet();

  // 一条 MMCL 总帧带两台子命令；发出后只轮询，两台都到位后才发下一段
  _cfg.group->multi_clear();

  for (size_t i = 0U; i < 2U; i++)
  {
    const int32_t delta = (i == 0U) ? delta1 : delta2;

    // 到位目标 = 本段起点实测 counts + 本段指令增量（counts）。
    // 用增量而不是“世界上标 × counts/脉冲”：编码器零位与软件原点即使差一个常数，
    // “这一段有没有走完”依然能判准；不动的电机目标就等于起点，自然落在窗口内。
    _cmd_target[i] = _cur_counts[i] + static_cast<int32_t>(std::lround(static_cast<double>(delta) * static_cast<double>(_counts_per_pulse)));

    if (delta == 0)
    {
      _last_delta[i] = 0;
      continue; // 本段这台不动：45° 对角只有一台参与
    }

    const int32_t mag = (delta < 0) ? -delta : delta;

    // 速度按脉冲数分摊，两台同时到达；acc 相同即可保持加速段比例
    uint32_t rpm = static_cast<uint32_t>(std::lround(static_cast<double>(_cfg.rpm) * static_cast<double>(mag) / static_cast<double>(max_mag)));
    if (rpm < 1U)
    {
      rpm = 1U;
    }
    if (rpm > 5000U)
    {
      rpm = 5000U;
    }

    Status qs = Status::OK;

    if (_cfg.move_mode == MoveMode::Quick)
    {
      // 参数变了（或首次）才补发 0xF1，其余情况直接 0xFC
      if (!_quick_valid[i] || (_quick_rpm[i] != static_cast<uint16_t>(rpm)) || (_quick_acc[i] != _cfg.acc))
      {
        qs = _cfg.group->queue_quick_position_params(addr[i], static_cast<uint16_t>(rpm), _cfg.acc,
                                                     emmv5::PositionMode::Relative);
        if (qs == Status::OK)
        {
          _quick_valid[i] = true;
          _quick_rpm[i]   = static_cast<uint16_t>(rpm);
          _quick_acc[i]   = _cfg.acc;
        }
      }

      // 0xFC 没有方向字节：靠 delta 的符号表方向
      if (qs == Status::OK)
      {
        qs = _cfg.group->queue_quick_position(addr[i], delta);
      }
    }
    else
    {
      // 0xFD：相对模式 + 方向字节 + 正幅值
      const emmv5::Direction dir = (delta > 0) ? emmv5::Direction::CW : emmv5::Direction::CCW;
      qs = _cfg.group->queue_position(addr[i], dir, static_cast<uint16_t>(rpm), _cfg.acc, mag,
                                      emmv5::PositionMode::Relative);
    }

    if (qs != Status::OK)
    {
      // 参数帧已经发出去的，下次必须重发；直接作废缓存
      _quick_valid[i] = false;
      _cfg.group->multi_clear();
      return qs;
    }

    _last_delta[i] = delta;
  }

  const Status st = _cfg.group->multi_send();
  if (st != Status::OK)
  {
    return st;
  }

  // 命令发出后到第一次读位置之间必须留间隔（用户实测要求 5 ms）。
  // 控制命令**没有应答**，这段时间不是等应答，而是给驱动器解析 / 建档的时间；
  // 紧接着发 0x36 会把还没处理完的 MMCL 顶掉，表现为“命令不执行”。
  _read_hold = xTaskGetTickCount() + pdMS_TO_TICKS(READ_GAP_MS);

  // 记录本段预期：起点用当前实测，方向与行程由指令增量得到
  for (size_t i = 0U; i < 2U; i++)
  {
    const int32_t delta = (i == 0U) ? delta1 : delta2;

    _seg_from[i]   = _cur_counts[i];
    _seg_dir[i]    = (delta > 0) ? 1 : ((delta < 0) ? -1 : 0);
    _seg_travel[i] = static_cast<int32_t>(std::lround(std::fabs(static_cast<double>(delta)) * static_cast<double>(_counts_per_pulse)));
  }

  // 新的一段重新计到位确认拍数
  _arrive_ok = 0U;

  // 段超时给得非常大：宁可一直等两台回复到位，也不要误判 TIMEOUT 把正常运动停机。
  // 真需要中断请用 stop()。
  _seg_tick    = xTaskGetTickCount();
  _seg_timeout = pdMS_TO_TICKS(SEG_TIMEOUT_MS);

  return Status::OK;
}

template <typename Group>
bool CorexyT<Group>::_read_position(uint8_t addr, int32_t &out)
{
  int32_t value = 0;

  // 发请求前先丢掉遗留应答：否则会把上一条请求的结果当成本次读数
  while (_cfg.group->get_position(addr, value))
  {
  }

  if (_cfg.group->request_position(addr) != Status::OK)
  {
    return false;
  }

  // 半双工：一问一答，收到应答才能发下一条查询（3 B 请求 + 8 B 应答约 1 ms）
  for (uint32_t waited = 0U; waited < REPLY_TIMEOUT_MS; waited++)
  {
    (void)_cfg.group->update(1U);

    if (_cfg.group->get_position(addr, value))
    {
      out = value;
      return true;
    }
  }

  return false;
}

template <typename Group>
void CorexyT<Group>::_refresh_position()
{
  _pos = Point(-static_cast<float>(_cur_counts[0] - _cur_counts[1]) / (2.0f * _counts_per_mm),
               -static_cast<float>(_cur_counts[0] + _cur_counts[1]) / (2.0f * _counts_per_mm));
}

template <typename Group>
void CorexyT<Group>::_fault(Status reason)
{
  _state      = State::Fault;
  _last_error = reason;

  // 停机带间隔，保证不会被旧目标带走
  _stop_motors();
}

template <typename Group>
void CorexyT<Group>::_stop_motors()
{
  // 同 P6：发命令前先让总线安静，否则驱动器正忙于回 0x36，收不进停止帧
  _quiet();

  // 逐台停止 + 广播停止；均使用手册里的立即停止帧
  (void)_cfg.group->stop(_cfg.addr1);
  vTaskDelay(pdMS_TO_TICKS(SEND_GAP_MS));
  (void)_cfg.group->stop(_cfg.addr2);
  vTaskDelay(pdMS_TO_TICKS(SEND_GAP_MS));
  (void)_cfg.group->stop_all();
  vTaskDelay(pdMS_TO_TICKS(SEND_GAP_MS));

  _read_hold = xTaskGetTickCount() + pdMS_TO_TICKS(30U);
}

template <typename Group>
bool CorexyT<Group>::_inside(Point p, const Rect &r)
{
  return (p.x >= r.x0) && (p.x <= r.x1) && (p.y >= r.y0) && (p.y <= r.y1);
}

template <typename Group>
int32_t CorexyT<Group>::_pulses(Point p, size_t axis) const
{
  // CoreXY 逆解：p1 = -k(X + Y)，p2 = +k(X - Y)，k = 脉冲/mm
  const double value = (axis == 0U) ? (-static_cast<double>(_pulses_per_mm) * (static_cast<double>(p.x) + static_cast<double>(p.y)))
                                    : (static_cast<double>(_pulses_per_mm) * (static_cast<double>(p.x) - static_cast<double>(p.y)));

  return static_cast<int32_t>(std::lround(value));
}

template <typename Group>
bool CorexyT<Group>::_finite(float v)
{
  return (v == v) && (v < 1.0e30f) && (v > -1.0e30f);
}

// ----------------
