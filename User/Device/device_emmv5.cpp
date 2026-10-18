#include "device_emmv5.hpp"

#include "FreeRTOS.h" // IWYU pragma: keep（portMAX_DELAY）

// ---------------- 构造与生命周期 ----------------

DeviceEmmv5::DeviceEmmv5(const Config &cfg)
  : _uart(cfg.uart),
    _addr(cfg.addr)
{
  // 构造函数只做赋值；地址校验和状态复位在 init() 里
}

Status DeviceEmmv5::init()
{
  // 0 是广播地址：实例带广播地址会把别人的到位帧也算成自己的，必须拦住
  if (_addr == 0)
  {
    return Status::BAD_ARG;
  }

  _scanner.reset();
  _ready = true;
  return Status::OK;
}


// ----------------
// ---------------- 发送 ----------------

Status DeviceEmmv5::_send(const emmv5::Frame &frame)
{
  if (!_ready)
  {
    return Status::NOT_INIT;
  }

  if (frame.size == 0)
  {
    return Status::BAD_ARG;
  }

  return _uart.send(frame.data, frame.size, portMAX_DELAY, nullptr);
}


// ----------------
// ---------------- 常用命令 ----------------

Status DeviceEmmv5::enable(bool on, bool sync)
{
  return _send(emmv5::enable(_addr, on, sync));
}

Status DeviceEmmv5::set_velocity(emmv5::Direction dir, uint16_t rpm, uint8_t accel, bool sync)
{
  return _send(emmv5::velocity(_addr, dir, rpm, accel, sync));
}

Status DeviceEmmv5::set_position(emmv5::Direction dir, uint16_t rpm, uint8_t accel, uint32_t pulses, emmv5::PositionMode mode, bool sync)
{
  return _send(emmv5::position(_addr, dir, rpm, accel, pulses, mode, sync));
}

Status DeviceEmmv5::set_quick_position_params(uint16_t rpm, uint8_t accel, emmv5::PositionMode mode, bool sync)
{
  return _send(emmv5::quick_position_params(_addr, rpm, accel, mode, sync));
}

Status DeviceEmmv5::set_quick_position(int32_t pulses)
{
  return _send(emmv5::quick_position(_addr, pulses));
}

Status DeviceEmmv5::stop(bool sync)
{
  return _send(emmv5::stop(_addr, sync));
}

Status DeviceEmmv5::sync_start()
{
  return _send(emmv5::sync_start(_addr));
}

Status DeviceEmmv5::calibrate_encoder()
{
  return _send(emmv5::calibrate_encoder(_addr));
}

Status DeviceEmmv5::zero_position()
{
  return _send(emmv5::zero_position(_addr));
}

Status DeviceEmmv5::clear_stall()
{
  return _send(emmv5::clear_stall(_addr));
}

Status DeviceEmmv5::reset()
{
  return _send(emmv5::reset(_addr));
}

Status DeviceEmmv5::restore_defaults()
{
  return _send(emmv5::restore_defaults(_addr));
}

Status DeviceEmmv5::home(emmv5::HomingMode mode, bool sync)
{
  return _send(emmv5::home(_addr, mode, sync));
}

Status DeviceEmmv5::abort_home()
{
  return _send(emmv5::abort_home(_addr));
}

Status DeviceEmmv5::read(emmv5::SystemParam param)
{
  return _send(emmv5::read(_addr, param));
}

Status DeviceEmmv5::auto_report(emmv5::SystemParam param, uint16_t period_ms)
{
  return _send(emmv5::auto_report(_addr, param, period_ms));
}

Status DeviceEmmv5::set_pid(bool store, uint32_t kp, uint32_t ki, uint32_t kd)
{
  const emmv5::PidParams params(kp, ki, kd, store);
  return _send(emmv5::set_pid(_addr, params));
}

Status DeviceEmmv5::set_id(bool store, uint8_t id)
{
  return _send(emmv5::set_id(_addr, store, id));
}

Status DeviceEmmv5::set_microstep(bool store, uint8_t microstep)
{
  return _send(emmv5::set_microstep(_addr, store, microstep));
}

Status DeviceEmmv5::set_protection(bool store, uint16_t over_temp_c, uint16_t over_current_ma, uint16_t detect_ms)
{
  const emmv5::ProtectionParams params(over_temp_c, over_current_ma, detect_ms, store);
  return _send(emmv5::set_protection(_addr, params));
}


// ----------------
// ---------------- 接收 ----------------

bool DeviceEmmv5::update()
{
  if (!_ready)
  {
    return false;
  }

  bool    hit = false;
  uint8_t buf[RX_CHUNK_SIZE];

  // 最多读 RX_MAX_CHUNKS 块：数据持续到达时也要把控制权还给调用任务
  for (size_t chunk = 0; chunk < RX_MAX_CHUNKS; chunk++)
  {
    size_t n = 0;
    if (_uart.receive(buf, sizeof(buf), 0, &n) != Status::OK || n == 0)
    {
      break; // 暂时没有数据
    }

    if (feed_rx(buf, n))
    {
      hit = true;
    }
  }

  return hit;
}

bool DeviceEmmv5::feed_rx(const uint8_t *data, size_t size)
{
  if (!_ready)
  {
    return false;
  }

  return _scanner.feed(_addr, data, size);
}

// ----------------
