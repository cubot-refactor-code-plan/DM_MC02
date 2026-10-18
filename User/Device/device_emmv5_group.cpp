#include "device_emmv5_group.hpp"

#include "FreeRTOS.h" // IWYU pragma: keep（portMAX_DELAY）

#include <string.h> // memcpy

// ---------------- 构造与生命周期 ----------------

DeviceEmmv5Group::DeviceEmmv5Group(const Config &cfg)
  : _uart(cfg.uart),
    _count(cfg.count)
{
  for (size_t i = 0; i < MAX_MOTORS; i++)
  {
    _motor[i].addr = cfg.addr[i];
  }
}

Status DeviceEmmv5Group::init()
{
  if (_count == 0 || _count > MAX_MOTORS)
  {
    return Status::BAD_ARG;
  }

  // 地址必须 1~255 且不能重复，否则分发和到位判断都会串台
  for (size_t i = 0; i < _count; i++)
  {
    if (_motor[i].addr == 0)
    {
      return Status::BAD_ARG;
    }

    for (size_t j = 0; j < i; j++)
    {
      if (_motor[j].addr == _motor[i].addr)
      {
        return Status::BAD_ARG;
      }
    }
  }

  for (size_t i = 0; i < _count; i++)
  {
    _motor[i].scanner.reset();
    _motor[i].in_position = false;
  }

  _multi_payload = 0;
  _ready         = true;
  return Status::OK;
}


// ----------------
// ---------------- 私有查找与发送 ----------------

size_t DeviceEmmv5Group::index_of(uint8_t addr) const
{
  for (size_t i = 0; i < _count; i++)
  {
    if (_motor[i].addr == addr)
    {
      return i;
    }
  }
  return _count;
}

bool DeviceEmmv5Group::_is_member(uint8_t addr) const
{
  return index_of(addr) < _count;
}

Status DeviceEmmv5Group::_send_for(uint8_t addr, const emmv5::Frame &frame)
{
  if (!_ready)
  {
    return Status::NOT_INIT;
  }

  if (!_is_member(addr) || frame.size == 0)
  {
    return Status::BAD_ARG;
  }

  return _uart.send(frame.data, frame.size, portMAX_DELAY, nullptr);
}

Status DeviceEmmv5Group::_queue_for(uint8_t addr, const emmv5::Frame &frame)
{
  if (!_ready)
  {
    return Status::NOT_INIT;
  }

  if (!_is_member(addr) || frame.size == 0)
  {
    return Status::BAD_ARG;
  }

  // 先保证整帧放得下，再拷贝；绝不留下半条子命令
  if (_multi_payload + frame.size > MULTI_PAYLOAD_MAX)
  {
    return Status::FULL;
  }

  memcpy(_multi + MULTI_HEADER_SIZE + _multi_payload, frame.data, frame.size);
  _multi_payload += frame.size;
  return Status::OK;
}


// ----------------
// ---------------- 直接命令 ----------------

Status DeviceEmmv5Group::enable(uint8_t addr, bool on, bool sync)
{
  return _send_for(addr, emmv5::enable(addr, on, sync));
}

Status DeviceEmmv5Group::set_velocity(uint8_t addr, emmv5::Direction dir, uint16_t rpm, uint8_t accel, bool sync)
{
  return _send_for(addr, emmv5::velocity(addr, dir, rpm, accel, sync));
}

Status DeviceEmmv5Group::set_position(uint8_t addr, emmv5::Direction dir, uint16_t rpm, uint8_t accel, uint32_t pulses, emmv5::PositionMode mode, bool sync)
{
  return _send_for(addr, emmv5::position(addr, dir, rpm, accel, pulses, mode, sync));
}

Status DeviceEmmv5Group::set_quick_position_params(uint8_t addr, uint16_t rpm, uint8_t accel, emmv5::PositionMode mode, bool sync)
{
  return _send_for(addr, emmv5::quick_position_params(addr, rpm, accel, mode, sync));
}

Status DeviceEmmv5Group::set_quick_position(uint8_t addr, int32_t pulses)
{
  return _send_for(addr, emmv5::quick_position(addr, pulses));
}

Status DeviceEmmv5Group::stop(uint8_t addr, bool sync)
{
  return _send_for(addr, emmv5::stop(addr, sync));
}

Status DeviceEmmv5Group::sync_start(uint8_t addr)
{
  return _send_for(addr, emmv5::sync_start(addr));
}

Status DeviceEmmv5Group::sync_start_all()
{
  if (!_ready)
  {
    return Status::NOT_INIT;
  }

  const emmv5::Frame frame = emmv5::sync_start(emmv5::BROADCAST);
  return _uart.send(frame.data, frame.size, portMAX_DELAY, nullptr);
}

Status DeviceEmmv5Group::home(uint8_t addr, emmv5::HomingMode mode, bool sync)
{
  return _send_for(addr, emmv5::home(addr, mode, sync));
}

Status DeviceEmmv5Group::read(uint8_t addr, emmv5::SystemParam param)
{
  return _send_for(addr, emmv5::read(addr, param));
}


// ----------------
// ---------------- MMCL 多机命令 ----------------

void DeviceEmmv5Group::multi_clear()
{
  _multi_payload = 0;
}

Status DeviceEmmv5Group::multi_send(uint8_t target)
{
  if (!_ready)
  {
    return Status::NOT_INIT;
  }

  if (_multi_payload == 0)
  {
    return Status::OK;
  }

  const size_t total = emmv5::build_multi_frame(target, _multi + MULTI_HEADER_SIZE, _multi_payload, _multi, sizeof(_multi));
  if (total == 0)
  {
    return Status::BAD_ARG; // 理论上不会发生：_queue_for 已经保证整帧放得下
  }

  const Status status = _uart.send(_multi, total, portMAX_DELAY, nullptr);
  if (status == Status::OK)
  {
    _multi_payload = 0; // 发送失败时保留，调用方可以重试
  }
  return status;
}

Status DeviceEmmv5Group::queue_enable(uint8_t addr, bool on, bool sync)
{
  return _queue_for(addr, emmv5::enable(addr, on, sync));
}

Status DeviceEmmv5Group::queue_velocity(uint8_t addr, emmv5::Direction dir, uint16_t rpm, uint8_t accel, bool sync)
{
  return _queue_for(addr, emmv5::velocity(addr, dir, rpm, accel, sync));
}

Status DeviceEmmv5Group::queue_position(uint8_t addr, emmv5::Direction dir, uint16_t rpm, uint8_t accel, uint32_t pulses, emmv5::PositionMode mode, bool sync)
{
  return _queue_for(addr, emmv5::position(addr, dir, rpm, accel, pulses, mode, sync));
}

Status DeviceEmmv5Group::queue_quick_position_params(uint8_t addr, uint16_t rpm, uint8_t accel, emmv5::PositionMode mode, bool sync)
{
  return _queue_for(addr, emmv5::quick_position_params(addr, rpm, accel, mode, sync));
}

Status DeviceEmmv5Group::queue_quick_position(uint8_t addr, int32_t pulses)
{
  return _queue_for(addr, emmv5::quick_position(addr, pulses));
}

Status DeviceEmmv5Group::queue_stop(uint8_t addr, bool sync)
{
  return _queue_for(addr, emmv5::stop(addr, sync));
}

Status DeviceEmmv5Group::queue_sync_start(uint8_t addr)
{
  return _queue_for(addr, emmv5::sync_start(addr));
}

Status DeviceEmmv5Group::queue_home(uint8_t addr, emmv5::HomingMode mode, bool sync)
{
  return _queue_for(addr, emmv5::home(addr, mode, sync));
}


// ----------------
// ---------------- 接收 ----------------

uint32_t DeviceEmmv5Group::update()
{
  if (!_ready)
  {
    return 0;
  }

  uint32_t hits = 0;
  uint8_t  buf[RX_CHUNK_SIZE];

  // 最多读 RX_MAX_CHUNKS 块：数据持续到达时也要把控制权还给调用任务
  for (size_t chunk = 0; chunk < RX_MAX_CHUNKS; chunk++)
  {
    size_t n = 0;
    if (_uart.receive(buf, sizeof(buf), 0, &n) != Status::OK || n == 0)
    {
      break; // 暂时没有数据
    }

    hits |= feed_rx(buf, n);
  }

  return hits;
}

uint32_t DeviceEmmv5Group::feed_rx(const uint8_t *data, size_t size)
{
  if (!_ready || data == nullptr || size == 0)
  {
    return 0;
  }

  uint32_t hits = 0;

  for (size_t i = 0; i < _count; i++)
  {
    if (_motor[i].scanner.feed(_motor[i].addr, data, size))
    {
      _motor[i].in_position = true;
      hits |= (1UL << i);
    }
  }

  return hits;
}

bool DeviceEmmv5Group::in_position(size_t index) const
{
  return _ready && (index < _count) && _motor[index].in_position;
}

uint32_t DeviceEmmv5Group::take_in_position()
{
  if (!_ready)
  {
    return 0;
  }

  uint32_t mask = 0;

  for (size_t i = 0; i < _count; i++)
  {
    if (_motor[i].in_position)
    {
      mask |= (1UL << i);
      _motor[i].in_position = false;
    }
  }

  return mask;
}

// ----------------
