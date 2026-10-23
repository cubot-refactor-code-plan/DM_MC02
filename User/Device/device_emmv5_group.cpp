#include "device_emmv5_group.hpp"

#include "FreeRTOS.h" // IWYU pragma: keep（pdMS_TO_TICKS / taskENTER_CRITICAL）

#include <string.h> // memcpy

// ---------------- 模板实例化 ----------------

// 显式实例化：必须覆盖 device_cfg.cpp 里用到的串口类型，否则链接时找不到成员函数
template class DeviceEmmv5GroupT<BspUartT<uart_detail::DmaTransport, 128, 128>>;

// ----------------
// ---------------- 构造与生命周期 ----------------

template <typename Uart>
DeviceEmmv5GroupT<Uart>::DeviceEmmv5GroupT(const Config &cfg) : _uart(cfg.uart),
                                                                _count(cfg.count)
{
  for (size_t i = 0; i < MAX_MOTORS; i++)
  {
    _motor[i].addr = cfg.addr[i];
  }
}

template <typename Uart>
Status DeviceEmmv5GroupT<Uart>::init()
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
    _motor[i].inpos_scanner.reset();
    _motor[i].position_scanner.reset();
    _motor[i].status_scanner.reset();
    _motor[i].in_position    = false;
    _motor[i].position       = 0;
    _motor[i].status         = 0;
    _motor[i].position_valid = false;
    _motor[i].status_valid   = false;
  }

  _multi_payload = 0;

  // 半双工总线锁：必须在 _ready 置位、第一次发送之前就存在。
  // 递归锁允许同一任务在“组装 MMCL 事务”里再调单帧发送接口而不自锁。
  if (_bus_lock == nullptr)
  {
    _bus_lock = xSemaphoreCreateRecursiveMutex();

    if (_bus_lock == nullptr)
    {
      return Status::FULL; // 堆不够
    }
  }

  _ready = true;

  return Status::OK;
}

template <typename Uart>
DeviceEmmv5GroupT<Uart>::~DeviceEmmv5GroupT()
{
  if (_bus_lock != nullptr)
  {
    vSemaphoreDelete(_bus_lock);
    _bus_lock = nullptr;
  }
}

// ----------------
// ---------------- 私有查找与发送 ----------------

template <typename Uart>
size_t DeviceEmmv5GroupT<Uart>::index_of(uint8_t addr) const
{
  // 配置可能传入超过 MAX_MOTORS 的地址数；init() 会拒绝，但查找时也要自己夹住
  const size_t count = (_count > MAX_MOTORS) ? MAX_MOTORS : _count;

  for (size_t i = 0; i < count; i++)
  {
    if (_motor[i].addr == addr)
    {
      return i;
    }
  }

  return _count;
}

template <typename Uart>
bool DeviceEmmv5GroupT<Uart>::_is_member(uint8_t addr) const
{
  return index_of(addr) < _count;
}


// ----------------
// ---------------- 总线互斥 ----------------

template <typename Uart>
Status DeviceEmmv5GroupT<Uart>::lock_bus(uint32_t timeout_ms)
{
  if (!_ready || (_bus_lock == nullptr))
  {
    return Status::NOT_INIT;
  }

  if (xSemaphoreTakeRecursive(_bus_lock, pdMS_TO_TICKS(timeout_ms)) != pdTRUE)
  {
    return Status::TIMEOUT;
  }

  return Status::OK;
}

template <typename Uart>
void DeviceEmmv5GroupT<Uart>::unlock_bus()
{
  if (_bus_lock != nullptr)
  {
    (void)xSemaphoreGiveRecursive(_bus_lock);
  }
}

template <typename Uart>
Status DeviceEmmv5GroupT<Uart>::_send_for(uint8_t addr, const emmv5::Frame &frame)
{
  if (!_ready)
  {
    return Status::NOT_INIT;
  }

  if (!_is_member(addr) || !frame.valid())
  {
    return Status::BAD_ARG;
  }

  // 半双工：整帧发送期间独占总线，其他任务不得插入任何字节
  const Status ls = lock_bus();
  if (ls != Status::OK)
  {
    return ls;
  }

  // 20 ms 入队超时：短帧足够；缓冲卡死时返回 TIMEOUT，不无限挂住调用任务
  const Status status = _uart.send(frame.data, frame.size, pdMS_TO_TICKS(20U), nullptr);

  unlock_bus();
  return status;
}

template <typename Uart>
Status DeviceEmmv5GroupT<Uart>::_queue_for(uint8_t addr, const emmv5::Frame &frame)
{
  if (!_ready)
  {
    return Status::NOT_INIT;
  }

  if (!_is_member(addr) || !frame.valid())
  {
    return Status::BAD_ARG;
  }

  // 先保证整帧放得下，再拷贝；绝不留下半条子命令
  if (_multi_payload + frame.size > MULTI_PAYLOAD_MAX)
  {
    return Status::FULL;
  }

  // 拼帧也是总线事务的一部分：跟多任务下其他子命令串行
  const Status ls = lock_bus();
  if (ls != Status::OK)
  {
    return ls;
  }

  memcpy(_multi + emmv5::MULTI_PREFIX_SIZE + _multi_payload, frame.data, frame.size);
  _multi_payload += frame.size;

  unlock_bus();
  return Status::OK;
}


// ----------------
// ---------------- 直接命令 ----------------

template <typename Uart>
Status DeviceEmmv5GroupT<Uart>::enable(uint8_t addr, bool on)
{
  return _send_for(addr, emmv5::enable(addr, on));
}

template <typename Uart>
Status DeviceEmmv5GroupT<Uart>::set_velocity(uint8_t addr, emmv5::Direction dir, uint16_t rpm, uint8_t accel)
{
  if (!_valid_direction(dir) || !_valid_rpm(rpm))
  {
    return Status::BAD_ARG;
  }

  return _send_for(addr, emmv5::velocity(addr, dir, rpm, accel));
}

template <typename Uart>
Status DeviceEmmv5GroupT<Uart>::set_position(uint8_t addr, emmv5::Direction dir, uint16_t rpm, uint8_t accel, int32_t pulses, emmv5::PositionMode mode)
{
  if (!_valid_direction(dir) || !_valid_rpm(rpm))
  {
    return Status::BAD_ARG;
  }

  return _send_for(addr, emmv5::position(addr, dir, rpm, accel, pulses, mode));
}

template <typename Uart>
Status DeviceEmmv5GroupT<Uart>::set_quick_position_params(uint8_t addr, uint16_t rpm, uint8_t accel, emmv5::PositionMode mode)
{
  if (!_valid_rpm(rpm))
  {
    return Status::BAD_ARG;
  }

  return _send_for(addr, emmv5::quick_position_params(addr, rpm, accel, mode));
}

template <typename Uart>
Status DeviceEmmv5GroupT<Uart>::set_quick_position(uint8_t addr, int32_t pulses)
{
  return _send_for(addr, emmv5::quick_position(addr, pulses));
}

template <typename Uart>
Status DeviceEmmv5GroupT<Uart>::stop(uint8_t addr)
{
  return _send_for(addr, emmv5::stop(addr));
}

template <typename Uart>
Status DeviceEmmv5GroupT<Uart>::stop_all()
{
  if (!_ready)
  {
    return Status::NOT_INIT;
  }

  const emmv5::Frame frame = emmv5::stop(emmv5::BROADCAST);
  if (!frame.valid())
  {
    return Status::BAD_ARG;
  }

  const Status ls = lock_bus();
  if (ls != Status::OK)
  {
    return ls;
  }

  const Status status = _uart.send(frame.data, frame.size, pdMS_TO_TICKS(20U), nullptr);

  unlock_bus();
  return status;
}

template <typename Uart>
Status DeviceEmmv5GroupT<Uart>::zero_position(uint8_t addr)
{
  return _send_for(addr, emmv5::zero_position(addr));
}

template <typename Uart>
Status DeviceEmmv5GroupT<Uart>::clear_protection(uint8_t addr)
{
  return _send_for(addr, emmv5::clear_protection(addr));
}

template <typename Uart>
Status DeviceEmmv5GroupT<Uart>::set_control_mode(uint8_t addr, bool store, uint8_t mode)
{
  if (mode > 3U)
  {
    return Status::BAD_ARG;
  }

  return _send_for(addr, emmv5::set_control_mode(addr, store, mode));
}

template <typename Uart>
Status DeviceEmmv5GroupT<Uart>::read(uint8_t addr, emmv5::SystemParam param)
{
  return _send_for(addr, emmv5::read(addr, param));
}

template <typename Uart>
Status DeviceEmmv5GroupT<Uart>::request_position(uint8_t addr)
{
  return read(addr, emmv5::SystemParam::CurrentPosition);
}

template <typename Uart>
Status DeviceEmmv5GroupT<Uart>::request_status(uint8_t addr)
{
  return read(addr, emmv5::SystemParam::StatusFlag);
}


// ----------------
// ---------------- MMCL 多机命令 ----------------

template <typename Uart>
const uint8_t *DeviceEmmv5GroupT<Uart>::last_multi_frame(size_t &len) const
{
  len = _last_frame_len;
  return _last_frame;
}

template <typename Uart>
void DeviceEmmv5GroupT<Uart>::multi_clear()
{
  // 清缓冲同样要排队：否则可能清掉别的任务刚攒进去的子命令
  if (lock_bus() == Status::OK)
  {
    _multi_payload = 0;
    unlock_bus();
  }
}

template <typename Uart>
Status DeviceEmmv5GroupT<Uart>::multi_send(uint8_t target)
{
  if (!_ready)
  {
    return Status::NOT_INIT;
  }

  if (_multi_payload == 0)
  {
    return Status::OK;
  }

  const size_t total = emmv5::build_multi_frame(target, _multi + emmv5::MULTI_PREFIX_SIZE, _multi_payload, _multi, sizeof(_multi));
  if (total == 0)
  {
    return Status::BAD_ARG; // 理论上不会发生：_queue_for 已经保证整帧放得下
  }

  // 组帧 + 发送 + 清缓冲必须是一个不可分割的总线事务
  const Status ls = lock_bus();
  if (ls != Status::OK)
  {
    return ls;
  }

  const Status status = _uart.send(_multi, total, pdMS_TO_TICKS(20U), nullptr);

  // 记下实际发出的字节，供控制台 dump 对账
  _last_frame_len = (total < sizeof(_last_frame)) ? total : sizeof(_last_frame);
  memcpy(_last_frame, _multi, _last_frame_len);

  // 成功就清空；超时可能已经有一部分字节进了流缓冲，不能把整帧再发一遍，
  // 否则总线上会出现“半帧 + 完整帧”。这里统一清空，由上层决定是否重建命令。
  _multi_payload = 0;

  unlock_bus();
  return status;
}

template <typename Uart>
Status DeviceEmmv5GroupT<Uart>::queue_enable(uint8_t addr, bool on)
{
  return _queue_for(addr, emmv5::enable(addr, on));
}

template <typename Uart>
Status DeviceEmmv5GroupT<Uart>::queue_velocity(uint8_t addr, emmv5::Direction dir, uint16_t rpm, uint8_t accel)
{
  if (!_valid_direction(dir) || !_valid_rpm(rpm))
  {
    return Status::BAD_ARG;
  }

  return _queue_for(addr, emmv5::velocity(addr, dir, rpm, accel));
}

template <typename Uart>
Status DeviceEmmv5GroupT<Uart>::queue_position(uint8_t addr, emmv5::Direction dir, uint16_t rpm, uint8_t accel, int32_t pulses, emmv5::PositionMode mode)
{
  if (!_valid_direction(dir) || !_valid_rpm(rpm))
  {
    return Status::BAD_ARG;
  }

  return _queue_for(addr, emmv5::position(addr, dir, rpm, accel, pulses, mode));
}

template <typename Uart>
Status DeviceEmmv5GroupT<Uart>::queue_quick_position_params(uint8_t addr, uint16_t rpm, uint8_t accel, emmv5::PositionMode mode)
{
  if (!_valid_rpm(rpm))
  {
    return Status::BAD_ARG;
  }

  return _queue_for(addr, emmv5::quick_position_params(addr, rpm, accel, mode));
}

template <typename Uart>
Status DeviceEmmv5GroupT<Uart>::queue_quick_position(uint8_t addr, int32_t pulses)
{
  return _queue_for(addr, emmv5::quick_position(addr, pulses));
}

template <typename Uart>
Status DeviceEmmv5GroupT<Uart>::queue_stop(uint8_t addr)
{
  return _queue_for(addr, emmv5::stop(addr));
}


// ----------------
// ---------------- 接收 ----------------

template <typename Uart>
uint32_t DeviceEmmv5GroupT<Uart>::update(uint32_t timeout_ms)
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

    // 只有第一块按调用方给的超时等；后续块一律 0，把已到的字节一次排空
    const uint32_t wait_ms = (chunk == 0U) ? timeout_ms : 0U;

    if (_uart.receive(buf, sizeof(buf), wait_ms, &n) != Status::OK || n == 0)
    {
      break; // 超时或暂时没有数据
    }

    hits |= feed_rx(buf, n);
  }

  return hits;
}

template <typename Uart>
uint32_t DeviceEmmv5GroupT<Uart>::feed_rx(const uint8_t *data, size_t size)
{
  if (!_ready || data == nullptr || size == 0U)
  {
    return 0;
  }

  uint32_t hits = 0;

  // 每台电机都有自己的三个扫描器；同一块字节交给全组扫一遍，
  // 由扫描器内部的“地址 + 功能码”条件决定归属，避免在设备层手写协议解析。
  for (size_t i = 0; i < _count; i++)
  {
    bool alive = false;

    if (_motor[i].inpos_scanner.feed(_motor[i].addr, data, size))
    {
      _publish_in_position(i);
      hits |= (1UL << i);
      alive = true;
    }

    int32_t position = 0;
    if (_motor[i].position_scanner.feed(_motor[i].addr, data, size, position))
    {
      _publish_position(i, position);
      alive = true;
    }

    uint8_t status = 0;
    if (_motor[i].status_scanner.feed(_motor[i].addr, data, size, status))
    {
      _publish_status(i, status);
      alive = true;
    }

    if (alive)
    {
      (void)_motor[i].online.refresh_task();
    }
  }

  return hits;
}

template <typename Uart>
bool DeviceEmmv5GroupT<Uart>::get_position(uint8_t addr, int32_t &out)
{
  if (!_ready)
  {
    return false;
  }

  const size_t index = index_of(addr);
  if (index >= _count)
  {
    return false;
  }

  taskENTER_CRITICAL();

  const bool valid = _motor[index].position_valid;
  if (valid)
  {
    out                          = _motor[index].position;
    _motor[index].position_valid = false;
  }

  taskEXIT_CRITICAL();
  return valid;
}

template <typename Uart>
bool DeviceEmmv5GroupT<Uart>::get_status(uint8_t addr, uint8_t &out)
{
  if (!_ready)
  {
    return false;
  }

  const size_t index = index_of(addr);
  if (index >= _count)
  {
    return false;
  }

  taskENTER_CRITICAL();

  const bool valid = _motor[index].status_valid;
  if (valid)
  {
    out                        = _motor[index].status;
    _motor[index].status_valid = false;
  }

  taskEXIT_CRITICAL();
  return valid;
}

template <typename Uart>
bool DeviceEmmv5GroupT<Uart>::is_online(uint8_t addr) const
{
  if (!_ready)
  {
    return false;
  }

  const size_t index = index_of(addr);
  if (index >= _count)
  {
    return false;
  }

  return _motor[index].online.is_online() == Status::OK;
}

template <typename Uart>
void DeviceEmmv5GroupT<Uart>::_publish_position(size_t index, int32_t position)
{
  // 解析在调用 update() 的任务，取数可能在另一个任务。用短临界区保证“值 + 有效标志”成对可见
  taskENTER_CRITICAL();
  _motor[index].position       = position;
  _motor[index].position_valid = true;
  taskEXIT_CRITICAL();
}

template <typename Uart>
void DeviceEmmv5GroupT<Uart>::_publish_status(size_t index, uint8_t status)
{
  taskENTER_CRITICAL();
  _motor[index].status       = status;
  _motor[index].status_valid = true;
  taskEXIT_CRITICAL();
}

template <typename Uart>
void DeviceEmmv5GroupT<Uart>::_publish_in_position(size_t index)
{
  taskENTER_CRITICAL();
  _motor[index].in_position = true;
  taskEXIT_CRITICAL();
}

template <typename Uart>
bool DeviceEmmv5GroupT<Uart>::_valid_rpm(uint16_t rpm)
{
  return rpm <= 5000U;
}

template <typename Uart>
bool DeviceEmmv5GroupT<Uart>::_valid_direction(emmv5::Direction dir)
{
  return (dir == emmv5::Direction::CW) || (dir == emmv5::Direction::CCW);
}

template <typename Uart>
bool DeviceEmmv5GroupT<Uart>::in_position(size_t index) const
{
  if (!_ready || index >= _count)
  {
    return false;
  }

  taskENTER_CRITICAL();
  const bool value = _motor[index].in_position;
  taskEXIT_CRITICAL();
  return value;
}

template <typename Uart>
uint32_t DeviceEmmv5GroupT<Uart>::get_in_position()
{
  if (!_ready)
  {
    return 0;
  }

  uint32_t mask = 0;

  // 接收任务写、控制任务读并清零：必须和 _publish_in_position() 用同一把短临界区
  taskENTER_CRITICAL();
  for (size_t i = 0; i < _count; i++)
  {
    if (_motor[i].in_position)
    {
      mask |= (1UL << i);
      _motor[i].in_position = false;
    }
  }
  taskEXIT_CRITICAL();

  return mask;
}

// ----------------
