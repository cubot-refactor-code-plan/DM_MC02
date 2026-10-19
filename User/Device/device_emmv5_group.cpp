#include "device_emmv5_group.hpp"

#include "FreeRTOS.h" // IWYU pragma: keep（pdMS_TO_TICKS / taskENTER_CRITICAL）

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
  _ready         = true;

  // 每个组实例在 init() 里创建自己的内部 RX 任务；重复 init 不会重复创建。
  // 先置 _ready 再创建任务：万一调度器已在运行，任务第一拍就能正常收包。
  if (_task_handle == nullptr)
  {
    if (xTaskCreate(_rx_task_entry, "emmv5g", RX_TASK_STACK_WORDS, this, RX_TASK_PRIORITY, &_task_handle) != pdPASS)
    {
      _task_handle = nullptr;
      _ready       = false;
      return Status::FULL; // 堆不够或 TCB 分配失败
    }
  }

  return Status::OK;
}

DeviceEmmv5Group::~DeviceEmmv5Group()
{
  // 全局/静态对象通常不会走到这里；显式销毁时先把内部任务停掉，避免悬空 this
  if (_task_handle != nullptr)
  {
    vTaskDelete(_task_handle);
    _task_handle = nullptr;
  }
}

/**
 * @brief 内部 RX 任务入口
 *
 * @note 这是 DeviceEmmv5Group 独有的入口地址；DeviceEmmv5 用自己的入口函数。
 *       同类多实例共享入口地址，但 this 不同，各自有独立 TCB 和栈。
 */
void DeviceEmmv5Group::_rx_task_entry(void *argument)
{
  auto *self = static_cast<DeviceEmmv5Group *>(argument);

  if (self != nullptr)
  {
    self->_run_rx_loop();
  }

  vTaskDelete(nullptr); // 正常不会走到；参数为空时自删，避免空任务
}

/**
 * @brief 内部接收循环：固定 10 ms 调一次 update()
 */
void DeviceEmmv5Group::_run_rx_loop()
{
  TickType_t wake_time = xTaskGetTickCount();

  for (;;)
  {
    (void)update();
    vTaskDelayUntil(&wake_time, pdMS_TO_TICKS(RX_TASK_PERIOD_MS));
  }
}


// ----------------
// ---------------- 私有查找与发送 ----------------

size_t DeviceEmmv5Group::index_of(uint8_t addr) const
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

  if (!_is_member(addr) || !frame.valid())
  {
    return Status::BAD_ARG;
  }

  // 20 ms 入队超时：短帧足够；缓冲卡死时返回 TIMEOUT，不无限挂住调用任务
  return _uart.send(frame.data, frame.size, pdMS_TO_TICKS(20U), nullptr);
}

Status DeviceEmmv5Group::_queue_for(uint8_t addr, const emmv5::Frame &frame)
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

  memcpy(_multi + emmv5::MULTI_PREFIX_SIZE + _multi_payload, frame.data, frame.size);
  _multi_payload += frame.size;
  return Status::OK;
}


// ----------------
// ---------------- 直接命令 ----------------

Status DeviceEmmv5Group::enable(uint8_t addr, bool on)
{
  return _send_for(addr, emmv5::enable(addr, on));
}

Status DeviceEmmv5Group::set_velocity(uint8_t addr, emmv5::Direction dir, uint16_t rpm, uint8_t accel)
{
  if (!_valid_direction(dir) || !_valid_rpm(rpm))
  {
    return Status::BAD_ARG;
  }

  return _send_for(addr, emmv5::velocity(addr, dir, rpm, accel));
}

Status DeviceEmmv5Group::set_position(uint8_t addr, emmv5::Direction dir, uint16_t rpm, uint8_t accel, int32_t pulses, emmv5::PositionMode mode)
{
  if (!_valid_direction(dir) || !_valid_rpm(rpm))
  {
    return Status::BAD_ARG;
  }

  return _send_for(addr, emmv5::position(addr, dir, rpm, accel, pulses, mode));
}

Status DeviceEmmv5Group::set_quick_position_params(uint8_t addr, uint16_t rpm, uint8_t accel, emmv5::PositionMode mode)
{
  if (!_valid_rpm(rpm))
  {
    return Status::BAD_ARG;
  }

  return _send_for(addr, emmv5::quick_position_params(addr, rpm, accel, mode));
}

Status DeviceEmmv5Group::set_quick_position(uint8_t addr, int32_t pulses)
{
  return _send_for(addr, emmv5::quick_position(addr, pulses));
}

Status DeviceEmmv5Group::stop(uint8_t addr)
{
  return _send_for(addr, emmv5::stop(addr));
}

Status DeviceEmmv5Group::stop_all()
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

  return _uart.send(frame.data, frame.size, pdMS_TO_TICKS(20U), nullptr);
}

Status DeviceEmmv5Group::home(uint8_t addr, emmv5::HomingMode mode)
{
  if (!_valid_homing(mode))
  {
    return Status::BAD_ARG;
  }

  return _send_for(addr, emmv5::home(addr, mode));
}

Status DeviceEmmv5Group::zero_position(uint8_t addr)
{
  return _send_for(addr, emmv5::zero_position(addr));
}

Status DeviceEmmv5Group::calibrate_encoder(uint8_t addr)
{
  return _send_for(addr, emmv5::calibrate_encoder(addr));
}

Status DeviceEmmv5Group::clear_stall(uint8_t addr)
{
  return _send_for(addr, emmv5::clear_stall(addr));
}

Status DeviceEmmv5Group::reset(uint8_t addr)
{
  return _send_for(addr, emmv5::reset(addr));
}

Status DeviceEmmv5Group::restore_defaults(uint8_t addr)
{
  return _send_for(addr, emmv5::restore_defaults(addr));
}

Status DeviceEmmv5Group::abort_home(uint8_t addr)
{
  return _send_for(addr, emmv5::abort_home(addr));
}

Status DeviceEmmv5Group::set_microstep(uint8_t addr, bool store, uint8_t microstep)
{
  return _send_for(addr, emmv5::set_microstep(addr, store, microstep));
}

Status DeviceEmmv5Group::set_motor_direction(uint8_t addr, bool store, emmv5::Direction dir)
{
  if (!_valid_direction(dir))
  {
    return Status::BAD_ARG;
  }

  return _send_for(addr, emmv5::set_motor_direction(addr, store, dir));
}

Status DeviceEmmv5Group::set_control_mode(uint8_t addr, bool store, emmv5::ControlMode mode)
{
  if (!_valid_control_mode(mode))
  {
    return Status::BAD_ARG;
  }

  return _send_for(addr, emmv5::set_control_mode(addr, store, mode));
}

Status DeviceEmmv5Group::set_position_window(uint8_t addr, bool store, uint16_t window)
{
  return _send_for(addr, emmv5::set_position_window(addr, store, window));
}

Status DeviceEmmv5Group::set_protection(uint8_t addr, bool store, uint16_t over_temp_c, uint16_t over_current_ma, uint16_t detect_ms)
{
  const emmv5::ProtectionParams params(over_temp_c, over_current_ma, detect_ms, store);
  return _send_for(addr, emmv5::set_protection(addr, params));
}

Status DeviceEmmv5Group::read(uint8_t addr, emmv5::SystemParam param)
{
  return _send_for(addr, emmv5::read(addr, param));
}

Status DeviceEmmv5Group::request_position(uint8_t addr)
{
  return read(addr, emmv5::SystemParam::CurrentPosition);
}

Status DeviceEmmv5Group::request_status(uint8_t addr)
{
  return read(addr, emmv5::SystemParam::StatusFlag);
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

  const size_t total = emmv5::build_multi_frame(target, _multi + emmv5::MULTI_PREFIX_SIZE, _multi_payload, _multi, sizeof(_multi));
  if (total == 0)
  {
    return Status::BAD_ARG; // 理论上不会发生：_queue_for 已经保证整帧放得下
  }

  const Status status = _uart.send(_multi, total, pdMS_TO_TICKS(20U), nullptr);

  // 成功就清空；超时可能已经有一部分字节进了流缓冲，不能把整帧再发一遍，
  // 否则总线上会出现“半帧 + 完整帧”。这里统一清空，由上层决定是否重建命令。
  _multi_payload = 0;
  return status;
}

Status DeviceEmmv5Group::queue_enable(uint8_t addr, bool on)
{
  return _queue_for(addr, emmv5::enable(addr, on));
}

Status DeviceEmmv5Group::queue_velocity(uint8_t addr, emmv5::Direction dir, uint16_t rpm, uint8_t accel)
{
  if (!_valid_direction(dir) || !_valid_rpm(rpm))
  {
    return Status::BAD_ARG;
  }

  return _queue_for(addr, emmv5::velocity(addr, dir, rpm, accel));
}

Status DeviceEmmv5Group::queue_position(uint8_t addr, emmv5::Direction dir, uint16_t rpm, uint8_t accel, int32_t pulses, emmv5::PositionMode mode)
{
  if (!_valid_direction(dir) || !_valid_rpm(rpm))
  {
    return Status::BAD_ARG;
  }

  return _queue_for(addr, emmv5::position(addr, dir, rpm, accel, pulses, mode));
}

Status DeviceEmmv5Group::queue_quick_position_params(uint8_t addr, uint16_t rpm, uint8_t accel, emmv5::PositionMode mode)
{
  if (!_valid_rpm(rpm))
  {
    return Status::BAD_ARG;
  }

  return _queue_for(addr, emmv5::quick_position_params(addr, rpm, accel, mode));
}

Status DeviceEmmv5Group::queue_quick_position(uint8_t addr, int32_t pulses)
{
  return _queue_for(addr, emmv5::quick_position(addr, pulses));
}

Status DeviceEmmv5Group::queue_stop(uint8_t addr)
{
  return _queue_for(addr, emmv5::stop(addr));
}

Status DeviceEmmv5Group::queue_home(uint8_t addr, emmv5::HomingMode mode)
{
  if (!_valid_homing(mode))
  {
    return Status::BAD_ARG;
  }

  return _queue_for(addr, emmv5::home(addr, mode));
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
  if (!_ready || data == nullptr || size == 0U)
  {
    return 0;
  }

  uint32_t hits = 0;

  for (size_t i = 0; i < _count; i++)
  {
    bool alive = false;

    if (_motor[i].inpos_scanner.feed(_motor[i].addr, data, size))
    {
      _motor[i].in_position = true;
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

bool DeviceEmmv5Group::take_position(uint8_t addr, int32_t &out)
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
    out                              = _motor[index].position;
    _motor[index].position_valid     = false;
  }

  taskEXIT_CRITICAL();
  return valid;
}

bool DeviceEmmv5Group::take_status(uint8_t addr, uint8_t &out)
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
    out                          = _motor[index].status;
    _motor[index].status_valid   = false;
  }

  taskEXIT_CRITICAL();
  return valid;
}

bool DeviceEmmv5Group::is_online(uint8_t addr) const
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

void DeviceEmmv5Group::_publish_position(size_t index, int32_t position)
{
  // 接收在内部 RX 任务；取数可能在控制任务。用短临界区保证“值 + 有效标志”成对可见
  taskENTER_CRITICAL();
  _motor[index].position       = position;
  _motor[index].position_valid = true;
  taskEXIT_CRITICAL();
}

void DeviceEmmv5Group::_publish_status(size_t index, uint8_t status)
{
  taskENTER_CRITICAL();
  _motor[index].status       = status;
  _motor[index].status_valid = true;
  taskEXIT_CRITICAL();
}

bool DeviceEmmv5Group::_valid_rpm(uint16_t rpm)
{
  return rpm <= 5000U;
}

bool DeviceEmmv5Group::_valid_direction(emmv5::Direction dir)
{
  return (dir == emmv5::Direction::CW) || (dir == emmv5::Direction::CCW);
}

bool DeviceEmmv5Group::_valid_homing(emmv5::HomingMode mode)
{
  return static_cast<uint8_t>(mode) <= 3U;
}

bool DeviceEmmv5Group::_valid_control_mode(emmv5::ControlMode mode)
{
  return static_cast<uint8_t>(mode) <= 3U;
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
