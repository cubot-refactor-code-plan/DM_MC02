#include "device_emmv5.hpp"

#include "FreeRTOS.h" // IWYU pragma: keep（pdMS_TO_TICKS / taskENTER_CRITICAL）

// ---------------- 模板实例化 ----------------

// 显式实例化：必须覆盖 device_cfg.cpp 里用到的串口类型，否则链接时找不到成员函数
template class DeviceEmmv5T<BspUartT<uart_detail::DmaTransport, 128, 128>>;
template class DeviceEmmv5T<BspUartT<uart_detail::ItTransport, 128, 32>>;

// ----------------
// ---------------- 构造与生命周期 ----------------

template <typename Uart>
DeviceEmmv5T<Uart>::DeviceEmmv5T(const Config &cfg)
  : _uart(cfg.uart),
    _addr(cfg.addr),
    _online(ONLINE_TIMEOUT_MS)
{
  // 构造函数只做赋值；地址校验和状态复位在 init() 里
}

template <typename Uart>
Status DeviceEmmv5T<Uart>::init()
{
  // 0 是广播地址：实例带广播地址会把别人的到位帧也算成自己的，必须拦住
  if (_addr == 0)
  {
    return Status::BAD_ARG;
  }

  _inpos_scanner.reset();
  _position_scanner.reset();
  _status_scanner.reset();
  _position_valid = false;
  _status_valid   = false;
  _ready          = true;

  // 每个实例在 init() 里创建自己的内部 RX 任务；重复 init 不会重复创建。
  // 先置 _ready 再创建任务：万一调度器已在运行，任务第一拍就能正常收包。
  if (_task_handle == nullptr)
  {
    if (xTaskCreate(_rx_task_entry, "emmv5", RX_TASK_STACK_WORDS, this, RX_TASK_PRIORITY, &_task_handle) != pdPASS)
    {
      _task_handle = nullptr;
      _ready       = false;
      return Status::FULL; // 堆不够或 TCB 分配失败
    }
  }

  return Status::OK;
}

template <typename Uart>
DeviceEmmv5T<Uart>::~DeviceEmmv5T()
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
 * @note 这是 DeviceEmmv5T<Uart> 独有的入口地址；DeviceEmmv5T<Uart>Group 用自己的入口函数。
 *       同类多实例共享入口地址，但 this 不同，各自有独立 TCB 和栈。
 */
template <typename Uart>
void DeviceEmmv5T<Uart>::_rx_task_entry(void *argument)
{
  auto *self = static_cast<DeviceEmmv5T<Uart> *>(argument);

  if (self != nullptr)
  {
    self->_run_rx_loop();
  }

  vTaskDelete(nullptr); // 正常不会走到；参数为空时自删，避免空任务
}

/**
 * @brief 内部接收循环：固定 10 ms 调一次 update()
 */
template <typename Uart>
void DeviceEmmv5T<Uart>::_run_rx_loop()
{
  TickType_t wake_time = xTaskGetTickCount();

  for (;;)
  {
    (void)update();
    vTaskDelayUntil(&wake_time, pdMS_TO_TICKS(RX_TASK_PERIOD_MS));
  }
}


// ----------------
// ---------------- 发送 ----------------

template <typename Uart>
Status DeviceEmmv5T<Uart>::_send(const emmv5::Frame &frame)
{
  if (!_ready)
  {
    return Status::NOT_INIT;
  }

  // 协议层在非法枚举/超长时会返回未封口的 Frame，这里统一挡掉
  if (!frame.valid())
  {
    return Status::BAD_ARG;
  }

  // 20 ms 足够一帧短命令入队；缓冲卡死时把 TIMEOUT 交给调用方，不无限挂起任务
  return _uart.send(frame.data, frame.size, pdMS_TO_TICKS(20U), nullptr);
}


// ----------------
// ---------------- 常用命令 ----------------

template <typename Uart>
Status DeviceEmmv5T<Uart>::enable(bool on)
{
  return _send(emmv5::enable(_addr, on));
}

template <typename Uart>
Status DeviceEmmv5T<Uart>::set_velocity(emmv5::Direction dir, uint16_t rpm, uint8_t accel)
{
  if (!_valid_direction(dir) || !_valid_rpm(rpm))
  {
    return Status::BAD_ARG;
  }

  return _send(emmv5::velocity(_addr, dir, rpm, accel));
}

template <typename Uart>
Status DeviceEmmv5T<Uart>::set_position(emmv5::Direction dir, uint16_t rpm, uint8_t accel, int32_t pulses, emmv5::PositionMode mode)
{
  if (!_valid_direction(dir) || !_valid_rpm(rpm))
  {
    return Status::BAD_ARG;
  }

  return _send(emmv5::position(_addr, dir, rpm, accel, pulses, mode));
}

template <typename Uart>
Status DeviceEmmv5T<Uart>::set_quick_position_params(uint16_t rpm, uint8_t accel, emmv5::PositionMode mode)
{
  if (!_valid_rpm(rpm))
  {
    return Status::BAD_ARG;
  }

  return _send(emmv5::quick_position_params(_addr, rpm, accel, mode));
}

template <typename Uart>
Status DeviceEmmv5T<Uart>::set_quick_position(int32_t pulses)
{
  return _send(emmv5::quick_position(_addr, pulses));
}

template <typename Uart>
Status DeviceEmmv5T<Uart>::stop()
{
  return _send(emmv5::stop(_addr));
}

template <typename Uart>
Status DeviceEmmv5T<Uart>::zero_position()
{
  return _send(emmv5::zero_position(_addr));
}

template <typename Uart>
Status DeviceEmmv5T<Uart>::read(emmv5::SystemParam param)
{
  return _send(emmv5::read(_addr, param));
}

template <typename Uart>
Status DeviceEmmv5T<Uart>::request_position()
{
  return read(emmv5::SystemParam::CurrentPosition);
}

template <typename Uart>
Status DeviceEmmv5T<Uart>::request_status()
{
  return read(emmv5::SystemParam::StatusFlag);
}

// ----------------
// ---------------- 接收 ----------------

template <typename Uart>
bool DeviceEmmv5T<Uart>::update()
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

template <typename Uart>
bool DeviceEmmv5T<Uart>::feed_rx(const uint8_t *data, size_t size)
{
  if (!_ready || data == nullptr || size == 0U)
  {
    return false;
  }

  bool alive = false;

  // 1) 到位帧：保留原有语义，返回值只反映它
  const bool in_position = _inpos_scanner.feed(_addr, data, size);
  alive                  = alive || in_position;

  // 2) 0x36 位置应答：锁存最新值，等控制任务来取
  int32_t position = 0;
  if (_position_scanner.feed(_addr, data, size, position))
  {
    _publish_position(position);
    alive = true;
  }

  // 3) 0x3A 状态应答：同样只锁存，不在接收上下文里做业务判断
  uint8_t status = 0;
  if (_status_scanner.feed(_addr, data, size, status))
  {
    _publish_status(status);
    alive = true;
  }

  // 只要收到任何一类有效应答，就认为链路还活着
  if (alive)
  {
    (void)_online.refresh_task();
  }

  return in_position;
}

template <typename Uart>
bool DeviceEmmv5T<Uart>::get_position(int32_t &out)
{
  taskENTER_CRITICAL();

  const bool valid = _position_valid;
  if (valid)
  {
    out             = _position;
    _position_valid = false;
  }

  taskEXIT_CRITICAL();
  return valid;
}

template <typename Uart>
bool DeviceEmmv5T<Uart>::get_status(uint8_t &out)
{
  taskENTER_CRITICAL();

  const bool valid = _status_valid;
  if (valid)
  {
    out           = _status;
    _status_valid = false;
  }

  taskEXIT_CRITICAL();
  return valid;
}

template <typename Uart>
bool DeviceEmmv5T<Uart>::is_online() const
{
  return _online.is_online() == Status::OK;
}

template <typename Uart>
void DeviceEmmv5T<Uart>::_publish_position(int32_t position)
{
  taskENTER_CRITICAL();
  _position       = position;
  _position_valid = true;
  taskEXIT_CRITICAL();
}

template <typename Uart>
void DeviceEmmv5T<Uart>::_publish_status(uint8_t status)
{
  taskENTER_CRITICAL();
  _status       = status;
  _status_valid = true;
  taskEXIT_CRITICAL();
}

template <typename Uart>
bool DeviceEmmv5T<Uart>::_valid_rpm(uint16_t rpm)
{
  return rpm <= 5000U;
}

template <typename Uart>
bool DeviceEmmv5T<Uart>::_valid_direction(emmv5::Direction dir)
{
  return (dir == emmv5::Direction::CW) || (dir == emmv5::Direction::CCW);
}

// ----------------
