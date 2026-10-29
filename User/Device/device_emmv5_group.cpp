#include "device_emmv5_group.hpp"

#include "FreeRTOS.h" // IWYU pragma: keep（pdMS_TO_TICKS / taskENTER_CRITICAL）

#include <string.h> // memcpy

// ---------------- 模板实例化 ----------------

// 显式实例化：必须覆盖 device_cfg.cpp 里用到的串口类型，否则链接时找不到成员函数
template class DeviceEmmv5GroupT<BspUartT<uart_detail::DmaTransport, 128, 128>>;

// ----------------
// ---------------- 构造与生命周期 ----------------

/** @brief 只保存串口引用与地址表，地址个数由地址表的 0 结尾推导 */
template <typename Uart>
DeviceEmmv5GroupT<Uart>::DeviceEmmv5GroupT(const Config &cfg) : _uart(cfg.uart)
{
  // 只把地址表拷进来；重复 / 为 0 的地址交给 init() 拒绝，
  // 总线锁也放到 init() 里建（构造函数里不碰 FreeRTOS API）。
  for (size_t i = 0; i < MAX_MOTORS; i++)
  {
    _motor[i].addr = cfg.addr[i];
  }

  // 地址 0 不是合法地址，所以「前导非 0」就是组内电机数；MAX_MOTORS 天然封顶
  _count = 0;
  while (_count < MAX_MOTORS && cfg.addr[_count] != 0U)
  {
    ++_count;
  }
}

/** @brief 校验地址表、复位接收状态与 MMCL 缓冲，并创建总线锁 */
template <typename Uart>
Status DeviceEmmv5GroupT<Uart>::init()
{
  if (_count == 0)
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

  // 各电机的复位值与 3 个协议扫描器的初值全部在 device_emmv5_group.hpp / emmv5_protocol.hpp
  // 的成员声明里（NSDMI），这里不再逐字段清一遍

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

/** @brief 释放总线锁（全局实例不会走到这里，只为测试 / 局部对象不泄漏） */
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

/** @brief 地址 → 下标；不在组内返回 _count */
template <typename Uart>
size_t DeviceEmmv5GroupT<Uart>::index_of(uint8_t addr) const
{
  // 约定：找不到返回 _count，调用方一律用 `index >= _count` 判非法。
  for (size_t i = 0; i < _count; i++)
  {
    if (_motor[i].addr == addr)
    {
      return i;
    }
  }

  return _count;
}

/** @brief 地址是否在组内 */
template <typename Uart>
bool DeviceEmmv5GroupT<Uart>::_is_member(uint8_t addr) const
{
  return index_of(addr) < _count;
}


// ----------------
// ---------------- 总线互斥 ----------------

/** @brief 取半双工总线锁（递归；timeout_ms = 0 相当于 try-lock） */
template <typename Uart>
Status DeviceEmmv5GroupT<Uart>::lock_bus(uint32_t timeout_ms)
{
  if (!_ready || (_bus_lock == nullptr))
  {
    return Status::NOT_INIT;
  }

  // timeout_ms = 0 相当于 try-lock；递归锁让同一任务可以层层嵌套而不会自锁
  if (xSemaphoreTakeRecursive(_bus_lock, pdMS_TO_TICKS(timeout_ms)) != pdTRUE)
  {
    return Status::TIMEOUT;
  }

  return Status::OK;
}

/** @brief 释放总线锁，必须与 lock_bus() 成对 */
template <typename Uart>
void DeviceEmmv5GroupT<Uart>::unlock_bus()
{
  // 与 lock_bus() 成对，必须同任务调用；多给/少给会让递归计数错位
  if (_bus_lock != nullptr)
  {
    (void)xSemaphoreGiveRecursive(_bus_lock);
  }
}

/** @brief 直接发一帧：校验地址与帧有效性 → 加锁 → 入队 → 解锁 */
template <typename Uart>
Status DeviceEmmv5GroupT<Uart>::_send_for(uint8_t addr, const emmv5::Frame &frame)
{
  if (!_ready)
  {
    return Status::NOT_INIT;
  }

  if (!_is_member(addr) || !frame.valid())
  {
    // frame 有效性由协议层保证：未封口 / 已溢出 / 空帧一律不发，避免把截断帧送上总线
    return Status::BAD_ARG;
  }

  // 半双工：整帧发送期间独占总线，其他任务不得插入任何字节
  const Status ls = lock_bus();
  if (ls != Status::OK)
  {
    return ls;
  }

  // 短帧入队：正常几 ms 内就排空，卡死时靠 TX_TIMEOUT_MS 退出
  const Status status = _uart.send(frame.data, frame.size, pdMS_TO_TICKS(TX_TIMEOUT_MS), nullptr);

  unlock_bus();
  return status;
}

/** @brief 把一帧追加到 MMCL 缓冲；放不下返回 FULL，不留半条子命令 */
template <typename Uart>
Status DeviceEmmv5GroupT<Uart>::_queue_for(uint8_t addr, const emmv5::Frame &frame)
{
  if (!_ready)
  {
    return Status::NOT_INIT;
  }

  if (!_is_member(addr) || !frame.valid())
  {
    // 截断帧不能攒进总帧：否则整帧 MMCL 会带着半条子命令发出去
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

/** @brief 使能 / 关闭一台电机（无应答命令） */
template <typename Uart>
Status DeviceEmmv5GroupT<Uart>::enable(uint8_t addr, bool on)
{
  return _send_for(addr, emmv5::enable(addr, on));
}

/** @brief 速度模式 */
template <typename Uart>
Status DeviceEmmv5GroupT<Uart>::set_velocity(uint8_t addr, emmv5::Direction dir, uint16_t rpm, uint8_t accel)
{
  if (!_valid_direction(dir) || !_valid_rpm(rpm))
  {
    return Status::BAD_ARG;
  }

  return _send_for(addr, emmv5::velocity(addr, dir, rpm, accel));
}

/** @brief 位置模式（方向字节 + 正幅值；相对/绝对由 mode 决定） */
template <typename Uart>
Status DeviceEmmv5GroupT<Uart>::set_position(uint8_t addr, emmv5::Direction dir, uint16_t rpm, uint8_t accel, int32_t pulses, emmv5::PositionMode mode)
{
  if (!_valid_direction(dir) || !_valid_rpm(rpm))
  {
    return Status::BAD_ARG;
  }

  return _send_for(addr, emmv5::position(addr, dir, rpm, accel, pulses, mode));
}

/** @brief 只设快速位置参数，不产生运动 */
template <typename Uart>
Status DeviceEmmv5GroupT<Uart>::set_quick_position_params(uint8_t addr, uint16_t rpm, uint8_t accel, emmv5::PositionMode mode)
{
  if (!_valid_rpm(rpm))
  {
    return Status::BAD_ARG;
  }

  return _send_for(addr, emmv5::quick_position_params(addr, rpm, accel, mode));
}

/** @brief 快速位置（没有方向字节，方向靠脉冲符号） */
template <typename Uart>
Status DeviceEmmv5GroupT<Uart>::set_quick_position(uint8_t addr, int32_t pulses)
{
  return _send_for(addr, emmv5::quick_position(addr, pulses));
}

/** @brief 立即停止一台电机 */
template <typename Uart>
Status DeviceEmmv5GroupT<Uart>::stop(uint8_t addr)
{
  return _send_for(addr, emmv5::stop(addr));
}

/** @brief 广播立即停止（地址 0） */
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

  const Status status = _uart.send(frame.data, frame.size, pdMS_TO_TICKS(TX_TIMEOUT_MS), nullptr);

  unlock_bus();
  return status;
}

/** @brief 当前位置清零（编码器零位 = 上电位置） */
template <typename Uart>
Status DeviceEmmv5GroupT<Uart>::zero_position(uint8_t addr)
{
  return _send_for(addr, emmv5::zero_position(addr));
}

/** @brief 解除堵转保护 */
template <typename Uart>
Status DeviceEmmv5GroupT<Uart>::clear_protection(uint8_t addr)
{
  return _send_for(addr, emmv5::clear_protection(addr));
}

/** @brief 改控制模式（0 关闭 / 1 开环 / 2 闭环 / 3 脉冲方向） */
template <typename Uart>
Status DeviceEmmv5GroupT<Uart>::set_control_mode(uint8_t addr, bool store, uint8_t mode)
{
  if (mode > 3U)
  {
    return Status::BAD_ARG;
  }

  return _send_for(addr, emmv5::set_control_mode(addr, store, mode));
}

/** @brief 读一次系统参数（有应答，由 update() 解析） */
template <typename Uart>
Status DeviceEmmv5GroupT<Uart>::read(uint8_t addr, emmv5::SystemParam param)
{
  return _send_for(addr, emmv5::read(addr, param));
}

/** @brief 请求一次 0x36 实时位置 */
template <typename Uart>
Status DeviceEmmv5GroupT<Uart>::request_position(uint8_t addr)
{
  return read(addr, emmv5::SystemParam::CurrentPosition);
}

/** @brief 请求一次 0x3A 状态标志 */
template <typename Uart>
Status DeviceEmmv5GroupT<Uart>::request_status(uint8_t addr)
{
  return read(addr, emmv5::SystemParam::StatusFlag);
}


// ----------------
// ---------------- MMCL 多机命令 ----------------

/** @brief 取最近一次发出的 MMCL 字节（诊断用，长度写入 len） */
template <typename Uart>
const uint8_t *DeviceEmmv5GroupT<Uart>::last_multi_frame(size_t &len) const
{
  len = _last_frame_len;
  return _last_frame;
}

/** @brief 清空 MMCL 缓冲 */
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

/** @brief 把缓冲封成 MMCL 总帧并发送；无论成败都清空缓冲 */
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

  const Status status = _uart.send(_multi, total, pdMS_TO_TICKS(TX_TIMEOUT_MS), nullptr);

  // 记下实际发出的字节，供控制台 dump 对账
  _last_frame_len = (total < sizeof(_last_frame)) ? total : sizeof(_last_frame);
  memcpy(_last_frame, _multi, _last_frame_len);

  // 成功就清空；超时可能已经有一部分字节进了流缓冲，不能把整帧再发一遍，
  // 否则总线上会出现“半帧 + 完整帧”。这里统一清空，由上层决定是否重建命令。
  _multi_payload = 0;

  unlock_bus();
  return status;
}

/** @brief 向 MMCL 缓冲追加「使能」子命令 */
template <typename Uart>
Status DeviceEmmv5GroupT<Uart>::queue_enable(uint8_t addr, bool on)
{
  return _queue_for(addr, emmv5::enable(addr, on));
}

/** @brief 追加「速度模式」子命令 */
template <typename Uart>
Status DeviceEmmv5GroupT<Uart>::queue_velocity(uint8_t addr, emmv5::Direction dir, uint16_t rpm, uint8_t accel)
{
  if (!_valid_direction(dir) || !_valid_rpm(rpm))
  {
    return Status::BAD_ARG;
  }

  return _queue_for(addr, emmv5::velocity(addr, dir, rpm, accel));
}

/** @brief 追加「位置模式」子命令 */
template <typename Uart>
Status DeviceEmmv5GroupT<Uart>::queue_position(uint8_t addr, emmv5::Direction dir, uint16_t rpm, uint8_t accel, int32_t pulses, emmv5::PositionMode mode)
{
  if (!_valid_direction(dir) || !_valid_rpm(rpm))
  {
    return Status::BAD_ARG;
  }

  return _queue_for(addr, emmv5::position(addr, dir, rpm, accel, pulses, mode));
}

/** @brief 追加「快速位置参数」子命令 */
template <typename Uart>
Status DeviceEmmv5GroupT<Uart>::queue_quick_position_params(uint8_t addr, uint16_t rpm, uint8_t accel, emmv5::PositionMode mode)
{
  if (!_valid_rpm(rpm))
  {
    return Status::BAD_ARG;
  }

  return _queue_for(addr, emmv5::quick_position_params(addr, rpm, accel, mode));
}

/** @brief 追加「快速位置」子命令 */
template <typename Uart>
Status DeviceEmmv5GroupT<Uart>::queue_quick_position(uint8_t addr, int32_t pulses)
{
  return _queue_for(addr, emmv5::quick_position(addr, pulses));
}

/** @brief 追加「立即停止」子命令 */
template <typename Uart>
Status DeviceEmmv5GroupT<Uart>::queue_stop(uint8_t addr)
{
  return _queue_for(addr, emmv5::stop(addr));
}


// ----------------
// ---------------- 接收 ----------------

/** @brief 读串口字节并按地址分发；返回本次命中到位帧的电机位图 */
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
  // （一问一答的场景下每拍最多两块：0x36 应答 8 B + 可能的 0x3A 应答 4 B）
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

/** @brief 一块字节喂给组内每台电机的三个扫描器，并刷新在线计时 */
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
      // 任何一类有效应答都算在线：只看 0x36 会在只查状态的阶段把电机误判成离线
      (void)_motor[i].online.refresh_task();
    }
  }

  return hits;
}

/** @brief 取走一台最近一次的 0x36 位置（读后清标志） */
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

  // 读后清标志：同一条位置应答只能被取走一次，否则调用方会把同一个旧值当新数据
  const bool valid = _motor[index].position_valid;
  if (valid)
  {
    out                          = _motor[index].position;
    _motor[index].position_valid = false;
  }

  taskEXIT_CRITICAL();
  return valid;
}

/** @brief 取走一台最近一次的 0x3A 状态（读后清标志） */
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

  // 同理：状态字节也只交付一次
  const bool valid = _motor[index].status_valid;
  if (valid)
  {
    out                        = _motor[index].status;
    _motor[index].status_valid = false;
  }

  taskEXIT_CRITICAL();
  return valid;
}

/** @brief 最近 ONLINE_TIMEOUT_MS 内是否收到过有效应答 */
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

  // 计时由 sys_task 里的 Online::update() 推进，这里只查锁存结果；
  // 所以本类不建任务，但要求调用方的刷新周期小于 ONLINE_TIMEOUT_MS。
  return _motor[index].online.is_online() == Status::OK;
}

/** @brief 锁存位置并置有效标志（短临界区） */
template <typename Uart>
void DeviceEmmv5GroupT<Uart>::_publish_position(size_t index, int32_t position)
{
  // 解析在调用 update() 的任务，取数可能在另一个任务。用短临界区保证“值 + 有效标志”成对可见
  taskENTER_CRITICAL();
  _motor[index].position       = position;
  _motor[index].position_valid = true;
  taskEXIT_CRITICAL();
}

/** @brief 锁存状态并置有效标志（短临界区） */
template <typename Uart>
void DeviceEmmv5GroupT<Uart>::_publish_status(size_t index, uint8_t status)
{
  taskENTER_CRITICAL();
  _motor[index].status       = status;
  _motor[index].status_valid = true;
  taskEXIT_CRITICAL();
}

/** @brief 置累计到位标志（短临界区） */
template <typename Uart>
void DeviceEmmv5GroupT<Uart>::_publish_in_position(size_t index)
{
  taskENTER_CRITICAL();
  _motor[index].in_position = true;
  taskEXIT_CRITICAL();
}

/** @brief rpm 是否在说明书范围内（0~5000） */
template <typename Uart>
bool DeviceEmmv5GroupT<Uart>::_valid_rpm(uint16_t rpm)
{
  // 0~5000 是说明书给的范围；0 也合法（驱动器按 0 速处理）
  return rpm <= 5000U;
}

/** @brief 方向枚举是否合法 */
template <typename Uart>
bool DeviceEmmv5GroupT<Uart>::_valid_direction(emmv5::Direction dir)
{
  return (dir == emmv5::Direction::CW) || (dir == emmv5::Direction::CCW);
}

/** @brief 只查该台的累计到位标志，不清零 */
template <typename Uart>
bool DeviceEmmv5GroupT<Uart>::in_position(size_t index) const
{
  // 只查询不清零：给“看看这台到没到”用，不影响 get_in_position() 的位图
  if (!_ready || index >= _count)
  {
    return false;
  }

  taskENTER_CRITICAL();
  const bool value = _motor[index].in_position;
  taskEXIT_CRITICAL();
  return value;
}

/** @brief 取走累计到位位图并清零 */
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
