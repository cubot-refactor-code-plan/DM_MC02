#include "online_check.hpp"

#include "FreeRTOS.h" // IWYU pragma: keep
#include "task.h"

#include <stdint.h>


namespace
{
/** @brief 在调度器运行后用任务临界区保护 Online 链表与状态。 */
class ScopedTaskCritical
{
public:
  ScopedTaskCritical()
    : _active(xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
  {
    if (_active)
    {
      taskENTER_CRITICAL();
    }
  }

  ~ScopedTaskCritical()
  {
    if (_active)
    {
      taskEXIT_CRITICAL();
    }
  }

private:
  bool _active;
};

///< 上次 update() 的 tick；用于按真实经过时间累计离线计时
TickType_t s_last_tick = 0U;
} // namespace


// 离线计时以「毫秒」为单位，依赖 1 tick == 1 ms
static_assert(configTICK_RATE_HZ == 1000U,
              "online_check requires configTICK_RATE_HZ == 1000 (1 tick = 1 ms)");


Online *Online::_head = nullptr;
Online *Online::_tail = nullptr;


Online::Online(uint16_t timeout_gap)
  : _cnt(timeout_gap == 0U ? 1U : timeout_gap),
    _timeout_gap(timeout_gap == 0U ? 1U : timeout_gap),
    _statu(Status::TIMEOUT),
    _next(nullptr)
{
  const ScopedTaskCritical lock;
  if (_tail == nullptr)
  {
    _head = this;
    _tail = this;
  }
  else
  {
    _tail->_next = this;
    _tail        = this;
  }
}


Online::~Online()
{
  const ScopedTaskCritical lock;

  Online *previous = nullptr;
  Online *current  = _head;
  while ((current != nullptr) && (current != this))
  {
    previous = current;
    current  = current->_next;
  }

  if (current == this)
  {
    if (previous == nullptr)
    {
      _head = _next;
    }
    else
    {
      previous->_next = _next;
    }

    if (_tail == this)
    {
      _tail = previous;
    }
  }

  _next = nullptr;
}


Status Online::refresh_task(void)
{
  const ScopedTaskCritical lock;
  _cnt   = 0U;
  _statu = Status::OK;
  return Status::OK;
}


Status Online::refresh_isr(void)
{
  const UBaseType_t interrupt_mask = taskENTER_CRITICAL_FROM_ISR();
  _cnt   = 0U;
  _statu = Status::OK;
  taskEXIT_CRITICAL_FROM_ISR(interrupt_mask);
  return Status::OK;
}


Status Online::isOnline(void) const
{
  const ScopedTaskCritical lock;
  const Status result = _statu;
  return result;
}


Status Online::update(void)
{
  const ScopedTaskCritical lock;

  // 按「真实经过的毫秒」推进，与 update() 的调用周期解耦（1 tick = 1 ms）
  const TickType_t now        = xTaskGetTickCount();
  const uint32_t   elapsed_ms = static_cast<uint32_t>(now - s_last_tick);
  s_last_tick                 = now;

  for (Online *item = _head; item != nullptr; item = item->_next)
  {
    const uint32_t sum = static_cast<uint32_t>(item->_cnt) + elapsed_ms;
    item->_cnt = (sum > static_cast<uint32_t>(UINT16_MAX))
                   ? static_cast<uint16_t>(UINT16_MAX)
                   : static_cast<uint16_t>(sum); // 饱和递增，避免回绕后误判在线

    item->_statu = (item->_cnt >= item->_timeout_gap) ? Status::TIMEOUT : Status::OK;
  }

  return Status::OK;
}
