#include "online_check.hpp"

#include "FreeRTOS.h" // IWYU pragma: keep
#include "task.h"

#include <stdint.h>


// ---------------- 私有实现 ----------------


namespace
{
/** @brief 在调度器运行后用任务临界区保护 Online 链表与状态 */
class ScopedTaskCritical
{
public:
  ScopedTaskCritical() : _active(xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
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
} // namespace


// ----------------
// ---------------- 成员变量 ----------------


Online *Online::_head = nullptr;
Online *Online::_tail = nullptr;


Online::Online(uint16_t timeout_gap) : _last_refresh_tick(xTaskGetTickCount() - pdMS_TO_TICKS(timeout_gap == 0U ? 1U : timeout_gap)), _timeout_gap(timeout_gap == 0U ? 1U : timeout_gap), _statu(Status::TIMEOUT), _next(nullptr)
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


// ----------------
// ---------------- 公有接口 ----------------


Status Online::refresh_task(void)
{
  const ScopedTaskCritical lock;
  _last_refresh_tick = xTaskGetTickCount();
  _statu             = Status::OK;
  return Status::OK;
}


Status Online::refresh_isr(void)
{
  const UBaseType_t interrupt_mask = taskENTER_CRITICAL_FROM_ISR();
  _last_refresh_tick               = xTaskGetTickCountFromISR();
  _statu                           = Status::OK;
  taskEXIT_CRITICAL_FROM_ISR(interrupt_mask);
  return Status::OK;
}


Status Online::is_online(void) const
{
  const ScopedTaskCritical lock;
  return _statu;
}


Status Online::update(void)
{
  const ScopedTaskCritical lock;
  const TickType_t        now = xTaskGetTickCount();

  for (Online *item = _head; item != nullptr; item = item->_next)
  {
    // 无符号差值，tick 回绕安全
    const bool timed_out =
      (now - item->_last_refresh_tick) >= pdMS_TO_TICKS(item->_timeout_gap);
    item->_statu = timed_out ? Status::TIMEOUT : Status::OK;
  }

  return Status::OK;
}


// ----------------
