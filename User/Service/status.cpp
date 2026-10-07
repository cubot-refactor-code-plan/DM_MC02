#include "status.hpp"

#include "task.h" // IWYU pragma: keep（vTaskDelete）


// ---------------- 生命周期 ----------------


/** @brief 创建本实例的事件组：已创建过就保留原句柄直接返回，不重建（重建会泄漏句柄） */
Status EventState::init(void)
{
  if (_handle != nullptr)
  {
    return Status::OK;
  }

  _handle = xEventGroupCreate();
  if (_handle == nullptr)
  {
    return Status::FULL;
  }
  (void)xEventGroupSetBits(_handle, status_bit(Status::OK));
  return Status::OK;
}

/** @brief 初始化全部完成时置运行位；已有初始化失败记录则不置位，使"失败后不再进入运行态"成为单向约束 */
Status EventState::complete_init(void)
{
  if (_handle == nullptr)
  {
    return Status::NOT_INIT;
  }
  // 有失败记录说明初始化没走完整，不置运行位
  if ((xEventGroupGetBits(_handle) & INIT_FAIL_BIT) != 0U)
  {
    return Status::NOT_SUPPORTED;
  }

  (void)xEventGroupSetBits(_handle, RUNNING_BIT);
  return Status::OK;
}


// ----------------
// ---------------- 错误登记 ----------------


/** @brief 记录一个错误码；不动运行位，运行期错误走这里 */
Status EventState::error(Status statu)
{
  if (_handle == nullptr)
  {
    return Status::NOT_INIT;
  }
  if (statu == Status::OK)
  {
    return Status::BAD_ARG;
  }
  
  (void)xEventGroupSetBits(_handle, status_bit(statu));
  (void)xEventGroupClearBits(_handle, status_bit(Status::OK));
  return Status::OK;
}

/** @brief 记录初始化失败：先停掉本实例的运行期动作，再记错误码留下原因 */
Status EventState::init_error(Status statu)
{
  if (_handle == nullptr)
  {
    return Status::NOT_INIT;
  }
  // INIT_FAIL 与 RUNNING 先处理完再记错误码：失败位置位就一定已经停掉了运行位
  (void)xEventGroupSetBits(_handle, INIT_FAIL_BIT);
  (void)xEventGroupClearBits(_handle, RUNNING_BIT);
  return error(statu);
}


// ----------------
// ---------------- 查询与等待 ----------------


/** @brief 事件组是否已创建 */
bool EventState::is_ready(void) const
{
  return _handle != nullptr;
}

/** @brief 本实例当前是否处于运行态 */
bool EventState::running(void) const
{
  return is_ready() && ((xEventGroupGetBits(_handle) & RUNNING_BIT) != 0U);
}

/** @brief 已记录的最小错误码；按位号从小到大扫，取到的即枚举值最小的那个 */
Status EventState::error_code(void) const
{
  if (_handle == nullptr)
  {
    return Status::NOT_INIT;
  }

  // 位号即枚举值：从 1 扫到 STATUS_MAX - 1（bit 0 是 OK 位，不算错误）
  const EventBits_t bits = xEventGroupGetBits(_handle);
  for (uint32_t value = 1U; value < STATUS_MAX; ++value)
  {
    if ((bits & (1U << value)) != 0U)
    {
      return static_cast<Status>(value);
    }
  }
  return Status::OK;
}

/**
 * @brief 阻塞等待本实例进入运行态；未 init 或等到初始化失败时删除当前任务
 * @note 同时等运行位与失败位：只等运行位的话，初始化失败时它永不置位，任务会永久阻塞。
 */
void EventState::wait_running(void) const
{
  if (_handle == nullptr) // 未 init，没有可等待的对象
  {
    vTaskDelete(NULL);
    return;
  }

  // xWaitForAllBits=pdFALSE：两位中任意一位置位即返回
  const EventBits_t bits = xEventGroupWaitBits(_handle, RUNNING_BIT | INIT_FAIL_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
  if ((bits & RUNNING_BIT) == 0U)
  {
    vTaskDelete(NULL); // 初始化失败，任务自行退出
  }
}


// ----------------
// ---------------- 构造与析构 ----------------


/** @brief 释放事件组；未 init 时无副作用 */
EventState::~EventState()
{
  if (_handle != nullptr)
  {
    vEventGroupDelete(_handle);
    _handle = nullptr;
  }
}


// ----------------
