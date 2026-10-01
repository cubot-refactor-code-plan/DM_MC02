#include "status.hpp"

#include "task.h" // IWYU pragma: keep（vTaskDelete）

EventGroupHandle_t sys_event = nullptr; ///< 系统状态事件组

Status sys_flag_init(void)
{
  sys_event = xEventGroupCreate();
  if (sys_event == nullptr)
  {
    return Status::FULL;
  }
  xEventGroupSetBits(sys_event, (1U << (uint8_t)Status::OK));
  return Status::OK;
}

Status sys_flag_set(Status statu)
{
  if (sys_event == nullptr)
  {
    return Status::NOT_INIT;
  }
  if (statu == Status::OK)
  {
    return Status::BAD_ARG;
  }
  xEventGroupSetBits(sys_event, (1U << (uint8_t)statu));
  xEventGroupClearBits(sys_event, (1U << (uint8_t)Status::OK));
  return Status::OK;
}

void sys_complete_init(void)
{
  if ((xEventGroupGetBits(sys_event) & SYS_FLAG_INIT_FAIL_BIT) == 0U)
  {
    xEventGroupSetBits(sys_event, SYS_FLAG_RUNNING_BIT);
  }
}

void sys_init_error(void)
{
  xEventGroupSetBits(sys_event, SYS_FLAG_INIT_FAIL_BIT);
  xEventGroupClearBits(sys_event, SYS_FLAG_RUNNING_BIT);
}

Status sys_init_error(Status statu)
{
  sys_init_error();
  return sys_flag_set(statu);
}

uint32_t sys_flag_wait(Status statu, uint32_t timeout)
{
  // pdFALSE=退出时不清标志，pdTRUE=等到所有指定位都置位
  return xEventGroupWaitBits(sys_event, (1U << (uint8_t)statu), pdFALSE, pdTRUE, timeout);
}

bool sys_flag_running(void)
{
  return sys_event != nullptr && (xEventGroupGetBits(sys_event) & SYS_FLAG_RUNNING_BIT) != 0U;
}

void sys_flag_wait_running(void)
{
  if (sys_event == nullptr) // 连事件组都建不起来，那就别运行了
  {
    vTaskDelete(NULL);
  }

  (void)xEventGroupWaitBits(sys_event, SYS_FLAG_RUNNING_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
  if (!sys_flag_running())
  {
    vTaskDelete(NULL);
  }
}
