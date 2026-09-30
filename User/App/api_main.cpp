#include "api_main.h"
#include "FreeRTOS.h" // IWYU pragma: keep
#include "main.h"     // IWYU pragma: keep
#include "stdio.h"
#include "task.h"


/* BSP */
#include "bsp_cfg.hpp"

/* Device */
#include "device_cfg.hpp" // IWYU pragma: keep

/* 任务声明 */
#include "app_task.hpp"

/* 测试任务声明（开关见 app_test.hpp） */
#include "app_test.hpp" // IWYU pragma: keep


/**
 * @brief FreeRTOS后的相关初始化
 *
 * @note 在main.c的MX_FREERTOS_INIT函数中调用
 *       用于创建FreeRTOS任务和初始化外设驱动
 *
 */
void all_init()
{
  /* 初始化BSP设备 */
  bsp_init();

  /* 维护任务：sys_task 为 10 ms（Online 计时 + UART/CAN 断链兜底，优先级 +7） */
  configASSERT(xTaskCreate(sys_task, "sys", 256, NULL, tskIDLE_PRIORITY + 7, NULL) == pdPASS);

  /* 按键任务：200 ms 轮询（短按/长按发不同提示音，优先级 +5） */
  configASSERT(xTaskCreate(key_task, "key", 256, NULL, tskIDLE_PRIORITY + 5, NULL) == pdPASS);

}


/**
 * @brief 默认任务，这个原本命名为_start_default_task。但是每次开FreeRTOS这个里面，默认是这个名字
 *
 * @note 保留这个名字，但是其他任务要类似：_start_default_task
 * @param argument 任务参数
 */
extern "C" void StartDefaultTask(void *argument)
{
  for (;;)
  {

    vTaskDelay(100);
  }
}
