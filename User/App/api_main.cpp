#include "api_main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h" // IWYU pragma: keep
#include "stdio.h"

/* BSP */
#include "bsp_cfg.hpp"

/* Device */
#include "device_cfg.hpp"

/* Protocol */
#include "protocol_cfg.hpp"

/* 任务声明 */
#include "app_task.hpp"
#include "app_test.hpp"


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

  /* 初始化协议层 */
  protocol_init();

  /* 维护任务：sys_task 为 10 ms（Online 计时 + 串口 TX 兜底，优先级 +7）；
     dji_motor_task 为 1 ms（电机接收/发送，优先级更高 +8） */
  configASSERT(xTaskCreate(sys_task, "sys", 256, NULL, tskIDLE_PRIORITY + 7, NULL) == pdPASS);

}


/**
 * @brief 默认任务，这个原本命名为_start_default_task。但是每次开FreeRTOS这个里面，默认是这个名字
 *
 * @note 保留这个名字，但是其他任务要类似：_start_default_task
 * @param argument 任务参数
 */
extern "C" void StartDefaultTask(void *argument)
{
  (void)argument; // 未使用参数

  for (;;)
  {
    
    vTaskDelay(1); // 1 tick（等价于 osDelay(1)）
  }
}
