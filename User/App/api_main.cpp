#include "api_main.h"
#include "FreeRTOS.h"
#include "main.h" // IWYU pragma: keep
#include "stdio.h"
#include "task.h"


/* BSP */
#include "bsp_cfg.hpp"

/* Device */
#include "device_cfg.hpp"

/* 任务声明 */
#include "app_task.hpp"

/* 测试任务声明（开关见 app_test.hpp） */
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

  /* 维护任务：sys_task 为 10 ms（Online 计时 + UART/CAN 断链兜底，优先级 +7） */
  configASSERT(xTaskCreate(sys_task, "sys", 256, NULL, tskIDLE_PRIORITY + 7, NULL) == pdPASS);

  /* 测试任务：can_test 为 10 ms（CAN1 <-> CAN3 互测，优先级 +5；关闭开关即不参与编译） */
#if APP_TEST_CAN_ENABLED
  configASSERT(xTaskCreate(can_test_task, "can_test", 512, NULL, tskIDLE_PRIORITY + 5, NULL) == pdPASS);
#endif

  /* 测试任务：key_test 为 200 ms（按键短按/长按提示音，优先级 +5；关闭开关即不参与编译） */
#if APP_TEST_KEY_ENABLED
  configASSERT(xTaskCreate(key_test_task, "key_test", 256, NULL, tskIDLE_PRIORITY + 5, NULL) == pdPASS);
#endif
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
