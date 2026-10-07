#include "api_main.h"
#include "FreeRTOS.h" // IWYU pragma: keep
#include "main.h"     // IWYU pragma: keep
#include "stdio.h"
#include "task.h"


/* BSP */
#include "bsp_cfg.hpp"

/* Device */
#include "device_cfg.hpp" // IWYU pragma: keep

/* Service */
#include "service_cfg.hpp"

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
 *       初始化顺序：
 *         1. service_init()  —— Service 层（系统状态标志）
 *         2. bsp_init()      —— BSP 层外设初始化
 *         3. device_init()   —— 设备层初始化
 *         4. 创建各任务      —— 任务入口先等运行态
 *         5. sys_state.complete_init() —— 置运行位
 */
void all_init()
{
  /* 各层 init 内部自带 configASSERT，失败就地停机，因此这里直接调用、不需要再验一次 */

  /* Service 层：系统状态标志，须最先（后面各层要靠它上报失败） */
  service_init();

  /* BSP 层：外设（含各 CAN 外设） */
  bsp_init();

  /* 设备层：BSP 之后 */
  device_init();

#if APP_TEST_CAN_RECOVERY_ENABLED
  can_recovery_test_init();
  configASSERT(xTaskCreate(can_recovery_test_task, "can_fault", 512, NULL, tskIDLE_PRIORITY + 5, NULL) == pdPASS);
#endif

#if APP_TEST_ONLINE_CHECK_ENABLED
  configASSERT(xTaskCreate(online_check_test_task, "online_test", 256, NULL, tskIDLE_PRIORITY + 3, NULL) == pdPASS);
#endif

#if APP_TEST_DJI_GROUP_ENABLED
  configASSERT(xTaskCreate(dji_motor_group_test_task, "dji_grp", 512, NULL, tskIDLE_PRIORITY + 5, NULL) == pdPASS);
#endif

#if APP_TEST_R9DS_ENABLED
  configASSERT(xTaskCreate(r9ds_test_task, "r9ds", 512, NULL, tskIDLE_PRIORITY + 5, NULL) == pdPASS);
#endif

  /* 维护任务：sys_task 为 10 ms（Online 计时 + UART/CAN 断链兜底，优先级 +7） */
  configASSERT(xTaskCreate(sys_task, "sys", 256, NULL, tskIDLE_PRIORITY + 7, NULL) == pdPASS);

  /* 按键任务：200 ms 轮询（短按/长按发不同提示音，优先级 +5） */
  configASSERT(xTaskCreate(key_task, "key", 256, NULL, tskIDLE_PRIORITY + 5, NULL) == pdPASS);

  /* 置运行标志：任务入口的 wait_running() 在此之后才放行 */
  configASSERT(sys_state.complete_init() == Status::OK);
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
