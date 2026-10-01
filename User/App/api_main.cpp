#include "api_main.h"
#include "FreeRTOS.h" // IWYU pragma: keep
#include "main.h"     // IWYU pragma: keep
#include "task.h"

#include <stdint.h>
#include <stdio.h>

/* BSP */
#include "bsp_cfg.hpp"

/* Device */
#include "device_cfg.hpp"

/* Protocol */
#include "protocol_cfg.hpp"

/* Service */
#include "can_bus.hpp"
#include "status.hpp"

/* 任务声明 */
#include "app_task.hpp"
#include "app_test.hpp"


// ---------------- 全局变量定义 ----------------

/* C620/M3508 CAN1 实机测试诊断量（可在调试器 Live Watch 中查看） */
volatile Status   can1_statu                = Status::NOT_INIT;
volatile uint32_t can1_tx_ok_count          = 0;
volatile uint32_t can1_tx_full_count        = 0;
volatile uint32_t can1_rx_count             = 0;
volatile uint32_t can1_feedback_202_count   = 0;
volatile uint32_t can1_last_rx_id           = 0;
volatile int16_t  c620_speed_rpm            = 0;
volatile uint16_t c620_peak_abs_speed_rpm   = 0;
volatile int16_t  c620_given_current        = 0;
volatile uint8_t  c620_temperature          = 0;
volatile uint32_t can1_last_error_code      = 0;
volatile uint32_t can1_tx_error_count       = 0;
volatile uint32_t can1_rx_error_count       = 0;
volatile uint32_t can1_bus_off              = 0;


/**
 * @brief FreeRTOS后的相关初始化
 *
 * @note 在main.c的MX_FREERTOS_INIT函数中调用
 *       用于创建FreeRTOS任务和初始化外设驱动
 *
 *       初始化顺序：
 *         1. 初始化系统状态标志
 *         2. bsp_init()     —— 外设 BSP 初始化
 *         3. can_bus_init() —— CAN 总线初始化（回退缓冲）
 *         4. device_init()  —— 设备层初始化
 *         5. protocol_init()—— 协议层初始化
 *         6. ...
 */
void all_init()
{
  // 初始化系统状态标志
  configASSERT(sys_flag_init() == Status::OK);

  /* 初始化BSP设备 */
  bsp_init();

  /* 初始化 CAN 总线（须在 bsp_init() 之后） */
  configASSERT(can_bus_init() == Status::OK);

  /* 初始化设备 */
  device_init();

  /* 初始化协议层 */
  protocol_init();

#if APP_TEST_DJI_MOTOR_ENABLED
  // 注册必须早于运行标志，接收任务等待初始化完成后开始轮询。
  dji_motor_test_init();

// ----------------
#endif

#if APP_TEST_CAN_RECOVERY_ENABLED
  can_recovery_test_init();
#endif

  /* 系统维护与 CAN 发送任务（均为 1 kHz，CAN 发送优先级更高） */
  configASSERT(xTaskCreate(sys_task, "sys", 256, NULL, tskIDLE_PRIORITY + 7, NULL) == pdPASS);
  configASSERT(xTaskCreate(can_rx_task, "can_rx", 512, NULL, tskIDLE_PRIORITY + 8, NULL) == pdPASS);
  configASSERT(xTaskCreate(can_tx_task, "can_tx", 512, NULL, tskIDLE_PRIORITY + 8, NULL) == pdPASS);

#if APP_TEST_CAN_RECOVERY_ENABLED
  configASSERT(xTaskCreate(can_recovery_test_task, "can_fault", 512, NULL, tskIDLE_PRIORITY + 5, NULL) == pdPASS);
#endif

#if APP_TEST_ONLINE_CHECK_ENABLED
  configASSERT(xTaskCreate(online_check_test_task,
                           "online_test",
                           256,
                           NULL,
                           tskIDLE_PRIORITY + 3,
                           NULL) == pdPASS);
#endif

#if APP_TEST_DJI_MOTOR_ENABLED
  configASSERT(xTaskCreate(dji_motor_test_task,
                           "motor_test",
                           256,
                           NULL,
                           tskIDLE_PRIORITY + 5,
                           NULL) == pdPASS);
#endif

#if APP_TEST_DM_MOTOR_ENABLED
  configASSERT(xTaskCreate(dm_motor_test_task,
                           "dm_test",
                           512,
                           NULL,
                           tskIDLE_PRIORITY + 5,
                           NULL) == pdPASS);
#endif

#if APP_TEST_QSPI_FLASH_ENABLED
  configASSERT(xTaskCreate(qspi_flash_test_task,
                           "qspi_test",
                           512,
                           NULL,
                           tskIDLE_PRIORITY + 4,
                           NULL) == pdPASS);
#endif

#if APP_TEST_DWT_ENABLED
  configASSERT(xTaskCreate(dwt_test_task, "dwt_test", 512, NULL, tskIDLE_PRIORITY + 3, NULL) == pdPASS);
#endif

  sys_complete_init();

}


/**
 * @brief ST的默认任务，弱定义，在这里定义，方便寻找
 *
 * @note 任务名保留 CubeMX 生成的 StartDefaultTask（便于 CubeMX 重新生成时同名兼容）；
 *       自定义任务函数命名遵循小写蛇形规范（如 task_menu）。
 *
 * @note freertos调用cpp的函数，需要extern "C"修饰，避免C++的名称修饰导致找不到函数
 *
 * @param argument 任务参数
 */
extern "C" void StartDefaultTask(void *argument)
{
  (void)argument; // 未使用参数

  /* Flash 复位等待和 TinyUSB RTOS 对象都要求调度器已经运行。 */
  bsp_usb.init();

  for (;;)
  {
    bsp_usb.task();

#if APP_TEST_USB_TRANSPORT_ENABLED
    usb_transport_test_step();
#endif

#if APP_TEST_UART_TRANSPORT_ENABLED
    uart_transport_test_step(); // USART1 收发测试：回显 + 周期心跳
#endif

    vTaskDelay(pdMS_TO_TICKS(1U));
  }
}
