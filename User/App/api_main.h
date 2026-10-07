/**
 * @file api_main.h
 * @author Rh
 * @brief 应用层与STM32的接口，放置最直接运行的任务
 * @version 0.1
 * @date 2026-01-20
 *
 * @note 各个外设的中断回调函数，放到了各个的BSP中实现，方便查找
 *       对于中断得到的数据，在这里创建任务等待消息队列+处理即可
 *
 * @copyright Copyright (c) 2026
 *
 * @details 使用说明：
 *          - app_init()：在main.c的main函数中调用，用于无RTOS的初始化
 *          - freertos_init()：在FreeRTOS初始化阶段调用，用于创建任务和初始化外设
 */

#ifndef __API_MAIN_H__
#define __API_MAIN_H__

#include "FreeRTOS.h"  // IWYU pragma: keep
#include "semphr.h"

#ifdef __cplusplus
extern "C"
{
#endif

  /* ==================== 初始化函数 ==================== */

  /**
   * @brief FreeRTOS后的相关初始化
   */
  void all_init(void);


  /* ==================== 任务函数声明 ==================== */

  /**
   * @brief 默认任务
   * @param argument 任务参数
   */
  void StartDefaultTask(void *argument);

  /**
   * @brief CAN接收后处理任务
   * @param argument 任务参数
   */
  void _can_rx_handler_task(void *argument);

  /**
   * @brief USB接收后处理任务
   *
   * @param argument 任务参数
   */
  void _usb_rx_handler_task(void *argument);

  void _usb_tx_handler_task(void *argument);


#ifdef __cplusplus
}
#endif

#endif // __API_MAIN_H__
