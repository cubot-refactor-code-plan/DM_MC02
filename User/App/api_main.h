/**
 * @file api_main.h
 * @author Rh
 * @brief 应用层与STM32和FreeRTOS底层的接口，放置最直接运行的任务
 * @version 0.1
 * @date 2026-01-20
 *
 * @copyright Copyright (c) 2026
 *
 * @note 各个外设的中断回调函数，放到了各个的BSP中实现，方便查找
 *
 * @note 使用说明：
 *
 *          - all_init()：在FreeRTOS初始化阶段(MX_FREERTOS_Init)中调用，用于初始化外设和创建任务
 *          - StartDefaultTask()：默认任务，创建各业务任务并循环运行
 *          - 业务任务声明见 app_task.hpp，测试任务声明见 app_test.hpp
 */

#ifndef __API_MAIN_H__
#define __API_MAIN_H__

#include "FreeRTOS.h" // IWYU pragma: keep
#include "semphr.h"


#ifdef __cplusplus
extern "C"
{
#endif

  // ---------------- 初始化函数 ----------------

  /**
   * @brief FreeRTOS后的相关初始化
   */
  void all_init(void);


  // ----------------
  // ---------------- 任务函数声明 ----------------

  /**
   * @brief 默认任务
   * @param argument 任务参数
   */
  void StartDefaultTask(void *argument);


#ifdef __cplusplus

  // ----------------
}
#endif


#endif // __API_MAIN_H__
