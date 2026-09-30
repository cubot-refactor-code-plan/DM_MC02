/**
 * @file app_test.hpp
 * @author Rh
 * @brief 应用层测试任务开关与声明
 * @version 0.1
 * @date 2026-09-27
 *
 * @copyright Copyright (c) 2026
 *
 *       包住全部代码，任务函数以 extern "C" 声明（FreeRTOS 以 C 方式调用）；
 *       由 all_init() 统一创建。
 */

#ifndef __APP_TEST_HPP__
#define __APP_TEST_HPP__

/** @brief DWT 计时验证：打印绝对时间轴 / 1s 标定 / 1ms 忙等自测 */
#define APP_TEST_DWT_ENABLED 1

/** @brief UART 极限测试：uart8 <-> uart9 对接，双向同时压满并逐个包比对 */
#define APP_TEST_UART_ENABLED 1


#ifdef __cplusplus
extern "C"
{
#endif

  /** @brief DWT 计时测试任务（串口打印） */
  void dwt_test_task(void *argument);

  /** @brief UART 极限测试：uart8 发、uart9 收 */
  void uart_test_task_8to9(void *argument);

  /** @brief UART 极限测试：uart9 发、uart8 收 */
  void uart_test_task_9to8(void *argument);

  /** @brief UART 极限测试：每秒汇报两个方向的统计（独立任务，不干扰收发） */
  void uart_test_report(void *argument);

#ifdef __cplusplus
}
#endif

#endif // __APP_TEST_HPP__
