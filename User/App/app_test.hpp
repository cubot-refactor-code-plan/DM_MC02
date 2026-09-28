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


#ifdef __cplusplus
extern "C"
{
#endif

  /** @brief DWT 计时测试任务（串口打印） */
  void dwt_test_task(void *argument);

#ifdef __cplusplus
}
#endif

#endif // __APP_TEST_HPP__
