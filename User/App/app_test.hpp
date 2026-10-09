/**
 * @file app_test.hpp
 * @author Rh
 * @brief 应用层测试任务开关与声明
 * @version 0.2
 * @date 2026-10-03
 *
 * @copyright Copyright (c) 2026
 *
 *       包住全部代码，任务函数以 extern "C" 声明（FreeRTOS 以 C 方式调用）；
 *       由 all_init() 统一创建。
 */

#ifndef __APP_TEST_HPP__
#define __APP_TEST_HPP__

#include "status.hpp" // IWYU pragma: keep


/** @brief CAN1 零输出故障注入与 Bus-Off 恢复实机测试 */
#define APP_TEST_CAN_RECOVERY_ENABLED 0

/** @brief 编译并创建 Online 实机自检任务 */
#define APP_TEST_ONLINE_CHECK_ENABLED 0

/** @brief CAN3 DJI 电机组通讯自检（3508/2006/6020 混挂，成员表见 device_cfg.cpp） */
#define APP_TEST_DJI_GROUP_ENABLED 0

/** @brief R9DS 遥控接收机 SBUS 收帧自检（UART5，只收不发；接收机需拨到 SBUS 模式） */
#define APP_TEST_R9DS_ENABLED 1

/** @brief 舵机限速轨迹自检（servo1 @PE13 TIM1_CH3，开环来回扫角，打印目标/实际角度/脉宽） */
#define APP_TEST_SERVO_ENABLED 0


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

  /** @brief Online 状态机自检任务 */
  void online_check_test_task(void *argument);

  /**
   * @brief CAN1 故障注入与恢复测试的初始化
   *
   * @note 总线就绪与否在这里用 configASSERT 验，失败就地停机；
   *       结论同时记进 can_recovery_test_init_status，方便调试器直接看。
   */
  void can_recovery_test_init(void);

  /** @brief CAN1 零输出故障注入与 Bus-Off 恢复测试任务 */
  void can_recovery_test_task(void *argument);

  /** @brief CAN3 DJI 电机组通讯自检任务（开环正弦 + 轮询汇报） */
  void dji_motor_group_test_task(void *argument);

  /** @brief R9DS SBUS 收帧自检任务（打印原始帧 / 16 通道 / 映射后的通道与诊断计数） */
  void r9ds_test_task(void *argument);

  /** @brief 舵机限速轨迹自检任务（1 kHz update()，200 ms 打印目标/实际角度/脉宽） */
  void servo_test_task(void *argument);

#ifdef __cplusplus
}
#endif

#endif // __APP_TEST_HPP__
