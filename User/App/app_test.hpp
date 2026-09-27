/**
 * @file app_test.hpp
 * @author Rh
 * @brief 应用层测试任务开关与声明
 * @version 0.1
 * @date 2026-09-27
 *
 * @copyright Copyright (c) 2026
 *
 * @note 约定：User/App/test/<模块>/<模块>_test.cpp 内用 #if APP_TEST_XXX_ENABLED
 *       包住全部代码，任务函数以 extern "C" 声明（FreeRTOS 以 C 方式调用）；
 *       由 all_init() 统一创建，关闭开关即完全不参与编译。
 */

#ifndef __APP_TEST_HPP__
#define __APP_TEST_HPP__

/**
 * @brief CAN1 <-> CAN3 物理互连收发验证
 * @note 1=CAN1 发 0x123 / CAN3 收，CAN3 发 0x321 / CAN1 收；10 ms 周期各 1 万帧，
 *       进度与诊断经 USART1（PA9/PA10, 115200）打印。
 * @note 开启前请确认两路 CANH/CANL 已正确互连（不交叉）且总线有 120Ω 终端。
 */
#define APP_TEST_CAN_ENABLED 0


/** @brief 按键测试：PRESS=3kHz/40ms，SHORT=4kHz/100ms，LONG=2kHz/500ms（串口只打印 SHORT/LONG） */
#define APP_TEST_KEY_ENABLED 1


#ifdef __cplusplus
extern "C"
{
#endif

  /**
   * @brief CAN1 <-> CAN3 互测任务（发满后转入诊断心跳，便于拔线观察 TEC/REC/BusOff）
   * @param argument 任务参数（未使用，NULL）
   */
  void can_test_task(void *argument);

  /** @brief 按键提示音测试任务 */
  void key_test_task(void *argument);

#ifdef __cplusplus
}
#endif

#endif // __APP_TEST_HPP__
