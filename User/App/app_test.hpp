/**
 * @file app_test.hpp
 * @author Rh
 * @brief 应用层测试任务声明
 * @version 0.1
 * @date 2026-08-14
 *
 * @copyright Copyright (c) 2026
 *
 * @note 测试任务函数均以 extern "C" 声明（FreeRTOS 以 C 方式调用）。
 *       任务实现位于 User/App/test/，由 all_init() 统一创建。
 */

#ifndef __APP_TEST_HPP__
#define __APP_TEST_HPP__

/** @brief 编译并创建 Online 实机自检任务；设为 0 可停用。 */
#define APP_TEST_ONLINE_CHECK_ENABLED 0

/** @brief CAN1 零输出故障注入与 Bus-Off 恢复实机测试。 */
#define APP_TEST_CAN_RECOVERY_ENABLED 0

/** @brief 编译并创建 CAN1 ID2 M3508 低电流转动测试；设为 0 可停用。 */
#define APP_TEST_DJI_MOTOR_ENABLED 0

/** @brief 编译并创建 CAN2 ID1 达妙普通固件四模式实机测试；设为 0 可停用。 */
#define APP_TEST_DM_MOTOR_ENABLED 0

/** @brief 启用当前 USB 类的周期发送及 HID 回环测试；设为 0 可停用。 */
#define APP_TEST_USB_TRANSPORT_ENABLED 0

/** @brief 旧名称兼容；新代码应使用 APP_TEST_USB_TRANSPORT_ENABLED。 */
#define APP_TEST_USB_CDC_ENABLED APP_TEST_USB_TRANSPORT_ENABLED

/** @brief 启用 USART1 串口收发实机测试（回显 + 周期心跳）；设为 0 可停用。 */
#define APP_TEST_UART_TRANSPORT_ENABLED 0

/** @brief 编译并创建 W25Q64JV 擦写、映射与 XIP 实机测试；完成后应设回 0。 */
#define APP_TEST_QSPI_FLASH_ENABLED 0

/** @brief 编译并创建 DWT 计时标定测试；设为 0 可停用。 */
#define APP_TEST_DWT_ENABLED 0

/** @brief CAN3 三设备联调自检（达妙 IMU + M2006 + GM6020）；设为 0 可停用。 */
#define APP_TEST_CAN3_DEVICE_ENABLED 0

/** @brief CAN3 达妙 IMU 自检（读探测 + 数据帧打印 + 状态监视）；设为 0 可停用。 */
#define APP_TEST_CAN3_IMU_ENABLED 0

#ifdef __cplusplus
extern "C"
{
#endif

  /** @brief 在系统运行标志置位前初始化 CAN1 ID2 测试电机。 */
  void dji_motor_test_init(void);
  void can_recovery_test_init(void);
  void can_recovery_test_task(void *argument);

  /**
   * @brief Online 状态机自检任务
   * @param argument 任务参数（未使用，NULL）
   * @note 仅在 APP_TEST_ONLINE_CHECK_ENABLED 非零时创建。
   */
  void online_check_test_task(void *argument);

  /**
   * @brief CAN1 ID2 M3508 低电流实机测试任务
   * @param argument 任务参数（未使用，NULL）
   * @note 仅在 APP_TEST_DJI_MOTOR_ENABLED 非零时创建。
   * @note 启用后保持零输出，调试器写 dji_motor_test_arm=1 才执行一次双向测试。
   * @warning 正反向各输出原始电流指令 512 持续 600 ms，测试时必须架空或固定电机。
   */
  void dji_motor_test_task(void *argument);

  /**
   * @brief CAN2 ID1 达妙普通固件四模式实机测试任务
   * @param argument 任务参数（未使用，NULL）
   * @note 仅在 APP_TEST_DM_MOTOR_ENABLED 非零时创建。
   * @warning 测试会临时切换电机模式，每种模式运行约 4 秒；结束后会失能并恢复原模式。
   */
  void dm_motor_test_task(void *argument);

  /**
   * @brief 板载 W25Q64JV 间接读写、Memory-Mapped 和 XIP 实机测试任务。
   * @param argument 任务参数（未使用，NULL）。
   * @warning 启用后会改写外置 Flash 最后两个扇区和 0x00100000 附近的 XIP 测试区。
   */
  void qspi_flash_test_task(void *argument);

  /**
   * @brief DWT 计时标定任务（USART1 每秒打印一次耗时对比）
   * @param argument 任务参数（未使用，NULL）
   * @note 仅在 APP_TEST_DWT_ENABLED 非零时创建；需要 PC 端接 USART1 才能看到输出。
   * @note 校验 1 s 延时是否 ≈1.000 s（不符就是 CPU 频率算错）与 delay_ms(1) 是否 ≈0.001 s。
   */
  void dwt_test_task(void *argument);

  /**
   * @brief CAN3 两台 DJI 电机自检任务（M2006/C610 + GM6020 电流模式）
   * @param argument 任务参数（未使用，NULL）
   * @note 仅在 APP_TEST_CAN3_DEVICE_ENABLED 非零时创建。
   * @note 上电保持零输出，调试器写 can3_test_arm=1 才执行一次转动测试；
   *       写 can3_test_abort=1 随时终止；结果全在 can3_test_* 观察变量里。
   * @warning 会真实驱动电机（M2006 给 3000 / GM6020 给 5000 原始电流，输出轴 60 rpm 硬保护），必须架空或固定。
   */
  void can3_device_test_task(void *argument);

  /**
   * @brief CAN3 达妙 IMU 自检任务（读探测 + CAN_ID 扫描 + 状态监视）
   * @param argument 任务参数（未使用，NULL）
   * @note 仅在 APP_TEST_CAN3_IMU_ENABLED 非零时创建。
   * @note 上电发一次读探测（读操作一定有应答帧），观察 1 s；若仍离线则扫描
   *       CAN_ID 1..127；之后只在状态变化时打印。
   * @note 调试器写 can3_imu_test_scan=1 可随时重新扫描（模块改完配置不用重启板子）。
   */
  void can3_imu_test_task(void *argument);

  /**
   * @brief 当前 USB 类的非阻塞实机测试步骤，由 USB 服务任务每毫秒调用。
   * @note CDC 周期发送固定帧；HID 周期发送固定报告并回显主机 OUT Report。
   */
  void usb_transport_test_step(void);

  /**
   * @brief USART1 串口收发实机测试步骤，由 StartDefaultTask 的 1ms 循环调用。
   * @note PC 发什么就原样回显什么；空闲时周期发送心跳（含累计收发字节数）。
   */
  void uart_transport_test_step(void);

#ifdef __cplusplus
}
#endif

#endif // __APP_TEST_HPP__
