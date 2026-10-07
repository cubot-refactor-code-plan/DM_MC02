/**
 * @file device_cfg.hpp
 * @author Rh
 * @brief 设备层统一管理 —— device_init() 与全局实例声明
 * @version 0.3
 * @date 2026-10-04
 *
 * @copyright Copyright (c) 2026
 *
 * @note 设备实例在此声明、在 device_cfg.cpp 中实例化；本层的初始化动作全部收进
 *       device_init()，与 bsp_init() / service_init() 对齐。
 */

#ifndef __DEVICE_CFG_HPP__
#define __DEVICE_CFG_HPP__

#include "device_buzzer.hpp"
#include "device_r9ds.hpp"
#include "dji_motor_group.hpp"
#include "status.hpp"


// ---------------- 函数声明 ----------------

/**
 * @brief 逐个初始化本层设备
 *
 * @note 设备自己不写全局状态；初始化失败在这里用 configASSERT 就地停机，因此不需要返回值。
 */
void device_init(void);


// ----------------
// ---------------- 全局声明 ----------------

extern DeviceBuzzer  buzzer;         ///< 蜂鸣器，绑定 BSP 层的 bsp_pwm_buzzer
extern DjiMotorGroup can3_dji_group; ///< CAN3 上的 DJI 电机组，成员表见 device_cfg.cpp
extern DeviceR9ds    r9ds;           ///< R9DS 遥控接收机，绑定 BSP 层的 bsp_uart5（UART5，SBUS）


// ----------------
#endif // __DEVICE_CFG_HPP__
