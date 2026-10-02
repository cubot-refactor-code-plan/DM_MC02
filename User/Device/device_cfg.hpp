/**
 * @file device_cfg.hpp
 * @author Rh
 * @brief 设备层统一管理 —— extern 声明与 device_init()
 * @version 0.1
 * @date 2026-07-21
 *
 * @copyright Copyright (c) 2026
 *
 * @note 设备实例在此统一声明，在 device_cfg.cpp 中统一实例化。
 *       使用者只需 include 此头文件即可访问所有设备实例。
 *       device_init() 暂为空：蜂鸣器只需绑定 PWM 通道（PWM 由 bsp_init() 启动）；
 *       电机 / IMU 业务接入时在此补实例化、extern 声明与初始化。
 */

#ifndef __DEVICE_CFG_HPP__
#define __DEVICE_CFG_HPP__

#include "bsp_cfg.hpp"     // IWYU pragma: keep (bsp_pwm_buzzer)
#include "device_buzzer.hpp"


// ---------------- 函数声明 ----------------

void device_init();


// ----------------
// ---------------- 全局声明 ----------------

///< 蜂鸣器：硬件是 BSP 层的 bsp_pwm_buzzer，本类只加音调/音长语义
extern DeviceBuzzer buzzer;


// ----------------
#endif // __DEVICE_CFG_HPP__
