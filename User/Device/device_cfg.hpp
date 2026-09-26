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
 *       当前为底层库形态：仅 LCD 在 device_init() 中初始化；
 *       电机/IMU 业务接入时在此补实例化与 extern 声明。
 */

#ifndef __DEVICE_CFG_HPP__
#define __DEVICE_CFG_HPP__


/* ==================== 函数声明 ==================== */

void device_init();


/* ==================== 全局声明 ==================== */


#endif // __DEVICE_CFG_HPP__
