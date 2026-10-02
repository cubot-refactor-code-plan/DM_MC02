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
 *       device_init() 只做设备注册：CanBus 的节点注册表在 can_rx_task 启动后立即冻结，
 *       所以必须在调度器启动前（all_init() 中 can_bus_init() 之后）调用。
 *       蜂鸣器无需初始化：它的 PWM 通道由 bsp_init() 启动。
 */

#ifndef __DEVICE_CFG_HPP__
#define __DEVICE_CFG_HPP__

#include "bsp_cfg.hpp" // IWYU pragma: keep (bsp_pwm_buzzer)
#include "device_buzzer.hpp"
#include "dji_motor.hpp"
#include "dm_imu.hpp"


// ---------------- 函数声明 ----------------

void device_init();


// ----------------
// ---------------- 全局声明 ----------------

///< 蜂鸣器：硬件是 BSP 层的 bsp_pwm_buzzer，本类只加音调/音长语义
extern DeviceBuzzer buzzer;


// ---------------- CAN3 设备 ----------------

///< 达妙 IMU：CAN_ID 0x58（发送目标）、MST_ID 0x59（数据帧 ID），主动模式 100 Hz
extern DmImu dm_imu;

///< DJI M2006 + C610 电调（电流控制）：ID 1 → 控制帧 0x200 槽位 0、反馈 0x201
extern DjiMotor<Motor2006> m2006;

///< DJI GM6020（电流控制）：ID 1 → 控制帧 0x1FE 槽位 0、反馈 0x205
extern DjiMotor<Motor6020> gm6020;


// ----------------
#endif // __DEVICE_CFG_HPP__
