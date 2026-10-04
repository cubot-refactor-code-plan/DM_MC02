/**
 * @file service_cfg.hpp
 * @author Rh
 * @brief Service 层统一管理 —— 全局实例的 extern 声明
 * @version 0.1
 * @date 2026-10-02
 *
 * @copyright Copyright (c) 2026
 *
 * @note 全局实例在此统一声明，在 service_cfg.cpp 中统一实例化；
 *       使用者只需 include 此头文件即可访问全部 Service 层实例。
 */

#ifndef __SERVICE_CFG_HPP__
#define __SERVICE_CFG_HPP__

#include "can_bus.hpp"


// ---------------- 全局实例 ----------------

/**
 * @brief CAN 总线实例
 * @note 构造时只绑定硬件句柄，回退缓冲由 can_bus_init() 创建。
 */
extern CanBus bus_can1; ///< CAN1
extern CanBus bus_can2; ///< CAN2
extern CanBus bus_can3; ///< CAN3


// ----------------
#endif // __SERVICE_CFG_HPP__
