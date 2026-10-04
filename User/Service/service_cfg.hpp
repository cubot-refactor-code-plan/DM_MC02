/**
 * @file service_cfg.hpp
 * @author Rh
 * @brief Service 层统一管理 —— service_init() 与全局实例声明
 * @version 0.3
 * @date 2026-10-04
 *
 * @copyright Copyright (c) 2026
 *
 * @note 全局实例在此声明、在 service_cfg.cpp 中实例化；本层的初始化动作全部收进
 *       service_init()，与 BSP 层的 bsp_init()、设备层的 device_init() 对齐。
 */

#ifndef __SERVICE_CFG_HPP__
#define __SERVICE_CFG_HPP__

#include "status.hpp"


// ---------------- 函数声明 ----------------

/**
 * @brief Service 层统一初始化
 *
 * @return OK=全部就绪；其余=第一个失败实例返回的状态码
 * @note 须在调度器启动后调用（EventState 要创建事件组），且早于各任务入口的 wait_running()
 */
Status service_init(void);


// ----------------
// ---------------- 全局声明 ----------------

extern EventState sys_state; ///< 整个 MCU 系统的状态


// ----------------
#endif // __SERVICE_CFG_HPP__
