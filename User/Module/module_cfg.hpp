/**
 * @file module_cfg.hpp
 * @author Rh
 * @brief 模块层统一管理 —— module_init() 与全局实例声明
 * @version 0.1
 * @date 2026-10-22
 *
 * @copyright Copyright (c) 2026
 *
 * @note 模块实例在此声明、在 module_cfg.cpp 中实例化；本层的初始化动作全部收进
 *       module_init()，与 bsp_init() / device_init() 对齐。
 */

#ifndef __MODULE_CFG_HPP__
#define __MODULE_CFG_HPP__

#include "corexy.hpp"
#include "device_cfg.hpp"


// ---------------- 函数声明 ----------------

/**
 * @brief 初始化模块层
 *
 * @note 只做配置校验，不发串口命令（清零编码器由 corexy_task 启动阶段完成）；
 *       失败在这里 configASSERT 就地停机，因此不需要返回值。
 */
void module_init(void);

// ----------------
// ---------------- 全局声明 ----------------

/// CoreXY 模块类型（绑定到 emm_group 的具体电机组类型）
using CorexyModule = CorexyT<decltype(emm_group)>;

/// CoreXY 平面机构：emm_group 上的地址 1、2，理论限位 190×200，安全限位 (1,1)~(189,199)
extern CorexyModule corexy;

// ----------------
#endif // __MODULE_CFG_HPP__
