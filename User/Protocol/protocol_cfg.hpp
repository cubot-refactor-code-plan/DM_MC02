/**
 * @file protocol_cfg.hpp
 * @author Rh
 * @brief 协议层配置 —— 全局协议实例 + 初始化入口
 * @version 0.1
 * @date 2026-08-14
 *
 * @copyright Copyright (c) 2026
 *
 * @details cfg 只负责两件事：
 *            - 创建协议类全局实例（在 protocol_cfg.cpp 定义）
 *            - 初始化：protocol_init() 调用各协议类的 init()
 *          其余逻辑全部封装在各自的协议类中。
 *
 * @note 当前为底层库形态，尚未绑定具体协议实例；
 *       接入上位机协议时仿照 bsp_cfg / device_cfg 的组织方式：
 *       实例化放 .cpp，extern 声明放 .hpp，init 统一在 protocol_init()。
 */

#ifndef __PROTOCOL_CFG_HPP__
#define __PROTOCOL_CFG_HPP__

// ---------------- 协议层初始化 ----------------

/**
 * @brief 协议层统一初始化
 * @note  在 osKernelStart() 之前调用（由 all_init() 调用）
 */
void protocol_init(void);


// ----------------
#endif // __PROTOCOL_CFG_HPP__
