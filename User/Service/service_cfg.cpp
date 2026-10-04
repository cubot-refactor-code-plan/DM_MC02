#include "service_cfg.hpp"


// ---------------- 函数定义 ----------------


/** @brief Service 层统一初始化：逐个初始化本层实例，返回第一个失败的状态码 */
Status service_init(void)
{
  return sys_state.init();
}


// ----------------
// ---------------- 全局实例 ----------------


EventState sys_state; ///< 整个 MCU 系统的状态


// ----------------
