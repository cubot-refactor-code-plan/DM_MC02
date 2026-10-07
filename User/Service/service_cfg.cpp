#include "service_cfg.hpp"

#include "FreeRTOS.h" // IWYU pragma: keep (configASSERT)


// ---------------- 函数定义 ----------------


/** @brief Service 层统一初始化：失败就地停机 */
void service_init(void)
{
  configASSERT(sys_state.init() == Status::OK);
}


// ----------------
// ---------------- 全局实例 ----------------


EventState sys_state; ///< 整个 MCU 系统的状态


// ----------------
