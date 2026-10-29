#include "module_cfg.hpp"

#include "FreeRTOS.h" // IWYU pragma: keep (configASSERT)
#include "status.hpp" // IWYU pragma: keep


// ---------------- 全局实例 ----------------


// CoreXY：USART2（RS485）地址 1、2；上电清零后世界原点 = 上电位置。
// 理论限位 (0,0)~(190,200)，安全限位 (1,1)~(188,198)（x1/y1 距理论边有 2 mm 余量）。
// 几何（40 mm/圈、3200 脉冲/圈、65536 counts/圈）固定写在 CorexyT 的私有常量里。
CorexyT<decltype(emm_group)> corexy({
  .group      = &emm_group,
  .addr1      = 1U,
  .addr2      = 2U,
  .theory     = {0.0f, 0.0f, 190.0f, 200.0f},
  .safe       = {2.0f, 2.0f, 188.0f, 198.0f},
  .rpm        = 300U,
  .acc        = 230U,
  .arrive_tol = 0.20f,
});


// ----------------
// ---------------- 函数定义 ----------------


/** @brief 初始化模块层：只校验配置，不发命令 */
void module_init(void)
{
  configASSERT(corexy.init() == Status::OK);
}

// ----------------
