#include "module_cfg.hpp"

#include "FreeRTOS.h" // IWYU pragma: keep (configASSERT)
#include "status.hpp" // IWYU pragma: keep


// ---------------- 全局实例 ----------------


// CoreXY：USART2（RS485）地址 1、2；上电清零后世界原点 = 上电位置。
// 理论限位 (0,0)~(190,200)，安全限位 (1,1)~(189,199)，默认 300 RPM / acc 230 / 到位窗口 0.20 mm。
// 几何：2GT 节距 2 mm × 20 齿 = 40 mm/圈；16 细分 = 3200 脉冲/圈；编码器 65536 counts/圈。
CorexyT<decltype(emm_group)>
  corexy({&emm_group, 1U, 2U, {0.0f, 0.0f, 190.0f, 200.0f}, {1.0f, 1.0f, 189.0f, 199.0f}, 40.0f, 3200U, 65536U, 300U, 230U, 0.20f});


// ----------------
// ---------------- 函数定义 ----------------


/** @brief 初始化模块层：只校验配置，不发命令 */
void module_init(void)
{
  configASSERT(corexy.init() == Status::OK);
}

// ----------------
