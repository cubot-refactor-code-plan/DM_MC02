#include "service_cfg.hpp"

#include "bsp_cfg.hpp" // IWYU pragma: keep (bsp_can1/2/3)


// ---------------- 全局实例 ----------------

CanBus bus_can1({&bsp_can1}); ///< CAN1
CanBus bus_can2({&bsp_can2}); ///< CAN2
CanBus bus_can3({&bsp_can3}); ///< CAN3


// ----------------
