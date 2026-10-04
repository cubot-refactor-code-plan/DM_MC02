#include "device_cfg.hpp"

#include "bsp_cfg.hpp" // IWYU pragma: keep（bsp_pwm_buzzer）


// ---------------- 全局实例 ----------------


DeviceBuzzer buzzer(bsp_pwm_buzzer);

// CAN3 上的 DJI 电机组：按反馈标识符 0x201 ~ 0x20B 的顺序填 11 位，空位写 nullptr。
// 位置 0~3 放 M3508/M2006；位置 4~7 三种都能放；位置 8~10 只能放 GM6020（电流模式）。
// 当前只挂 1 台 GM6020（必须电流模式）：电调 ID 1 → 反馈 0x205 → 位置 4（控制帧 0x1FE 槽位 0）。
static const DjiMotorGroup::Member gm6020_1 {DjiMotorModel::GM6020};

DjiMotorGroup can3_dji_group({&bsp_can3,
                              nullptr, nullptr, nullptr, nullptr,
                              &gm6020_1, nullptr, nullptr, nullptr,
                              nullptr, nullptr, nullptr});


// ----------------
// ---------------- 函数定义 ----------------


/** @brief 逐个初始化本层设备，返回第一个失败的 Status */
Status device_init(void)
{
  return can3_dji_group.init();
}


// ----------------
