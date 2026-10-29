#include "device_cfg.hpp"

#include "bsp_cfg.hpp" // IWYU pragma: keep（bsp_pwm_buzzer）
#include "FreeRTOS.h"  // IWYU pragma: keep (configASSERT)
#include "status.hpp"  // IWYU pragma: keep（device_init 里的 Status）


// ---------------- 全局实例 ----------------


DeviceBuzzer buzzer(bsp_pwm_buzzer);

// CAN3 上的 DJI 电机组：按反馈标识符 0x201 ~ 0x20B 的顺序填 11 位，空位写 nullptr。
// 当前挂 2 台 GM6020（必须电流模式）：
//   位置 4 → 反馈 0x205（电调 ID 1）→ 控制帧 0x1FE 槽 0
//   位置 9 → 反馈 0x20A（电调 ID 6）→ 控制帧 0x2FE 槽 1
// 位置 / 电调 ID / 控制帧对照表见 dji_motor_group.md。
static const DjiMotorGroupT<decltype(bsp_can3)>::Member gm6020_1 { .model = DjiMotorModel::GM6020 };
static const DjiMotorGroupT<decltype(bsp_can3)>::Member gm6020_6 { .model = DjiMotorModel::GM6020 };

// `.member` 数组内部仍按位置写（C++ 指定初始化器不支持数组下标），
// 但外层字段名已指明语义，不再靠数位置。
DjiMotorGroupT<decltype(bsp_can3)> can3_dji_group({
  .can    = &bsp_can3,
  .member = {nullptr, nullptr, nullptr, nullptr, &gm6020_1, nullptr, nullptr, nullptr, nullptr, &gm6020_6, nullptr},
});

// R9DS 遥控接收机：走 UART5（SBUS，100000/8E2 + 外部反相），由 bsp_uart_it5 中断接收，不占 DMA。
// 四个摇杆的零飘偏移用默认全 0：死区已经把"没动杆但不在 1000"处理掉了。
DeviceR9dsT<decltype(bsp_uart_it5)> r9ds(bsp_uart_it5);

// EmmV5 多电机组：一条总线挂多台，地址 1、2 为 CoreXY 两轴，3 为独立旋转轴。
// 地址必须与电机侧 UartAddr 一致；CoreXY 模块只用 1、2（见 module_cfg.cpp）。
DeviceEmmv5GroupT<decltype(bsp_uart2)> emm_group({ .uart = bsp_uart2, .addr = {1, 2, 3} });

// 舵机：BSP 层那 4 路排针 PWM 已配成 50 Hz / 20 ms、1 计数 = 1 µs，脉宽可直接按 µs 填。
// Config 参数顺序 = {通道, 脉宽下限, 脉宽上限, 行程, 限速, 加速度, 反向, 微调}。
// ⚠ 脉宽端点与限速/加速度是**占位值，等实机标定**；限速写 0 = 不限速。
// ⚠ 必须由同一个任务周期调用 servoN.update()（唯一的脉宽写入者），详见 device_servo.md。
DeviceServo servo1({ .pwm = &bsp_pwm1, .pulse_min_us = 500.0f, .pulse_max_us = 2500.0f, .range_deg = 270.0f, .max_speed_dps = 0.0f, .max_accel_dps2 = 0.0f });
DeviceServo servo2({ .pwm = &bsp_pwm2, .pulse_min_us = 500.0f, .pulse_max_us = 2500.0f, .range_deg = 180.0f, .max_speed_dps = 300.0f, .max_accel_dps2 = 1500.0f });
DeviceServo servo3({ .pwm = &bsp_pwm3, .pulse_min_us = 500.0f, .pulse_max_us = 2500.0f, .range_deg = 180.0f, .max_speed_dps = 300.0f, .max_accel_dps2 = 1500.0f });
DeviceServo servo4({ .pwm = &bsp_pwm4, .pulse_min_us = 500.0f, .pulse_max_us = 2500.0f, .range_deg = 180.0f, .max_speed_dps = 300.0f, .max_accel_dps2 = 1500.0f });


// ----------------
// ---------------- 函数定义 ----------------


/** @brief 逐个初始化本层设备：失败就地停机 */
void device_init(void)
{
  // can3_dji_group 的校验与折算已在构造阶段完成（成员表错误会在全局构造时 assert 停机）
  configASSERT(r9ds.init() == Status::OK);
  configASSERT(emm_group.init() == Status::OK);

  // 舵机：init() 只校验配置、不写脉宽，所以这一步不会让舵机动起来
  configASSERT(servo1.init() == Status::OK); // PE13 TIM1_CH3
  configASSERT(servo2.init() == Status::OK); // PE9  TIM1_CH1
  configASSERT(servo3.init() == Status::OK); // PA2  TIM2_CH3
  configASSERT(servo4.init() == Status::OK); // PA0  TIM2_CH1
}


// ----------------
