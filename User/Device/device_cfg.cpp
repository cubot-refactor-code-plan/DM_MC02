#include "device_cfg.hpp"

#include "FreeRTOS.h" // IWYU pragma: keep (configASSERT)
#include "bsp_cfg.hpp"  // IWYU pragma: keep（bsp_pwm_buzzer）


// ---------------- 全局实例 ----------------


DeviceBuzzer buzzer(bsp_pwm_buzzer);

// CAN3 上的 DJI 电机组：按反馈标识符 0x201 ~ 0x20B 的顺序填 11 位，空位写 nullptr。
// 位置 0~3 放 M3508/M2006；位置 4~7 三种都能放；位置 8~10 只能放 GM6020（电流模式）。
// 当前挂 2 台 GM6020（必须电流模式），都在混挂区、同走控制帧 0x1FE：
//   电调 ID 1 → 反馈 0x205 → 位置 4 → 0x1FE 槽位 0
//   电调 ID 2 → 反馈 0x206 → 位置 5 → 0x1FE 槽位 1
static const DjiMotorGroup::Member gm6020_1 {DjiMotorModel::GM6020};
static const DjiMotorGroup::Member gm6020_6 {DjiMotorModel::GM6020};

DjiMotorGroup can3_dji_group({&bsp_can3,
                              nullptr, nullptr, nullptr, nullptr,
                              &gm6020_1, nullptr, nullptr, nullptr,
                              nullptr, &gm6020_6, nullptr});

// R9DS 遥控接收机：走 UART5（SBUS，100000/8E2 + 外部反相，见 bsp_cfg.cpp 的 bsp_uart5）。
// 四个摇杆的零飘偏移用默认全 0：死区已经把"没动杆但不在 1000"处理掉了。
DeviceR9ds r9ds(bsp_uart5);

// 舵机：BSP 层那 4 路排针预留的 PWM 已经配成 50 Hz / 20 ms、1 计数 = 1 µs，
// 所以脉宽可以直接按 µs 填。Config 参数顺序 = {通道, 脉宽下限, 脉宽上限, 行程, 限速, 加速度, 反向, 微调}。
// ⚠ 脉宽端点与限速/加速度仍是**占位值，等实机标定**：
//   - 500 ~ 2500 µs 是常见标称值，手上这只未必一样（先 set_pulse_us() 试出真实端点再改）；
//   - 限速 300 °/s + 加速度 1500 °/s² 是一组保守起步值（270° 走满约 1.1 s：0.2 s 加速 30°
//     + 0.7 s 匀速 210° + 0.2 s 减速 30°）；
//     限速写 0 就是不限速（update() 下一拍直接把脉宽落到目标，与 BspPwm 的瞬时语义一致）；
//   - 能用起来的前提：由**一个任务周期调用 servoN.update()**，它是唯一的脉宽写入者
//     （线程模型见 device_servo.hpp 文件头，set_angle() / off() 可以跨任务）。
DeviceServo servo1({&bsp_pwm1, 500.0f, 2500.0f, 270.0f, 300.0f, 1500.0f});
DeviceServo servo2({&bsp_pwm2, 500.0f, 2500.0f, 180.0f, 300.0f, 1500.0f});
DeviceServo servo3({&bsp_pwm3, 500.0f, 2500.0f, 180.0f, 300.0f, 1500.0f});
DeviceServo servo4({&bsp_pwm4, 500.0f, 2500.0f, 180.0f, 300.0f, 1500.0f});


// ----------------
// ---------------- 函数定义 ----------------


/** @brief 逐个初始化本层设备：失败就地停机 */
void device_init(void)
{
  configASSERT(can3_dji_group.init() == Status::OK);
  configASSERT(r9ds.init() == Status::OK);

  // 舵机：init() 只校验配置、不写脉宽，所以这一步不会让舵机动起来
  configASSERT(servo1.init() == Status::OK); // PE13 TIM1_CH3
  configASSERT(servo2.init() == Status::OK); // PE9  TIM1_CH1
  configASSERT(servo3.init() == Status::OK); // PA2  TIM2_CH3
  configASSERT(servo4.init() == Status::OK); // PA0  TIM2_CH1
}


// ----------------
