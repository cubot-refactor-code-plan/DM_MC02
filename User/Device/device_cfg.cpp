#include "device_cfg.hpp"

#include "service_cfg.hpp"


// ---------------- 全局实例 ----------------

///< 蜂鸣器：PWM 通道（bsp_pwm_buzzer）已由 bsp_init() 启动，这里只做绑定
DeviceBuzzer buzzer(bsp_pwm_buzzer);

///< CAN3：达妙 IMU（CAN_ID 0x58 / MST_ID 0x59，在线超时 100 ms）
///< 每次上电发一次「设间隔 1 ms + 切主动模式」报文；save_params = false，不写模块 flash
DmImu dm_imu({bus_can3, 0x58U, 0x59U, 100U, 1U});

///< CAN3：M2006 + C610（ID 1；ratio 传 0 = 用型号默认减速比 36）
DjiMotor<Motor2006> m2006({bus_can3, 1U});

///< CAN3：GM6020（ID 1，电流控制模式；ratio 传 0 = 1）
DjiMotor<Motor6020> gm6020({bus_can3, 1U, 0.0f, 0.0f, DjiMotorControlMode::CURRENT});


// ----------------
// ---------------- 初始化函数 ----------------

/**
 * @brief 设备层统一初始化
 *
 * @note 必须在调度器启动前调用（all_init() 中位于 can_bus_init() 之后）：CanBus 的
 *       节点注册表在 can_rx_task 启动后立即冻结，之后再注册一律失败；
 *       各设备的 init() 内部已用 sys_init_error() 上报失败，这里不重复处理。
 *
 * @note 蜂鸣器无需初始化：它的 PWM 通道由 bsp_init() 启动。
 *
 *       DeviceEmmV5 这类“带到位信号量”的设备接入时的顺序（参考）：
 *         1. api_main 创建到位信号量 motor_xy_sem / motor_z_sem
 *         2. emm_motor_*.init()
 *         3. emm_motor_*.set_in_pos_sem(...) 绑定到位信号量
 *         4. DeviceEmmV5::create_rx_tasks() 创建每路电机的到位接收任务
 */
void device_init()
{
  (void)dm_imu.init();  // 注册 0x59 接收节点 + 发一次「设间隔 1 ms + 切主动模式」报文（不写 flash）
  (void)m2006.init();   // 注册 0x200 槽位 0 + 反馈节点 0x201
  (void)gm6020.init();  // 注册 0x1FE 槽位 0 + 反馈节点 0x205
}

// ----------------
