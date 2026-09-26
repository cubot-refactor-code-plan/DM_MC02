#include "device_cfg.hpp"


/* ==================== 全局实例化 ==================== */

/* ==================== 初始化函数 ==================== */

/**
 * @brief 设备层统一初始化
 *
 * @note LCD 在此初始化（复位、背光、寄存器配置，SPI1 DMA）；
 *       当前为底层库形态：电机不初始化（不绑定到位信号量、不创建接收任务）。
 *
 *       业务接入电机时的完整初始化顺序（参考）：
 *         1. api_main 创建到位信号量 motor_xy_sem / motor_z_sem
 *         2. emm_motor_*.init()
 *         3. emm_motor_*.set_in_pos_sem(...) 绑定到位信号量
 *         4. DeviceEmmV5::create_rx_tasks() 创建每路电机的到位接收任务
 */
void device_init()
{
  /* TODO: 电机业务接入时在此初始化（见上方注释） */
}
