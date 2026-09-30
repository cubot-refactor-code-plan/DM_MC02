/**
 * @file bsp_cfg.hpp
 * @author ALL
 * @brief BSP 层的全局初始化入口与全局实例声明
 * @version 0.1
 * @date 2026-04-18
 *
 * @copyright Copyright (c) 2026
 */

#ifndef __BSP_CFG_HPP__
#define __BSP_CFG_HPP__

#include "bsp_buzzer.hpp"
#include "bsp_can.hpp"
#include "bsp_dwt.hpp"
#include "bsp_gpio.hpp"
#include "bsp_key.hpp"
#include "bsp_uart.hpp"


// ---------------- 函数声明 ----------------

/**
 * @brief BSP 层统一初始化
 *
 * @note 须在 FreeRTOS 内核启动后调用（各驱动内部要创建 RTOS 对象）
 */
void bsp_init();


// ----------------
// ---------------- 全局声明 ----------------

extern BspDwt bsp_dwt; ///< 内核 CYCCNT 计时（不占外设、不依赖中断）


extern BspCan bsp_can1; ///< CAN1
extern BspCan bsp_can2; ///< CAN2
extern BspCan bsp_can3; ///< CAN3


extern BspUart<128> bsp_uart1;  ///< USART1
extern BspUart<128> bsp_uart3;  ///< USART3
extern BspUart<128> bsp_uart4;  ///< UART4
extern BspUart<128> bsp_uart5;  ///< UART5（只接收，未配 TX DMA）
extern BspUart<128> bsp_uart7;  ///< UART7
extern BspUart<128> bsp_uart8;  ///< UART8
extern BspUart<128> bsp_uart9;  ///< UART9
extern BspUart<128> bsp_uart10; ///< USART10


extern BspGpio power_24v_2;  ///< 电源控制
extern BspGpio power_24v_1;  ///< 电源控制
extern BspGpio power_5v;     ///< 电源控制
extern BspGpio gyro_acc_cs;  ///< IMU 加速度计片选
extern BspGpio gyro_gyro_cs; ///< IMU 陀螺仪片选
extern BspGpio btb_gpio;     ///< BTB 扩展 IO
extern BspGpio lcd_cs;       ///< LCD 片选
extern BspGpio lcd_blk;      ///< LCD 背光
extern BspGpio lcd_res;      ///< LCD 复位
extern BspGpio lcd_dc;       ///< LCD 数据/命令选择


extern BspBuzzer bsp_buzzer; ///< 无源蜂鸣器（TIM12 CH2 / PB15）
extern BspKey    key_user;   ///< 用户按键（PA15，低有效）

// ----------------


#endif // __BSP_CFG_HPP__