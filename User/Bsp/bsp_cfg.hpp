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

#include "bsp_can.hpp"
#include "bsp_dwt.hpp"
#include "bsp_gpio.hpp"
#include "bsp_key.hpp"
#include "bsp_pwm.hpp"
#include "bsp_uart.hpp"


// ---------------- 函数声明 ----------------

/**
 * @brief BSP 层统一初始化
 *
 * @note 须在 FreeRTOS 内核启动后调用（各驱动内部要创建 RTOS 对象）。
 *       每一条驱动的 init() 都在函数内用 configASSERT 验过，失败就地停机，因此不需要返回值。
 */
void bsp_init(void);


// ----------------
// ---------------- 全局声明 ----------------

extern BspDwt bsp_dwt; ///< 内核 CYCCNT 计时（不占外设、不依赖中断）


extern BspCan bsp_can1; ///< CAN1
extern BspCan bsp_can2; ///< CAN2
extern BspCan bsp_can3; ///< CAN3


extern BspUart<128> bsp_uart1;  ///< USART1
extern BspUart<128> bsp_uart3;  ///< USART3
extern BspUart<128> bsp_uart4;  ///< UART4
extern BspUart<128> bsp_uart5;  ///< UART5（SBUS：100000/8E2 + 外部反相；只接收，未配 TX DMA）
extern BspUart<128> bsp_uart7;  ///< UART7
extern BspUart<128> bsp_uart8;  ///< UART8
extern BspUart<128> bsp_uart9;  ///< UART9
extern BspUart<128> bsp_uart10; ///< USART10


extern BspGpio power_24v_2;  ///< 电源控制
extern BspGpio power_24v_1;  ///< 电源控制
extern BspGpio power_5v;     ///< 电源控制
extern BspGpio gyro_acc_cs;  ///< IMU 加速度计片选
extern BspGpio gyro_gyro_cs; ///< IMU 陀螺仪片选
extern BspGpio btb_pa5;  ///< BTB 扩展 IO（PA5）
extern BspGpio btb_pe14; ///< BTB 扩展 IO（PE14）
extern BspGpio btb_pd10; ///< BTB 扩展 IO（PD10）
extern BspGpio spi1_cs;  ///< SPI1 片选（原 LCD 接口，PE15）


extern BspPwm bsp_pwm1; ///< PE13 TIM1_CH3 排针预留（舵机 50 Hz）
extern BspPwm bsp_pwm2; ///< PE9  TIM1_CH1 排针预留（舵机 50 Hz）
extern BspPwm bsp_pwm3; ///< PA2  TIM2_CH3 排针预留（舵机 50 Hz）
extern BspPwm bsp_pwm4; ///< PA0  TIM2_CH1 排针预留（舵机 50 Hz）


extern BspPwm bsp_pwm_gyro;   ///< PB1  TIM3_CH4  陀螺仪
extern BspPwm bsp_pwm_buzzer; ///< PB15 TIM12_CH2 无源蜂鸣器（由 Device 层 DeviceBuzzer 驱动）


extern BspKey key_user; ///< 用户按键（PA15，低有效）

// ----------------


#endif // __BSP_CFG_HPP__