/**
 * @file bsp_cfg.hpp
 * @author ALL
 * @brief 作为bsp层的总初始化和extern部分，更方便管理和移植
 * @version 0.1
 * @date 2026-04-18
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef __BSP_CFG_HPP__
#define __BSP_CFG_HPP__

#include "bsp_can.hpp"
#include "bsp_uart.hpp"


// ---------------- 函数声明 ----------------

void bsp_init();


// ----------------
// ---------------- 全局声明 ----------------

///< CAN 一共三个 都初始化了
extern BspCan bsp_can1;
extern BspCan bsp_can2;
extern BspCan bsp_can3;


///< USART
extern BspUart<64, 8> bsp_usart1;


///< GPIO 输出引脚（电源控制、片选等）
extern BspGpio power_24v_2;
extern BspGpio power_24v_1;
extern BspGpio power_5v;
extern BspGpio gyro_acc_cs;
extern BspGpio gyro_gyro_cs;
extern BspGpio btb_gpio;
extern BspGpio lcd_cs;
extern BspGpio lcd_blk;
extern BspGpio lcd_res;
extern BspGpio lcd_dc;


///< 蜂鸣器
extern BspBuzzer bsp_buzzer;

///< 按键
extern BspKey key_user;

// ----------------


#endif // __BSP_CFG_HPP__