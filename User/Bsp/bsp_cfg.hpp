/**
 * @file bsp_cfg.hpp
 * @author ALL
 * @brief 作为bsp层的总初始化和extern部分，更方便管理和移植
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
#include "bsp_usb.hpp"


// ---------------- 函数声明 ----------------

void bsp_init();


// ----------------
// ---------------- 全局声明 ----------------

///< DWT 计时（内核 CYCCNT，无外设；bsp_init() 中最先初始化）
extern BspDwt bsp_dwt;


///< CAN 一共三个 都初始化了
extern BspCan bsp_can1;
extern BspCan bsp_can2;
extern BspCan bsp_can3;


///< UART
extern BspUart<128> bsp_uart1;
extern BspUart<128> bsp_uart3;
extern BspUart<128> bsp_uart4;
extern BspUart<128> bsp_uart5;
extern BspUart<128> bsp_uart7;
extern BspUart<128> bsp_uart8;
extern BspUart<128> bsp_uart9;
extern BspUart<128> bsp_uart10;


///< GPIO 输出引脚（电源控制、片选、BTB 扩展）
extern BspGpio power_24v_2;
extern BspGpio power_24v_1;
extern BspGpio power_5v;
extern BspGpio gyro_acc_cs;
extern BspGpio gyro_gyro_cs;
extern BspGpio spi1_cs;
extern BspGpio btb_pa5;
extern BspGpio btb_pe14;
extern BspGpio btb_pd10;


///< 蜂鸣器
extern BspBuzzer bsp_buzzer;

///< 按键
extern BspKey key_user;

///< USB
extern BspUsb &bsp_usb;


// ----------------
#endif // __BSP_CFG_HPP__
