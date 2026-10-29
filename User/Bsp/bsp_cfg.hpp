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

// ----------------
// ---------------- 全局实例 ----------------

// 全部实例都是 C++17 **inline 变量**：定义只写在这里一次，不再需要 .cpp 里再定义一遍，
// 也不会出现「extern 声明类型 / .cpp 定义类型不同步」这类问题。
// 新增或修改一路串口，只改本文件的一行。
//
// ⚠ .dma_buffer 落在 RAM_D1（DMA1/DMA2 访问不到 DTCM）。链接脚本里该段是**可加载段**
//   （>RAM_D1 AT> FLASH），启动时代码会先把初值拷进 RAM_D1、再跑 .init_array，
//   因此常量初始化和动态初始化都正确 —— 这是它不能写成 NOLOAD 的原因。

inline BspDwt bsp_dwt {}; ///< 内核 CYCCNT 计时（CPU 频率由 init() 从 RCC 读出，没有配置项）


// CAN：{.hfdcan = 句柄, .name = 调试名}
inline BspCanT<16, 16> bsp_can1({.hfdcan = &hfdcan1, .name = "CAN1"}); ///< CAN1
inline BspCanT<16, 16> bsp_can2({.hfdcan = &hfdcan2, .name = "CAN2"}); ///< CAN2
inline BspCanT<16, 16> bsp_can3({.hfdcan = &hfdcan3, .name = "CAN3"}); ///< CAN3


// UART：{.huart = 句柄, .mode = 收发模式, .baudrate = 波特率}
// Config::baudrate 只用于估算 TX 卡死阈值；真正的波特率与帧格式由 CubeMX 写进 huartX。
// 中断实例不强制放 .dma_buffer，bsp_uart8 沿用该段便于统一。
inline __attribute__((section(".dma_buffer.bsp_uart1"))) BspUartT<uart_detail::DmaTransport, 128, 128> bsp_uart1({.huart = &huart1, .mode = UartMode::TxRx, .baudrate = 115200U}); ///< USART1

inline __attribute__((section(".dma_buffer.bsp_uart2"))) BspUartT<uart_detail::DmaTransport, 128, 128> bsp_uart2({.huart = &huart2, .mode = UartMode::TxRx, .baudrate = 921600U}); ///< USART2（RS485，硬件DE）

inline __attribute__((section(".dma_buffer.bsp_uart3"))) BspUartT<uart_detail::DmaTransport, 128, 128> bsp_uart3({.huart = &huart3, .mode = UartMode::TxRx, .baudrate = 115200U}); ///< USART3

inline __attribute__((section(".dma_buffer.bsp_uart4"))) BspUartT<uart_detail::DmaTransport, 128, 128> bsp_uart4({.huart = &huart4, .mode = UartMode::TxRx, .baudrate = 115200U}); ///< UART4

inline __attribute__((section(".dma_buffer.bsp_uart7"))) BspUartT<uart_detail::DmaTransport, 128, 128> bsp_uart7({.huart = &huart7, .mode = UartMode::TxRx, .baudrate = 921600U}); ///< UART7

inline __attribute__((section(".dma_buffer.bsp_uart8"))) BspUartT<uart_detail::ItTransport, 128, 128> bsp_uart8({.huart = &huart8, .mode = UartMode::TxRx, .baudrate = 921600U}); ///< UART8（可改DMA收发）

inline __attribute__((section(".dma_buffer.bsp_uart9"))) BspUartT<uart_detail::DmaTransport, 128, 128> bsp_uart9({.huart = &huart9, .mode = UartMode::TxRx, .baudrate = 921600U}); ///< UART9

inline __attribute__((section(".dma_buffer.bsp_uart10"))) BspUartT<uart_detail::DmaTransport, 128, 128> bsp_uart10({.huart = &huart10, .mode = UartMode::TxRx, .baudrate = 921600U}); ///< USART10

// UART5 = SBUS：只收不发，中断接收；不使用 DMA
inline BspUartT<uart_detail::ItTransport, 128, 32> bsp_uart_it5({.huart = &huart5, .mode = UartMode::RxOnly, .baudrate = 100000U}); ///< 接收机（SBUS，中断发，TX 只用 32）


// GPIO 输出引脚：{.port = 端口, .pin = 引脚掩码}
inline BspGpio power_24v_2({.port = POWER_24V_2_GPIO_Port, .pin = POWER_24V_2_Pin}); ///< 电源控制 PC13
inline BspGpio power_24v_1({.port = POWER_24V_1_GPIO_Port, .pin = POWER_24V_1_Pin}); ///< 电源控制 PC14
inline BspGpio power_5v({.port = POWER_5V_GPIO_Port, .pin = POWER_5V_Pin});          ///< 电源控制 PC15

inline BspGpio gyro_acc_cs({.port = GYRO_ACC_CS_GPIO_Port, .pin = GYRO_ACC_CS_Pin});    ///< IMU加速度计片选 PC0
inline BspGpio gyro_gyro_cs({.port = GYRO_GYRO_CS_GPIO_Port, .pin = GYRO_GYRO_CS_Pin}); ///< IMU陀螺仪片选 PC3

inline BspGpio btb_pa5({.port = BTB_PA5_GPIO_Port, .pin = BTB_PA5_Pin});    ///< BTB 扩展 IO PA5
inline BspGpio btb_pe14({.port = BTB_PE14_GPIO_Port, .pin = BTB_PE14_Pin}); ///< BTB 扩展 IO PE14
inline BspGpio btb_pd10({.port = BTB_PD10_GPIO_Port, .pin = BTB_PD10_Pin}); ///< BTB 扩展 IO PD10

inline BspGpio spi1_cs({.port = SPI1_CS_GPIO_Port, .pin = SPI1_CS_Pin}); ///< SPI1 片选（原 LCD 接口，PE15）


// PWM 通道：{.htim, .channel, .timer_clk_hz, .prescaler, .period[, .duty_pct]}
// 本驱动不改寄存器；寄存器信息必须与 CubeMX 保持一致，否则换算是错的
// 同一定时器的两个通道共用计数器与 ARR：PE9/PE13 必须同频，PA0/PA2 必须同频。
inline BspPwm bsp_pwm1({.htim = &htim1, .channel = TIM_CHANNEL_3, .timer_clk_hz = 275000000UL, .prescaler = 274U, .period = 19999U}); ///< PE13 TIM1_CH3 排针预留（舵机 50 Hz）
inline BspPwm bsp_pwm2({.htim = &htim1, .channel = TIM_CHANNEL_1, .timer_clk_hz = 275000000UL, .prescaler = 274U, .period = 19999U}); ///< PE9  TIM1_CH1 排针预留（舵机 50 Hz）
inline BspPwm bsp_pwm3({.htim = &htim2, .channel = TIM_CHANNEL_3, .timer_clk_hz = 275000000UL, .prescaler = 274U, .period = 19999U}); ///< PA2  TIM2_CH3 排针预留（舵机 50 Hz）
inline BspPwm bsp_pwm4({.htim = &htim2, .channel = TIM_CHANNEL_1, .timer_clk_hz = 275000000UL, .prescaler = 274U, .period = 19999U}); ///< PA0  TIM2_CH1 排针预留（舵机 50 Hz）

inline BspPwm bsp_pwm_gyro({.htim = &htim3, .channel = TIM_CHANNEL_4, .timer_clk_hz = 275000000UL, .prescaler = 23U, .period = 9999U});    ///< PB1  TIM3_CH4  陀螺仪
inline BspPwm bsp_pwm_buzzer({.htim = &htim12, .channel = TIM_CHANNEL_2, .timer_clk_hz = 275000000UL, .prescaler = 23U, .period = 1999U}); ///< PB15 TIM12_CH2 无源蜂鸣器（由 Device 层 DeviceBuzzer 驱动）


// 按键：配置在构造时存入，init() 只做校验 + 读一次初始电平（要等 MX_GPIO_Init 跑过）
inline BspKey key_user({.port = KEY_GPIO_Port, .pin = KEY_Pin, .active_low = true, .debounce_cnt = 1U, .long_press_cnt = 5U}); ///< 用户按键 PA15，低有效

// ----------------


#endif // __BSP_CFG_HPP__
