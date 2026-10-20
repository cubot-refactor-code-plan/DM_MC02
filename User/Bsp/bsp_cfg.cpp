#include "bsp_cfg.hpp"

#include "FreeRTOS.h" // IWYU pragma: keep (configASSERT)

/**
 * @brief BSP 层统一初始化
 *
 * @note  必须在 FreeRTOS 内核启动后调用（这些驱动内部要创建 RTOS 对象）。
 *        每一条驱动的 init() 都已在下面用 configASSERT 验过，失败就地停机，因此本函数无返回值。
 *        串口的模板实例化只能在 bsp_uart.cpp，其余 BSP 全局实例都在 bsp_cfg.cpp 中定义。
 *        本函数初始化的外设：
 *
 *        ✅ DWT        → bsp_dwt.init()       [内核 CYCCNT 计时，无外设]
 *        ✅ CAN1/2/3   → bsp_can1/2/3.init()  [Message Buffer 收发]
 *        ✅ USART1     → bsp_uart1.init()     [IDLE RX DMA + FreeRTOS stream buffer]
 *        ✅ USART2     → bsp_uart2.init()     [RS485，硬件 DE；IDLE RX DMA + TX DMA]
 *        ✅ USART3     → bsp_uart3.init()     [IDLE RX DMA + FreeRTOS stream buffer]
 *        ✅ UART4      → bsp_uart4.init()     [IDLE RX DMA + FreeRTOS stream buffer]
 *        ✅ UART5      → bsp_uart_it5.init()  [SBUS 中断收发]
 *        ✅ UART7      → bsp_uart7.init()     [IDLE RX DMA + FreeRTOS stream buffer]
 *        ✅ UART8      → bsp_uart8.init()     [IDLE RX IT + FreeRTOS stream buffer]
 *        ✅ UART9      → bsp_uart9.init()     [IDLE RX DMA + FreeRTOS stream buffer]
 *        ✅ USART10    → bsp_uart10.init()    [IDLE RX DMA + FreeRTOS stream buffer]
 *        ✅ GPIO       → MX_GPIO_Init()       [BspGpio 仅封装]
 *        ✅ KEY (PA15) → key_user.init(...)   [纯软件轮询消抖，无 ISR；200ms 轮询 → 200ms 消抖, 1s 长按]
 *        ✅ PWM×6      → bsp_pwm*.init()      [TIM1/2/3/12 各通道，先清 CCR 再启动 → 上电 0% 占空比]
 *
 */
void bsp_init(void)
{
  // DWT 计时（内核 CYCCNT，不依赖 FreeRTOS/中断）：最先初始化，供其它驱动记时间戳
  configASSERT(bsp_dwt.init() == Status::OK);

  // CAN（init() 失败则这一路不可用，用 configASSERT 立刻停机）
  configASSERT(bsp_can1.init() == Status::OK);
  configASSERT(bsp_can2.init() == Status::OK);
  configASSERT(bsp_can3.init() == Status::OK);

  // UART（同样依赖 FreeRTOS 流缓冲区，失败则立刻停机）
  configASSERT(bsp_uart1.init() == Status::OK);
  configASSERT(bsp_uart2.init() == Status::OK); // USART2：RS485 硬件 DE
  configASSERT(bsp_uart3.init() == Status::OK);
  configASSERT(bsp_uart4.init() == Status::OK);
  configASSERT(bsp_uart7.init() == Status::OK);
  configASSERT(bsp_uart8.init() == Status::OK);
  configASSERT(bsp_uart9.init() == Status::OK);
  configASSERT(bsp_uart10.init() == Status::OK);

  // UART5 = SBUS：帧格式校验由 DeviceR9ds::init() 负责
  configASSERT(bsp_uart_it5.init() == Status::OK);

  // PWM×6（参数取自 CubeMX；先清 CCR 再启动 → 上电全部 0% 占空比）
  configASSERT(bsp_pwm1.init() == Status::OK);       // PE13 TIM1_CH3  排针预留（舵机 50 Hz）
  configASSERT(bsp_pwm2.init() == Status::OK);       // PE9  TIM1_CH1  排针预留（舵机 50 Hz）
  configASSERT(bsp_pwm3.init() == Status::OK);       // PA2  TIM2_CH3  排针预留（舵机 50 Hz）
  configASSERT(bsp_pwm4.init() == Status::OK);       // PA0  TIM2_CH1  排针预留（舵机 50 Hz）
  configASSERT(bsp_pwm_gyro.init() == Status::OK);   // PB1  TIM3_CH4  陀螺仪
  configASSERT(bsp_pwm_buzzer.init() == Status::OK); // PB15 TIM12_CH2 无源蜂鸣器

  // 按键（纯软件轮询消抖，200ms轮询 → 200ms消抖, 1s长按）
  configASSERT(key_user.init({KEY_GPIO_Port, KEY_Pin, true, 1U, 5U}) == Status::OK);
}


// ---------------- DWT ----------------

BspDwt bsp_dwt; ///< 内核 CYCCNT 计时（CPU 频率由 init() 从 RCC 读出，没有配置项）


// ----------------
// ---------------- CAN ----------------

BspCan bsp_can1({&hfdcan1, "CAN1"}); ///< 构造参数：{句柄, 调试名}
BspCan bsp_can2({&hfdcan2, "CAN2"});
BspCan bsp_can3({&hfdcan3, "CAN3"});


// ----------------
// ---------------- UART ----------------

/**
 * @brief UART 全局实例，构造参数为 {句柄, 模式, 波特率}
 *
 * @note __attribute__((section(".dma_buffer"))) 把这些实例放进 .dma_buffer 段：
 *       该段在 STM32H723XG_FLASH.ld 中定义在 RAM_D1，而普通全局变量所在的 .bss 在 DTCMRAM；
 *       DTCM 不能被 DMA 访问，DMA 实例里含收发缓冲区，所以必须放到这里。
 *       中断实例不要求放这里；bsp_uart8 测试期间仍沿用该段，便于与 DMA 实例统一。
 *
 * @note Config 里的 baudrate 只给发送侧用（算一包发完要多久、判 TX 卡不卡死）；接收侧的波特率
 *       与帧格式一律由 CubeMX 写进 huartX，本驱动一个字段都不改 —— 所以协议参数要么在 CubeMX
 *       里配，要么就是错的，不能在 Config 里改。
 */
__attribute__((section(".dma_buffer")))
BspUartT<uart_detail::DmaTransport, 128, 128> bsp_uart1({&huart1, UartMode::TxRx, 115200U});

__attribute__((section(".dma_buffer")))
BspUartT<uart_detail::DmaTransport, 128, 128> bsp_uart2({&huart2, UartMode::TxRx, 115200U}); // RS485，DE 由 USART2 硬件控制

__attribute__((section(".dma_buffer")))
BspUartT<uart_detail::DmaTransport, 128, 128> bsp_uart3({&huart3, UartMode::TxRx, 115200U});

__attribute__((section(".dma_buffer")))
BspUartT<uart_detail::DmaTransport, 128, 128> bsp_uart4({&huart4, UartMode::TxRx, 115200U});

__attribute__((section(".dma_buffer")))
BspUartT<uart_detail::DmaTransport, 128, 128> bsp_uart7({&huart7, UartMode::TxRx, 921600U});

__attribute__((section(".dma_buffer")))
BspUartT<uart_detail::ItTransport, 128, 128> bsp_uart8({&huart8, UartMode::TxRx, 921600U});

__attribute__((section(".dma_buffer")))
BspUartT<uart_detail::DmaTransport, 128, 128> bsp_uart9({&huart9, UartMode::TxRx, 921600U});

__attribute__((section(".dma_buffer")))
BspUartT<uart_detail::DmaTransport, 128, 128> bsp_uart10({&huart10, UartMode::TxRx, 921600U});

// UART5 = SBUS：只收不发，中断接收；不使用 DMA
BspUartT<uart_detail::ItTransport, 128, 32> bsp_uart_it5({&huart5, UartMode::RxOnly, 100000U});


// ----------------
// ---------------- GPIO 输出引脚 ----------------

// 电源控制
BspGpio power_24v_2({POWER_24V_2_GPIO_Port, POWER_24V_2_Pin}); // PC13
BspGpio power_24v_1({POWER_24V_1_GPIO_Port, POWER_24V_1_Pin}); // PC14
BspGpio power_5v({POWER_5V_GPIO_Port, POWER_5V_Pin});          // PC15

// IMU 片选
BspGpio gyro_acc_cs({GYRO_ACC_CS_GPIO_Port, GYRO_ACC_CS_Pin});    // PC0
BspGpio gyro_gyro_cs({GYRO_GYRO_CS_GPIO_Port, GYRO_GYRO_CS_Pin}); // PC3

// BTB 扩展 IO
BspGpio btb_pa5({BTB_PA5_GPIO_Port, BTB_PA5_Pin});    // PA5
BspGpio btb_pe14({BTB_PE14_GPIO_Port, BTB_PE14_Pin}); // PE14
BspGpio btb_pd10({BTB_PD10_GPIO_Port, BTB_PD10_Pin}); // PD10

// SPI1 片选（原 LCD 接口，经反向线延伸到二层）
BspGpio spi1_cs({SPI1_CS_GPIO_Port, SPI1_CS_Pin}); // PE15


// ----------------
// ---------------- PWM 通道 ----------------

/**
 * @brief 排针预留的 4 路舵机 PWM
 *
 * @note 参数取自 CubeMX：定时器时钟 275 MHz（APB 137.5 MHz × 2），PSC = 275-1、ARR = 20000-1
 *       ⇒ 计数器 1 MHz、周期 20000 个计数 = **50 Hz / 20 ms，1 个计数正好 1 µs**，
 *       于是脉宽可以直接按 µs 理解：set_pulse_us(1500.0f) 就是舵机中位。
 * @note PSC 由 MX_TIM1_Init() / MX_TIM2_Init() 写入，本驱动只用它做换算、不改寄存器；
 *       这两个数必须与 CubeMX 保持一致，否则脉宽/频率换算是错的（bsp_pwm.hpp 有 @warning）。
 *
 * @warning 同一定时器的两个通道共用计数器与 ARR：PE9/PE13 必须同频，PA0/PA2 必须同频。
 */
BspPwm bsp_pwm1({&htim1, TIM_CHANNEL_3, 275000000UL, 274U, 19999U}); // PE13 TIM1_CH3
BspPwm bsp_pwm2({&htim1, TIM_CHANNEL_1, 275000000UL, 274U, 19999U}); // PE9  TIM1_CH1
BspPwm bsp_pwm3({&htim2, TIM_CHANNEL_3, 275000000UL, 274U, 19999U}); // PA2  TIM2_CH3
BspPwm bsp_pwm4({&htim2, TIM_CHANNEL_1, 275000000UL, 274U, 19999U}); // PA0  TIM2_CH1

///< 陀螺仪 PWM：TIM3_CH4 PB1
BspPwm bsp_pwm_gyro({&htim3, TIM_CHANNEL_4, 275000000UL, 23U, 9999U});

///< 无源蜂鸣器：TIM12_CH2 PB15（由 Device 层 DeviceBuzzer 驱动）
BspPwm bsp_pwm_buzzer({&htim12, TIM_CHANNEL_2, 275000000UL, 23U, 1999U});


// ----------------
// ---------------- 按键 ----------------

BspKey key_user; ///< 用户按键 PA15，低有效


// ----------------