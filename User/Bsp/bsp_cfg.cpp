#include "bsp_cfg.hpp"

/**
 * @brief bsp层整体的初始化
 *
 * @note  必须在 FreeRTOS 内核启动后调用（因为大部分 BSP 驱动内部创建 RTOS 对象）。
 *        目前除了串口的模板实例化需要在 bsp_uart.cpp 中定义，其他 BSP 全局实例化都在 bsp_cfg.cpp 中定义。
 *        当前已初始化的外设一览：
 *
 *        ✅ DWT        → bsp_dwt.init()       [内核 CYCCNT 计时，无外设；供其它驱动记时间戳]
 *        ✅ CAN1/2/3   → bsp_can1/2/3.init()  [Message Buffer 收发]
 *        ✅ USART1     → bsp_uart1.init()     [IDLE RX DMA + FreeRTOS stream buffer]
 *        ✅ USART3     → bsp_uart3.init()     [IDLE RX DMA + FreeRTOS stream buffer]
 *        ✅ UART4      → bsp_uart4.init()     [IDLE RX DMA + FreeRTOS stream buffer]
 *        ✅ UART5      → bsp_uart5.init()     [IDLE RX DMA，无发送功能]
 *        ✅ UART7      → bsp_uart7.init()     [IDLE RX DMA + FreeRTOS stream buffer]
 *        ✅ UART8      → bsp_uart8.init()     [IDLE RX DMA + FreeRTOS stream buffer]
 *        ✅ UART9      → bsp_uart9.init()     [IDLE RX DMA + FreeRTOS stream buffer]
 *        ✅ USART10    → bsp_uart10.init()    [IDLE RX DMA + FreeRTOS stream buffer]
 *        ✅ GPIO       → MX_GPIO_Init()       [BspGpio 仅封装]
 *        ✅ KEY (PA15) → key_user.init(...)   [纯软件轮询消抖，无 ISR，使用rtos进行轮询：200ms轮询 → 200ms消抖, 1s长按]
 *        ✅ USB        → BspUsb::instance()   [由默认任务启动后初始化]
 *
 *        本工程尚未封装（CubeMX 已配置）：SPI1 / I2C2 / PWM1~4 / TIM3_CH4。
 *
 */
void bsp_init()
{
  // ── DWT 计时（内核 CYCCNT，不依赖 FreeRTOS/中断）：最先初始化，供其它驱动记时间戳 ──
  bsp_dwt.init();

  // ── FreeRTOS 驱动的 CAN ──
  bsp_can1.init();
  bsp_can2.init();
  bsp_can3.init();

  // ── FreeRTOS 驱动的 UART ──
  bsp_uart1.init();
  bsp_uart3.init();
  bsp_uart4.init();
  bsp_uart5.init();
  bsp_uart7.init();
  bsp_uart8.init();
  bsp_uart9.init();
  bsp_uart10.init();

  // ── 按键（纯软件轮询消抖，200ms 轮询 → 200ms 消抖, 1s 长按）──
  key_user.init({KEY_GPIO_Port, KEY_Pin, true, 1U, 5U}); // 200ms×1 消抖, 200ms×5=1s 长按

  // ── 蜂鸣器（TIM12 CH2 PB15 PWM 无源蜂鸣器）──
  // 构造时已完成配置（BspBuzzer::Config 默认值即本板参数），没有 init()
}


// ---------------- DWT ----------------

/**
 * @brief 全局实例化
 *
 * @note 无配置项（CPU 频率由 init() 从 RCC 读出），全局只需要一个实例；
 *       供 delta_s()/time_s()/delay_us() 等使用，详见 bsp_dwt.hpp。
 */
BspDwt bsp_dwt;


// ----------------
// ---------------- CAN ----------------

/**
 * @brief 全局实例化
 * @param CAN句柄
 * @param 实例名称（用于资源命名）
 * @param CAN工作模式（默认正常模式）
 */
BspCan bsp_can1({&hfdcan1, "CAN1"});
BspCan bsp_can2({&hfdcan2, "CAN2"});
BspCan bsp_can3({&hfdcan3, "CAN3"});


// ----------------
// ---------------- UART 但模板实例化需要在BspUart中实现 ----------------

/**
 * @brief 全局实例化
 * @param 第一个串口句柄
 * @param 第二个是是否启用发送逻辑
 * @note 这个 __attribute__((section(".dma_buffer"))) 是把他放到dtcm区域外，在.ld格式文件下实现的
 *
 */
__attribute__((section(".dma_buffer"))) BspUart<128> bsp_uart1({&huart1, true});
__attribute__((section(".dma_buffer"))) BspUart<128> bsp_uart3({&huart3, true});
__attribute__((section(".dma_buffer"))) BspUart<128> bsp_uart4({&huart4, true});
///< UART5 仅接收：CubeMX 未配 TX DMA，发送功能关闭（transmit_enable=false）
__attribute__((section(".dma_buffer"))) BspUart<128> bsp_uart5({&huart5, false});
__attribute__((section(".dma_buffer"))) BspUart<128> bsp_uart7({&huart7, true});
__attribute__((section(".dma_buffer"))) BspUart<128> bsp_uart8({&huart8, true});
__attribute__((section(".dma_buffer"))) BspUart<128> bsp_uart9({&huart9, true});
__attribute__((section(".dma_buffer"))) BspUart<128> bsp_uart10({&huart10, true});


// ----------------
// ---------------- GPIO 输出引脚 ----------------

///< 电源控制
BspGpio power_24v_2({POWER_24V_2_GPIO_Port, POWER_24V_2_Pin}); // PC13
BspGpio power_24v_1({POWER_24V_1_GPIO_Port, POWER_24V_1_Pin}); // PC14
BspGpio power_5v({POWER_5V_GPIO_Port, POWER_5V_Pin});          // PC15

///< IMU 片选
BspGpio gyro_acc_cs({GYRO_ACC_CS_GPIO_Port, GYRO_ACC_CS_Pin});    // PC0
BspGpio gyro_gyro_cs({GYRO_GYRO_CS_GPIO_Port, GYRO_GYRO_CS_Pin}); // PC3

///< SPI1 片选（原 LCD 接口改造后作 SPI1 用）
BspGpio spi1_cs({SPI1_CS_GPIO_Port, SPI1_CS_Pin}); // PE15

///< BTB 扩展 IO（经反向线到二层）
BspGpio btb_pa5({BTB_PA5_GPIO_Port, BTB_PA5_Pin});    // PA5
BspGpio btb_pe14({BTB_PE14_GPIO_Port, BTB_PE14_Pin}); // PE14
BspGpio btb_pd10({BTB_PD10_GPIO_Port, BTB_PD10_Pin}); // PD10


// ----------------
// ---------------- 蜂鸣器 ----------------

///< TIM12 CH2 (PB15) PWM 无源蜂鸣器
BspBuzzer bsp_buzzer;


// ----------------
// ---------------- 按键 ----------------

///< 用户按键 PA15, 低有效
BspKey key_user;


// ----------------
// ---------------- USB ----------------

/**
 * @brief USB BSP 全局单例引用定义。
 *
 */
BspUsb &bsp_usb = BspUsb::instance();

// ----------------
