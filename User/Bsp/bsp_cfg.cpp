#include "bsp_cfg.hpp"

#include "FreeRTOS.h" // IWYU pragma: keep (configASSERT)

/**
 * @brief bsp层整体的初始化
 *
 * @note  必须在 FreeRTOS 内核启动后调用（因为大部分 BSP 驱动内部创建 RTOS 对象）。
 *        目前除了串口的模板实例化需要在 bsp_uart.cpp 中定义，其他 BSP 全局实例化都在 bsp_cfg.cpp 中定义。
 *        当前已初始化的外设一览：
 *
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
 *        ✅ KEY (PA15) → key_user.init(...)   [纯软件轮询消抖，无 ISR；200ms 轮询 → 200ms 消抖, 1s 长按]
 *        ✅ USB        → BspUsb::instance()   [由默认任务启动后初始化]
 *        ✅ 蜂鸣器     → bsp_buzzer          [TIM12 CH2 PB15，配置在全局构造时传入，无需 init]
 *
 */
extern "C" int __io_putchar(int ch)
{
  // FreeRTOS 驱动的 CAN（init() 失败 = 该路 CAN 静默不可用，必须当场发现）
  configASSERT(bsp_can1.init() == Status::OK);
  configASSERT(bsp_can2.init() == Status::OK);
  configASSERT(bsp_can3.init() == Status::OK);

  // FreeRTOS 驱动的 UART
  configASSERT(bsp_uart1.init() == Status::OK);
  configASSERT(bsp_uart3.init() == Status::OK);
  configASSERT(bsp_uart4.init() == Status::OK);
  configASSERT(bsp_uart5.init() == Status::OK);
  configASSERT(bsp_uart7.init() == Status::OK);
  configASSERT(bsp_uart8.init() == Status::OK);
  configASSERT(bsp_uart9.init() == Status::OK);
  configASSERT(bsp_uart10.init() == Status::OK);

  // 按键（纯软件轮询消抖，200ms 轮询 → 200ms 消抖, 1s 长按）
  //    长按时间 = long_press_cnt × 轮询周期（从电平跳变起算）= 5 × 200ms = 1s
  key_user.init({KEY_GPIO_Port, KEY_Pin, true, 1U, 5U});
}


// ---------------- CAN ----------------

/**
 * @brief 全局实例化
 * @param CAN句柄
 *
 * @note Config 只有单个字段时，`BspCan x({&hfdcan1})` 会与拷贝构造产生二义
 *       （GCC: call of overloaded 'BspCan(<brace-enclosed initializer list>)' is
 *       ambiguous），故这里显式构造 BspCan::Config。
 */
BspCan bsp_can1(BspCan::Config{&hfdcan1});
BspCan bsp_can2(BspCan::Config{&hfdcan2});
BspCan bsp_can3(BspCan::Config{&hfdcan3});


// ----------------
// ---------------- UART ----------------

/**
 * @brief 全局实例化
 * @param 第一个串口句柄
 * @param 第二个是串口接收模式
 * @param 第三个是是否启用发送逻辑
 * @note 这个 __attribute__((section(".dma_buffer"))) 是把他放到dtcm区域外，在.ld格式文件下实现的
 *
 */
__attribute__((section(".dma_buffer"))) BspUart<64, 8> bsp_usart1(&huart1, ReceiveMode::SINGLE_BUFFER, true, 1); // 添加实例ID为6


// ----------------
// ---------------- GPIO 输出引脚 ----------------

///< 电源控制
BspGpio power_24v_2({POWER_24V_2_GPIO_Port, POWER_24V_2_Pin}); // PC13
BspGpio power_24v_1({POWER_24V_1_GPIO_Port, POWER_24V_1_Pin}); // PC14
BspGpio power_5v({POWER_5V_GPIO_Port, POWER_5V_Pin});          // PC15

///< IMU 片选
BspGpio gyro_acc_cs({GYRO_ACC_CS_GPIO_Port, GYRO_ACC_CS_Pin});    // PC0
BspGpio gyro_gyro_cs({GYRO_GYRO_CS_GPIO_Port, GYRO_GYRO_CS_Pin}); // PC3

///< BTB 扩展 IO
BspGpio btb_gpio({BTB_GPIO_GPIO_Port, BTB_GPIO_Pin}); // PE14

///< LCD 控制
BspGpio lcd_cs({LCD_CS_GPIO_Port, LCD_CS_Pin});    // PE15
BspGpio lcd_blk({LCD_BLK_GPIO_Port, LCD_BLK_Pin}); // PB10
BspGpio lcd_res({LCD_RES_GPIO_Port, LCD_RES_Pin}); // PB11
BspGpio lcd_dc({LCD_DC_GPIO_Port, LCD_DC_Pin});    // PD10


// ----------------
// ---------------- 蜂鸣器 ----------------

///< TIM12 CH2 (PB15) PWM 无源蜂鸣器
///< 默认配置即为 TIM12 CH2 / 6MHz 基频，换定时器时传入 BspBuzzer::Config 即可
BspBuzzer bsp_buzzer;


// ----------------
// ---------------- 按键 ----------------

///< 用户按键 PA15, 低有效
BspKey key_user;


// ----------------
