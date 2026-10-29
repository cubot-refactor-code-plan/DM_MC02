#include "bsp_cfg.hpp"

#include "FreeRTOS.h" // IWYU pragma: keep (configASSERT)

/**
 * @brief BSP 层统一初始化
 *
 * @note  在 osKernelInitialize() 之后、osKernelStart() 之前调用（MX_FREERTOS_Init → all_init）：
 *        此时可以创建 StreamBuffer / Mutex 等对象，但不能阻塞等待。
 *        每一条驱动的 init() 都已在下面用 configASSERT 验过，失败就地停机，因此本函数无返回值。
 *        串口的模板实例化在 bsp_uart.cpp，其余 BSP 全局实例都在本文件定义。
 *
 *        初始化顺序：DWT → CAN×3 → UART×9 → PWM×6 → 按键；实例与绑定资源见 bsp_cfg.md。
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

  // 按键（纯软件轮询消抖，200ms轮询 → 200ms消抖, 1s长按）；配置已在构造时存入
  configASSERT(key_user.init() == Status::OK);
}
