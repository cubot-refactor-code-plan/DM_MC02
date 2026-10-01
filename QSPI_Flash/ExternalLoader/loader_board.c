#include "loader_board.h"

#include <stdint.h>

#include "main.h"
#include "octospi.h"

OSPI_HandleTypeDef hospi2;
volatile int       loader_board_stage;
volatile uint32_t  loader_fault_cfsr;
volatile uint32_t  loader_fault_hfsr;
volatile uint32_t  loader_fault_bfar;
volatile uint32_t  loader_fault_mmfar;

extern uint32_t __loader_stack_top__;
void            SysTick_Handler(void);
void            OCTOSPI2_IRQHandler(void);

static void Default_Handler(void)
{
  while (1)
  {
  }
}

static void Fault_Handler(void)
{
  loader_fault_cfsr  = SCB->CFSR;
  loader_fault_hfsr  = SCB->HFSR;
  loader_fault_bfar  = SCB->BFAR;
  loader_fault_mmfar = SCB->MMFAR;
  while (1)
  {
  }
}

__attribute__((section(".isr_vector"), used))
const uintptr_t loader_vectors[16U + OCTOSPI2_IRQn + 1U] = {
  [0]                           = (uintptr_t)&__loader_stack_top__,
  [1 ... 2]                     = (uintptr_t)Default_Handler,
  [3 ... 6]                     = (uintptr_t)Fault_Handler,
  [7 ... 14]                    = (uintptr_t)Default_Handler,
  [15]                          = (uintptr_t)SysTick_Handler,
  [16 ...(15U + OCTOSPI2_IRQn)] = (uintptr_t)Default_Handler,
  [16U + OCTOSPI2_IRQn]         = (uintptr_t)OCTOSPI2_IRQHandler};

void SysTick_Handler(void)
{
  HAL_IncTick();
}

void Error_Handler(void)
{
  while (1)
  {
  }
}

/** @brief 使用板载 24 MHz HSE 配置 550 MHz SYSCLK 和 275 MHz HCLK。 */
static int loader_clock_init(void)
{
  RCC_OscInitTypeDef oscillator = {0};
  RCC_ClkInitTypeDef clock      = {0};

  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);
  while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY))
  {
  }

  oscillator.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  oscillator.HSEState       = RCC_HSE_ON;
  oscillator.PLL.PLLState   = RCC_PLL_ON;
  oscillator.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
  oscillator.PLL.PLLM       = 3U;
  oscillator.PLL.PLLN       = 68U;
  oscillator.PLL.PLLP       = 1U;
  oscillator.PLL.PLLQ       = 4U;
  oscillator.PLL.PLLR       = 2U;
  oscillator.PLL.PLLRGE     = RCC_PLL1VCIRANGE_3;
  oscillator.PLL.PLLVCOSEL  = RCC_PLL1VCOWIDE;
  oscillator.PLL.PLLFRACN   = 6144U;
  if (HAL_RCC_OscConfig(&oscillator) != HAL_OK)
  {
    return -1;
  }

  clock.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
  clock.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  clock.SYSCLKDivider  = RCC_SYSCLK_DIV1;
  clock.AHBCLKDivider  = RCC_HCLK_DIV2;
  clock.APB3CLKDivider = RCC_APB3_DIV2;
  clock.APB1CLKDivider = RCC_APB1_DIV2;
  clock.APB2CLKDivider = RCC_APB2_DIV2;
  clock.APB4CLKDivider = RCC_APB4_DIV2;
  return HAL_RCC_ClockConfig(&clock, FLASH_LATENCY_3) == HAL_OK ? 0 : -1;
}

/** @brief 按 DM-MC-Board02 引脚初始化 OCTOSPI2 Port 1 Quad 模式。 */
static int loader_ospi_init(void)
{
  GPIO_InitTypeDef         gpio             = {0};
  RCC_PeriphCLKInitTypeDef peripheral_clock = {0};
  OSPIM_CfgTypeDef         manager          = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  peripheral_clock.PeriphClockSelection = RCC_PERIPHCLK_OSPI;
  peripheral_clock.OspiClockSelection   = RCC_OSPICLKSOURCE_D1HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&peripheral_clock) != HAL_OK)
  {
    return -1;
  }
  __HAL_RCC_OCTOSPIM_CLK_ENABLE();
  __HAL_RCC_OSPI2_CLK_ENABLE();

  gpio.Mode  = GPIO_MODE_AF_PP;
  gpio.Pull  = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

  gpio.Pin       = GPIO_PIN_1;
  gpio.Alternate = GPIO_AF9_OCTOSPIM_P1;
  HAL_GPIO_Init(GPIOA, &gpio);
  gpio.Pin       = GPIO_PIN_3;
  gpio.Alternate = GPIO_AF6_OCTOSPIM_P1;
  HAL_GPIO_Init(GPIOA, &gpio);

  gpio.Pin       = GPIO_PIN_0;
  gpio.Alternate = GPIO_AF4_OCTOSPIM_P1;
  HAL_GPIO_Init(GPIOB, &gpio);
  gpio.Pin       = GPIO_PIN_2;
  gpio.Alternate = GPIO_AF9_OCTOSPIM_P1;
  HAL_GPIO_Init(GPIOB, &gpio);

  gpio.Pin       = GPIO_PIN_11;
  gpio.Alternate = GPIO_AF9_OCTOSPIM_P1;
  HAL_GPIO_Init(GPIOD, &gpio);
  gpio.Pin       = GPIO_PIN_11;
  gpio.Alternate = GPIO_AF11_OCTOSPIM_P1;
  HAL_GPIO_Init(GPIOE, &gpio);

  hospi2.Instance                   = OCTOSPI2;
  hospi2.Init.FifoThreshold         = 8U;
  hospi2.Init.DualQuad              = HAL_OSPI_DUALQUAD_DISABLE;
  hospi2.Init.MemoryType            = HAL_OSPI_MEMTYPE_MICRON;
  hospi2.Init.DeviceSize            = 23U;
  hospi2.Init.ChipSelectHighTime    = 1U;
  hospi2.Init.FreeRunningClock      = HAL_OSPI_FREERUNCLK_DISABLE;
  hospi2.Init.ClockMode             = HAL_OSPI_CLOCK_MODE_3;
  hospi2.Init.WrapSize              = HAL_OSPI_WRAP_NOT_SUPPORTED;
  hospi2.Init.ClockPrescaler        = 3U;
  hospi2.Init.SampleShifting        = HAL_OSPI_SAMPLE_SHIFTING_HALFCYCLE;
  hospi2.Init.DelayHoldQuarterCycle = HAL_OSPI_DHQC_DISABLE;
  hospi2.Init.ChipSelectBoundary    = 0U;
  hospi2.Init.DelayBlockBypass      = HAL_OSPI_DELAY_BLOCK_BYPASSED;
  hospi2.Init.MaxTran               = 0U;
  hospi2.Init.Refresh               = 0U;
  if (HAL_OSPI_Init(&hospi2) != HAL_OK)
  {
    return -1;
  }

  manager.ClkPort   = 1U;
  manager.NCSPort   = 1U;
  manager.IOLowPort = HAL_OSPIM_IOPORT_1_LOW;
  return HAL_OSPIM_Config(&hospi2, &manager, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) == HAL_OK ? 0 : -1;
}

void HAL_OSPI_MspInit(OSPI_HandleTypeDef *handle)
{
  (void)handle;
}

int loader_board_init(void)
{
  loader_board_stage = 1;
  SCB->CPACR |= (3UL << (10U * 2U)) | (3UL << (11U * 2U));
  __DSB();
  __ISB();
  SCB->VTOR = (uint32_t)(uintptr_t)loader_vectors;
  __DSB();
  __ISB();
  HAL_Init();
  loader_board_stage = 2;
  if (loader_clock_init() != 0)
  {
    loader_board_stage = -2;
    return -1;
  }
  loader_board_stage = 3;
  if (loader_ospi_init() != 0)
  {
    loader_board_stage = -3;
    return -1;
  }
  loader_board_stage = 4;
  return 0;
}
