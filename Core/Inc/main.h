/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define POWER_24V_2_Pin GPIO_PIN_13
#define POWER_24V_2_GPIO_Port GPIOC
#define POWER_24V_1_Pin GPIO_PIN_14
#define POWER_24V_1_GPIO_Port GPIOC
#define POWER_5V_Pin GPIO_PIN_15
#define POWER_5V_GPIO_Port GPIOC
#define GYRO_ACC_CS_Pin GPIO_PIN_0
#define GYRO_ACC_CS_GPIO_Port GPIOC
#define GYRO_GYRO_CS_Pin GPIO_PIN_3
#define GYRO_GYRO_CS_GPIO_Port GPIOC
#define PWM4_TIM2_CH1_Pin GPIO_PIN_0
#define PWM4_TIM2_CH1_GPIO_Port GPIOA
#define PWM3_TIM2_CH3_Pin GPIO_PIN_2
#define PWM3_TIM2_CH3_GPIO_Port GPIOA
#define BTB_PA5_Pin GPIO_PIN_5
#define BTB_PA5_GPIO_Port GPIOA
#define PWM_GYRO_TIM3_CH4_Pin GPIO_PIN_1
#define PWM_GYRO_TIM3_CH4_GPIO_Port GPIOB
#define PWM2_TIM1_CH1_Pin GPIO_PIN_9
#define PWM2_TIM1_CH1_GPIO_Port GPIOE
#define GYRO_ACC_INT_Pin GPIO_PIN_10
#define GYRO_ACC_INT_GPIO_Port GPIOE
#define GYRO_ACC_INT_EXTI_IRQn EXTI15_10_IRQn
#define GYRO_GYRO_INT_Pin GPIO_PIN_12
#define GYRO_GYRO_INT_GPIO_Port GPIOE
#define GYRO_GYRO_INT_EXTI_IRQn EXTI15_10_IRQn
#define PWM1_TIM1_CH3_Pin GPIO_PIN_13
#define PWM1_TIM1_CH3_GPIO_Port GPIOE
#define BTB_PE14_Pin GPIO_PIN_14
#define BTB_PE14_GPIO_Port GPIOE
#define SPI1_CS_Pin GPIO_PIN_15
#define SPI1_CS_GPIO_Port GPIOE
#define PWM_Buzzer_TIM12_CH2_Pin GPIO_PIN_15
#define PWM_Buzzer_TIM12_CH2_GPIO_Port GPIOB
#define BTB_PD10_Pin GPIO_PIN_10
#define BTB_PD10_GPIO_Port GPIOD
#define KEY_Pin GPIO_PIN_15
#define KEY_GPIO_Port GPIOA
#define KEY_EXTI_IRQn EXTI15_10_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
