/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32f1xx_hal.h"

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
#define LIPO_ADC_Pin GPIO_PIN_0
#define LIPO_ADC_GPIO_Port GPIOA
#define BUT_SOS_Pin GPIO_PIN_1
#define BUT_SOS_GPIO_Port GPIOB
#define BUT_SOS_EXTI_IRQn EXTI1_IRQn
#define BUT_ADC_Pin GPIO_PIN_2
#define BUT_ADC_GPIO_Port GPIOB
#define BUT_ADC_EXTI_IRQn EXTI2_IRQn
#define MCU_TXD3_Pin GPIO_PIN_10
#define MCU_TXD3_GPIO_Port GPIOB
#define MCU_RXD3_Pin GPIO_PIN_11
#define MCU_RXD3_GPIO_Port GPIOB
#define MCU_BUZZER_Pin GPIO_PIN_15
#define MCU_BUZZER_GPIO_Port GPIOB
#define GSM_PWRKEY_Pin GPIO_PIN_8
#define GSM_PWRKEY_GPIO_Port GPIOA
#define MCU_TXD1_Pin GPIO_PIN_9
#define MCU_TXD1_GPIO_Port GPIOA
#define MCU_RXD1_Pin GPIO_PIN_10
#define MCU_RXD1_GPIO_Port GPIOA
#define GPS_RESET_Pin GPIO_PIN_11
#define GPS_RESET_GPIO_Port GPIOA
#define GSM_STATUS_Pin GPIO_PIN_12
#define GSM_STATUS_GPIO_Port GPIOA
#define MCU_LED1_Pin GPIO_PIN_4
#define MCU_LED1_GPIO_Port GPIOB
#define MCU_LED2_Pin GPIO_PIN_5
#define MCU_LED2_GPIO_Port GPIOB
#define MCU_LED3_Pin GPIO_PIN_6
#define MCU_LED3_GPIO_Port GPIOB
#define MCU_LED4_Pin GPIO_PIN_7
#define MCU_LED4_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
