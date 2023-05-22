/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mainFSM.h"
#include "din_dout.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void vStartTimer();
void vStopTimer();
void vTimerInit(uint16_t timeout);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_R_Pin GPIO_PIN_4
#define LED_R_GPIO_Port GPIOA
#define LED_G_Pin GPIO_PIN_5
#define LED_G_GPIO_Port GPIOA
#define S1_Pin GPIO_PIN_0
#define S1_GPIO_Port GPIOB
#define S2_Pin GPIO_PIN_1
#define S2_GPIO_Port GPIOB
#define S3_Pin GPIO_PIN_2
#define S3_GPIO_Port GPIOB
#define S4_Pin GPIO_PIN_10
#define S4_GPIO_Port GPIOB
#define K2_Pin GPIO_PIN_12
#define K2_GPIO_Port GPIOB
#define K4_Pin GPIO_PIN_13
#define K4_GPIO_Port GPIOB
#define K6_Pin GPIO_PIN_14
#define K6_GPIO_Port GPIOB
#define K8_Pin GPIO_PIN_15
#define K8_GPIO_Port GPIOB
#define EN_Pin GPIO_PIN_8
#define EN_GPIO_Port GPIOA
#define SW8_Pin GPIO_PIN_15
#define SW8_GPIO_Port GPIOA
#define SW7_Pin GPIO_PIN_3
#define SW7_GPIO_Port GPIOB
#define SW6_Pin GPIO_PIN_4
#define SW6_GPIO_Port GPIOB
#define SW5_Pin GPIO_PIN_5
#define SW5_GPIO_Port GPIOB
#define SW4_Pin GPIO_PIN_6
#define SW4_GPIO_Port GPIOB
#define SW3_Pin GPIO_PIN_7
#define SW3_GPIO_Port GPIOB
#define SW2_Pin GPIO_PIN_8
#define SW2_GPIO_Port GPIOB
#define SW1_Pin GPIO_PIN_9
#define SW1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
