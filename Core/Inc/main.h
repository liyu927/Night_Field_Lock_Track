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
#include "stm32f4xx_hal.h"

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
#define LED_Pin GPIO_PIN_0
#define LED_GPIO_Port GPIOC
#define SPI2_CS_Pin GPIO_PIN_1
#define SPI2_CS_GPIO_Port GPIOC
#define ADC1_LEFT_TOP_Pin GPIO_PIN_0
#define ADC1_LEFT_TOP_GPIO_Port GPIOA
#define ADC1_LEFT_BOTTOM_Pin GPIO_PIN_1
#define ADC1_LEFT_BOTTOM_GPIO_Port GPIOA
#define ADC1_RIGHT_TOP_Pin GPIO_PIN_2
#define ADC1_RIGHT_TOP_GPIO_Port GPIOA
#define ADC1_RIGHT_BOTTOM_Pin GPIO_PIN_3
#define ADC1_RIGHT_BOTTOM_GPIO_Port GPIOA
#define SPI1_DC_Pin GPIO_PIN_4
#define SPI1_DC_GPIO_Port GPIOA
#define SPI1_RST_Pin GPIO_PIN_6
#define SPI1_RST_GPIO_Port GPIOA
#define SPI1_CS_Pin GPIO_PIN_0
#define SPI1_CS_GPIO_Port GPIOB
#define SPI1_BLK_Pin GPIO_PIN_1
#define SPI1_BLK_GPIO_Port GPIOB
#define TIM1_EN_Pin GPIO_PIN_7
#define TIM1_EN_GPIO_Port GPIOE
#define TIM8_EN_Pin GPIO_PIN_8
#define TIM8_EN_GPIO_Port GPIOE
#define CTP_INT_Pin GPIO_PIN_15
#define CTP_INT_GPIO_Port GPIOE
#define CTP_RST_Pin GPIO_PIN_12
#define CTP_RST_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
