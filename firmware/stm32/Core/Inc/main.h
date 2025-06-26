/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#define RADIO_CE_Pin GPIO_PIN_0
#define RADIO_CE_GPIO_Port GPIOA
#define RADIO_MOSI_Pin GPIO_PIN_1
#define RADIO_MOSI_GPIO_Port GPIOA
#define SYS_RESET_Pin GPIO_PIN_4
#define SYS_RESET_GPIO_Port GPIOA
#define ETH_INT_Pin GPIO_PIN_5
#define ETH_INT_GPIO_Port GPIOA
#define LCD_MOSI_Pin GPIO_PIN_7
#define LCD_MOSI_GPIO_Port GPIOA
#define ETH_SCK_Pin GPIO_PIN_0
#define ETH_SCK_GPIO_Port GPIOB
#define ETH_CS_Pin GPIO_PIN_1
#define ETH_CS_GPIO_Port GPIOB
#define SYS_LED_Pin GPIO_PIN_2
#define SYS_LED_GPIO_Port GPIOB
#define LCD_BL_PWM_Pin GPIO_PIN_10
#define LCD_BL_PWM_GPIO_Port GPIOB
#define RADIO_CS_Pin GPIO_PIN_12
#define RADIO_CS_GPIO_Port GPIOB
#define RADIO_SCK_Pin GPIO_PIN_13
#define RADIO_SCK_GPIO_Port GPIOB
#define RADIO_IRQ_Pin GPIO_PIN_14
#define RADIO_IRQ_GPIO_Port GPIOB
#define ETH_MOSI_Pin GPIO_PIN_10
#define ETH_MOSI_GPIO_Port GPIOA
#define RADIO_MISO_Pin GPIO_PIN_11
#define RADIO_MISO_GPIO_Port GPIOA
#define ETH_MISO_Pin GPIO_PIN_12
#define ETH_MISO_GPIO_Port GPIOA
#define LCD_CS_Pin GPIO_PIN_15
#define LCD_CS_GPIO_Port GPIOA
#define LCD_SCK_Pin GPIO_PIN_3
#define LCD_SCK_GPIO_Port GPIOB
#define LCD_MISO_Pin GPIO_PIN_4
#define LCD_MISO_GPIO_Port GPIOB
#define LCD_DC_RS_Pin GPIO_PIN_6
#define LCD_DC_RS_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
