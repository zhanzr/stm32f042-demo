/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "stm32f0xx_hal.h"

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
#define ADC_CHAN_NO 3
#define VREF_MV 3300
#define RS_Pin GPIO_PIN_0
#define RS_GPIO_Port GPIOF
#define RW_Pin GPIO_PIN_1
#define RW_GPIO_Port GPIOF
#define E_Pin GPIO_PIN_1
#define E_GPIO_Port GPIOA
#define VCP_TX_Pin GPIO_PIN_2
#define VCP_TX_GPIO_Port GPIOA
#define DB0_Pin GPIO_PIN_3
#define DB0_GPIO_Port GPIOA
#define DB1_Pin GPIO_PIN_4
#define DB1_GPIO_Port GPIOA
#define DB2_Pin GPIO_PIN_5
#define DB2_GPIO_Port GPIOA
#define DB3_Pin GPIO_PIN_6
#define DB3_GPIO_Port GPIOA
#define DB4_Pin GPIO_PIN_7
#define DB4_GPIO_Port GPIOA
#define DB5_Pin GPIO_PIN_0
#define DB5_GPIO_Port GPIOB
#define DB6_Pin GPIO_PIN_1
#define DB6_GPIO_Port GPIOB
#define DB7_Pin GPIO_PIN_8
#define DB7_GPIO_Port GPIOA
#define CS1_Pin GPIO_PIN_9
#define CS1_GPIO_Port GPIOA
#define RST_Pin GPIO_PIN_10
#define RST_GPIO_Port GPIOA
#define CS2_Pin GPIO_PIN_11
#define CS2_GPIO_Port GPIOA
#define CS3_Pin GPIO_PIN_12
#define CS3_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define VCP_RX_Pin GPIO_PIN_15
#define VCP_RX_GPIO_Port GPIOA
#define LD3_Pin GPIO_PIN_3
#define LD3_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
