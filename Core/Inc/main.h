/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32l4xx_hal.h"

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
#define SELECT_B_Pin GPIO_PIN_2
#define SELECT_B_GPIO_Port GPIOC
#define SELECT_C_Pin GPIO_PIN_3
#define SELECT_C_GPIO_Port GPIOC
#define SRCLR_RESET_Pin GPIO_PIN_5
#define SRCLR_RESET_GPIO_Port GPIOA
#define RCLK_LATCH_Pin GPIO_PIN_6
#define RCLK_LATCH_GPIO_Port GPIOA
#define SRCLK_CLOCK_Pin GPIO_PIN_7
#define SRCLK_CLOCK_GPIO_Port GPIOA
#define SELECT_A_Pin GPIO_PIN_0
#define SELECT_A_GPIO_Port GPIOB
#define INH5_Pin GPIO_PIN_10
#define INH5_GPIO_Port GPIOB
#define INH8_Pin GPIO_PIN_7
#define INH8_GPIO_Port GPIOC
#define INH6_Pin GPIO_PIN_8
#define INH6_GPIO_Port GPIOA
#define INH7_Pin GPIO_PIN_9
#define INH7_GPIO_Port GPIOA
#define INH1_Pin GPIO_PIN_10
#define INH1_GPIO_Port GPIOA
#define INH2_Pin GPIO_PIN_3
#define INH2_GPIO_Port GPIOB
#define INH4_Pin GPIO_PIN_4
#define INH4_GPIO_Port GPIOB
#define INH3_Pin GPIO_PIN_5
#define INH3_GPIO_Port GPIOB
#define SER_DATA_Pin GPIO_PIN_6
#define SER_DATA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
