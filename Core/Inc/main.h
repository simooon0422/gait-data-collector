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
#define selectBr_Pin GPIO_PIN_13
#define selectBr_GPIO_Port GPIOC
#define rinh2_Pin GPIO_PIN_2
#define rinh2_GPIO_Port GPIOC
#define rinh1_Pin GPIO_PIN_3
#define rinh1_GPIO_Port GPIOC
#define selectCr_Pin GPIO_PIN_0
#define selectCr_GPIO_Port GPIOA
#define rinh3_Pin GPIO_PIN_4
#define rinh3_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define SRCLR_RESET_Pin GPIO_PIN_6
#define SRCLR_RESET_GPIO_Port GPIOA
#define RCLK_LATCH_Pin GPIO_PIN_7
#define RCLK_LATCH_GPIO_Port GPIOA
#define SER_DATA_Pin GPIO_PIN_7
#define SER_DATA_GPIO_Port GPIOC
#define SRCLK_CLOCK_Pin GPIO_PIN_6
#define SRCLK_CLOCK_GPIO_Port GPIOB
#define selectAr_Pin GPIO_PIN_7
#define selectAr_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
