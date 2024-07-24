/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// matrix size
#define rowNum 168
#define colNum 56

#define muxNumber 8

#define LINE_MAX_LENGTH	5

static char line_buffer[LINE_MAX_LENGTH + 1];
static uint32_t line_length;
uint16_t touch_list[rowNum][colNum];
uint8_t led_brightness;
uint8_t buzzer_volume;

typedef struct
{
   GPIO_TypeDef* GPIOx;
   uint16_t GPIO_Pin;
} port_and_pin_t;


port_and_pin_t inhibits[] = {
		{INH1_GPIO_Port, INH1_Pin},
		{INH2_GPIO_Port, INH2_Pin},
		{INH3_GPIO_Port, INH3_Pin},
		{INH4_GPIO_Port, INH4_Pin},
		{INH5_GPIO_Port, INH5_Pin},
		{INH6_GPIO_Port, INH6_Pin},
		{INH7_GPIO_Port, INH7_Pin},
		{INH8_GPIO_Port, INH8_Pin},
};

uint16_t map(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void chooseMux(uint8_t mux)
{
	for (int i = 0; i < muxNumber; i++)
	{
		if (i == mux)
		{
			HAL_GPIO_WritePin(inhibits[i].GPIOx, inhibits[i].GPIO_Pin, GPIO_PIN_RESET);
		}
		else
		{
			HAL_GPIO_WritePin(inhibits[i].GPIOx, inhibits[i].GPIO_Pin, GPIO_PIN_SET);
		}

	}
}

//Function sending values from array over UART to PC
void sendValues(const uint16_t a[][colNum]) {
	for (int i = 0; i <rowNum; i++)
	{
		for (int j = 0; j < colNum; j++)
		{
			printf("%d", touch_list[i][j]);
		}
	}
	printf("%d", led_brightness);
	printf("%d", buzzer_volume);
	printf("\n");
}


//Function to use printf() with UART
int __io_putchar(int ch)
{
  if (ch == '\n') {
    __io_putchar('\r');
  }

  HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, HAL_MAX_DELAY);

  return 1;
}

uint16_t readPressureValue()
{
	  HAL_ADC_Start(&hadc1);
	  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	  return HAL_ADC_GetValue(&hadc1);
}

void readPotentiometersValues()
{
	  HAL_ADC_Start(&hadc2);
	  HAL_ADC_PollForConversion(&hadc2, HAL_MAX_DELAY);
	  led_brightness = map(HAL_ADC_GetValue(&hadc2), 0, 63, 0, 5);
	  HAL_ADC_PollForConversion(&hadc2, HAL_MAX_DELAY);
	  buzzer_volume = map(HAL_ADC_GetValue(&hadc2), 0, 63, 0, 5);
}

void shiftLowBit()
{
	HAL_GPIO_WritePin(RCLK_LATCH_GPIO_Port, RCLK_LATCH_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SER_DATA_GPIO_Port, SER_DATA_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SRCLK_CLOCK_GPIO_Port, SRCLK_CLOCK_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SRCLK_CLOCK_GPIO_Port, SRCLK_CLOCK_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(RCLK_LATCH_GPIO_Port, RCLK_LATCH_Pin, GPIO_PIN_SET);
}



void shiftHighBit()
{
	HAL_GPIO_WritePin(RCLK_LATCH_GPIO_Port, RCLK_LATCH_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SER_DATA_GPIO_Port, SER_DATA_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SRCLK_CLOCK_GPIO_Port, SRCLK_CLOCK_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SRCLK_CLOCK_GPIO_Port, SRCLK_CLOCK_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(RCLK_LATCH_GPIO_Port, RCLK_LATCH_Pin, GPIO_PIN_SET);
}

void select_read_channel(int channel) {
	switch (channel) {
		case 0:
			HAL_GPIO_WritePin(SELECT_A_GPIO_Port, SELECT_A_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SELECT_B_GPIO_Port, SELECT_B_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SELECT_C_GPIO_Port, SELECT_C_Pin, GPIO_PIN_RESET);
			break;
		case 1:
			HAL_GPIO_WritePin(SELECT_A_GPIO_Port, SELECT_A_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(SELECT_B_GPIO_Port, SELECT_B_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SELECT_C_GPIO_Port, SELECT_C_Pin, GPIO_PIN_RESET);
			break;
		case  2:
			HAL_GPIO_WritePin(SELECT_A_GPIO_Port, SELECT_A_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SELECT_B_GPIO_Port, SELECT_B_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(SELECT_C_GPIO_Port, SELECT_C_Pin, GPIO_PIN_RESET);
			break;
		case  3:
			HAL_GPIO_WritePin(SELECT_A_GPIO_Port, SELECT_A_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(SELECT_B_GPIO_Port, SELECT_B_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(SELECT_C_GPIO_Port, SELECT_C_Pin, GPIO_PIN_RESET);
			break;
		case  4:
			HAL_GPIO_WritePin(SELECT_A_GPIO_Port, SELECT_A_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SELECT_B_GPIO_Port, SELECT_B_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SELECT_C_GPIO_Port, SELECT_C_Pin, GPIO_PIN_SET);
			break;
		case 5:
			HAL_GPIO_WritePin(SELECT_A_GPIO_Port, SELECT_A_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(SELECT_B_GPIO_Port, SELECT_B_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SELECT_C_GPIO_Port, SELECT_C_Pin, GPIO_PIN_SET);
			break;
		case 6:
			HAL_GPIO_WritePin(SELECT_A_GPIO_Port, SELECT_A_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SELECT_B_GPIO_Port, SELECT_B_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(SELECT_C_GPIO_Port, SELECT_C_Pin, GPIO_PIN_SET);
			break;
	    case 7:
	    	HAL_GPIO_WritePin(SELECT_A_GPIO_Port, SELECT_A_Pin, GPIO_PIN_SET);
	    	HAL_GPIO_WritePin(SELECT_B_GPIO_Port, SELECT_B_Pin, GPIO_PIN_SET);
	    	HAL_GPIO_WritePin(SELECT_C_GPIO_Port, SELECT_C_Pin, GPIO_PIN_SET);
	        break;
	    default:
	    	break;
	}
}

void readValues()
{
	uint8_t currentMux = 0;

	shiftHighBit();

	for (int i = 0; i < rowNum; i++)
	{
		currentMux = 0;
		for (int j = 0; j < colNum; j++)
		{
			if (j % 8 == 0)
			{
				chooseMux(currentMux);
				currentMux++;
			}

			select_read_channel(j % 8);
			uint16_t a_val = readPressureValue();
			if(a_val > 40) a_val = 40;
			touch_list[i][j] = map(a_val, 0, 40, 0, 9);
		}

		shiftLowBit();
	}
}


//Function adding characters to buffer
void line_append(uint8_t value)
{
	if (value == '\r' || value == '\n') {
		if (line_length > 0) {
			line_buffer[line_length] = '\0';
			if (strcmp(line_buffer, "ok") == 0)
			{
				readValues();
				readPotentiometersValues();
				sendValues(touch_list);
			} else
			{
				printf("wrong\n");
			}
			line_length = 0;
		}
	}
	else {
		if (line_length >= LINE_MAX_LENGTH) {
			line_length = 0;
		}
		line_buffer[line_length++] = value;
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM6_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);

  for (int i = 0; i < rowNum; i++)
  {
    for (int j = 0; j < colNum; j++)
    {
      touch_list[i][j] = 0;
    }
  }

  HAL_GPIO_WritePin(SRCLR_RESET_GPIO_Port, SRCLR_RESET_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(SRCLR_RESET_GPIO_Port, SRCLR_RESET_Pin, GPIO_PIN_SET);

  while (1)
  {
//	  HAL_TIM_Base_Start(&htim6);
//	  readValues();
//	  sendValues(touch_list);
//	  HAL_TIM_Base_Stop(&htim6);
//	  printf("%lu \n", __HAL_TIM_GET_COUNTER(&htim6));
//	  __HAL_TIM_SET_COUNTER(&htim6, 0);

	  uint8_t value;
	  if (HAL_UART_Receive(&huart2, &value, 1, 0) == HAL_OK) line_append(value);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 16;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
