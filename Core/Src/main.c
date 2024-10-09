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
#include "spi.h"
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
#define ROW_NUM 168
#define COL_NUM 56
#define MUX_NUM 8
#define TOUCH_ARRAY_SIZE ROW_NUM*COL_NUM
#define SEND_ARRAY_SIZE TOUCH_ARRAY_SIZE + 3
#define SPI_CMD_SEND_DATA 0x01
#define RX_BUFFER_SIZE 1


volatile uint8_t dataReady = 0;

uint8_t touchList[SEND_ARRAY_SIZE];
uint8_t led_brightness;
uint8_t buzzer_volume;
uint8_t dummyArray[SEND_ARRAY_SIZE];
uint8_t spiRxBuffer[RX_BUFFER_SIZE];
uint8_t uartRxBuffer[RX_BUFFER_SIZE];

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

//Function to use printf() with UART
int __io_putchar(int ch)
{
  if (ch == '\n') {
    __io_putchar('\r');
  }

  HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, HAL_MAX_DELAY);

  return 1;
}

// Function for mapping values
uint16_t map(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//Function for choosing current multiplexer
void chooseMux(uint8_t mux)
{
	for (int i = 0; i < MUX_NUM; i++)
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

// Function for sending values over UART
void sendValuesUART() {
	for (int i = 0; i < SEND_ARRAY_SIZE-1; i++)
	{
		printf("%d", touchList[i]);
	}
	printf("\n");
}

// Function for reading analog value of pressure on the mat
uint16_t readPressureValue()
{
	  HAL_ADC_Start(&hadc1);
	  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	  return HAL_ADC_GetValue(&hadc1);
}

// Function for reading analog values from potentiometers - LED brightness and Buzzer volume
void readPotentiometersValues()
{
	  HAL_ADC_Start(&hadc2);
	  HAL_ADC_PollForConversion(&hadc2, HAL_MAX_DELAY);
	  led_brightness = map(HAL_ADC_GetValue(&hadc2), 0, 63, 0, 5);
	  HAL_ADC_PollForConversion(&hadc2, HAL_MAX_DELAY);
	  buzzer_volume = map(HAL_ADC_GetValue(&hadc2), 0, 63, 0, 5);
}

//Function for writing LOW bit on shift registers
void shiftLowBit()
{
	HAL_GPIO_WritePin(RCLK_LATCH_GPIO_Port, RCLK_LATCH_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SER_DATA_GPIO_Port, SER_DATA_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SRCLK_CLOCK_GPIO_Port, SRCLK_CLOCK_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SRCLK_CLOCK_GPIO_Port, SRCLK_CLOCK_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(RCLK_LATCH_GPIO_Port, RCLK_LATCH_Pin, GPIO_PIN_SET);
}


//Function for writing HIGH bit on shift registers
void shiftHighBit()
{
	HAL_GPIO_WritePin(RCLK_LATCH_GPIO_Port, RCLK_LATCH_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SER_DATA_GPIO_Port, SER_DATA_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SRCLK_CLOCK_GPIO_Port, SRCLK_CLOCK_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SRCLK_CLOCK_GPIO_Port, SRCLK_CLOCK_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(RCLK_LATCH_GPIO_Port, RCLK_LATCH_Pin, GPIO_PIN_SET);
}

// Function for choosing current channel on multiplexer
void selectReadChannel(int channel) {
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

// Function for reading values from the mat and writing them into array
void readValues()
{
	uint8_t currentMux = 0;

	shiftHighBit();

	for (int i = 0; i < ROW_NUM; i++)
	{
		currentMux = 0;
		for (int j = 0; j < COL_NUM; j++)
		{
			if (j % 8 == 0)
			{
				chooseMux(currentMux);
				currentMux++;
			}

			selectReadChannel(j % 8);
			uint16_t adc_val = readPressureValue();
			if(adc_val > 30) adc_val = 30;
			uint16_t index = i * COL_NUM + j;
			touchList[index] = map(adc_val, 0, 30, 0, 9);
		}

		shiftLowBit();
	}
	readPotentiometersValues();
	touchList[TOUCH_ARRAY_SIZE] = led_brightness;
	touchList[TOUCH_ARRAY_SIZE+1] = buzzer_volume;
	touchList[TOUCH_ARRAY_SIZE+2] = 100; // Add number 100 to mark correct transmission
}

// Function for handling UART interrupt
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
    	if (dataReady == 1){
    		sendValuesUART();
    		dataReady = 0;
    	}
        HAL_UART_Receive_IT(&huart2, uartRxBuffer, RX_BUFFER_SIZE);
    }
}

// Function for handling SPI interrupt
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if (hspi->Instance == SPI2)
	{
		  if (dataReady == 1)
		  {
			  HAL_SPI_Transmit(&hspi2, touchList, SEND_ARRAY_SIZE, HAL_MAX_DELAY);
			  dataReady = 0;
		  }
		  else
		  {
			  HAL_SPI_Transmit(&hspi2, dummyArray, SEND_ARRAY_SIZE, HAL_MAX_DELAY);
		  }
		  HAL_SPI_Receive_IT(&hspi2, spiRxBuffer, sizeof(spiRxBuffer));
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
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  // Calibrate ADC
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);

  // Prepare touchList and dummyArray
  for (int i = 0; i < SEND_ARRAY_SIZE; i++)
  {
     touchList[i] = 0;
  }

  for (int i = 0; i < SEND_ARRAY_SIZE; i++)
  {
	  dummyArray[i] = 1;
  }

  // Reset shift registers
  HAL_GPIO_WritePin(SRCLR_RESET_GPIO_Port, SRCLR_RESET_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(SRCLR_RESET_GPIO_Port, SRCLR_RESET_Pin, GPIO_PIN_SET);

  // Initialize UART and SPI in interrupts
  uint8_t initBuffer;
  HAL_UART_Receive_IT(&huart2, &initBuffer, 1);
  HAL_SPI_Receive_IT(&hspi2, spiRxBuffer, sizeof(spiRxBuffer));

  while (1)
  {
	  readValues();
	  dataReady = 1;
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
