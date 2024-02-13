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

#define ORANGE__LED_ON 0x0100
#define RED_LED_ON 0x00000040
#define GREEN_LED_ON 0x0200
#define BLUE_LED_ON 0x0080

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{  
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();
	
	RCC->AHBENR = RCC->AHBENR | RCC_AHBENR_GPIOCEN; // Enable the GPIOC clock
	RCC->AHBENR = RCC->AHBENR | RCC_AHBENR_GPIOAEN; // Enable the GPIOA clock

	HAL_Delay(2);
	
	// GPIO C (LED configuration)
	GPIOC->MODER = GPIOC->MODER | 0x00055000; // Set the GPIO C mode to general output
	GPIOC->OTYPER = 0x00000000; // Set to push pull
	GPIOC->OSPEEDR = 0x00000000; // Set to low speed across the GPIO port
	GPIOC->PUPDR = 0x00000000; // Set to no pull up or pull down across the GPIO port

	// GPIO A (Push button configuration)
	GPIOA->MODER = 0x00000000;
	GPIOA->OSPEEDR = 0x00000000;
	GPIOA->PUPDR = GPIOA->PUPDR | 0x00000002;
	
	GPIOC->ODR = GPIOC->ODR | RED_LED_ON; // Initialize to red led on
	
	// Varaible to use for debounce
	uint32_t debounce = 0x00000000;
	uint8_t currentValue = 0x00;
	uint8_t switchAllowed = 0x00;
	
	// Red and Blue only
  while (1)
  {
		currentValue = (uint8_t)(GPIOA->IDR & 0x1); // Get the latest push button value
		debounce = debounce << 1;
		debounce = debounce | currentValue;
		
		if(debounce == 0xFFFFFFFF && switchAllowed == 0x01)
		{
			switchAllowed = 0x00;
			
			// Switch the LED
			if(GPIOC->ODR == RED_LED_ON)
			{
				GPIOC->ODR = BLUE_LED_ON;
			}
			else
			{
				GPIOC->ODR = RED_LED_ON;
			}
		}
		else if (debounce != 0xFFFFFFFF)
		{
			switchAllowed = 0x01;
		}
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
