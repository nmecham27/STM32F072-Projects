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

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();

  SystemClock_Config();
	
	// Enable the clock for the tim2, tim3, and GPIOC peripherals
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM3EN;
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;

	// Timer 2 configuration
	// Enable the update event
	// Set the prescaler to 8000 so our timer has a period of 1ms
	// Set the auto reload register to 0xFA	= 250 in order to get
	// a timer of 4Hz
	TIM2->DIER |= 0x1;
	TIM2->PSC = 0x1F3F; //7999+1=8000
	TIM2->ARR = 0xFA;
	
	// Timer 3 configuration
	// Set the prescaler to 1000 to give a timer period of 40KHz
	// Set the ARR to 50 in order to give a period of 800 Hz
	// Set the CCR1 & CCR2 to a value of 10 for a 20% duty cycle based on the 800 Hz
	// Configure channel 1 & 2 to be PWM output
	TIM3->PSC = 0xC8;
	TIM3->ARR = 0x0032;
	TIM3->CCR1 = 0xA; //0xA gives 80% duty cycle since pwm mode 2
	TIM3->CCR2 = 0xA; //0xA gives 20% duty cyclc since pwm mode 1
	TIM3->CCMR1 = 0x6878;
	TIM3->CCER = 0x11;
	TIM3->CR1 = 0x1;

	// Enable the timer interrupts
	NVIC_EnableIRQ(TIM2_IRQn);
	NVIC_SetPriority(TIM2_IRQn, 1); // Set with a high priority
	
	// LED config
	// Set the LED pins to general purpose output mode, push-pull, low speed, and no pull-up or pull-down
	GPIOC->MODER = 0x0005A000;
	GPIOC->OTYPER &= 0x00000000;
	GPIOC->OSPEEDR &= 0x00000000;
	GPIOC->PUPDR &= 0x00000000;
	
	GPIOC->ODR |= 0x100; // Set the green, orange, red and blue LEDs
	
	TIM2->SR = 0x00; // Clear the timer
	TIM2->CR1 = 0x5; // Enable the TIM2 timer, enable the interrupt, and set so only a underflow or overflow event causes interrupt

  while (1)
  {
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

void TIM2_IRQHandler(void)
{
	GPIOC->ODR ^= 0x300; // Toggle the green and orange LEDs
	TIM2->SR = 0x00; // Clear the timer
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
