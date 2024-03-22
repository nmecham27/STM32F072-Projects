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
#include "stdbool.h"

// UART defines
#define BAUD_RATE 115200
#define USART_TX_READY_MASK 0x80
#define CARRIAGE_RETURN 0x0D
#define LINE_FEED 0x0A
#define EOT 0x04

// ADC defines
#define EOC_MASK 0x00000004

// Comment out to remove debug prints
#define DEBUG

enum Led_e
{
	RED = 0,
	BLUE,
	GREEN,
	ORANGE
};


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

// Functions to send a character to the host
void SendString(const char* string);
void SendChar(char character);
void DefaultConfigLeds();
void ToggleLed(enum Led_e led);
void TurnOnLed(enum Led_e led);
void TurnOffLed(enum Led_e led);
void DefaultConfigUart();
void DefaultConfigADC();
void DefaultConfigDAC();

// Sine Wave: 8-bit, 32 samples/cycle
const uint8_t sine_table[32] = {127,151,175,197,216,232,244,251,254,251,244,
232,216,197,175,151,127,102,78,56,37,21,9,2,0,2,9,21,37,56,78,102};
// Triangle Wave: 8-bit, 32 samples/cycle
const uint8_t triangle_table[32] = {0,15,31,47,63,79,95,111,127,142,158,174,
190,206,222,238,254,238,222,206,190,174,158,142,127,111,95,79,63,47,31,15};
// Sawtooth Wave: 8-bit, 32 samples/cycle
const uint8_t sawtooth_table[32] = {0,7,15,23,31,39,47,55,63,71,79,87,95,103,
111,119,127,134,142,150,158,166,174,182,190,198,206,214,222,230,238,246};
// Square Wave: 8-bit, 32 samples/cycle
const uint8_t square_table[32] = {254,254,254,254,254,254,254,254,254,254,
254,254,254,254,254,254,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

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
	
	// Configure the LEDs
	DefaultConfigLeds();
	
	// Configure UART for debugging
	DefaultConfigUart();
	
	DefaultConfigADC();
	
	DefaultConfigDAC();
	
	HAL_Delay(1000);

	TurnOffLed(RED);
	TurnOffLed(BLUE);
	TurnOffLed(GREEN);
	TurnOffLed(ORANGE);

	uint32_t adcData = 0;
	uint8_t tableIndex = 0;

	// Doing both ADC and DAC stuff in the whle loop
  /* Infinite loop */
  while (1)
  {
		HAL_Delay(1);
		
		DAC1->DHR8R1 = sawtooth_table[tableIndex++];
		DAC1->SWTRIGR |= 0x1;
		if(tableIndex == sizeof(sawtooth_table))
		{
			tableIndex = 0;
		}
		
		// Lab part 1
		if( ((ADC1->ISR & EOC_MASK) >> 2) == 1)
		{
			// Get the converted data
			adcData = ADC1->DR;
			
			if(adcData <= 0x40)
			{
				TurnOnLed(BLUE);
				TurnOffLed(RED);
				TurnOffLed(GREEN);
				TurnOffLed(ORANGE);
			}
			else if(adcData > 0x40 && adcData <= 0x80)
			{
				TurnOnLed(ORANGE);
				TurnOnLed(BLUE);
				TurnOffLed(RED);
				TurnOffLed(GREEN);
			}
			else if(adcData > 0x80 && adcData <= 0xC0)
			{
				TurnOnLed(ORANGE);
				TurnOnLed(BLUE);
				TurnOnLed(RED);
				TurnOffLed(GREEN);
			}
			else
			{
				TurnOnLed(ORANGE);
				TurnOnLed(BLUE);
				TurnOnLed(RED);
				TurnOnLed(GREEN);
			}
		}
  }
}

void DefaultConfigDAC()
{
	RCC->APB1ENR |= RCC_APB1ENR_DACEN;
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	
	// Configure PA4 as the DAC out
	GPIOA->MODER = 0x00000C00;
	
	DAC1->CR |= 0x00000039;
}

void DefaultConfigADC()
{
	RCC->APB2ENR |= RCC_APB2ENR_ADCEN;
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	
	// Configure PA1 as the ADC in pin
	GPIOA->MODER = 0x0000000C;
	
	// Configure to continuous mode, 8 bit res, and software trigger
	ADC1->CFGR1 |= 0x00002010;
	
	// Select channel 1 to be enabled
	ADC1->CHSELR |= 0x00000002;
	
	// Go through the calibration process
	/* (1) Ensure that ADEN = 0 */
	/* (2) Clear ADEN by setting ADDIS*/
	/* (3) Clear DMAEN */
	/* (4) Launch the calibration by setting ADCAL */
	/* (5) Wait until ADCAL=0 */
	if ((ADC1->CR & ADC_CR_ADEN) != 0) /* (1) */
	{
	ADC1->CR |= ADC_CR_ADDIS; /* (2) */
	}
	while ((ADC1->CR & ADC_CR_ADEN) != 0)
	{
	/* For robust implementation, add here time-out management */
	}
	ADC1->CFGR1 &= ~ADC_CFGR1_DMAEN; /* (3) */
	ADC1->CR |= ADC_CR_ADCAL; /* (4) */
	while ((ADC1->CR & ADC_CR_ADCAL) != 0) /* (5) */
	{
	/* For robust implementation, add here time-out management */
	}
	
	// Enable the ADC
	ADC1->CR |= 0x00000005;
	
	TurnOnLed(RED);
	TurnOnLed(BLUE);
	TurnOnLed(GREEN);
	TurnOnLed(ORANGE);
}

void DefaultConfigLeds()
{
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	
	GPIOC->MODER |= 0x00055000;
}

void SendChar(char character)
{
	// Could potentially sit here forever
	while( (USART1->ISR & USART_TX_READY_MASK) == 0 );
	
	USART1->TDR = character;
}

void SendString(const char* string)
{
	uint8_t index = 0;
	
	// Test for null and stop if found
	while(string[index] != '\0')
	{
		SendChar(string[index]);
		++index;
	}
	
	SendChar(CARRIAGE_RETURN);
	SendChar(LINE_FEED);
}

void ToggleLed(enum Led_e led)
{
	switch(led)
	{
		case RED:
			GPIOC->ODR ^= 0x40;
			break;
		case BLUE:
			GPIOC->ODR ^= 0x80;
			break;
		case GREEN:
			GPIOC->ODR ^= 0x200;
			break;
		case ORANGE:
			GPIOC->ODR ^= 0x100;
			break;
		default:
			SendString("PROBLEM!");
			break;
	}
}

void TurnOnLed(enum Led_e led)
{
	switch(led)
	{
		case RED:
			GPIOC->ODR |= 0x40;
			break;
		case BLUE:
			GPIOC->ODR |= 0x80;
			break;
		case GREEN:
			GPIOC->ODR |= 0x200;
			break;
		case ORANGE:
			GPIOC->ODR |= 0x100;
			break;
		default:
			SendString("PROBLEM!");
			break;
	}
}

void TurnOffLed(enum Led_e led)
{
	switch(led)
	{
		case RED:
			GPIOC->ODR &= 0xFFBF;
			break;
		case BLUE:
			GPIOC->ODR &= 0xFF7F;
			break;
		case GREEN:
			GPIOC->ODR &= 0xFDFF;
			break;
		case ORANGE:
			GPIOC->ODR &= 0xFEFF;
			break;
		default:
			SendString("PROBLEM!");
			break;
	}
}

void DefaultConfigUart()
{
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	
	// Set the USART1 baud rate prescaler to give a baud rate of 115200
	USART1->BRR = HAL_RCC_GetHCLKFreq()/BAUD_RATE;
	
	// Configure GPIOA pins 10 (RX) and 9 (TX) to use the their alternate functions
	GPIOA->MODER = 0x00280000;
	GPIOA->AFR[1] = 0x110;
	
	// Turn on the USART1 with the TX and RX enabled
	USART1->CR1 |= 0xD;
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
