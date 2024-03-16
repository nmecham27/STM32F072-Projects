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

// I2C defines
#define PRESC 0x1
#define SCLL 0x13
#define SCLH 0xF
#define SDADEL 0x2
#define SCLDEL 0x4
#define TXIS 0x00000002
#define NACKF 0x00000010
#define TC 0x00000040
#define RXNE 0x00000004
#define WHO_AM_I_REG 0x0F

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

// Functions to configure the I2C
void DefaultConfigI2C();
void i2cSendData(uint8_t data);
uint8_t i2cReadData(uint8_t data);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();
	
	// Configure the UART for debug prints
	DefaultConfigUart();
	
	// Configure the I2C
	DefaultConfigI2C();
	
	// Configure the LEDs
	DefaultConfigLeds();
	
	TurnOnLed(GREEN);
	
	uint8_t whoAmIValue = i2cReadData(WHO_AM_I_REG);
	
	if(whoAmIValue == 0xD3)
	{
		TurnOnLed(RED);
	}
	else
	{
		TurnOnLed(BLUE);
	}
	
  while (1)
  {
  }
}

void i2cSendData(uint8_t data)
{
	
}

uint8_t i2cReadData(uint8_t data)
{
	SendString("Starting I2C read");
	
	uint8_t returnData = 0xFF; // Use FF as an invalid data for right now
	
	// Address of the slave device
	// bytes to transmit = 1
	// write transaction
	// start bit 1
	I2C2->CR2 |= 0x10069;
	I2C2->CR2 |= 0x02000;
	
	
	while( ((I2C2->ISR & NACKF) == 0) && ((I2C2->ISR & TXIS) == 0) )
	{
		SendString("105: Waiting for NACKF or TXIS");
	}
	
	// If no error move on
	if( ((I2C2->ISR & NACKF) == 0) && ((I2C2->ISR & TXIS) == 1) )
	{
		SendString("111: TXIS set to 1");
		
		I2C2->TXDR = data;
		
		while( (I2C2->ISR & TC) == 0 )
		{
			SendString("119: Waiting for TC");
		}
		
		if( (I2C2->ISR & TC) == 1 )
		{
			// Address of the slave device
			// bytes to transmit = 1
			// read transaction
			// start bit 1
			I2C2->CR2 |= 0x12469;
			
			while( ((I2C2->ISR & NACKF) == 0) && ((I2C2->ISR & RXNE) == 0) && ((I2C2->ISR & TC) == 0) )
			{
				SendString("136: Waiting for NACK, RXNE or TC to change");
			}
			
			if( (I2C2->ISR & TC) == 1 )
			{
				// Grab the return data
				returnData = I2C2->RXDR;
				
				// Set stop bit
				I2C2->CR2 |= 0x04000;
			}
		}
	}
	
	return returnData;
}

void DefaultConfigI2C()
{
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN | RCC_AHBENR_GPIOBEN;
	RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
	
	// Set PB11 and PB13 to their alternate functions
	GPIOB->MODER |= 0x18800000;
	GPIOB->AFR[1] |= 0x1000;
	GPIOB->OTYPER |= 0x2800;
	GPIOB->ODR = 0x0400;
	
	// Set PC0 to output push-pull
	GPIOC->MODER |= 0x1;
	GPIOC->ODR |= 0x01;
	
	// Configure standard 100 MHz timing
	I2C2->TIMINGR |= (PRESC << 28) | (SCLDEL << 20) | (SDADEL << 16) | (SCLH << 8) | (SCLL << 0);
	
	I2C2->CR1 |= 0x1;
}

void DefaultConfigLeds()
{
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	
	GPIOC->MODER |= 0x00055000;
	//GPIOC->OTYPER &= 0x00000000;
	//GPIOC->OSPEEDR &= 0x00000000;
	//GPIOC->PUPDR &= 0x00000000;
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