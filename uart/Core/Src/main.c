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

#define BAUD_RATE 115200
#define USART_TX_READY_MASK 0x80
#define USART_RX_READY_MASK 0x20
#define CARRIAGE_RETURN 0x0D
#define LINE_FEED 0x0A
#define EOT 0x04

enum Led_e
{
	RED = 0,
	BLUE,
	GREEN,
	ORANGE
};

// Define a const string to send to the user
static const char INVALID_KEY_ERROR_MESSAGE[] = {"Invalid key pressed"};
	
static const char RED_LED_TOGGLE[] = {"Red LED toggled"};
static const char RED_LED_ON[] = {"Red LED turned on"};
static const char RED_LED_OFF[] = {"Red LED turned off"};
	
static const char BLUE_LED_TOGGLE[] = {"Blue LED toggled"};
static const char BLUE_LED_ON[] = {"Blue LED turned on"};
static const char BLUE_LED_OFF[] = {"Blue LED turned off"};
	
static const char GREEN_LED_TOGGLE[] = {"Green LED toggled"};
static const char GREEN_LED_ON[] = {"Green LED turned on"};
static const char GREEN_LED_OFF[] = {"Green LED turned off"};
	
static const char ORANGE_LED_TOGGLE[] = {"Orange LED toggled"};
static const char ORANGE_LED_ON[] = {"Orange LED turned on"};
static const char ORANGE_LED_OFF[] = {"Orange LED turned off"};

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

// Functions to send a character to the host
void SendString(const char* string);
void SendChar(char character);
void DefaultConfigLeds();
void ToggleLed(enum Led_e led);
void TurnOnLed(enum Led_e led);
void TurnOffLed(enum Led_e led);

static volatile char gRxData = 0;
static volatile bool gDataReady = false;


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
	// A local variable to store the most recent command byte
	char localRxData = 0;
	
	// A flag to track which command byte we are on
	bool firstByteValid = false;
	
	enum Led_e ledSelected;
	
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();
	
	// Configure the USART1 and GPIOC (LED) peripherals
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN | RCC_AHBENR_GPIOAEN;
	
	// Set the USART1 baud rate prescaler to give a baud rate of 115200
	USART1->BRR = HAL_RCC_GetHCLKFreq()/BAUD_RATE;
	
	// Configure GPIOA pins 10 (RX) and 9 (TX) to use the their alternate functions
	GPIOA->MODER = 0x00280000;
	GPIOA->AFR[1] = 0x110;
	
	// Configure the LEDs in the default settings so we can turn them on/off
	DefaultConfigLeds();
	
	// Configure the USART1 interrupt
	NVIC_EnableIRQ(USART1_IRQn);
	NVIC_SetPriority(USART1_IRQn, 3); // Set with low priority
	
	// Turn on the USART1 with the TX and RX enabled
	// also enable the RXNE interrupt
	USART1->CR1 |= 0x2D;
	
  while (1)
  {
		while( gDataReady == false);
		
		// Save global state and clear the flag
		localRxData = gRxData;
		gDataReady = false;
		
		if( firstByteValid == false )
		{
			if( localRxData == 'r' )
			{
				ledSelected = RED;
				firstByteValid = true;
			}
			else if( localRxData == 'b' )
			{
				ledSelected = BLUE;
				firstByteValid = true;
			}
			else if( localRxData == 'g' )
			{
				ledSelected = GREEN;
				firstByteValid = true;
			}
			else if( localRxData == 'o' )
			{
				ledSelected = ORANGE;
				firstByteValid = true;
			}
			else
			{
				SendString(INVALID_KEY_ERROR_MESSAGE);
			}
		}
		else
		{
			if( localRxData == '0' )
			{
				TurnOffLed(ledSelected);
			}
			else if( localRxData == '1' )
			{
				TurnOnLed(ledSelected);
			}
			else if( localRxData == '2' )
			{
				ToggleLed(ledSelected);
			}
			else
			{
				SendString(INVALID_KEY_ERROR_MESSAGE);
			}
			
			firstByteValid = false; // Return to the state of parsing first command byte
		}
  }
}

void DefaultConfigLeds()
{
	GPIOC->MODER = 0x00055000;
	GPIOC->OTYPER &= 0x00000000;
	GPIOC->OSPEEDR &= 0x00000000;
	GPIOC->PUPDR &= 0x00000000;
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
			SendString(RED_LED_TOGGLE);
			break;
		case BLUE:
			GPIOC->ODR ^= 0x80;
			SendString(BLUE_LED_TOGGLE);
			break;
		case GREEN:
			GPIOC->ODR ^= 0x200;
			SendString(GREEN_LED_TOGGLE);
			break;
		case ORANGE:
			GPIOC->ODR ^= 0x100;
			SendString(ORANGE_LED_TOGGLE);
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
			SendString(RED_LED_ON);
			break;
		case BLUE:
			GPIOC->ODR |= 0x80;
			SendString(BLUE_LED_ON);
			break;
		case GREEN:
			GPIOC->ODR |= 0x200;
			SendString(GREEN_LED_ON);
			break;
		case ORANGE:
			GPIOC->ODR |= 0x100;
			SendString(ORANGE_LED_ON);
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
			SendString(RED_LED_OFF);
			break;
		case BLUE:
			GPIOC->ODR &= 0xFF7F;
			SendString(BLUE_LED_OFF);
			break;
		case GREEN:
			GPIOC->ODR &= 0xFDFF;
			SendString(GREEN_LED_OFF);
			break;
		case ORANGE:
			GPIOC->ODR &= 0xFEFF;
			SendString(ORANGE_LED_OFF);
			break;
		default:
			SendString("PROBLEM!");
			break;
	}
}

void USART1_IRQHandler(void)
{
	gRxData = USART1->RDR;
	gDataReady = true;
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
