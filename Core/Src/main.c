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
#include "main.h"

void SystemClock_Config(void);
void transmitChar(char c);
void transmitString(char string[]);
void USART3_4_IRQHandler();
void updateServoX(int direction);
void updateServoY(int direction);

const int NDATA = 5;
volatile int new_data = 0;
// [space, up, down, left, right]
volatile char commands[5];

// variables
volatile int space = 0;
volatile int up = 0;
volatile int down = 0;
volatile int left = 0;
volatile int right = 0;

int currX = 3500;
int currY = 0;
int delay = 0;

const int MAX = 5000;
const int MIN = 950;
const int DELAY_TIME = 10000;
const int INCS = 50;

const int UP = 0;
const int DOWN = 1;
const int LEFT = 2;
const int RIGHT = 3;

void transmitChar(char c) {
	// continue looping if transmit data register is empty, continue when there is data
	while ( (USART3->ISR & 128) == 0) {
	}
	
	USART3->TDR = c;
	return;
}

void transmitString(char string[]) {
		int i = 0;
		while (string[i] != 0) {
			transmitChar(string[i]);
			i++;
		}
		return;
}

/**
	* Interrupt for when data is ready to be read. Store
	* data into commands array.
	*
	*/
void USART3_4_IRQHandler() {
	if (new_data == 0) {
		commands[0] = USART3->RDR;
		new_data = 1;
	}
	else if (new_data == 1) {
		commands[1] = USART3->RDR;
		new_data = 2;
	}
	else if (new_data == 2) {
		commands[2] = USART3->RDR;
		new_data = 3;
	}
	else if (new_data == 3) {
		commands[3] = USART3->RDR;
		new_data = 4;
	}
	else if (new_data == 4) {
		commands[4] = USART3->RDR;
		new_data = 5;
	}

}

void updateServoX(int direction) {
	
	if (delay < DELAY_TIME) {
		delay++;
		return;
	}
	
	delay = 0;
	if (direction == LEFT) currX += INCS;
	else if (direction == RIGHT) currX -= INCS;
	else return;
	
	if (currX > MAX) currX = MAX;
	else if (currX < MIN) currX = MIN;
	
	TIM3->CR1 &= ~(1 << 0);
	TIM3->CCR1 = currX;
	TIM3->CR1 |= (1 << 0);
}

void updateServoY(int direction) {
	
	if (delay < DELAY_TIME) {
		delay++;
		return;
	}
	
	delay = 0;
	if (direction == UP) currY += INCS;
	else if (direction == DOWN) currY -= INCS;
	else return;
	
	if (currY > MAX) currY = MAX;
	else if (currY < MIN) currY = MIN;
	
	TIM3->CR1 &= ~(1 << 0);
	TIM3->CCR2 = currY;
	TIM3->CR1 |= (1 << 0);
}


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();
	
	// LED SETUP ################################################################################################
	__HAL_RCC_GPIOC_CLK_ENABLE(); // Enable the GPIOC clock in the RCC
	// GREEN  -> 9
	// ORANGE -> 8
	// BLUE		-> 7
	// RED		-> 6
	// Set up a configuration struct to pass to the initialization function
	GPIO_InitTypeDef initStr = {GPIO_PIN_9, // GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | 
	GPIO_MODE_OUTPUT_PP,
	GPIO_SPEED_FREQ_LOW,
	GPIO_NOPULL};
	HAL_GPIO_Init(GPIOC, &initStr); // Initialize pins PC6, PC7, PC8 & PC9
	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET); // Start PC6 reset
	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET); // Start PC7 reset
	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET); // Start PC8 reset
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET); // Start PC9 reset
	// UART SETUP ################################################################################################
	// USART3_RX = PC5
	// USART3_TX = PC4
	// PI_TX = 8
	// PI_RX = 10
	// USB-UART Transmit 	(TX) -> STM32F0 Receive 	(RX)
	// USB-UART Receive 	(RX) -> STM32F0 Transmit 	(TX)
	
	// enable system clock as USART3 clock
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
	
	// setting to Alternate Function Mode
	// PC4
	GPIOC->MODER |= (1 << 9);
	GPIOC->MODER &= ~(1 << 8);
	
	// PC5
	GPIOC->MODER |= (1 << 11);
	GPIOC->MODER &= ~(1 << 10);

	// selecting alternate functions AF1
	// PC4
	GPIOC->AFR[0] &= ~(1 << 19);
	GPIOC->AFR[0] &= ~(1 << 18);
	GPIOC->AFR[0] &= ~(1 << 17);
	GPIOC->AFR[0] |= (1 << 16);
	
	// PC5
	GPIOC->AFR[0] &= ~(1 << 23);
	GPIOC->AFR[0] &= ~(1 << 22);
	GPIOC->AFR[0] &= ~(1 << 21);
	GPIOC->AFR[0] |= (1 << 20);
	
	// setting Baud rate to be 115200
	// USART3->BRR &= ~(1 << 20);	// disable auto buad rate
	
	USART3->CR1 |= (1 << 3);	// enable TX
	USART3->CR1 |= (1 << 2);  // enable RX
	
	int f = HAL_RCC_GetHCLKFreq();
	int target = 115200;
	
	USART3->BRR = f / target ;
	
	// interrupt enable
	USART3->CR1 |= (1 << 5);
	NVIC_EnableIRQ(29);
	NVIC_SetPriority(29, 1);

	USART3->CR1 |= (1 << 0);		// enable USART
	
	// PWM SETUP ################################################################################################
	
	// Initialize LEDs 
  // Blue LED (PC7)
	// set to Alternate Function
  GPIOC->MODER &= ~(1 << 14);
  GPIOC->MODER |= (1 << 15);
	
  GPIOC->OTYPER &= ~(1 << 7);
  GPIOC->OSPEEDR &= ~(1 << 14);
  GPIOC->OSPEEDR &= ~(1 << 15);
  GPIOC->PUPDR &= ~(1 << 14);
  GPIOC->PUPDR &= ~(1 << 15);

  // Red LED (PC6)
	// set to Alternate Function
  GPIOC->MODER &= ~(1 << 12);
  GPIOC->MODER |= (1 << 13);
	
  GPIOC->OTYPER &= ~(1 << 6);
  GPIOC->OSPEEDR &= ~(1 << 12);
  GPIOC->OSPEEDR &= ~(1 << 13);
  GPIOC->PUPDR &= ~(1 << 12); 
  GPIOC->PUPDR &= ~(1 << 13); 
	
	// ENABLE TIM3
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

	// Frequency target = 50, Fclock = 8 000 000
	// 50 = 8 000 000 / ( (PSC + 1) * ARR )
	// PSC + 1 = 4, PSC = 3
	// ARR = 50000
  TIM3->PSC = 3;
  TIM3->ARR = 50000;
	
	// PC6 -> CHANNEL 1 
	// SET OUTPUT for channel 1
	TIM3->CCMR1 &= ~(1 << 1);
	TIM3->CCMR1 &= ~(1 << 0);	
	// PWM Mode 1 for channel 1
	TIM3->CCMR1 |= (1 << 6);
	TIM3->CCMR1 |= (1 << 5);
	TIM3->CCMR1 &= ~(1 << 4);
	// ENABLE OUTPUT for channel 1
	TIM3->CCER |= (1 << 0);
	// ENABLE PRELOAD for channel 1
	TIM3->CCMR1 |= (1 << 3);
	
	// PC7 -> CHANNEL 2
	// SET OUTPUT for channel 2
	TIM3->CCMR1 &= ~(1 << 9);
	TIM3->CCMR1 &= ~(1 << 8);
	// PWM Mode 1 for channel 2
	TIM3->CCMR1 |= (1 << 14);
	TIM3->CCMR1 |= (1 << 13);
	TIM3->CCMR1 &= ~(1 << 12);
	// ENABLE OUTPUT for channel 2
	TIM3->CCER |= (1 << 4);
	// ENABLE PRELOAD for channel 2
	TIM3->CCMR1 |= (1 << 11);
	
	// ADJUST DUTY-CYCLE WHERE FOR SERVOS (NOT THOROUGHLY TESTED)
	// 0 deg 		-> 2500
	// 45 deg 	-> 3750
	// 90 deg 	-> 4550
	
  TIM3->CCR1 = currX;
  TIM3->CCR2 = currY;
	
	// ENABLE TIMER 3
	TIM3->CR1 |= (1 << 0);
	

  while (1)
  {
		new_data = 0;
		
		// wait until all data (5 bytes) are received.
		while(new_data != 5) {
			
			if (up == 1) {
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
				updateServoY(UP);
				continue;
			}
			if (down == 1) {
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
				updateServoY(DOWN);
				continue;
			}
			
			if (left == 1) {
				continue;
			}
			
			if (right == 1) {
				continue;
			}
			
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
			delay = DELAY_TIME + 1;
			TIM3->CR1 &= ~(1 << 0);
			TIM3->CCR2 = 0;
			TIM3->CR1 |= (1 << 0);
		}
		
		// PARSE COMMANDS
		if (commands[0] == '1') space = 1; // space
		else space = 0;
		
		if (commands[1] == '1') up = 1;	// up
		else up = 0;
		
		if (commands[2] == '1') down = 1; // down
		else down = 0;
		
		if (commands[3] == '1') left = 1; // left
		else left = 0;
		
		if (commands[4] == '1') right = 1; // right
		else right = 0;

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
