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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "step_motor.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
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

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Used for calibrate
uint32_t total_pulses = 0;   // Record the total pulses needed in half loop
uint32_t pulseCount = 0;	// Record the count of current pulses
uint8_t loop_num = 0; 	// Record the current loop

// Used for accelaration
// #define MAX_PULSES 200
uint32_t maxPulses = 800;
uint32_t startTime; // To keep track of when acceleration started
uint32_t lastTime;
uint32_t freqArray[100000];
uint32_t decelerateCount = 0;
uint32_t constantT = 75;
uint32_t R0 = 64e6 / 60000;
uint32_t Rs = 64e6 / 12000;
uint32_t curR = 64e6 / 60000;

volatile uint8_t isPressed = 0; 	// Check if we pressed user button
volatile uint8_t isAccelerating = 1; 	// Check if we need to accelerate
volatile uint8_t isDecelerating = 0; 	// Check if we need to decelerate

// DEBUG VARIABLE
//volatile uint16_t ii = 0;
//volatile uint16_t jj = 0;
//volatile uint16_t kk = 0;
//volatile uint32_t curPeriod = 0;
//volatile uint32_t curElapseTime = 0;

// Playing Variable
// volatile uint8_t isPlaying = 0;
uint32_t curNoteIndex = 0;
// Frequency array for "Twinkle, Twinkle, Little Star"
uint32_t noteFrequencies[] = {
    // 1 1 5 5 6 6 5
    523 * 7, 523 * 7, 392 * 7, 392 * 7, 440 * 7, 440 * 7, 392 * 7,
    // 4 4 3 3 2 2 1
    698 * 7, 698 * 7, 659 * 7, 659 * 7, 587 * 7, 587 * 7, 523 * 7,
    // 5 5 4 4 3 3 2
    392 * 7, 392 * 7, 698 * 7, 698 * 7, 659 * 7, 659 * 7, 587 * 7,
    // 5 5 4 4 3 3 2
    392 * 7, 392 * 7, 698 * 7, 698 * 7, 659 * 7, 659 * 7, 587 * 7,
    // 1 1 5 5 6 6 5
    523 * 7, 523 * 7, 392 * 7, 392 * 7, 440 * 7, 440 * 7, 392 * 7,
    // 4 4 3 3 2 2 1
    698 * 7, 698 * 7, 659 * 7, 659 * 7, 587 * 7, 587 * 7, 523 * 7
};


size_t noteSize = sizeof(noteFrequencies) / sizeof(noteFrequencies[0]);

//----------------
//Support code (copy&paste):
void mywrite (unsigned char *st, int nby)
{
	if (nby) HAL_UART_Transmit(&huart3,st,nby,10);
}

void myprintf (const char *fmt, ...)
{
	va_list arglist;
	int nby;
	unsigned char st[256];
	if (!fmt) return;
	va_start(arglist,fmt);
	nby = vsnprintf((char *)st,sizeof(st),fmt,arglist);
	va_end(arglist);
	if (nby < 0) { nby = sizeof(st)-1; } //print truncated string in case too long
	if (nby) HAL_UART_Transmit(&huart3,st,nby,10);
}

void setToCenter(){
	if(loop_num == 1){
	  pulseCount++;
	} else if(loop_num == 2){
		total_pulses--;
		if(total_pulses == 0){
			HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4); // Stop PWM
			HAL_GPIO_WritePin(GPIOG, GPIO_PIN_8, GPIO_PIN_SET);
			// HAL_Delay(5000);
		}
	}
}


void accelerationProcess(uint32_t maxFreq){
	 if(isPressed  == 1){
		if(pulseCount < maxPulses){
			// Get current time and elapsetime during a pulse
			 uint32_t currentTime = HAL_GetTick();
			 uint32_t elapseTime = currentTime - lastTime;
			 // curR represent latest frequency
			 // I need to initialize the array here so that it could store all the required value,
			 // If I put this array after I update curR, I won't come back to the initial period.
			 freqArray[pulseCount] = curR;	// Using a array to store the latest frequency

//           DEBUG VARIABLE
//			 curPeriod = 64e6 / curR ;
//			 curElapseTime = elapseTime;
			 // POSSIBLE BUG: Since curR is a integer, the initial value could become 1066, which could
			 // lead 64e6 / 1066 become 60037, a bit of different with 60000, and finally the period will
			 // recover to this value.

			 curR = curR + ((Rs - R0) * elapseTime) / constantT;	// Update formula
			 if(curR > maxFreq){
				 curR = maxFreq;
			 }
			 uint32_t newPeriod = 64e6 / curR ;	// Update the new period and duty cycle

			 TIM2->ARR = newPeriod;
			 TIM2->CCR4 = newPeriod / 2;
			 pulseCount++;
//			 DEBUG VARIABLE
//			 ii++;
		} else if(pulseCount >= maxPulses && pulseCount < (uint32_t) (maxPulses * 1.5)){
			// If I run into constant speed mode, just increase pulseCount and do
			// not change the speed now
			pulseCount++;
			// If I found that it already reach the time limit, change the state to
			// deceleration state
			if(pulseCount == (uint32_t) (maxPulses * 1.5)){
				isDecelerating = 1;
			}
//			 DEBUG VARIABLE
//			jj++;
		} else if(isDecelerating && decelerateCount <= maxPulses){
			// I set the deceleration period to MAX_PULSES pulses, then I'll retrieve
			// the value I stored at freqArray to make it come back to the original state
			if(decelerateCount < maxPulses){
				// Using a deceleration count, to track if we reach the deceleration limit
				uint32_t newPeriod = 64e6 / freqArray[maxPulses - decelerateCount - 1];
			    TIM2->ARR = newPeriod;
			    TIM2->CCR4 = newPeriod / 2;
			    decelerateCount++;
			    pulseCount++;

//			 	DEBUG VARIABLE
//				curPeriod = newPeriod;
// 				kk++;
			} else {
				// Deceleration is done, re-initialize the relevant variable
				isDecelerating = 0;
				pulseCount = 0;
				decelerateCount = 0;
				curR = R0;
				GPIO_PinState curDirection = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_10);
				GPIO_PinState newDirection = (curDirection == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET;
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, newDirection);
				// Code may not used when playing music
//				isPressed = 0;
				// Code need to be used when playing music
				curNoteIndex++;
//				HAL_TIM_PWM_Stop_IT(&htim2,TIM_CHANNEL_4);
// 				HAL_Delay(20);
//				HAL_TIM_PWM_Start_IT(&htim2,TIM_CHANNEL_4);
//			 	DEBUG VARIABLE
//			   kk++;
			}
		}
	 }
}

void playTheSong(uint32_t curFreq){
	uint32_t prevPulse = curFreq;
	accelerationProcess(curFreq);
	if(curFreq != prevPulse){
		HAL_TIM_PWM_Stop_IT(&htim2,TIM_CHANNEL_4);
		HAL_Delay(100);
		HAL_TIM_PWM_Start_IT(&htim2,TIM_CHANNEL_4);
	}
}

void HAL_TIM_PWM_PulseFinishedCallback (TIM_HandleTypeDef *htim){
	 if (htim == &htim2)
	 {
		 // The following function is used for calibrate
		  setToCenter();
		 // accelerationProcess(maxPulses);

		 // PLAYING MUSIC CODE
		  // HAL_TIM_PWM_Start_IT(&htim2,TIM_CHANNEL_4);
		 // If I press the user button here, it begins running
		 playTheSong(noteFrequencies[curNoteIndex]); // Play the current note
		 // curNoteIndex++;	// Increase the note index

		 // If it already reach the limit, reset all the values.
		 if(curNoteIndex == noteSize){
			curNoteIndex = 0;
			isPressed = 0;
			HAL_TIM_PWM_Stop_IT(&htim2,TIM_CHANNEL_4);
		 }

		 lastTime = HAL_GetTick();
	 }
}

//int a = 0;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
    if(GPIO_Pin == GPIO_PIN_13){
//        // Simple software debouncing: check if the button is still pressed after a delay
//      HAL_Delay(50); // 50 ms for debouncing delay
        isPressed = 1;
//		curNoteIndex = 0;
//      HAL_TIM_PWM_Start_IT(&htim2,TIM_CHANNEL_4);
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

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_SET);

  // Maximum: 12000, minimum: 60000
   uint16_t period = 60000;
   TIM2->ARR = period; // first period
   TIM2->CCR4 = period / 2; // first duty cycle (50%)
   HAL_GPIO_WritePin(GPIOG, GPIO_PIN_8, GPIO_PIN_RESET);
   HAL_TIM_PWM_Start_IT(&htim2,TIM_CHANNEL_4);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  uint8_t leftState = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_11);
	  uint8_t rightState = HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_3);

	  if(!leftState || !rightState){
		  // Increase the count of current loop
		  loop_num++;
		  if(loop_num == 2)	total_pulses = pulseCount / 2;
		  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4); // Stop PWM

		  GPIO_PinState curDirection = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_10);
		  GPIO_PinState newDirection = (curDirection == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET;
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, newDirection);
		  HAL_Delay(100);

		  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_8, GPIO_PIN_RESET);
		  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
	 }
	HAL_Delay(200);

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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 6400 - 1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PF3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : PE11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PD10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PG8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
