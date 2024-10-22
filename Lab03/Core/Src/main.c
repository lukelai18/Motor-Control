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
#include "stepper_motor.h"
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
// #define DEBOUNCE_DELAY 50  // milliseconds
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Used for calibrate X-axis
uint32_t total_pulsesX = 0;   // Record the total pulses needed in half loop
uint32_t pulseCountX = 0;	// Record the count of current pulses
uint8_t loop_numX = 0; 	// Record the current loop

// Used for calibrate Y-axis
uint32_t total_pulsesY = 0;   // Record the total pulses needed in half loop
uint32_t pulseCountY = 0;	// Record the count of current pulses
uint8_t loop_numY = 0;
volatile uint8_t isPressed = 0;

// Used for accelaration
// #define MAX_PULSES 200
//uint32_t maxPulses = 800;
//uint32_t startTime; // To keep track of when acceleration started
//uint32_t lastTime;
//uint32_t freqArray[100000];
//uint32_t decelerateCount = 0;
//uint32_t constantT = 75;
//uint32_t R0 = 64e6 / 60000;
//uint32_t Rs = 64e6 / 12000;
//uint32_t curR = 64e6 / 60000;
//
//volatile uint8_t isPressed = 0; 	// Check if we pressed user button
//volatile uint8_t isAccelerating = 1; 	// Check if we need to accelerate
//volatile uint8_t isDecelerating = 0; 	// Check if we need to decelerate

// Playing Variable
// volatile uint8_t isPlaying = 0;
//uint32_t curNoteIndex = 0;
//// Frequency array for "Twinkle, Twinkle, Little Star"
//uint32_t noteFrequencies[] = {
//    // 1 1 5 5 6 6 5
//	523 * 7, 523 * 7, 440 * 7, 523 * 7, 392 * 7, 440 * 7, 392 * 7
//    // 4 4 3 3 2 2 1
//};
//size_t noteSize = sizeof(noteFrequencies) / sizeof(noteFrequencies[0]);

//void accelerationProcess(uint32_t maxFreq, uint32_t* pulseCount, volatile uint32_t* ARR, volatile uint32_t* CCR, int Timer){
//	 if(isPressed  == 1){
//		if((*pulseCount) < maxPulses){
//			// Get current time and elapsetime during a pulse
//			 uint32_t currentTime = HAL_GetTick();
//			 uint32_t elapseTime = currentTime - lastTime;
//			 // curR represent latest frequency
//			 // I need to initialize the array here so that it could store all the required value,
//			 // If I put this array after I update curR, I won't come back to the initial period.
//			 freqArray[(*pulseCount)] = curR;	// Using a array to store the latest frequency
//
////           DEBUG VARIABLE
////			 curPeriod = 64e6 / curR ;
////			 curElapseTime = elapseTime;
//			 // POSSIBLE BUG: Since curR is a integer, the initial value could become 1066, which could
//			 // lead 64e6 / 1066 become 60037, a bit of different with 60000, and finally the period will
//			 // recover to this value.
//
//			 curR = curR + ((Rs - R0) * elapseTime) / constantT;	// Update formula
//			 if(curR > maxFreq){
//				 curR = maxFreq;
//			 }
//			 uint32_t newPeriod = 64e6 / curR ;	// Update the new period and duty cycle
//
//			 *ARR = newPeriod;
//			 *CCR = newPeriod / 2;
//			 (*pulseCount)++;
////			 DEBUG VARIABLE
////			 ii++;
//		} else if((*pulseCount) >= maxPulses && (*pulseCount) < (uint32_t) (maxPulses * 1.5)){
//			// If I run into constant speed mode, just increase pulseCount and do
//			// not change the speed now
//			(*pulseCount)++;
//			// If I found that it already reach the time limit, change the state to
//			// deceleration state
//			if((*pulseCount) == (uint32_t) (maxPulses * 1.5)){
//				isDecelerating = 1;
//			}
////			 DEBUG VARIABLE
////			jj++;
//		} else if(isDecelerating && decelerateCount <= maxPulses){
//			// I set the deceleration period to MAX_PULSES pulses, then I'll retrieve
//			// the value I stored at freqArray to make it come back to the original state
//			if(decelerateCount < maxPulses){
//				// Using a deceleration count, to track if we reach the deceleration limit
//				uint32_t newPeriod = 64e6 / freqArray[maxPulses - decelerateCount - 1];
//			    *ARR = newPeriod;
//			    *CCR = newPeriod / 2;
//			    decelerateCount++;
//			    (*pulseCount)++;
//
////			 	DEBUG VARIABLE
////				curPeriod = newPeriod;
//// 				kk++;
//			} else {
//				// Deceleration is done, re-initialize the relevant variable
//				isDecelerating = 0;
//				*pulseCount = 0;
//				decelerateCount = 0;
//				curR = R0;
//				// TODO: MODIFY IT
//				// Change the current direction
//				// if(Timer == 2){
//					GPIO_PinState curDirection = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_7);
//					GPIO_PinState newDirection = (curDirection == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET;
//					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, newDirection);
//				// }
//				// Code may not used when playing music
////				isPressed = 0;
//				// Code need to be used when playing music
//				curNoteIndex++;
////				HAL_TIM_PWM_Stop_IT(&htim2,TIM_CHANNEL_4);
//// 				HAL_Delay(20);
////				HAL_TIM_PWM_Start_IT(&htim2,TIM_CHANNEL_4);
////			 	DEBUG VARIABLE
////			   kk++;
//			}
//		}
//	 }
//}
//
//void playTheSong(uint32_t curFreq, uint32_t* pulseCount, volatile uint32_t* ARR, volatile uint32_t* CCR, int Timer){
//	uint32_t prevPulse = curFreq;
//	accelerationProcess(curFreq, pulseCount, ARR, CCR, Timer);
//	if(prevPulse != curFreq){
//		//if(Timer == 2){
//			HAL_TIM_PWM_Stop_IT(&htim2,TIM_CHANNEL_4);
//			HAL_Delay(100);
//			HAL_TIM_PWM_Start_IT(&htim2,TIM_CHANNEL_4);
//		// }
//	}
//}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
    if(GPIO_Pin == GPIO_PIN_13){
//        // Simple software debouncing: check if the button is still pressed after a delay
//      HAL_Delay(50); // 50 ms for debouncing delay
    	isPressed = 1;
//		curNoteIndex = 0;
//      HAL_TIM_PWM_Start_IT(&htim2,TIM_CHANNEL_4);
    }
}
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

void setToCenterX(){
	// Set X-axis to center
	if(loop_numX == 1){
	  pulseCountX++;
	} else if(loop_numX == 2){
		total_pulsesX--;
		if(total_pulsesX == 0){
			HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4); // Stop PWM
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET);
			// HAL_Delay(5000);
		}
	}
}

void setToCenterY(){
	// Set Y-axis to center
	if(loop_numY == 1){
	  pulseCountY++;
	} else if(loop_numY == 2){
		total_pulsesY--;
		if(total_pulsesY == 0){
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1); // Stop PWM
			HAL_GPIO_WritePin(GPIOF, GPIO_PIN_5, GPIO_PIN_SET);
			// HAL_Delay(5000);
		}
	}
}

void HAL_TIM_PWM_PulseFinishedCallback (TIM_HandleTypeDef *htim){
	 if (htim == &htim2)
	 {
		 // The following function is used for calibrate
		 // setToCenterX();

		 // PLAYING MUSIC CODE
		 // HAL_TIM_PWM_Start_IT(&htim2,TIM_CHANNEL_4);
		 // If I press the user button here, it begins running
//		 playTheSong(noteFrequencies[curNoteIndex], &pulseCountX, &(TIM2->ARR), &(TIM2->CCR4), 2); // Play the current note
//		 // curNoteIndex++;	// Increase the note index
//
//		 // If it already reach the limit, reset all the values.
//		 if(curNoteIndex == noteSize){
//			curNoteIndex = 0;
//			isPressed = 0;
//			HAL_TIM_PWM_Stop_IT(&htim2,TIM_CHANNEL_4);
//		 }
//
//		 lastTime = HAL_GetTick();

		 // DRAW THE CIRCLE CODE
//		 if(motorState.move_axis == X_AXIS){
//			 HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4);
//		 } else if(motorState.move_axis == Y_AXIS){
//			 HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
//		 }
		 new_position();
	 }
	 if(htim == &htim3)
	 {
		 // setToCenterY();
	     new_position();
	 }
//	 if(htim == &htim4){
//
//	 }
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
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  // Set up the initial direction for X-axis
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_SET);
  // Set up the initial direction for Y-axis
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_4, GPIO_PIN_SET);
  // Set up the initial direction for Z-axis
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_5, GPIO_PIN_RESET);

   // Maximum: 12000, minimum: 60000
   //  Set up the period in X-axis
//   uint16_t periodX = 20000;
//   TIM2->ARR = periodX; // first periodX
//   TIM2->CCR4 = periodX / 2; // first duty cycle (50%)
//   HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET); // Set up the enableX
//   HAL_TIM_PWM_Start_IT(&htim2,TIM_CHANNEL_4);
//   // Set up the period for Y-axis
//   uint16_t periodY = 20000;
//   TIM3->ARR = periodY; // first periodY
//   TIM3->CCR1 = periodY / 2; // first duty cycle (50%)
//   HAL_GPIO_WritePin(GPIOF, GPIO_PIN_5, GPIO_PIN_RESET); // Set up the enableY
//   HAL_TIM_PWM_Start_IT(&htim3,TIM_CHANNEL_1);
//    Set up the period for Z-axis
//   uint16_t periodZ = 60000;
//   TIM4->ARR = periodZ; // first periodY
//   TIM4->CCR4 = periodZ / 2; // first duty cycle (50%)
//   HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_RESET); // Set up the enableZ
//   HAL_TIM_PWM_Start_IT(&htim4,TIM_CHANNEL_4);


  while(!isPressed){
	  isPressed = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
	  HAL_Delay (20);
	  if(isPressed){
		  isPressed = 0;
		  break;
	  }
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	  HAL_ADC_Start(&hadc1); // start the adc
//
//	  HAL_ADC_PollForConversion(&hadc1, 100); // poll for conversion
//
//	  int adc_val = HAL_ADC_GetValue(&hadc1); // get the adc value
//	 // myprintf("%d\n\r", adc_val);
//
//	  HAL_ADC_Stop(&hadc1); // stop adc
//
//	  if(adc_val > 27000){
//		  HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4); // Stop PWM
//	  }
//	  HAL_Delay (200); // wait for 200ms

	  if(motorState.curQuadrant == NONEQUADRANT && motorState.state == STOP_STATE){
		  first_position(4, 0, 0, 4, COUNTERCLOCKWISE);
	  }

	  if(motorState.curQuadrant == FIRSTQUADRANT && motorState.state == STOP_STATE){
		  // DEBUG
//		  int curQuarant = (int)(motorState.curQuadrant);
//		  int curState = (int)motorState.state;
//		  myprintf("phase2 %d state% d\n\r", curQuarant, curState);
		  first_position(0, 4, -4, 0, COUNTERCLOCKWISE);
	  }

	  if(motorState.curQuadrant == SECONDQUADRANT && motorState.state == STOP_STATE){
		  first_position(-4, 0, 0, -4, COUNTERCLOCKWISE);
	  }

	  if(motorState.curQuadrant == THIRDQUADRANT && motorState.state == STOP_STATE){
		  first_position(0, -4, 0, 4, COUNTERCLOCKWISE);
	  }

	  uint8_t leftStateX = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_11);
	  uint8_t rightStateX = HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_3);
	  uint8_t leftStateY = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_12);
	  uint8_t rightStateY = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_13);

	  if(!leftStateX || !rightStateX){
		  // Increase the count of current loop
		  loop_numX++;
		  if(loop_numX == 2)	total_pulsesX = pulseCountX / 2;
		  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4); // Stop PWM

		  GPIO_PinState curDirectionX = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_7);
		  GPIO_PinState newDirectionX = (curDirectionX == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET;
		  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, newDirectionX);
		  HAL_Delay(100);

		  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);
		  HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_4);
	  }
	  if(!leftStateY || !rightStateY){
		  loop_numY++;
		  if(loop_numY == 2)	total_pulsesY = pulseCountY / 2;
		  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1); // Stop PWM

		  GPIO_PinState curDirectionY = HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_4);
		  GPIO_PinState newDirectionY = (curDirectionY == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET;
		  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_4, newDirectionY);
		  HAL_Delay(100);

		  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_5, GPIO_PIN_RESET);
		  HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_1);
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

  /** Macro to configure the PLL clock source
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSI);

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
