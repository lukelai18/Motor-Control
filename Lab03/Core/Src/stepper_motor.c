/*
 * stepper_motor.c
 *
 *  Created on: May 8, 2024
 *      Author: 39204
 */
#include "stepper_motor.h"
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <stdarg.h>
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "stm32h7xx_hal.h"

#define ACCURACY 1600
move_t motorState ={
	.curQuadrant = NONEQUADRANT,
	.state = STOP_STATE
};
int curPulse = 1;

////----------------
////Support code (copy&paste):
//void mywrite (unsigned char *st, int nby)
//{
//	if (nby) HAL_UART_Transmit(&huart3,st,nby,10);
//}
//
//void myprintf (const char *fmt, ...)
//{
//	va_list arglist;
//	int nby;
//	unsigned char st[256];
//	if (!fmt) return;
//	va_start(arglist,fmt);
//	nby = vsnprintf((char *)st,sizeof(st),fmt,arglist);
//	va_end(arglist);
//	if (nby < 0) { nby = sizeof(st)-1; } //print truncated string in case too long
//	if (nby) HAL_UART_Transmit(&huart3,st,nby,10);
//}


// Initialize in each quadrant, and this function doesn't have the move content
void first_position(int32_t start_x, int32_t start_y, int32_t stop_x, int32_t stop_y, int8_t direction){
//	if(pow(start_x, 2) + pow(start_y, 2) != pow(stop_x, 2) + pow(stop_y, 2)){
//		return;
//	}

	// Initialize the parameters
	motorState.f_e = 0;
	motorState.startX = start_x * ACCURACY;
	motorState.startY = start_y * ACCURACY;
	motorState.endX = stop_x * ACCURACY;
	motorState.endY = stop_y * ACCURACY;
	motorState.total_pulses = labs(motorState.startX - motorState.endX) +
			labs(motorState.startY - motorState.endY);
	motorState.state = RUNNING_STATE;

	// Set up the direction for x-axis and y-axis, then set the quadrant
	new_direction(motorState.startX, motorState.startY, direction);
	// Set up the first step, the direction, and the f_e here is still 0
	if(motorState.startX == 0){
		motorState.move_axis = X_AXIS;
		HAL_TIM_PWM_Start_IT(&htim2,TIM_CHANNEL_4);
		// motorState.f_e = motorState.f_e + motorState.x_dir * 2 * motorState.startX + 1;
	} else{
		motorState.move_axis = Y_AXIS;
		HAL_TIM_PWM_Start_IT(&htim3,TIM_CHANNEL_1);
//		motorState.startY = motorState.startY + motorState.y_dir;
		// motorState.f_e = motorState.f_e + motorState.y_dir * 2 * motorState.startY + 1;
	}
}


// Set the new direction and quadrant based on the position
void new_direction(int32_t posX, int32_t posY, int8_t direction){
	motorState.dir = direction;
	// If we draw this circle counter clockwise
	if(motorState.dir == COUNTERCLOCKWISE){
		// Check Right part firstly
		if(posX >= 0 && posY >= 0 && motorState.curQuadrant == NONEQUADRANT){
				// The first quadrant
				motorState.curQuadrant = FIRSTQUADRANT;
				motorState.x_dir = COUNTERCLOCKWISE;
				motorState.y_dir = CLOCKWISE;
		}
		else if(posX >= 0 && posY <= 0 && motorState.curQuadrant == THIRDQUADRANT){
				// The Fourth quadrant
				motorState.curQuadrant = FOURTHQUADRANT;
				motorState.x_dir = CLOCKWISE;
				motorState.y_dir = CLOCKWISE;
		}
		else if(posX <= 0 && posY >= 0 && motorState.curQuadrant == FIRSTQUADRANT){
				// The second quadrant
				motorState.curQuadrant = SECONDQUADRANT;
				motorState.x_dir = COUNTERCLOCKWISE;
				motorState.y_dir = COUNTERCLOCKWISE;
		}
		else if(posX <= 0 && posY <= 0 && motorState.curQuadrant == SECONDQUADRANT) {
				// The third quadrant
				motorState.curQuadrant = THIRDQUADRANT;
				motorState.x_dir = CLOCKWISE;
				motorState.y_dir = COUNTERCLOCKWISE;
		}
	}
//	else if(motorState.dir == CLOCKWISE){
//		// If we draw this circle clockwise
//		if(posX > 0){
//			if(posY >=0){
//				// The first quadrant
//				motorState.curQuadrant = FIRSTQUADRANT;
//				motorState.x_dir = CLOCKWISE;
//				motorState.y_dir = COUNTERCLOCKWISE;
//			} else{
//				// The Fourth quadrant
//				motorState.curQuadrant = FOURTHQUADRANT;
//				motorState.x_dir = COUNTERCLOCKWISE;
//				motorState.y_dir = COUNTERCLOCKWISE;
//			}
//		} else{
//			if(posY >=0){
//				// The second quadrant
//				motorState.curQuadrant = SECONDQUADRANT;
//				motorState.x_dir = CLOCKWISE;
//				motorState.y_dir = CLOCKWISE;
//			} else {
//				// The third quadrant
//				motorState.curQuadrant = THIRDQUADRANT;
//				motorState.x_dir = COUNTERCLOCKWISE;
//				motorState.y_dir = CLOCKWISE;
//			}
//		}
//	}
}

// Determine the new parameters after each pulses
void new_position(){
	if(motorState.total_pulses == 0){
		// DEBUG VARIABLE
		// int curPulse = (int)(motorState.total_pulses);
		// myprintf("%d\n\r", curPulse);
		motorState.state = STOP_STATE;
		if(motorState.curQuadrant == FOURTHQUADRANT){
			HAL_TIM_PWM_Stop_IT(&htim2, TIM_CHANNEL_4);
			HAL_TIM_PWM_Stop_IT(&htim3, TIM_CHANNEL_1);
		}
		return;
	}

//	// Get the new position in the X and Y axis
//	if(motorState.move_axis == X_AXIS){
//		if(motorState.dir == COUNTERCLOCKWISE){
//			motorState.startX--;
//		}
////		else{
////			motorState.startX++;
////		}
//	}
//
//	if(motorState.move_axis == Y_AXIS){
//		if(motorState.dir == COUNTERCLOCKWISE){
//			motorState.startY--;
//		}
////		else{
////			motorState.startY++;
////		}
//	}

	// Update the information of the motorState, including the position, move_axis
	if(motorState.dir == COUNTERCLOCKWISE){
		if(motorState.f_e < 0){
			if(motorState.curQuadrant == FIRSTQUADRANT){
				motorState.move_axis = Y_AXIS;
				motorState.startY = motorState.startY + motorState.y_dir;
				motorState.f_e = motorState.f_e + 2 * motorState.y_dir * motorState.startY + 1;
				// Here update the CCR and other values, then move
				begin_moving(0, 40000, motorState.y_dir);
			} else if(motorState.curQuadrant == SECONDQUADRANT){
				motorState.move_axis = X_AXIS;
				motorState.startX = motorState.startX + motorState.x_dir;
				motorState.f_e = motorState.f_e + 2 * motorState.x_dir * motorState.startX + 1;
				begin_moving(40000, 0, motorState.x_dir);
			} else if(motorState.curQuadrant == THIRDQUADRANT){
				motorState.move_axis = Y_AXIS;
				motorState.startY = motorState.startY + motorState.y_dir;
				motorState.f_e = motorState.f_e + 2 * motorState.y_dir * motorState.startY + 1;
				begin_moving(0, 40000, motorState.y_dir);
			} else if(motorState.curQuadrant == FOURTHQUADRANT){
				motorState.move_axis = X_AXIS;
				motorState.startX = motorState.startX + motorState.x_dir;
				motorState.f_e = motorState.f_e + 2 * motorState.x_dir * motorState.startX + 1;
				begin_moving(40000, 0, motorState.x_dir);
			}
		} else {
			if(motorState.curQuadrant == FIRSTQUADRANT){
				motorState.move_axis = X_AXIS;
				motorState.startX = motorState.startX + motorState.x_dir;
				motorState.f_e = motorState.f_e + 2 * motorState.x_dir * motorState.startX + 1;
				begin_moving(40000, 0, motorState.x_dir);
				// Here update the CCR and other values, then move
			} else if(motorState.curQuadrant == SECONDQUADRANT){
				motorState.move_axis = Y_AXIS;
				motorState.startY = motorState.startY + motorState.y_dir;
				motorState.f_e = motorState.f_e + 2 * motorState.y_dir * motorState.startY + 1;
				begin_moving(0, 40000, motorState.y_dir);
			} else if(motorState.curQuadrant == THIRDQUADRANT){
				motorState.move_axis = X_AXIS;
				motorState.startX = motorState.startX + motorState.x_dir;
				motorState.f_e = motorState.f_e + 2 * motorState.x_dir * motorState.startX + 1;
				begin_moving(40000, 0, motorState.x_dir);
			} else if(motorState.curQuadrant == FOURTHQUADRANT){
				motorState.move_axis = Y_AXIS;
				motorState.startY = motorState.startY + motorState.y_dir;
				motorState.f_e = motorState.f_e + 2 * motorState.y_dir * motorState.startY + 1;
				begin_moving(0, 40000, motorState.y_dir);
			}
		}
	}

	motorState.total_pulses--;
	// DEBUG VARIABLE
//	 int curPulse = (int)(motorState.total_pulses);
//	 myprintf("%d\n\r", curPulse);
// 	if(motorState.total_pulses == 0){
//		HAL_TIM_PWM_Stop_IT(&htim2,TIM_CHANNEL_4);
//		HAL_TIM_PWM_Stop_IT(&htim3,TIM_CHANNEL_1);
//	}
}

void begin_moving(uint32_t X_ARR, uint32_t Y_ARR, int8_t direction){
	// Check if we need to start X-axis and Y-axis
	if(X_ARR != 0){
		HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
		// HAL_Delay (1); // wait for 1ms
		// If we need to start X-axis, set up the duty cycle, enable, and direction, then start it
		TIM2 -> ARR = X_ARR;
		TIM2 -> CCR4 = X_ARR / 2;
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET); // Set up the enableX
		// Set up the direction for X-axis
		// If goes to the negative part of X-axis
		if(direction == COUNTERCLOCKWISE){
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_RESET);
		} else if(direction == CLOCKWISE){
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_SET);
		}
		HAL_TIM_PWM_Start_IT(&htim2,TIM_CHANNEL_4);
	} else{
		HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4);
		// HAL_Delay (1); // wait for 1ms
		TIM3->ARR = Y_ARR; // first periodY
		TIM3->CCR1 = Y_ARR / 2; // first duty cycle (50%)
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_5, GPIO_PIN_RESET); // Set up the enableY
		// Set up the direction for Y-axis
		// If goes to the negative part of Y-axis
		if(direction == COUNTERCLOCKWISE){
			HAL_GPIO_WritePin(GPIOF, GPIO_PIN_4, GPIO_PIN_SET);
		} else if(direction == CLOCKWISE){
			HAL_GPIO_WritePin(GPIOF, GPIO_PIN_4, GPIO_PIN_RESET);
		}
		HAL_TIM_PWM_Start_IT(&htim3,TIM_CHANNEL_1);
	}
}
