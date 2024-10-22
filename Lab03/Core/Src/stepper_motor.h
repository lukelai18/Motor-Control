/*
 * stepper_motor.h
 *
 *  Created on: May 8, 2024
 *      Author: 39204
 */
#include <stdint.h>

#ifndef SRC_STEPPER_MOTOR_H_
#define SRC_STEPPER_MOTOR_H_

#define STOP_STATE 0
#define RUNNING_STATE 1
#define COUNTERCLOCKWISE -1
#define CLOCKWISE 1
#define NONEQUADRANT 0
#define FIRSTQUADRANT 1
#define SECONDQUADRANT 2
#define THIRDQUADRANT 3
#define FOURTHQUADRANT 4
#define X_AXIS 1
#define Y_AXIS 2

typedef struct move
{
	int8_t state;
	int32_t f_e;
	int32_t startX;
	int32_t startY;
	int32_t endX;	// Used for calculate the total pulses
	int32_t endY;
	int32_t total_pulses;
	int8_t move_axis;	// Not sure if need it
	int8_t dir;		// The move direction of the circle, now set it as CounterClockwise
	int8_t x_dir;	// It can be set up in the initialize code, the direction will remain the same in the same quadrant
	int8_t y_dir;
	int8_t curQuadrant;	// It will remain the same
}move_t;

extern move_t motorState;
extern int curPulse;

void first_position(int32_t start_x, int32_t start_y, int32_t stop_x, int32_t stop_y, int8_t direction);

void new_direction(int32_t posX, int32_t posY, int8_t direction);

void new_position();

void begin_moving(uint32_t X_ARR, uint32_t Y_ARR, int8_t direction);

#endif /* SRC_STEPPER_MOTOR_H_ */
