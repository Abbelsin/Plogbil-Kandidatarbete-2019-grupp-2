/*
 * motorstyrning.h
 *
 *  Created on: 8 Apr 2019
 *      Author: itn
 */

#ifndef MOTORSTYRNING_H_
#define MOTORSTYRNING_H_
#include "stm32f4xx_hal.h"
#include "main.h"

int8_t motorPercentR;
int8_t motorPercentL;

void motor_R(int percent);
void motor_L(int percent);
void drivebackward(int percent);
void driveforward(int percent);
void turn_right(int percent);
void turn_left(int percent);

#endif /* MOTORSTYRNING_H_ */
