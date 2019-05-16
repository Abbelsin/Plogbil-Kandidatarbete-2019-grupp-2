/*
 * motorstyrning.c
 *
 *  Created on: 8 Apr 2019
 *      Author: itn
 */

#include "motorstyrning.h"

/*
PA7	PWM B (TIM3_CH2)	8
PA6	Dir A	5
PA5	Dir B (fram/back)	6
PB3	PWM A (TIM2_CH2)	7
*/
// 100% = 20000
// 1% = 200
//Kör höger motor med procent enligt percent
// GPIO_PIN_6 = DIR_A
// PMW A TIM2_CH2 = &htim2
void drive_R_regulated(int desired_percent)
{

	motor_R(desired_percent);

}
void motor_R(int percent)
{
	motorPercentR=percent;
	int speed = motorPercentR*200;
	if(speed>=0)
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
	}else{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
		speed=-speed;
	}
	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2, speed);
}
//Kör vänster motor med procent enligt percent
// GPIO_PIN_5 = DIR_B
// PMW A TIM3_CH2 = &htim3
void motor_L(int percent)
{
	motorPercentL=percent;
	int speed = motorPercentL*200;
	if(speed>=0)
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
	}else{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
		speed=-speed;
	}
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2, speed);
}
void drivebackward(int percent)
{
	motor_R(-percent);
	motor_L(-percent);
}
void driveforward(int percent)
{
	motor_R(percent);
	motor_L(percent);
}

void turn_right(int percent)
{
	motor_R(0);
	motor_L(percent);
}
void turn_left(int percent)
{
	motor_R(percent);
	motor_L(0);
}
void hardcode_right()
{
	motor_R(20);
	motor_L(100);
}
void hardcode_left()
{
	motor_R(100);
	motor_L(50);
}
void hardcode_u_turn()
{
	motor_R(100);
	motor_L(10);
}



