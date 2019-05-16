/*
 * varvtal.c
 *
 *  Created on: 8 Apr 2019
 *
 */
#include "varvtal.h"
#include "stm32f4xx_hal.h"
#include "motorstyrning.h"
/*

 */
#define MAX_SPEED_VALS 16
#define STEPS_PER_ROTATION 8
#define WHEEL_CIRC 188.5 //mm

int16_t counter_R;
int16_t counter_L;
uint32_t lastTimeR;
uint32_t lastTimeL;
uint32_t intervalTimeR;
uint32_t intervalTimeL;
const int IRdebounceInterval = 10;
int intervalHistoryR[MAX_SPEED_VALS] = {0}; //används för att få ett medelvärde på hastighet
int intervalHistoryL[MAX_SPEED_VALS] = {0};
int iR = 0;
int iL = 0;
uint32_t read_dist_R();

uint32_t read_dist_L();


// När rising edge triggas på hjul (left)
void speed_L_ISR()
{
	uint32_t currentMillis = millis();
	if(currentMillis-lastTimeL>IRdebounceInterval){
		if(motorPercentL>0)
			counter_L++;
		else if(motorPercentL<0)
			counter_L--;
		intervalHistoryL[iL++]=currentMillis-lastTimeL;
		if(iL>=MAX_SPEED_VALS)iL=0;
		lastTimeL = currentMillis;
	}
}

// När rsing edge triggas på hjul (right)
void speed_R_ISR()
{
	uint32_t currentMillis = millis();
	if(currentMillis-lastTimeR>IRdebounceInterval){
		if(motorPercentR>0)
			counter_R++;
		else if(motorPercentR<0)
			counter_R--;
		intervalHistoryR[iR++]=currentMillis-lastTimeR;
		if(iR>=MAX_SPEED_VALS)iR=0;
		lastTimeR = currentMillis;
	}
}
int read_speed_R(int N_mean)
{
	if(millis()-lastTimeR>1000){
		return 0;
	}else{
		int iStop=iR-N_mean;
		if(iStop<0) iStop+=MAX_SPEED_VALS;
		int meanInterval=0;
		for(int i = iR; i!=iStop-1; i--)
		{
			if(i<0)
				i+=MAX_SPEED_VALS;
			meanInterval += intervalHistoryR[i];
		}

		double varvPerSekund = ((N_mean*1000.0)/STEPS_PER_ROTATION)/meanInterval;
		double mmPerSekund = WHEEL_CIRC*varvPerSekund;
		if(motorPercentR>0)
			return (int)mmPerSekund;
		else if(motorPercentR<0)
			return -(int)mmPerSekund;
		else
			return 0;
	}
}
int read_speed_L(int N_mean)
{
	if(millis()-lastTimeL>1000){
		return 0;
	}else{
		int iStop=iR-N_mean;
		if(iStop<0) iStop+=MAX_SPEED_VALS;
		int meanInterval=0;
		for(int i = iL; i!=iStop-1; i--)
		{
			if(i<0)
				i+=MAX_SPEED_VALS;
			meanInterval += intervalHistoryL[i];
		}

		double varvPerSekund = ((N_mean*1000.0)/STEPS_PER_ROTATION)/meanInterval;
		double mmPerSekund = WHEEL_CIRC*varvPerSekund;

		if(motorPercentL>0)
			return (int)mmPerSekund;
		else if(motorPercentL<0)
			return -(int)mmPerSekund;
		else
			return 0;
	}
}
float read_rotation_deg() //returnerar rotationen sedan senaste counter reset
{
	return (read_counter_R()-read_counter_R())*360.0/26.0;
}
int16_t read_rotation_steps()//returnerar rotationen sedan senaste counter reset
{
	//ett varv är 26 steg
	return (read_counter_R()-read_counter_L());
}
uint32_t read_dist_R() //returnerar stäcka i mm sedan senaste counter reset
{
	uint32_t mmDistance = (WHEEL_CIRC/STEPS_PER_ROTATION)*counter_R;
	return mmDistance;
}
uint32_t read_dist_L() //returnerar stäcka i mm sedan senaste counter reset
{
	uint32_t mmDistance = (WHEEL_CIRC/STEPS_PER_ROTATION)*counter_L;
	return mmDistance;
}
int read_speed(int N_mean) //returnerar medelhastighet beräknat med önskat antal värden
{
	int iStop=iR-N_mean;
	if(iStop<0) iStop+=MAX_SPEED_VALS;
	int meanInterval=0;
	for(int i = iR; i!=iStop-1; i--)
	{
		if(i<0)
			i+=MAX_SPEED_VALS;
		meanInterval += intervalHistoryR[i];
	}
	iStop=iL-N_mean;
	if(iStop<0) iStop+=MAX_SPEED_VALS;
	for(int i = iL; i!=iStop-1; i--)
	{
		if(i<0)
			i+=MAX_SPEED_VALS;
		meanInterval += intervalHistoryL[i];
	}
	double varvPerSekund = ((N_mean*2000.0)/STEPS_PER_ROTATION)/meanInterval;
	double mmPerSekund = WHEEL_CIRC*varvPerSekund;

	return (int)mmPerSekund;
}
void reset_counter_L()
{
	counter_L=0;
}

void reset_counter_R()
{
	counter_R=0;
}

int read_counter_R()
{
	return counter_R;
}
int read_counter_L()
{
	return counter_L;
}
