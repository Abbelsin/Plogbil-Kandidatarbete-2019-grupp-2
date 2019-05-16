/*
 * DistSensor.c
 *
 *  Created on: 13 feb. 2018
 *      Author: Ibracadabra
 */
#include "DistSensor.h"


    uint32_t refSpadCount;
	uint8_t isApertureSpads;
	uint8_t VhvSettings;
	uint8_t PhaseCal;

void init_GPIO_Hezt(VL53L0X_Dev_t *x, uint8_t adress)
{
	//Baba Gustav Palmqvist la till delay d� han tror att transaktionerna inte hann g�ras ordentligt. Det �r ocks� m�jligt att den inte riktigt hann s�tta X_SHUT till p� innan den skrev till den. Citat i TP4027, Norrk�ping, kl.18:29, 16/2-2018.
	HAL_Delay(50);
	VL53L0X_DataInit(x);
	VL53L0X_StaticInit(x);
	VL53L0X_PerformRefCalibration(x, &VhvSettings, &PhaseCal); // Device Initialization
	VL53L0X_PerformRefSpadManagement(x, &refSpadCount, &isApertureSpads); // Device Initialization
	VL53L0X_SetDeviceMode(x, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
	VL53L0X_StartMeasurement(x);
	VL53L0X_SetDeviceAddress(x, adress);
}
