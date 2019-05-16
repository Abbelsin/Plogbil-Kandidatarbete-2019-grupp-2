/*
 * DistSensor.h
 *
 *  Created on: 13 feb. 2018
 *      Author: Ibracadabra
 */

#ifndef DISTSENSOR_H_
#define DISTSENSOR_H_
#include "stm32f4xx_hal.h"
#include "vl53l0x_api.h"
#include "vl53l0x_def.h"


void init_GPIO_Hezt(VL53L0X_Dev_t *x, uint8_t adress);


#endif /* DISTSENSOR_H_ */
