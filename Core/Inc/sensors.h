/*
 * sensors.h
 *
 *  Created on: Jun 10, 2022
 *      Author: pietro
 */

#ifndef SRC_SENSORS_H_
#define SRC_SENSORS_H_

#include "gyro.h"
#include "accel.h"
#include "mag.h"

void sensorsInit(sensorGyroInitFuncPtr, sensorGyroReadFuncPtr, sensorAccInitFuncPtr, sensorAccReadFuncPtr, sensorMagInitFuncPtr, sensorMagReadFuncPtr);
void sensorsUpdate(void);

#endif /* SRC_SENSORS_H_ */
