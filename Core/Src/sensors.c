/*
 * sensors.c
 *
 *  Created on: Jun 11, 2022
 *      Author: pietro
 */

#include "sensors.h"

void sensorsInit(sensorGyroInitFuncPtr initFnGyro, sensorGyroReadFuncPtr readFnGyro, sensorAccInitFuncPtr initFnAcc, sensorAccReadFuncPtr readFnAcc, sensorMagInitFuncPtr initFnMag, sensorMagReadFuncPtr readFnMag) {
	gyroInit(initFnGyro, readFnGyro);
	accInit(initFnAcc, readFnAcc);
	magInit(initFnMag, readFnMag);
}

void sensorsUpdate(void) {
    gyroUpdate();
	gyroFilter();
	gyroAccumulateMeasurements();
	accUpdate();
	magUpdate();
}
