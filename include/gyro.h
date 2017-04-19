#ifndef __GYRO_H__
#define __GYRO_H__

#include "debug.h"

int16_t Gyro_GetXAcc(void);
int16_t Gyro_GetYAcc(void);
int16_t Gyro_GetZAcc(void);
uint8_t Gyro_GetCalibration(void);
int16_t Gyro_GetAngle(void);

#endif /* __GYRO_H */
