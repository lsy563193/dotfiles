#ifndef __GYRO_H__
#define __GYRO_H__

#include "debug.h"

uint16_t Gyro_GetAngle(uint8_t id);
int16_t Gyro_GetXAcc(void);
int16_t Gyro_GetYAcc(void);
int16_t Gyro_GetZAcc(void);
void Gyro_SetAngle(int16_t angle, int16_t rate);
uint8_t Gyro_GetCalibration(void);

#endif /* __GYRO_H */
