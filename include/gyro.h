#ifndef __GYRO_H__
#define __GYRO_H__

#include "debug.h"

int16_t Gyro_GetXAcc(void);
int16_t Gyro_GetYAcc(void);
int16_t Gyro_GetZAcc(void);
uint8_t Gyro_GetCalibration(void);
int16_t Gyro_GetAngle(void);
void Gyro_SetAngle(int16_t angle);

void Set_Gyro_On(void);
bool Wait_For_Gyro_On(void);
void Set_Gyro_Off(void);
void Set_Gyro_Status(void);
void Reset_Gyro_Status(void);
uint8_t Is_Gyro_On(void);
#if GYRO_DYNAMIC_ADJUSTMENT
void Set_Gyro_Dynamic_On(void);
void Set_Gyro_Dynamic_Off(void);
#endif
#endif /* __GYRO_H */
