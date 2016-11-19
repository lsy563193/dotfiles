#ifndef __GYRO_H__
#define __GYRO_H__

#include "debug.h"

uint16_t Gyro_GetAngle(uint8_t id);
uint16_t Gyro_GetAngleRaw(uint8_t id);
int16_t Gyro_GetRaw(void);
int16_t Gyro_GetAngleDiff(uint16_t angle);
double Gyro_GetCorrection(void);
double Gyro_GetTiltAngle(void);
uint16_t Gyro_GetTiltDirection(void);
int16_t Gyro_GetRate(uint8_t id);
int16_t Gyro_GetXAcc(void);
int16_t Gyro_GetXAccDiff(void);
int16_t Gyro_GetYAcc(void);
int16_t Gyro_GetYAccDiff(void);
int16_t Gyro_GetZAcc(void);
int16_t Gyro_GetZAccDiff(void);
uint8_t Gyro_IsUpdated(void);
uint8_t Gyro_ParseMsg(uint8_t * msg);
uint16_t Gyro_GetEstimatedError(void);
void Gyro_SetOffset(int16_t offset);
void Gyro_SetAngle(int16_t angle, int16_t rate);
void Gyro_Reset(void);
void Gyro_Reset_With_Offset(int16_t offset);
uint8_t Gyro_IsEnabled(void);
uint8_t Gyro_GetCalibration(void);

void Gyro_ReceiveCharacter(uint8_t c);
uint8_t Gyro_Test(void);
void Gyro_Debug_Cmd(void);

void Gyro_SetImuOffset(int16_t offset);
uint16_t Gyro_GetImuAngle(uint8_t id);
void Gyro_SetImuAngle(int16_t angle, int16_t rate);

#endif /* __GYRO_H */
