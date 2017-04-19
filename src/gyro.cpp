#include <limits.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "path_planning.h"
#include "gyro.h"
#include "mathematics.h"

uint16_t gyro_angle[2];   //[3:3]   = Angle
uint16_t gyroAngle;
int16_t gyro_rate[2];     //[5:6]   = Rate
int16_t gyro_xacc;        //[7:8]   = X Acceleration
int16_t gyro_yacc;        //[9:10]  = Y Acceleration
int16_t gyro_zacc;        //[11:12] = Z Acceleration

int16_t gyro_xacc_offset;
int16_t gyro_yacc_offset;
int16_t gyro_zacc_offset;

uint16_t gyro_offset = 0;
int32_t gyro_odometer = 0;
uint16_t gyro_time = 0;

uint16_t gyro_imu_offset = 0;
uint16_t gyro_imu_angle[2];
int16_t gyro_imu_rate[2];

int16_t	gyro_raw = 0;

uint8_t gyro_calibration = 255;

int16_t Gyro_GetXAcc(void) {
	return gyro_xacc;
}

int16_t Gyro_GetYAcc(void) {
	return gyro_yacc;
}

int16_t Gyro_GetZAcc(void) {
	return gyro_zacc;
}

uint16_t Gyro_GetAngle(uint8_t id) {
	return (gyro_angle[id] - gyro_offset + 3600) % 3600;
}

/*
void Gyro_AdjustOffset(int16_t offset) {
	if (offset < 0) {
		offset *= -1;
		gyro_offset = (gyro_offset + 3600 - offset) % 3600;
	} else {
		gyro_offset = (gyro_offset + offset) % 3600;
	}
}

uint16_t Gyro_GetOffset(void) {
	return gyro_offset;
}
*/

void Gyro_SetAngle(int16_t angle, int16_t rate) {
	//gyro_offset = (gyro_angle[0] + 3600 - theta) % 3600;
	gyro_angle[1] = gyro_angle[0];
	gyro_angle[0] = angle;

	gyro_rate[1] = gyro_rate[0];
	gyro_rate[0] = rate;
}

uint8_t Gyro_GetCalibration(void) {
	return gyro_calibration;
}
