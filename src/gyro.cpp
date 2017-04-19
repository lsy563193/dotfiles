#include <limits.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "path_planning.h"
#include "gyro.h"
#include "mathematics.h"
#include "robot.hpp"

uint16_t gyro_angle[2];   //[3:3]   = Angle
int16_t gyro_rate[2];     //[5:6]   = Rate
int16_t gyro_xacc;        //[7:8]   = X Acceleration
int16_t gyro_yacc;        //[9:10]  = Y Acceleration
int16_t gyro_zacc;        //[11:12] = Z Acceleration

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

uint8_t Gyro_GetCalibration(void) {
	return gyro_calibration;
}

int16_t Gyro_GetAngle(void){
	auto angle = static_cast<int16_t>( (robot::instance()->robot_get_angle()) * 10);
	if(angle <0)
		angle += 3600;

	return angle;
}
