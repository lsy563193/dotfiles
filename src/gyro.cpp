#include <limits.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "path_planning.h"
#include "gyro.h"
#include "mathematics.h"

uint8_t F_GYRO = 0;
uint8_t Gyro_RxBuffer[13];
uint8_t Gyro_RxIndex = 0;
uint8_t Gyro_MsgStart = 0;

uint8_t gyro_idx;         //[2]     = Index
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

int16_t	gyro_raw = 0;

uint8_t gyro_calibration = 255;

volatile uint8_t Send_Data_Buf[15] = {0xaa, 0x00, 0x00, 0x00, 0x00,
                                        0x00, 0x00, 0x00, 0x00, 0x00,
                                        0x00, 0x00, 0x00, 0x00, 0x00};

void Gyro_Reset_Cmd(void)
{
        Send_Data_Buf[2] = 0xff;
        Send_Data_Buf[3] = 0xff;
        Send_Data_Buf[4] = 0xff;

        Send_Data_Buf[14] = 0XFD;

        //USART_DMA_String(15, (char *) Send_Data_Buf);
}

void Gyro_On_Cmd(void)
{
        Send_Data_Buf[2] = 0xee;
        Send_Data_Buf[3] = 0xee;
        Send_Data_Buf[4] = 0xee;

        Send_Data_Buf[14] = 0xca;

        //USART_DMA_String(15, (char *) Send_Data_Buf);
}

void Gyro_Debug_Cmd(void)
{
        Send_Data_Buf[2] = 0x66;
        Send_Data_Buf[3] = 0x66;
        Send_Data_Buf[4] = 0x66;

        Send_Data_Buf[14] = 0x32;

        //USART_DMA_String(15, (char *) Send_Data_Buf);
}

void Gyro_Calibration_Cmd(uint8_t NewState)
{
#ifdef GYRO_CALIBRATION
	if (NewState == 1) {
	        Send_Data_Buf[2] = 0x55;
	        Send_Data_Buf[3] = 0x55;
	        Send_Data_Buf[4] = 0x55;

	        Send_Data_Buf[14] = 0xff;
	} else {
	        Send_Data_Buf[2] = 0x44;
	        Send_Data_Buf[3] = 0x44;
	        Send_Data_Buf[4] = 0x44;

	        Send_Data_Buf[14] = 0xcc;
	}
	//USART_DMA_String(15, (char *) Send_Data_Buf);
#else
	NewState = NewState;
#endif
}

void Gyro_Cmd(uint8_t NewState) {
	if (NewState != 0 && !Gyro_IsEnabled()) {

		Gyro_On_Cmd();

		Gyro_Reset();
		gyro_xacc_offset = gyro_yacc_offset = gyro_zacc_offset = SHRT_MAX;
	} else {
		Gyro_Reset_Cmd();

		gyro_xacc_offset = gyro_yacc_offset = gyro_zacc_offset = SHRT_MAX;
	}
}

double Gyro_GetCorrection(void) {
	double k = sqrt(gyro_xacc * gyro_xacc + gyro_yacc * gyro_yacc + gyro_zacc * gyro_zacc), kc = 100;

	if (k > 1000 && k < 1100) {
		if (gyro_zacc != 0) {
			k = atan(sqrt(gyro_xacc * gyro_xacc + gyro_yacc * gyro_yacc) / gyro_zacc);
		} else {
			k = 1.57079632679;
		}
		k = cos(k);
		if (k > 0.01) {
			kc = 1 / k;
		}
	}

	return kc;
}

double Gyro_GetTiltAngle(void) {
	return atan(sqrt(gyro_xacc * gyro_xacc + gyro_yacc * gyro_yacc) / gyro_zacc) * 1800 / PI;
}

uint16_t Gyro_GetTiltDirection(void) {
	if (abs(gyro_xacc) < abs(gyro_yacc)) {
		return 0;
	} else {
		if (gyro_xacc >= 0) {
			return 900;
		} else {
			return 2700;
		}
	}
}

int16_t Gyro_GetXAcc(void) {
	return gyro_xacc;
}

int16_t Gyro_GetXAccDiff(void) {
	return (gyro_xacc - gyro_xacc_offset);
}

int16_t Gyro_GetYAcc(void) {
	return gyro_yacc;
}

int16_t Gyro_GetYAccDiff(void) {
	return (gyro_yacc - gyro_yacc_offset);
}

int16_t Gyro_GetZAcc(void) {
	return gyro_zacc;
}

int16_t Gyro_GetZAccDiff(void) {
	return (gyro_zacc - gyro_zacc_offset);
}

uint16_t Gyro_GetAngle(uint8_t id) {
	return (gyro_angle[id] - gyro_offset + 3600) % 3600;
}

int16_t Gyro_GetRaw(void) {
	return gyro_raw;
}

uint16_t Gyro_GetAngleRaw(uint8_t id) {
	return (gyro_angle[id] + 3600) % 3600;
}

int16_t Gyro_GetRate(uint8_t id) {
	return gyro_rate[id];
}

uint8_t Gyro_ParseMsg(uint8_t * msg) {
	uint8_t c, checksum = 0;
	int16_t i;
	int32_t l;

	++gyro_time;

	for (c = 0; c < 12; ++c) {
		checksum += *(msg + c);
	}

	if (checksum != *(msg + 12)) {
		return 0;
	} else {
		gyro_idx = *msg;
		gyro_rate[1] = gyro_rate[0];
		gyro_angle[1] = gyro_angle[0];
		gyro_rate[0] = (int16_t)((*(msg + 3) & 0xFF) | ((*(msg + 4) << 8) & 0xFF00)) * -1;

		i = (*(msg + 1) & 0xFF) | ((*(msg + 2) << 8) & 0xFF00);  //[3:4]   = Angle
		gyro_raw = i;
		l = (int32_t)i * -1;
		l = (l + 36000 + 5) % 36000;
		l /= 10;
		gyroAngle = (uint16_t)l;
		gyro_angle[0] = (uint16_t)l;
		gyro_odometer += gyro_angle[0] - gyro_angle[1];

		gyro_yacc = (*(msg + 5) & 0xFF) | ((*(msg + 6) << 8) & 0xFF00);  //[7:8]   = X Acceleration
		gyro_xacc = (*(msg + 7) & 0xFF) | ((*(msg + 8) << 8) & 0xFF00);  //[9:10]  = Y Acceleration
		gyro_xacc *= -1;
		gyro_zacc = (*(msg + 9) & 0xFF) | ((*(msg + 10) << 8) & 0xFF00);  //[11:12] = Z Acceleration
		gyro_zacc *= -1;

		gyro_calibration = (*(msg + 11) & 0xFF);

		if (gyro_xacc_offset == SHRT_MAX) {
			gyro_xacc_offset = gyro_xacc;
		}
		if (gyro_yacc_offset == SHRT_MAX) {
			gyro_yacc_offset = gyro_yacc;
		}
		if (gyro_zacc_offset == SHRT_MAX) {
			gyro_zacc_offset = gyro_zacc;
		}
		return 1;
	}
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

uint8_t Gyro_IsUpdated(void) {
	if (F_GYRO) {
		F_GYRO = 0;
		return 1;
	}
	return 0;
}

void Gyro_SetOffset(int16_t offset) {
	gyro_offset = (3600 + offset + gyro_offset) % 3600;
	printf("Gyro Offset: %d (%d)\n", gyro_offset, offset);
}

void Gyro_SetAngle(int16_t angle, int16_t rate) {
	//gyro_offset = (gyro_angle[0] + 3600 - theta) % 3600;
	gyro_angle[1] = gyro_angle[0];
	gyro_angle[0] = angle;

	gyro_rate[1] = gyro_rate[0];
	gyro_rate[0] = rate;
}

void Gyro_Reset() {
	gyro_calibration = 255;

	gyro_offset = gyro_angle[0];
	gyro_odometer = 0;
	gyro_time = 0;

	gyro_xacc_offset = gyro_xacc;
	gyro_yacc_offset = gyro_yacc;
	gyro_zacc_offset = gyro_zacc;
}

void Gyro_Reset_With_Offset(int16_t offset) {
	gyro_calibration = 255;

	gyro_offset = offset;
	gyro_odometer = 0;
	gyro_time = 0;

	gyro_xacc_offset = gyro_xacc;
	gyro_yacc_offset = gyro_yacc;
	gyro_zacc_offset = gyro_zacc;
}

uint8_t Gyro_IsEnabled(void) {
	return 1;
}

uint16_t Gyro_GetEstimatedError(void) {
	uint16_t error = abs(gyro_odometer / 100) + gyro_time / 1000;
	return (error > 450 ? 450 : error);
}

uint8_t Gyro_GetCalibration(void) {
	return gyro_calibration;
}

void Gyro_ReceiveCharacter(uint8_t c) {
	if (Gyro_MsgStart == 0 && c == 0xAA) {
		Gyro_MsgStart = 1;
	} else if (Gyro_MsgStart == 1) {
		if ( c == 0x00) {
			Gyro_MsgStart = 2;
		} else {
			Gyro_RxIndex = 0;
			Gyro_MsgStart = 0;
		}
	} else if (Gyro_MsgStart == 2) {
		Gyro_RxBuffer[Gyro_RxIndex] = c;
		Gyro_RxIndex++;
		if (Gyro_RxIndex == 13) {
			Gyro_RxIndex = 0;
			Gyro_MsgStart = 0;
			if (Gyro_ParseMsg(Gyro_RxBuffer)) {
				F_GYRO = 1;
			}
		}
	}
}

uint8_t Gyro_Test()
{
	uint8_t state = 1;

#ifndef GYRO_XV7011

	uint16_t angle_before, angle_after;

	angle_before = Gyro_GetAngle(0);

	/* delay 1 second and get the value again for comparing. */
	delay(10000);

	angle_after = Gyro_GetAngle(0);

	if (angle_before < 1800) {
		angle_before += 3600;
	}

	if (angle_after < 1800) {
		angle_after += 3600;
	}

	if (abs(angle_before - angle_after) > 5) {
		printf("%s %d %d %d\n", __FUNCTION__, __LINE__, angle_before, angle_after);
		state = 0;
	}
#endif

	return state;
}
