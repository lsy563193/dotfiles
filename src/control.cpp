#include <stdint.h>
#include <stdio.h>
#include <unistd.h>
#include "control.h"
#include "robot.hpp"
#include "common.h"

#include "crc8.h"
#include "serial.h"

static uint8_t ctl_data[19] = {0xAA, 0x55, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0xCC, 0x33};

void control_set_wheel_speed(int16_t left, int16_t right) {
	control_set_wheel_left_speed(left);
	control_set_wheel_right_speed(right);
}

void control_set_wheel_left_speed(int16_t val)
{
	control_set(CTL_WHEEL_LEFT_HIGH, (val >> 8) & 0xff);
	control_set(CTL_WHEEL_LEFT_LOW, val & 0xff);
}

void control_set_wheel_right_speed(int16_t val)
{
	control_set(CTL_WHEEL_RIGHT_HIGH, (val >> 8) & 0xff);
	control_set(CTL_WHEEL_RIGHT_LOW, val & 0xff);
}

void control_set_vaccum_pwr(uint8_t val)
{
	val = val < 0 ? 0 : val;
	val = val > 100 ? 100: val;

	control_set(CTL_VACCUM_PWR, ((int)(((float)val) * 2.55)) & 0xff);
}

void control_set_brush_left(uint8_t val)
{
	control_set(CTL_BRUSH_LEFT, val & 0xff);
}

void control_set_brush_right(uint8_t val)
{
	control_set(CTL_BRUSH_RIGHT, val & 0xff);
}

void control_set_brush_main(uint8_t val)
{
	control_set(CTL_BRUSH_MAIN, val & 0xff);
}

void control_set_buzzer(uint8_t val)
{
	control_set(CTL_BUZZER, val & 0xff);
}

void control_set_main_pwr(uint8_t val)
{
	control_set(CTL_MAIN_PWR, val & 0xff);
}

void control_set_led_red(uint8_t val)
{
	control_set(CTL_LED_RED, val & 0xff);
}

void control_set_led_green(uint8_t val)
{
	control_set(CTL_LED_GREEN, val & 0xff);
}

void control_set_gyro(uint8_t state, uint8_t calibration)
{
	control_set(CTL_GYRO, (state ? 0x2 : 0x0) | (calibration ? 0x1 : 0x0));
}

void control_stop_all(void)
{
	uint8_t i;

	for(i = 2; i < 17; i++) {
		if (i == 11)
			ctl_data[i] = 0x01;
		else
			ctl_data[i] = 0x00;
	}
	control_append_crc();
	serial_write(19, ctl_data);
}

void control_append_crc(void)
{
    uint8_t i;

#if 0
    int16_t crc = 0;

    for (i = 0; i < 15; i++) {
        crc += ctl_data[i];
    }
#endif

    ctl_data[16] = calcBufCrc8((char *)ctl_data, 16);
}


void control_set(uint8_t type, uint8_t val)
{
	if (type >= CTL_WHEEL_LEFT_HIGH && type <= CTL_GYRO) {
		ctl_data[type] = val;

		control_append_crc();
		serial_write(19, ctl_data);
	}
}
