#include <stdint.h>
#include <stdio.h>
#include <unistd.h>
#include "control.h"
#include "robot.hpp"
#include "common.h"

#define TAG	"Ctl. (%d):\t"

void control_set_wheel_speed(int16_t left, int16_t right) {
	robot::instance()->set_ctrl_data(CTL_WHEEL_LEFT_HIGH,(left>>8)&0xff);
	robot::instance()->set_ctrl_data(CTL_WHEEL_LEFT_LOW,left&0xff);	
	robot::instance()->set_ctrl_data(CTL_WHEEL_RIGHT_HIGH,(right>>8)&0xff);
	robot::instance()->set_ctrl_data(CTL_WHEEL_RIGHT_LOW,right&0xff);	
	robot::instance()->pub_ctrl_command();
}

void control_set_wheel_left_speed(int16_t val)
{
	robot::instance()->set_ctrl_data(CTL_WHEEL_LEFT_HIGH,(val>>8)&0xff);
	robot::instance()->set_ctrl_data(CTL_WHEEL_LEFT_LOW,val&0xff);	
	robot::instance()->pub_ctrl_command();
}

void control_set_wheel_right_speed(int16_t val)
{
	robot::instance()->set_ctrl_data(CTL_WHEEL_RIGHT_HIGH,(val>>8)&0xff);
	robot::instance()->set_ctrl_data(CTL_WHEEL_RIGHT_LOW,val&0xff);	
	robot::instance()->pub_ctrl_command();
}
void control_set_cleantool_pwr(uint8_t val){
	robot::instance()->set_ctrl_data(CTL_VACCUM_PWR,val&0xff);
	robot::instance()->set_ctrl_data(CTL_BRUSH_LEFT,val&0xff);
	robot::instance()->set_ctrl_data(CTL_BRUSH_RIGHT,val&0xff);
	robot::instance()->set_ctrl_data(CTL_BRUSH_MAIN,val&0xff);
	robot::instance()->pub_ctrl_command();
}
void control_set_vaccum_pwr(uint8_t val)
{
	control_set(CTL_VACCUM_PWR, val&0xff);
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

void control_set(uint8_t type, uint8_t val)
{
	if (type >= CTL_WHEEL_LEFT_HIGH && type <= CTL_GYRO) {
		robot::instance()->set_ctrl_data(type,val);

		robot::instance()->pub_ctrl_command();
	}
}
