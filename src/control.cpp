#include <stdint.h>

#include "crc8.h"
#include "log.h"
#include "serial.h"

#include "control.h"

#define TAG	"Ctl. (%d):\t"

static uint8_t ctl_data[16] = {0xAA, 0x55, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

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
	control_set(CTL_VACUUM_PWR, val & 0xff);
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

void control_set_main_pwr(uint8_t val)
{
	control_set(CTL_MAIN_PWR, val & 0xff);
}

void control_set_buzzer(uint8_t val)
{
	control_set(CTL_BUZZER, val & 0xff);
}

void control_append_crc(void)
{
	uint8_t i;

#if 0
	int16_t	crc = 0;

	for (i = 0; i < 15; i++) {
		crc += ctl_data[i];
	}
#endif

	ctl_data[15] = calcBufCrc8((char *)ctl_data, 15);

	log_msg(LOG_VERBOSE, TAG, __LINE__);
	for (i = 0; i < 16; i++) {
		log_msg(LOG_VERBOSE, "%02x ", ctl_data[i]);
	}
	log_msg(LOG_VERBOSE, "\n");
}

void control_set(ControlType type, uint8_t val)
{
	uint8_t updated = 0;

	if (type > CTL_HEADER_LOW && type < CTL_RESERVE_2) {
		log_msg(LOG_VERBOSE, TAG "set type: %d\tval: %02x\n", __LINE__, type, val);
		ctl_data[type] = val;
		updated = 1;
	}

	if (updated == 1) {
		control_append_crc();
		serial_write(16, ctl_data);
	}
}
