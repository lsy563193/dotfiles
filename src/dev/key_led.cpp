//
// Created by root on 11/17/17.
//
#include <serial.h>
#include <robotbase.h>
#include "key_led.h"
KeyLed led;

void KeyLed::set(uint16_t green, uint16_t red)
{
	// Set the brightnesss of the LED within range(0, 100).
	green = green < 100 ? green : 100;
	red = red < 100 ? red : 100;
	serial.setSendData(CTL_LED_RED, red & 0xff);
	serial.setSendData(CTL_LED_GREEN, green & 0xff);
}

void KeyLed::setMode(uint8_t type, uint8_t color, uint16_t time_ms)
{
	robotbase_led_type = type;
	robotbase_led_color = color;
	robotbase_led_cnt_for_one_cycle = time_ms / 20;
	live_led_cnt_for_switch = 0;
	robotbase_led_update_flag = true;
}

