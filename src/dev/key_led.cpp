//
// Created by root on 11/17/17.
//
#include <serial.h>
#include "key_led.h"
KeyLed key_led;

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
	led_type_ = type;
	led_color_ = color;
	led_cnt_for_one_cycle_ = static_cast<uint16_t>(time_ms / 20);
	live_led_cnt_for_switch_ = 0;
	led_update_flag_ = true;
}

uint8_t KeyLed::getColor()
{
	return led_color_;
}


void KeyLed::processLed()
{
//	printf("led_type:%d, ledcolor:%d.\n", led_type_, led_color_);
	if (!led_update_flag_)
		return;

	uint16_t led_brightness = 100;
	switch (led_type_)
	{
		case LED_FLASH:
		{
			if (live_led_cnt_for_switch_ > led_cnt_for_one_cycle_ / 2)
				led_brightness = 0;
			break;
		}
		case LED_BREATH:
		{
			if (live_led_cnt_for_switch_ > led_cnt_for_one_cycle_ / 2)
				led_brightness = static_cast<uint16_t>(led_brightness * (2 * (float)live_led_cnt_for_switch_ / (float)led_cnt_for_one_cycle_ - 1.0));
			else
				led_brightness = static_cast<uint16_t>(led_brightness * (1.0 - 2 * (float)live_led_cnt_for_switch_ / (float)led_cnt_for_one_cycle_));
			break;
		}
		default: //case LED_STEADY:
		{
			led_update_flag_ = false;
			break;
		}
	}

	if (live_led_cnt_for_switch_++ > led_cnt_for_one_cycle_)
		live_led_cnt_for_switch_ = 0;

	switch (led_color_)
	{
		case LED_GREEN:
		{
			set(led_brightness, 0);
			break;
		}
		case LED_ORANGE:
		{
			set(led_brightness, led_brightness);
			break;
		}
		case LED_RED:
		{
			set(0, led_brightness);
			break;
		}
		default: //case: LED_OFF:
		{
			set(0, 0);
			break;
		}
	}
//	printf("%s %d: live_led_cnt_for_switch_: %d, led_brightness: %d.\n", __FUNCTION__, __LINE__, live_led_cnt_for_switch_, led_brightness);
}

void KeyLed::wifi_led(KeyLed::state state)
{
	if (state == KeyLed::state::off)
		serial.setSendData(CTL_MIX,serial.getSendData(CTL_MIX) & 0XFE);//reset wifi led
	else if (state == KeyLed::state::on)
		serial.setSendData(CTL_MIX,serial.getSendData(CTL_MIX) & 0X01);//reset wifi led
}

