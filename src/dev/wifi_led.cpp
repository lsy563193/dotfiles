//
// Created by austin on 18-3-21.
//

#include "wifi_led.hpp"

WifiLed wifi_led;

void WifiLed::set(bool _switch)
{
	if (_switch)
		serial.setSendData(CTL_MIX, static_cast<uint8_t>(serial.getSendData(CTL_MIX) | 0x01));
	else
		serial.setSendData(CTL_MIX, static_cast<uint8_t>(serial.getSendData(CTL_MIX) & ~0x01));
}

void WifiLed::setMode(uint8_t type, bool _switch, uint16_t time_ms)
{
	led_type_ = type;
	led_cnt_for_one_cycle_ = static_cast<uint16_t>(time_ms / 20);
	led_switch_ = _switch;
	live_led_cnt_for_switch_ = 0;
	led_update_flag_ = true;
}

void WifiLed::processLed()
{
	if (!led_update_flag_)
		return;

	bool _switch = ON;
	switch (led_type_)
	{
		case LED_FLASH:
		{
			if (live_led_cnt_for_switch_ > led_cnt_for_one_cycle_ / 2)
				_switch = OFF;
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

	switch (led_switch_)
	{
		case ON:
		{
			set(_switch);
			break;
		}
		default: //case: OFF:
		{
			set(OFF);
			break;
		}
	}
	//ROS_INFO("%s %d: live_led_cnt_for_switch_: %d, led_brightness: %d.", __FUNCTION__, __LINE__, live_led_cnt_for_switch_, led_brightness);
}
