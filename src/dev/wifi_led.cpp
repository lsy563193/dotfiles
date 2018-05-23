//
// Created by austin on 18-3-21.
//

#include "ros/ros.h"
#include <serial.h>
#include "wifi_led.hpp"

WifiLed wifi_led;

void WifiLed::set(bool _switch)
{
	if (_switch)
		serial.setSendData(CTL_MIX, static_cast<uint8_t>(serial.getSendData(CTL_MIX) | 0x01));
	else
		serial.setSendData(CTL_MIX, static_cast<uint8_t>(serial.getSendData(CTL_MIX) & ~0x01));
}

void WifiLed::setMode(uint8_t type, WifiLed::state _switch, uint16_t time_ms)
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

	bool _switch = true;
	switch (led_type_)
	{
		case LED_FLASH:
		{
			if (live_led_cnt_for_switch_ > led_cnt_for_one_cycle_ / 2)
				_switch = false;
			break;
		}
		case LED_STEADY: //case LED_STEADY:
		{
			led_update_flag_ = false;
			break;
		}
	}

	if (live_led_cnt_for_switch_++ > led_cnt_for_one_cycle_)
		live_led_cnt_for_switch_ = 0;

	switch (led_switch_)
	{
		case state::on:
		{
			set(_switch);
			break;
		}
		case state::off:
		{
			set(false);
			break;
		}
		default:
			set(false);
			ROS_WARN("%s,%d,switch state input wrong,used default off state!!",__FUNCTION__,__LINE__);
	}
}
