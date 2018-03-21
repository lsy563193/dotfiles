//
// Created by austin on 18-3-21.
//

#ifndef PP_WIFI_LED_HPP
#define PP_WIFI_LED_HPP

#include <cstdint>
#include "key_led.h"
#include "robot.hpp"

// For wifi LED control

class WifiLed
{
public:
	void set(bool _switch);

// time_ms is used for both LED_FLASH type and LED_BREATH type, the default value is for LED_BREATH.
	void setMode(uint8_t type, bool _switch, uint16_t time_ms = 3000);

	void processLed();
private:

// For key_led control.
	uint8_t led_type_{LED_STEADY};
	bool led_update_flag_{false};
	bool led_switch_{OFF};
	uint16_t led_cnt_for_one_cycle_{0};
	uint16_t live_led_cnt_for_switch_{0};

};

extern WifiLed wifi_led;
#endif //PP_WIFI_LED_HPP
