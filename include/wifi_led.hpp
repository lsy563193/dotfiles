//
// Created by austin on 18-3-21.
//

#ifndef PP_WIFI_LED_HPP
#define PP_WIFI_LED_HPP

#include <cstdint>
#include "key_led.h"

// For wifi LED control

class WifiLed
{
public:
	enum state{
		off =0,
		on,
	};
	void set(bool _switch);

	void setMode(uint8_t type, state _switch, uint16_t time_ms = 600);

	void processLed();

	void enable();

	void disable();
private:

// For wifi_led control.
	uint8_t led_type_{LED_STEADY};
	bool led_update_flag_{false};
	state led_switch_{state::off};
	uint16_t led_cnt_for_one_cycle_{0};
	uint16_t live_led_cnt_for_switch_{0};

	bool enable_{true};

};

extern WifiLed wifi_led;
#endif //PP_WIFI_LED_HPP
