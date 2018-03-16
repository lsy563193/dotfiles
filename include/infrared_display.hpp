//
// Created by austin on 18-3-16.
//

#ifndef PP_INFRARED_DISPLAY_HPP
#define PP_INFRARED_DISPLAY_HPP

#include <cstdint>

class InfraredDisplay
{
public:
	InfraredDisplay() = default;
	~InfraredDisplay() = default;

	void displayNormalMsg(uint8_t step, uint16_t content);
	void displayErrorMsg(uint8_t step, uint16_t content, uint16_t error_code);
	void setOff();

};

extern InfraredDisplay infrared_display;
#endif //PP_INFRARED_DISPLAY_HPP
