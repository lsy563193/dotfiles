//
// Created by austin on 18-3-16.
//

#include "ros/ros.h"
#include "infrared_display.hpp"
#include "serial.h"

InfraredDisplay infrared_display;

void InfraredDisplay::displayNormalMsg(uint8_t step, uint16_t content)
{
	uint8_t stream = 0x00;
	stream |= 0x01 << 6;
	stream |= step;
	serial.setSendData(CTL_IR_CTRL, stream);

	serial.setSendData(CTL_IR_CONTENT_H, static_cast<uint8_t>(content >> 8));
	serial.setSendData(CTL_IR_CONTENT_L, static_cast<uint8_t>(content));

	ROS_INFO("%s %d: Step:%d, content:%d.", __FUNCTION__, __LINE__,
			 serial.getSendData(CTL_IR_CTRL) & 0x3F,
			 serial.getSendData(CTL_IR_CONTENT_H) << 8 | serial.getSendData(CTL_IR_CONTENT_L));
}

void InfraredDisplay::displayErrorMsg(uint8_t step, uint16_t content, uint16_t error_code)
{
	uint8_t stream = 0x00;
	stream |= 0x02 << 6;
	stream |= step;
	serial.setSendData(CTL_IR_CTRL, stream);

	serial.setSendData(CTL_IR_CONTENT_H, static_cast<uint8_t>(content >> 8));
	serial.setSendData(CTL_IR_CONTENT_L, static_cast<uint8_t>(content));

	serial.setSendData(CTL_IR_ERROR_CODE_H, static_cast<uint8_t>(error_code >> 8));
	serial.setSendData(CTL_IR_ERROR_CODE_L, static_cast<uint8_t>(error_code));

	ROS_INFO("%s %d: Step:%d, content:%d, error_code:%d.", __FUNCTION__, __LINE__,
			 serial.getSendData(CTL_IR_CTRL) & 0x3F,
			 serial.getSendData(CTL_IR_CONTENT_H) << 8 | serial.getSendData(CTL_IR_CONTENT_L),
			 serial.getSendData(CTL_IR_ERROR_CODE_H) << 8 | serial.getSendData(CTL_IR_ERROR_CODE_L));
}

void InfraredDisplay::setOff()
{
	uint8_t stream = 0x00;
	serial.setSendData(CTL_IR_CTRL, stream);

	serial.setSendData(CTL_IR_CONTENT_H, stream);
	serial.setSendData(CTL_IR_CONTENT_L, stream);

	serial.setSendData(CTL_IR_ERROR_CODE_H, stream);
	serial.setSendData(CTL_IR_ERROR_CODE_L, stream);

	ROS_INFO("%s %d: Infrared display set off.", __FUNCTION__, __LINE__);
}
