//
// Created by root on 11/17/17.
//

#include <serial.h>
#include <robot_timer.h>
#include "vacuum.h"

Vacuum vacuum;

void Vacuum::setTmpSpotState()
{
	bldcSpeed(Vac_Speed_Max);
	ROS_INFO("%s %d: Set vacuum temp mode", __FUNCTION__, __LINE__);
}

void Vacuum::setTmpLowState()
{
	bldcSpeed(Vac_Speed_Low);
	ROS_INFO("%s %d: Set vacuum temp mode", __FUNCTION__, __LINE__);
}

void Vacuum::setCleanState()
{
	if(is_max_clean_state_)
		bldcSpeed(Vac_Speed_Max);
	else
		bldcSpeed(Vac_Speed_Normal);

}

void Vacuum::stop(){
	bldcSpeed(0);
	ROS_INFO("%s,%d,vacuum set to stop",__FUNCTION__,__LINE__);
}

//------------------------------
void Vacuum::bldcSpeed(uint32_t S)
{
	// Set the power of BLDC, S should be in range(0, 100).
	S = S < 100 ? S : 100;
	serial.setSendData(CTL_VACCUM_PWR, S & 0xff);
}

void Vacuum::startExceptionResume(void) {
	uint8_t mix_byte = serial.getSendData(CTL_MIX);
	serial.setSendData(CTL_MIX, mix_byte | 0x02);
	ROS_INFO("%s %d: Start vacuum exception resume.", __FUNCTION__, __LINE__);
}

void Vacuum::resetExceptionResume(void)
{
	uint8_t mix_byte = serial.getSendData(CTL_MIX);
	serial.setSendData(CTL_MIX, mix_byte & 0xFD);
	ROS_INFO("%s %d: Stop vacuum exception resume.", __FUNCTION__, __LINE__);
}

