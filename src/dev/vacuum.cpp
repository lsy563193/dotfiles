//
// Created by root on 11/17/17.
//

#include <serial.h>
#include <robot_timer.h>
#include "vacuum.h"

Vacuum vacuum;

void Vacuum::setSpeedByUserSetMode()
{
	setForCurrentMaxMode(is_user_set_max_mode_);
	is_on_ = true;
}

void Vacuum::stop(){
	setSpeed(0);
	is_on_ = false;
	ROS_INFO("%s,%d,vacuum set to stop",__FUNCTION__,__LINE__);
}

//------------------------------
void Vacuum::setSpeed(uint32_t S)
{
	// Set the power of BLDC, S should be in range(0, 100).
	S = S < 100 ? S : 100;
	serial.setSendData(CTL_VACCUM_PWR, S & 0xff);
}

void Vacuum::startExceptionResume()
{
	uint8_t mix_byte = serial.getSendData(CTL_MIX);
	serial.setSendData(CTL_MIX, mix_byte | 0x02);
	ROS_INFO("%s %d: Start vacuum exception resume.", __FUNCTION__, __LINE__);
}

void Vacuum::resetExceptionResume()
{
	uint8_t mix_byte = serial.getSendData(CTL_MIX);
	serial.setSendData(CTL_MIX, mix_byte & 0xFD);
	ROS_INFO("%s %d: Stop vacuum exception resume.", __FUNCTION__, __LINE__);
}

void Vacuum::slowOperate()
{
	ROS_INFO("%s %d: Vacuum set to low.", __FUNCTION__, __LINE__);
	setSpeed(VacMode::vac_speed_low);
}

void Vacuum::fullOperate()
{
	ROS_INFO("%s %d: Vacuum set to max.", __FUNCTION__, __LINE__);
	setSpeed(VacMode::vac_speed_max);
}

void Vacuum::setForCurrentMaxMode(bool is_max)
{
	is_current_max_mode_ = is_max;
	if (is_current_max_mode_)
		fullOperate();
	else
		slowOperate();
}


void Vacuum::updateFilterTime(uint32_t addition_time)
{
	filter_time_ += addition_time;
	ROS_INFO("%s %d: Update filter operation time to %ds(%dh).", __FUNCTION__, __LINE__,
			 filter_time_, filter_time_ / 3600);
}

void Vacuum::resetFilterTime()
{
	filter_time_ = 0;
	ROS_INFO("%s %d: Reset filter operation time to 0.", __FUNCTION__, __LINE__);
}
