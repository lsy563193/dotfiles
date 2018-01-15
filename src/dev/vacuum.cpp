//
// Created by root on 11/17/17.
//

#include "pp.h"

Vacuum vacuum;

Vacuum::Vacuum()
{
	mode_=0;
//	mode_save_=0;
}

void Vacuum::setTmpMode(uint8_t mode)
{
	// Set the mode_ for vacuum.
	// The data should be Vac_Speed_Max/Vac_Speed_Normal/Vac_Speed_NormalL.
//	mode_ = mode_save_;
//	if (mode != Vac_Save)
//	{
//		mode_ = mode;
//		if (is_save)
//			mode_save_ = mode_;
//	}

	setSpeedByMode(mode);
	ROS_INFO("%s %d: Set vacuum mode:%d.", __FUNCTION__, __LINE__, mode);
}

void Vacuum::setMode(uint8_t mode)
{
	mode_ = mode;
	setSpeedByMode(mode);
}

void Vacuum::switchToNext()
{
	// Switch the vacuum mode_ between Max and Normal
	if (mode_ == Vac_Normal)
	{
		setMode(Vac_Max);
	} else
	{
		setMode(Vac_Normal);
	}
	// Process the vacuum mode_
	setSpeedByMode(mode_);
}
void Vacuum::setLastMode()
{
	setSpeedByMode(mode_);
}


void Vacuum::bldcSpeed(uint32_t S)
{
	// Set the power of BLDC, S should be in range(0, 100).
	S = S < 100 ? S : 100;
	serial.setSendData(CTL_VACCUM_PWR, S & 0xff);
}

void Vacuum::setSpeedByMode(uint8_t mode)
{
	// Set the power of BLDC according to different situation
	// Stop the BLDC if rGobot carries the water tank

	{
		// Set the BLDC power to max if robot in max mode_
		if (mode == Vac_Max)
		{
			bldcSpeed(Vac_Speed_Max);
		} else
		{
			// If work time less than 2 hours, the BLDC should be in normal level, but if more than 2 hours, it should slow down a little bit.
			if (robot_timer.getWorkTime() < Two_Hours)
			{
				bldcSpeed(Vac_Speed_Normal);
			} else
			{
				//ROS_INFO("%s %d: Work time more than 2 hours.", __FUNCTION__, __LINE__);
				bldcSpeed(Vac_Speed_NormalL);
			}
		}
	}
}

void Vacuum::stop(){
	bldcSpeed(0);
}

void Vacuum::startExceptionResume(void) {
	uint8_t omni_reset_byte = serial.getSendData(CTL_MIX);
	serial.setSendData(CTL_MIX, omni_reset_byte | 0x02);
}

void Vacuum::resetExceptionResume(void) {
	uint8_t omni_reset_byte = serial.getSendData(CTL_MIX);
	serial.setSendData(CTL_MIX, omni_reset_byte & ~0x06);
}

