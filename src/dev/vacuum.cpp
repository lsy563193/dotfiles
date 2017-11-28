//
// Created by root on 11/17/17.
//

#include "pp.h"

Vacuum vacuum;

Vacuum::Vacuum()
{
	mode_=0;
	mode_save_=0;
}
void Vacuum::mode(uint8_t mode, bool is_save)
{
	// Set the mode_ for vacuum.
	// The data should be Vac_Speed_Max/Vac_Speed_Normal/Vac_Speed_NormalL.
	mode_ = mode_save_;
	if (mode != Vac_Save)
	{
		mode_ = mode;
		if (is_save)
			mode_save_ = mode_;
	}

	set_speed_by_mode();
	ROS_INFO("%s ,%d mode_(%d),mode_save_(%d)", __FUNCTION__, __LINE__, mode, mode_save_);
}

void Vacuum::mode(uint8_t mode)
{
	if(mode <= Vac_Save)
		mode_ = mode;
	else{
		ROS_ERROR("%s,%d, variable error",__FUNCTION__,__LINE__);
	}
	set_speed_by_mode();
}

uint8_t Vacuum::mode(void)
{
	// Return the vacuum mode_
	return mode_;
}

void Vacuum::switchToNext(bool is_save)
{
	// Switch the vacuum mode_ between Max and Normal
	if (mode() == Vac_Normal)
	{
		mode(Vac_Max, is_save);
	} else
	{
		mode(Vac_Normal, is_save);
	}
	// Process the vacuum mode_
	set_speed_by_mode();
}

void Vacuum::bldc_speed(uint32_t S)
{
	// Set the power of BLDC, S should be in range(0, 100).
	S = S < 100 ? S : 100;
	controller.setSendData(CTL_VACCUM_PWR, S & 0xff);
}

void Vacuum::set_speed_by_mode(void)
{
	// Set the power of BLDC according to different situation
	// Stop the BLDC if rGobot carries the water tank

	{
		// Set the BLDC power to max if robot in max mode_
		if (mode() == Vac_Max)
		{
			bldc_speed(Vac_Speed_Max);
		} else
		{
			// If work time less than 2 hours, the BLDC should be in normal level, but if more than 2 hours, it should slow down a little bit.
			if (get_work_time() < Two_Hours)
			{
				bldc_speed(Vac_Speed_Normal);
			} else
			{
				//ROS_INFO("%s %d: Work time more than 2 hours.", __FUNCTION__, __LINE__);
				bldc_speed(Vac_Speed_NormalL);
			}
		}
	}
}


void Vacuum::stop(){
	bldc_speed(0);
}
