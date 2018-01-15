//
// Created by root on 11/17/17.
//

#include "pp.h"

Key key;

void Key::eliminate_jitter(bool key_triggered)
{
	if (key_triggered && !key.getPressStatus())
	{
		press_count_++;
		if (press_count_ > 0)
		{
			key.setPressStatus();
			press_count_ = 0;
			// When key 'clean' is triggered, it will set touch status.
			key.setTriggerStatus();
			triggered_start_time_stamp_ = ros::Time::now().toSec();
		}
	} else if (!key_triggered && key.getPressStatus())
	{
		release_count_++;
		if (release_count_ > 5)
		{
			key.resetPressStatus();
			release_count_ = 0;
		}
	} else
	{
		press_count_ = 0;
		release_count_ = 0;
	}
}

double Key::getPressTime()
{
	return (ros::Time::now().toSec() - triggered_start_time_stamp_);
}
