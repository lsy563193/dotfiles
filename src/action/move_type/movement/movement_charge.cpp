//
// Created by austin on 17-12-7.
//

#include <global.h>
#include "arch.hpp"
#include "dev.h"

MovementCharge::MovementCharge()
{
	ROS_INFO("%s %d: Start charge action.", __FUNCTION__, __LINE__);
	led.set_mode(LED_BREATH, LED_ORANGE);
	charger.setStart();
	usleep(30000);

	show_battery_info_time_stamp_ = time(NULL);

	speaker.play(VOICE_BATTERY_CHARGE);

}

MovementCharge::~MovementCharge()
{
	wheel.stop();
	ROS_INFO("%s %d: End movement turn for charger.", __FUNCTION__, __LINE__);
}

bool MovementCharge::isFinish()
{

	if (battery.isFull())
		return true;

	if (!turn_for_charger_)
	{
		// Check for charger connection.
		if (charger.getChargeStatus())
			disconnect_charger_count_ = 0;
		else
			disconnect_charger_count_++;

		if (disconnect_charger_count_ > 15)
		{
			if (directly_charge_)
			{
				charger.setStop();
				return true;
			}
			else
			{
				led.set_mode(LED_STEADY, LED_ORANGE);
				turn_for_charger_ = true;
				start_turning_time_stamp_ = ros::Time::now().toSec();
				turn_right_finish_ = false;
				ROS_INFO("%s %d: Start movement turn for charger.", __FUNCTION__, __LINE__);
			}
		}
	}

	if (turn_for_charger_)
	{
		if (charger.getChargeStatus())
			turn_for_charger_ = false;
		if (ros::Time::now().toSec() - start_turning_time_stamp_ > 3)
		{
			charger.setStop();
			return true;
		}
	}

	return false;
}

void MovementCharge::adjustSpeed(int32_t &left_speed, int32_t &right_speed)
{
	if (turn_for_charger_)
	{
		if (ros::Time::now().toSec() - start_turning_time_stamp_ > 1)
			turn_right_finish_ = true;

		if (turn_right_finish_)
		{
			wheel.setDirectionRight();
			left_speed = right_speed = 5;
		} else
		{
			wheel.setDirectionLeft();
			left_speed = right_speed = 5;
		}
	}
	else
		left_speed = right_speed = 0;
}

void MovementCharge::run()
{
		// Check for plan status.
	if (robot_timer.getPlanStatus())
	{
		beeper.play_for_command(VALID);
		speaker.play(VOICE_APPOINTMENT_DONE);
		ROS_WARN("%s %d: Plan received.", __FUNCTION__, __LINE__);
		robot_timer.resetPlanStatus();
	}
	else if (robot_timer.getPlanStatus() == 2)
	{
		beeper.play_for_command(VALID);
		speaker.play(VOICE_CANCEL_APPOINTMENT);
		ROS_WARN("%s %d: Plan cancel received.", __FUNCTION__, __LINE__);
		robot_timer.resetPlanStatus();
	}

	// Debug for charge info
	if (time(NULL) - show_battery_info_time_stamp_ > 5)
	{
		ROS_INFO("%s %d: battery voltage \033[32m%5.2f V\033[0m.", __FUNCTION__, __LINE__, (float)battery.getVoltage()/100.0);
		show_battery_info_time_stamp_ = time(NULL);
	}

	IMovement::run();
}
