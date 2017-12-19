//
// Created by austin on 17-12-6.
//

#include <pp.h>
#include "action.hpp"
#include "dev.h"

ActionCharge::ActionCharge(bool play_start_wav)
{
	ROS_INFO("%s %d: Start charge action.", __FUNCTION__, __LINE__);
	led.set_mode(LED_BREATH, LED_ORANGE);
	charger.setStart();
	usleep(30000);

	disconnect_charger_count_ = 0;
	show_battery_info_time_stamp_ = time(NULL);

	if (play_start_wav)
		speaker.play(VOICE_BATTERY_CHARGE);
}

ActionCharge::~ActionCharge()
{
	ROS_INFO("%s %d: End charge action.", __FUNCTION__, __LINE__);
}

bool ActionCharge::isFinish()
{
	if (battery.isFull())
	{
		charger.setStop();
		return true;
	}

	if (disconnect_charger_count_ > 15)
	{
		charger.setStop();
		return true;
	}

	return false;
}

void ActionCharge::run()
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

	// Check for charger connection.
	if (charger.getChargeStatus())
		disconnect_charger_count_ = 0;
	else
		disconnect_charger_count_++;

	// Debug for charge info
	if (time(NULL) - show_battery_info_time_stamp_ > 5)
	{
		ROS_INFO("%s %d: battery voltage \033[32m%5.2f V\033[0m.", __FUNCTION__, __LINE__, (float)battery.getVoltage()/100.0);
		show_battery_info_time_stamp_ = time(NULL);
	}
}
