//
// Created by austin on 17-12-7.
//

#include <movement.hpp>
#include "dev.h"

MovementCharge::MovementCharge()
{
	ROS_INFO("%s %d: Start charge action. Battery voltage \033[32m%5.2f V\033[0m.", __FUNCTION__, __LINE__, (float)battery.getVoltage()/100.0);
	wheel.stop();
	brush.stop();
	vacuum.stop();
	water_tank.stop(WaterTank::operate_option::swing_motor_and_pump);

	// For saving power.
	obs.control(OFF);
	gyro.setOff();

	if(lidar.isScanOriginalReady())
	{
		lidar.motorCtrl(OFF);
		lidar.setScanOriginalReady(0);
	}

	charger.setStart();
	usleep(30000);

	show_battery_info_time_stamp_ = time(NULL);

	directly_charge_ = charger.isDirected();

	speaker.play(VOICE_BATTERY_CHARGE);
	key_led.setMode(LED_BREATH, LED_ORANGE);

}

MovementCharge::~MovementCharge()
{
	wheel.stop();
	charger.setStop();
	obs.control(ON);
	ROS_INFO("%s %d: End movement charge.", __FUNCTION__, __LINE__);
}

bool MovementCharge::isFinish()
{
	if (!turn_for_charger_)
	{
		// Check for charger connection.
		if (charger.getChargeStatus())
			disconnect_charger_count_ = 0;
		else
			disconnect_charger_count_++;

		if (disconnect_charger_count_ > 25)
		{
			if (directly_charge_)
			{
				charger.setStop();
				return true;
			} else
			{
				key_led.setMode(LED_BREATH, LED_GREEN);
				turn_for_charger_ = true;
				start_turning_time_stamp_ = ros::Time::now().toSec();
				turn_right_finish_ = false;
				ROS_INFO("%s %d: Start turn for charger.", __FUNCTION__, __LINE__);
			}
		}

		if (charger.getChargeStatus() && battery.isFull())
		{
			if (battery_full_start_time_ == 0)
			{
				speaker.play(VOICE_BATTERY_CHARGE_DONE);
				battery_full_start_time_ = ros::Time::now().toSec();
			}

			// Show green key_led for 60s before going to sleep mode.
			if (ros::Time::now().toSec() - battery_full_start_time_ >= 60)
				key_led.setMode(LED_STEADY, LED_OFF);
			else
				key_led.setMode(LED_STEADY, LED_GREEN);
		}
	}
	else
	{
		if (charger.getChargeStatus())
		{
			turn_for_charger_ = false;
			key_led.setMode(LED_BREATH, LED_ORANGE);
		}
		if (ros::Time::now().toSec() - start_turning_time_stamp_ > 3)
		{
			MovementGoToCharger::is_turn_connect_failed_ = true;
			return true;
		}
		if (cliff.getStatus() == BLOCK_ALL)
			return true;
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
	// Debug for charge info
	if (time(NULL) - show_battery_info_time_stamp_ > 5)
	{
		ROS_INFO("%s %d: battery voltage \033[32m%5.2f V\033[0m.", __FUNCTION__, __LINE__, (float)battery.getVoltage()/100.0);
		show_battery_info_time_stamp_ = time(NULL);
	}

	IMovement::run();
}
