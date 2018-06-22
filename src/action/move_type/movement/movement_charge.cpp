//
// Created by austin on 17-12-7.
//

#include <movement.hpp>
#include <battery.h>
#include <wheel.hpp>
#include <brush.h>
#include <vacuum.h>
#include <water_tank.hpp>
#include <obs.h>
#include <gyro.h>
#include <lidar.hpp>
#include <charger.h>
#include <speaker.h>
#include <key_led.h>
#include <wifi_led.hpp>
#include <cliff.h>
#include <serial.h>
#include <wifi/wifi.h>
#include <mode.hpp>
#include <robot.hpp>

MovementCharge::MovementCharge()
{
	ROS_WARN("%s %d: Start. Battery voltage \033[32m%5.2f V\033[0m.", __FUNCTION__, __LINE__, (float)battery.getVoltage()/100.0);
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

	show_battery_info_time_stamp_ = 0;

	directly_charge_ = charger.isDirected();

	speaker.play(VOICE_BATTERY_CHARGE);
	key_led.setMode(LED_BREATH, LED_ORANGE);

	robot::instance()->setBatteryTooLowToClean(false);
	robot::instance()->setBatteryTooLowToMove(false);
	robot::instance()->setBatteryLowForGoingHome(false);
}

MovementCharge::~MovementCharge()
{
	wheel.stop();
	charger.setStop();
	obs.control(ON);
	wifi_led.enable();
	ROS_WARN("%s %d: End.", __FUNCTION__, __LINE__);
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

		ROS_WARN_COND(!charger.getChargeStatus(), "%s %d: Disconnect of charger.", __FUNCTION__, __LINE__);

		if (disconnect_charger_count_ > 25)
		{
			if (directly_charge_)
				return true;
			else
			{
				if (robot::instance()->batteryTooLowToClean())
					key_led.setMode(LED_BREATH, LED_ORANGE);
				else
					key_led.setMode(LED_BREATH, LED_GREEN);
				turn_for_charger_ = true;
				start_turning_time_stamp_ = ros::Time::now().toSec();
				turn_right_finish_ = false;
				battery_full_and_sleep_ = false;
				s_wifi.setWorkMode(Mode::md_idle);
				s_wifi.taskPushBack(S_Wifi::ACT::ACT_UPLOAD_STATUS);
				ROS_WARN("%s %d: Start turn for charger.", __FUNCTION__, __LINE__);
			}
		}

		if (battery.isFull())
			// Preventing battery drop back below full voltage before entering fake sleep.
			battery_full_ = true;

		if (ros::Time::now().toSec() - start_timer_ >= 10 && !battery_full_and_sleep_ && charger.getChargeStatus()
			&& battery_full_)
		{
			if (battery_full_start_time_ == 0)
			{
				speaker.play(VOICE_BATTERY_CHARGE_DONE);
				battery_full_start_time_ = ros::Time::now().toSec();
			}

			// Show green key_led for 60s before going to sleep mode.
			if (ros::Time::now().toSec() - battery_full_start_time_ >= 60)
			{
				wifi_led.disable();
				key_led.setMode(LED_STEADY, LED_OFF);
				battery_full_and_sleep_ = true;
				speaker.play(VOICE_SLEEP_UNOFFICIAL);
				s_wifi.setWorkMode(Mode::md_sleep);
				s_wifi.taskPushBack(S_Wifi::ACT::ACT_UPLOAD_STATUS);
				ROS_WARN("%s %d: Enter fake sleep during charge.", __FUNCTION__, __LINE__);
			}
			else
				key_led.setMode(LED_STEADY, LED_GREEN);
		}
	}
	else
	{
		if (charger.getChargeStatus())
		{
			wifi_led.enable();
			turn_for_charger_ = false;
			start_timer_ = ros::Time::now().toSec();
			battery_full_and_sleep_ = false;
			battery_full_start_time_ = 0;
			key_led.setMode(LED_BREATH, LED_ORANGE);
			s_wifi.setWorkMode(Mode::md_charge);
			s_wifi.taskPushBack(S_Wifi::ACT::ACT_UPLOAD_STATUS);
			ROS_WARN("%s %d: Turn for charger successfully.", __FUNCTION__, __LINE__);
		}
		if (ros::Time::now().toSec() - start_turning_time_stamp_ > 3)
		{
			MovementGoToCharger::is_turn_connect_failed_ = true;
			ROS_WARN("%s %d: Should move back first in next go to charger movement.", __FUNCTION__, __LINE__);
			return true;
		}
		if (cliff.allTriggered())
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
	// For charge command protection. (If MovementCharge destructs after a new MovementCharge constructor has finished,
	// it will cause charge failure.
	if (charger.isStop())
		charger.setStart();

	// Debug for charge info
	if (time(NULL) - show_battery_info_time_stamp_ > 30)
	{
		ROS_WARN("%s %d: Battery:%.1fv, cmd:%d, status:%d, isFull:%d.", __FUNCTION__,
				 __LINE__, (float) battery.getVoltage() / 100.0, serial.getSendData(CTL_CHARGER),
				 charger.getChargeStatus(), battery.isFull());
		show_battery_info_time_stamp_ = time(NULL);
	}


	IMovement::run();
}
