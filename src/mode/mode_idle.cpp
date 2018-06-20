#include <event_manager.h>
#include <serial.h>
#include <key.h>
#include <rcon.h>
#include <remote.hpp>
#include <move_type.hpp>
#include <beeper.h>
#include <water_tank.hpp>
#include <vacuum.h>
#include <speaker.h>
#include <charger.h>
#include <cliff.h>
#include <robot.hpp>
#include "error.h"
#include "mode.hpp"
#include "wifi/wifi.h"
#include "appointment.h"
#include "movement.hpp"
#include "battery.h"
#include "log.h"

#define RCON_TRIGGER_INTERVAL 180

ModeIdle::ModeIdle():
	bind_lock_(PTHREAD_MUTEX_INITIALIZER)
{
	ROS_WARN("%s %d: Entering Idle mode. Current battery voltage \033[32m%5.2f V\033[0m\n=========================" ,
			 __FUNCTION__, __LINE__, (float)battery.getVoltage()/100.0);
	event_manager_register_handler(this);
	event_manager_set_enable(true);
	serial.setWorkMode(IDLE_MODE);
	sp_action_.reset(new ActionIdle);
	action_i_ = ac_idle;
	system("unturbo_cpu.sh");

	key.resetTriggerStatus();
	c_rcon.resetStatus();
	remote.reset();
	appmt_obj.resetPlanStatus();
	event_manager_reset_status();

	s_wifi.resetReceivedWorkMode();

	/*---reset values for rcon handle---*/
	sp_state = st_pause.get() ;
	sp_state->init();
	mode_i_ = md_idle;
}

ModeIdle::~ModeIdle()
{
	event_manager_set_enable(false);
	sp_action_.reset();
	ROS_WARN("%s %d: Exit idle mode.", __FUNCTION__, __LINE__);
}

bool ModeIdle::isExit()
{
	if (plan_activated_status_)
	{
		if (robot_error.get() != ERROR_CODE_NONE)
		{
			if (robot_error.clear(robot_error.get()))
			{
				ROS_WARN("%s %d: Clear the error %x.", __FUNCTION__, __LINE__, robot_error.get());
				sp_state->init();
//				speaker.play(VOICE_CLEAR_ERROR_UNOFFICIAL, false);
			} else
			{
//				speaker.play(VOICE_CANCEL_APPOINTMENT_UNOFFICIAL, false);
				// Reset action idle for playing the error alarm.
				sp_action_.reset();
				sp_action_.reset(new ActionIdle);
			}
		}

		if (robot_error.get() != ERROR_CODE_NONE)
			ROS_INFO("%s %d: Error exists, so cancel the appointment.", __FUNCTION__, __LINE__);
		else if (readyToClean())
		{
			ROS_WARN("%s %d: Idle mode receives plan, change to navigation mode.", __FUNCTION__, __LINE__);
			setNextMode(cm_navigation);
			ACleanMode::plan_activation = true;
			return true;
		}
		plan_activated_status_ = false;
	}

	if (ev.key_clean_pressed)
	{
		ROS_WARN("%s %d: Idle mode receives remote clean or clean key, change to navigation mode.",
				 __FUNCTION__, __LINE__);
		setNextMode(cm_navigation);
		return true;
	}

	if (s_wifi.receivePlan1())
	{
		if (readyToClean(true, false))
		{
			setNextMode(cm_navigation);
			ROS_WARN("%s %d: Idle mode receives wifi plan1, change to navigation mode.",
					 __FUNCTION__, __LINE__);
			return true;
		}
		s_wifi.resetReceivedWorkMode();
		s_wifi.taskPushBack(S_Wifi::ACT::ACT_UPLOAD_STATUS);
	}

	if (ev.remote_follow_wall)
	{
		ROS_WARN("%s %d: Idle mode receives remote follow wall key, change to follow wall mode.",
				 __FUNCTION__, __LINE__);
		setNextMode(cm_wall_follow);
		return true;
	}

	if (s_wifi.receiveFollowWall())
	{
		if (readyToClean(true, false))
		{
			ROS_WARN("%s %d: Idle mode receives wifi follow wall, change to follow wall mode.",
					 __FUNCTION__, __LINE__);
			setNextMode(cm_wall_follow);
			return true;
		}
		s_wifi.receiveFollowWall();
		s_wifi.taskPushBack(S_Wifi::ACT::ACT_UPLOAD_STATUS);
	}

	if (ev.remote_spot)
	{
		ROS_WARN("%s %d: Idle mode receives remote spot key, change to spot mode.",
				 __FUNCTION__, __LINE__);
		setNextMode(cm_spot);
		return true;
	}

	if (s_wifi.receiveSpot())
	{
		if (readyToClean(true, false))
		{
			ROS_WARN("%s %d: Idle mode receives wifi spot, change to spot mode.",
					 __FUNCTION__, __LINE__);
			setNextMode(cm_spot);
			return true;
		}
		s_wifi.resetReceivedWorkMode();
		s_wifi.taskPushBack(S_Wifi::ACT::ACT_UPLOAD_STATUS);
	}

	if (ev.remote_home)
	{
		ROS_WARN("%s %d: Idle mode receives remote home key, change to exploration mode.",
				 __FUNCTION__, __LINE__);
		setNextMode(cm_exploration);
		return true;
	}

	if (s_wifi.receiveHome())
	{
		if (readyToClean(false, false))
		{
			ROS_WARN("%s %d: Idle mode receives wifi home button, change to exploration mode.",
					 __FUNCTION__, __LINE__);
			setNextMode(cm_exploration);
			return true;
		}
		s_wifi.resetReceivedWorkMode();
		s_wifi.taskPushBack(S_Wifi::ACT::ACT_UPLOAD_STATUS);
	}

	if (ev.key_long_pressed || s_wifi.receiveSleep())
	{
		ROS_WARN("%s %d: Idle mode detects long press key or wifi sleep button, change to sleep mode.",
				 __FUNCTION__, __LINE__);
		setNextMode(md_sleep);
		return true;
	}

	if (ev.charge_detect)
	{
		ROS_WARN("%s %d: Idle mode detects charger, change to charge mode.", __FUNCTION__, __LINE__);
		setNextMode(md_charge);
		return true;
	}

	if (ev.remote_direction_forward || ev.remote_direction_left || ev.remote_direction_right)
	{
		ROS_WARN("%s %d: Idle mode receives remote direction key, change to remote mode.",
				 __FUNCTION__, __LINE__);
		setNextMode(md_remote);
		if (ev.remote_direction_forward)
			MoveTypeRemote::forwardStart();
		else if (ev.remote_direction_left)
			MoveTypeRemote::leftStart();
		else if (ev.remote_direction_right)
			MoveTypeRemote::rightStart();
		return true;
	}

	if (ev.rcon_status)
	{
		ROS_WARN("%s %d: Idle mode receives rcon for over %ds, change to go to charger mode.", __FUNCTION__, __LINE__,
				 RCON_TRIGGER_INTERVAL);
		setNextMode(md_go_to_charger);
		ACleanMode::robot_trapped_warning = false;
		return true;
	}

	if (sp_action_->isTimeUp())
	{
		ROS_WARN("%s %d: Idle mode time up, change to sleep mode.", __FUNCTION__, __LINE__);
		setNextMode(md_sleep);
		return true;
	}

	return false;
}

void ModeIdle::remoteKeyHandler(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Remote key %x has been pressed.", __FUNCTION__, __LINE__, remote.get());

	bool valid = true;
	if (robot_error.get())
	{
		bool force_clear = true;
		robot_error.clear(robot_error.get(), force_clear);
	}

	bool check_battery = true;
	if (remote.isKeyTrigger(REMOTE_FORWARD) || remote.isKeyTrigger(REMOTE_LEFT)
		|| remote.isKeyTrigger(REMOTE_RIGHT) || remote.isKeyTrigger(REMOTE_HOME))
		check_battery = false;

	if (!readyToClean(check_battery))
	{
		// Reset action to refresh the timeout.
		sp_action_.reset();
		sp_action_.reset(new ActionIdle);
		s_wifi.taskPushBack(S_Wifi::ACT::ACT_UPLOAD_STATUS);
		valid = false;
	}

	if (valid)
	{
		beeper.beepForCommand(VALID);
		switch (remote.get())
		{
			case REMOTE_FORWARD:
			{
				ev.remote_direction_forward = true;
				break;
			}
			case REMOTE_LEFT:
			{
				ev.remote_direction_left = true;
				break;
			}
			case REMOTE_RIGHT:
			{
				ev.remote_direction_right = true;
				break;
			}
			case REMOTE_CLEAN:
			{
				ev.key_clean_pressed = true;
				break;
			}
			case REMOTE_SPOT:
			{
				ev.remote_spot = true;
				break;
			}
			case REMOTE_HOME:
			{
				ev.remote_home = true;
				break;
			}
			case REMOTE_WALL_FOLLOW:
			{
				ev.remote_follow_wall = true;
				break;
			}
			default: // REMOTE_PLAN/REMOTE_MAX is not handled here.
				break;
		}
	}
	remote.reset();
	ACleanMode::robot_trapped_warning = false;
}

void ModeIdle::remoteMax(bool state_now, bool state_last)
{
//	PP_INFO();
	if(water_tank.getStatus(WaterTank::operate_option::swing_motor)){
		beeper.beepForCommand(INVALID);
	}
	else{
		beeper.beepForCommand(VALID);
		vacuum.setForUserSetMaxMode(!vacuum.isUserSetMaxMode());
		speaker.play(vacuum.isUserSetMaxMode() ? VOICE_VACCUM_MAX : VOICE_VACUUM_NORMAL);
		setVacuum();
	}
	// Reset the start timer for action.
	sp_action_.reset();
	sp_action_.reset(new ActionIdle);
	remote.reset();
	ACleanMode::robot_trapped_warning = false;
}

/*void ModeIdle::lidarBumper(bool state_now, bool state_last)
{
	static uint16_t lidar_bumper_cnt = 0;
	if( ! s_wifi.isConnected()){
		MutexLock lock(&bind_lock_);
		lidar_bumper_cnt++;
		if(lidar_bumper_cnt >=250 && !trigger_wifi_smart_link_){
			lidar_bumper_cnt = 0;
			beeper.beepForCommand(VALID);
			trigger_wifi_smart_link_ = true;
			ROS_INFO("%s,%d smart link",__FUNCTION__,__LINE__);
		}
		else if(lidar_bumper_cnt >= 150 && trigger_wifi_smart_link_ ){
			lidar_bumper_cnt = 0;
			beeper.beepForCommand(VALID);
			trigger_wifi_smart_link_ = false;
			trigger_wifi_smart_ap_link_ = true;
			ROS_INFO("%s,%d,smart link ap",__FUNCTION__,__LINE__);
		}
	}
	else if(!trigger_wifi_rebind_ && s_wifi.isConnected())
	{
		MutexLock lock(&bind_lock_);
		lidar_bumper_cnt++;
		if(lidar_bumper_cnt >= 250){
			lidar_bumper_cnt = 0;
			beeper.beepForCommand(VALID);
			trigger_wifi_rebind_ = true;
			ROS_INFO("%s,%d, rebind",__FUNCTION__,__LINE__);
		}
	}
}*/

void ModeIdle::remoteWifi(bool state_now,bool state_last)
{
	ROS_INFO("%s,%d,wifi state = %d ",__FUNCTION__,__LINE__,s_wifi.isConnected());
	remote.reset();
	s_wifi.taskPushBack(S_Wifi::ACT::ACT_REBIND);
	s_wifi.taskPushBack(S_Wifi::ACT::ACT_SMART_LINK);

	ACleanMode::robot_trapped_warning = false;
}

void ModeIdle::remotePlan(bool state_now, bool state_last)
{
	ACleanMode::robot_trapped_warning = false;
	if (!plan_activated_status_ && appmt_obj.getPlanStatus() > 2)
	{
		appmt_obj.resetPlanStatus();
		appmt_obj.timesUp();
		INFO_YELLOW("Plan activated.");
		plan_activated_status_ = true;
	} else
		EventHandle::remotePlan(state_now, state_last);
}

void ModeIdle::chargeDetect(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Detect charge!", __FUNCTION__, __LINE__);
	ev.charge_detect = charger.getChargeStatus();
	ACleanMode::robot_trapped_warning = false;
}

void ModeIdle::keyClean(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: key clean.", __FUNCTION__, __LINE__);
	ACleanMode::robot_trapped_warning = false;
	beeper.beepForCommand(VALID);

	// Wait for key released.
	bool long_press = false;
	bool reset_wifi = false;
	while (key.getPressStatus())
	{
		if (!long_press && key.getPressTime() > 3)
		{
			ROS_WARN("%s %d: key clean long pressed to sleep.", __FUNCTION__, __LINE__);
			beeper.beepForCommand(VALID);
			long_press = true;
		}
		if (!reset_wifi && key.getPressTime() > 5)
		{
			ROS_WARN("%s %d: key clean long pressed to reset wifi.", __FUNCTION__, __LINE__);
			beeper.beepForCommand(VALID);
			reset_wifi = true;
		}
		usleep(20000);
	}
	ROS_WARN("%s %d: Key clean is released.", __FUNCTION__, __LINE__);

	if (reset_wifi)
	{
		s_wifi.taskPushBack(S_Wifi::ACT::ACT_REBIND);
		s_wifi.taskPushBack(S_Wifi::ACT::ACT_SMART_AP_LINK);
		sp_action_.reset();
		sp_action_.reset(new ActionIdle);
	}
	else if (long_press)
		ev.key_long_pressed = true;
	else
	{
		if (robot_error.get())
		{
			bool force_clear = true;
			robot_error.clear(robot_error.get(), force_clear);
			ROS_WARN("%s %d: Clear the error %x.", __FUNCTION__, __LINE__, robot_error.get());
			sp_state->init();
//			speaker.play(VOICE_CLEAR_ERROR_UNOFFICIAL);
		}
		else if (cliff.getStatus() == BLOCK_ALL)
		{
			ROS_WARN("%s %d: Remote key %x not valid because of robot lifted up.", __FUNCTION__, __LINE__,
					 remote.get());
			speaker.play(VOICE_ERROR_LIFT_UP);
		}
		else if (robot::instance()->isBatteryLow())
		{
			ROS_WARN("%s %d: Battery level low %4dmV(limit in %4dmV)", __FUNCTION__, __LINE__, battery.getVoltage(),
					 (int) BATTERY_READY_TO_CLEAN_VOLTAGE);
			sp_state->init();
			speaker.play(VOICE_BATTERY_LOW);
		}
		else
			ev.key_clean_pressed = true;
	}

	key.resetTriggerStatus();
}

void ModeIdle::rcon(bool state_now, bool state_last)
{
//	ROS_INFO("%s %d: rcon status: %8x.", __FUNCTION__, __LINE__, c_rcon.getStatus());
	if (robot_error.get() == ERROR_CODE_NONE && !robot::instance()->isBatteryLow2())
	{
		auto time_for_now_ = ros::Time::now().toSec();
//	ROS_WARN("%s %d: rcon signal. first: %lf, last: %lf, now: %lf", __FUNCTION__, __LINE__, first_time_seen_charger, last_time_seen_charger, time_for_now);
		if (time_for_now_ - last_time_seen_charger_ > 60.0)
		{
			/*---more than 1 min haven't seen charger, reset first_time_seen_charger---*/
			first_time_seen_charger_ = time_for_now_;
		} else
		{
			/*---received charger signal continuously, check if more than 3 mins---*/
			if (time_for_now_ - first_time_seen_charger_ > RCON_TRIGGER_INTERVAL)
				ev.rcon_status = c_rcon.getAll();
		}
		last_time_seen_charger_ = time_for_now_;
	}
	c_rcon.resetStatus();
}

bool ModeIdle::isFinish()
{
//	ROS_WARN("battery low(%d), battery ready to clean(%d)", robot::instance()->isBatteryLow(), battery.isReadyToClean());
	if (!robot::instance()->isBatteryLow() && !battery.isReadyToClean())
	{
		robot::instance()->setBatterLow(true);
		sp_state->init();
//		ROS_WARN("11111~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~`");
	}
	if (!robot::instance()->isBatteryLow2() && battery.isLow())
	{
		robot::instance()->setBatterLow2(true);
		sp_state->init();
//		ROS_ERROR("2222~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~`");
	}

	// For debug
	/*MutexLock lock(&bind_lock_);
	if(trigger_wifi_rebind_)
	{
		trigger_wifi_rebind_ = false;
		s_wifi.rebind();
	}
	else if(trigger_wifi_smart_link_ && !bumper.getLidarBumperStatus())
	{
		trigger_wifi_smart_link_ = false;
		s_wifi.smartLink();
	}
	else if(trigger_wifi_smart_ap_link_)
	{
		trigger_wifi_smart_ap_link_ = false;
		s_wifi.smartApLink();
	}*/

	return false;
}

bool ModeIdle::readyToClean(bool check_battery, bool check_error)
{
//	ROS_INFO("%s %d: Check battery (%d), check error (%d).", __FUNCTION__, __LINE__, check_battery, check_error);
	if (!check_error && robot_error.get())
	{
		bool force_clear = true;
		robot_error.clear(robot_error.get(), force_clear);
		ROS_WARN("%s %d: Clear the error %x.", __FUNCTION__, __LINE__, robot_error.get());
		sp_state->init();
	}
	if (robot_error.get())
	{
		ROS_WARN("%s %d: Remote key %x not valid because of error %d.", __FUNCTION__, __LINE__, remote.get(), robot_error.get());
		beeper.beepForCommand(INVALID);
		robot_error.alarm();
		return false;
	}
	else if (check_battery && !battery.isReadyToClean())
	{
		ROS_WARN("%s %d: Battery not ready to clean(Not reach %4dmV).", __FUNCTION__,
				 __LINE__, BATTERY_READY_TO_CLEAN_VOLTAGE);
		speaker.play(VOICE_BATTERY_LOW);
		return false;
	}
	else if (cliff.getStatus() == (BLOCK_LEFT | BLOCK_FRONT | BLOCK_RIGHT))
	{
		ROS_WARN("%s %d: Robot lifted up.", __FUNCTION__, __LINE__);
		speaker.play(VOICE_ERROR_LIFT_UP);
		return false;
	}
	else if (robot::instance()->isBatteryLow2())
	{
		ROS_WARN("%s %d: Battery level low %4dmV(limit in %4dmV)", __FUNCTION__, __LINE__, battery.getVoltage(), (int)LOW_BATTERY_STOP_VOLTAGE);
		sp_state->init();
		beeper.beepForCommand(INVALID);
		speaker.play(VOICE_BATTERY_LOW);
		return false;
	}

	return true;
}
