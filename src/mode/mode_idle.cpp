#include <infrared_display.hpp>
#include "event_manager.h"
#include "dev.h"
#include "error.h"
#include "mode.hpp"
#include "wifi/wifi.h"
#include "appointment.h"

#define RCON_TRIGGER_INTERVAL 180

ModeIdle::ModeIdle():
	bind_lock_(PTHREAD_MUTEX_INITIALIZER)
{
	ROS_INFO("%s %d: Entering Idle mode\n=========================" , __FUNCTION__, __LINE__);
	register_events();
	serial.setWorkMode(IDLE_MODE);
	setNextMode(md_idle);
	sp_action_.reset(new ActionIdle);
	action_i_ = ac_idle;

	key.resetTriggerStatus();
	c_rcon.resetStatus();
	remote.reset();
	appmt_obj.resetPlanStatus();
	event_manager_reset_status();

	ROS_INFO("%s %d: Current battery voltage \033[32m%5.2f V\033[0m.", __FUNCTION__, __LINE__, (float)battery.getVoltage()/100.0);
	/*---reset values for rcon handle---*/
	// todo: first_time_seen_charger_ does not mean as words in reality. It is just the time that enter this mode.
//	// todo:debug
//	infrared_display.displayErrorMsg(9, 1234, 101);
	sp_state = st_pause.get() ;
	sp_state->init();
	mode_i_ = md_idle;
}

ModeIdle::~ModeIdle()
{
	event_manager_set_enable(false);
	sp_action_.reset();
	ROS_INFO("%s %d: Exit idle mode.", __FUNCTION__, __LINE__);
}

bool ModeIdle::isExit()
{
	if(ev.key_clean_pressed || plan_activated_status_)
	{
		if (plan_activated_status_)
		{
			if (error.get() != ERROR_CODE_NONE)
			{
				if (error.clear(error.get()))
				{
					ROS_WARN("%s %d: Clear the error %x.", __FUNCTION__, __LINE__, error.get());
					sp_state->init();
//					speaker.play(VOICE_CLEAR_ERROR_UNOFFICIAL, false);
				} else
				{
//					speaker.play(VOICE_CANCEL_APPOINTMENT_UNOFFICIAL, false);
					// Reset action idle for playing the error alarm.
					sp_action_.reset(new ActionIdle);
				}
			}

			if (error.get() != ERROR_CODE_NONE)
				ROS_INFO("%s %d: Error exists, so cancel the appointment.", __FUNCTION__, __LINE__);
			else if (cliff.getStatus() & (BLOCK_LEFT | BLOCK_FRONT | BLOCK_RIGHT))
			{
				ROS_WARN("%s %d: Plan not activated not valid because of robot lifted up.", __FUNCTION__, __LINE__);
				speaker.play(VOICE_ERROR_LIFT_UP);
			} else if (!battery.isReadyToClean())
			{
				ROS_WARN("%s %d: Plan not activated not valid because of battery not ready to clean.", __FUNCTION__, __LINE__);
				speaker.play(VOICE_BATTERY_LOW);
			} else{
				ROS_WARN("%s %d: Idle mode receives plan, change to navigation mode.", __FUNCTION__, __LINE__);
				setNextMode(cm_navigation);
				ACleanMode::plan_activation_ = true;
				return true;
			}
			plan_activated_status_ = false;
		}
		else
		{
			ROS_WARN("%s %d: Idle mode receives remote clean or clean key, change to navigation mode.", __FUNCTION__, __LINE__);
			setNextMode(cm_navigation);
			return true;
		}
	}

	if(ev.remote_follow_wall)
	{
		ROS_WARN("%s %d: Idle mode receives remote follow wall key, change to follow wall mode.", __FUNCTION__, __LINE__);
		setNextMode(cm_wall_follow);
		return true;
	}

	if(ev.remote_spot)
	{
		ROS_WARN("%s %d: Idle mode receives remote spot key, change to spot mode.", __FUNCTION__, __LINE__);
		setNextMode(cm_spot);
		return true;
	}

	if(ev.remote_home)
	{
		ROS_WARN("%s %d: Idle mode receives remote home key, change to exploration mode.", __FUNCTION__, __LINE__);
		setNextMode(cm_exploration);
		return true;
	}
	if (ev.key_long_pressed)
	{
		ROS_WARN("%s %d: Idle mode detects long press key, change to sleep mode.", __FUNCTION__, __LINE__);
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
		ROS_WARN("%s %d: Idle mode receives remote direction key, change to remote mode.", __FUNCTION__, __LINE__);
		setNextMode(md_remote);
		return true;
	}

	if (ev.rcon_status)
	{
		ROS_WARN("%s %d: Idle mode receives rcon for over %ds, change to go to charger mode.", __FUNCTION__, __LINE__, RCON_TRIGGER_INTERVAL);
		setNextMode(md_go_to_charger);
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

void ModeIdle::register_events()
{
	event_manager_register_handler(this);
	event_manager_set_enable(true);
}

void ModeIdle::remoteKeyHandler(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Remote key %x has been pressed.", __FUNCTION__, __LINE__, remote.get());

	if (error.get())
	{
		if (remote.isKeyTrigger(REMOTE_CLEAN))
		{
			bool force_clear = true;
			if (error.clear(error.get(), force_clear))
			{
				ROS_WARN("%s %d: Clear the error %x.", __FUNCTION__, __LINE__, error.get());
				beeper.beepForCommand(VALID);
				sp_state->init();
//				speaker.play(VOICE_CLEAR_ERROR_UNOFFICIAL);
			}
			else
			{
				beeper.beepForCommand(INVALID);
				error.alarm();
			}
		}
		else
		{
			ROS_WARN("%s %d: Remote key %x not valid because of error %d.", __FUNCTION__, __LINE__, remote.get(), error.get());
			error.alarm();
			beeper.beepForCommand(INVALID);
		}
	}
	else if (cliff.getStatus() == BLOCK_ALL)
	{
		ROS_WARN("%s %d: Remote key %x not valid because of robot lifted up.", __FUNCTION__, __LINE__, remote.get());
		beeper.beepForCommand(INVALID);
		speaker.play(VOICE_ERROR_LIFT_UP);
	}
	else if (robot::instance()->isBatteryLow2())
	{
		ROS_WARN("%s %d: Battery level low %4dmV(limit in %4dmV)", __FUNCTION__, __LINE__, battery.getVoltage(), (int)LOW_BATTERY_STOP_VOLTAGE);
        sp_state->init();
		beeper.beepForCommand(INVALID);
		speaker.play(VOICE_BATTERY_LOW);
	}
    else if((!remote.isKeyTrigger(REMOTE_FORWARD) && !remote.isKeyTrigger(REMOTE_LEFT)
			  && !remote.isKeyTrigger(REMOTE_RIGHT) && !remote.isKeyTrigger(REMOTE_HOME))
			  && robot::instance()->isBatteryLow())
	{
		ROS_WARN("%s %d: Battery level low %4dmV(limit in %4dmV)", __FUNCTION__, __LINE__, battery.getVoltage(), (int)BATTERY_READY_TO_CLEAN_VOLTAGE);
        sp_state->init();
		beeper.beepForCommand(INVALID);
		speaker.play(VOICE_BATTERY_LOW);
	}
	else
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
}

void ModeIdle::remoteMax(bool state_now, bool state_last)
{
	PP_INFO();
	if(water_tank.checkEquipment(true)){
		beeper.beepForCommand(INVALID);
	}
	else{
		beeper.beepForCommand(VALID);
		vacuum.isMaxInClean(!vacuum.isMaxInClean());
		speaker.play(vacuum.isMaxInClean() ? VOICE_VACCUM_MAX : VOICE_CLEANING_NAVIGATION);
	}
	// Reset the start timer for action.
	sp_action_.reset();
	sp_action_.reset(new ActionIdle);
	remote.reset();
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

}

void ModeIdle::remotePlan(bool state_now, bool state_last)
{
	if (appmt_obj.getPlanStatus() > 2)
	{
		appmt_obj.resetPlanStatus();
		appmt_obj.timesUp();
		INFO_YELLOW("Plan activated.");
		// Sleep for 50ms cause the status 3 will be sent for 3 times.
		usleep(50000);
		plan_activated_status_ = true;
	} else
		EventHandle::remotePlan(state_now, state_last);
}

void ModeIdle::chargeDetect(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Detect charge!", __FUNCTION__, __LINE__);
	ev.charge_detect = charger.getChargeStatus();
}

void ModeIdle::keyClean(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: key clean.", __FUNCTION__, __LINE__);
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
		s_wifi.smartApLink();
		sp_action_.reset();
		sp_action_.reset(new ActionIdle);
	}
	else if (long_press)
		ev.key_long_pressed = true;
	else
	{
		if (error.get())
		{
			bool force_clear = true;
			if (error.clear(error.get(), force_clear))
			{
				ROS_WARN("%s %d: Clear the error %x.", __FUNCTION__, __LINE__, error.get());
                sp_state->init();
//				speaker.play(VOICE_CLEAR_ERROR_UNOFFICIAL);
			}
			else
				error.alarm();
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
	if (error.get() == ERROR_CODE_NONE)
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
