//
//	Created by lsy on 17-12-20.
//

#include <event_manager.h>
#include <dev.h>
#include "mode.hpp"

ModeGoToCharger::ModeGoToCharger()
{
	ROS_INFO("%s, %d: Entering go to charger mode\n=========================", __FUNCTION__, __LINE__);
	event_manager_register_handler(this);
	event_manager_reset_status();
	event_manager_set_enable(true);

	serial.setMainBoardMode(WORK_MODE);
	speaker.play(VOICE_BACK_TO_CHARGER, false);
	key_led.setMode(LED_STEADY, LED_ORANGE);
	sp_action_.reset(new ActionOpenGyro);
	action_i_ = ac_open_gyro;
}

ModeGoToCharger::~ModeGoToCharger()
{
	event_manager_set_enable(false);
	wheel.stop();
	brush.stop();
	vacuum.stop();
}

bool ModeGoToCharger::isExit()
{
	if(ev.cliff_all_triggered || ev.key_clean_pressed)
	{
		ROS_WARN("%s %d:.", __FUNCTION__, __LINE__);
		setNextMode(md_idle);
		ev.cliff_all_triggered = false;
		ev.key_clean_pressed = false;
		return true;
	}
	return false;
}

bool ModeGoToCharger::isFinish()
{
	if ((action_i_ != ac_exception_resume) && isExceptionTriggered())
	{
		ROS_WARN("%s %d: Exception triggered.", __FUNCTION__, __LINE__);
		action_i_ = ac_exception_resume;
		genNextAction();
	}

	if(sp_action_->isFinish())
	{
		PP_INFO();
		action_i_ = getNextAction();
		genNextAction();
		if(sp_action_ == nullptr)
		{
			if(ev.charge_detect)
				setNextMode(md_charge);
			else
				setNextMode(md_idle);
			return true;
		}
	}

	return false;
}

int ModeGoToCharger::getNextAction()
{
	PP_INFO();
	if(action_i_ == ac_open_gyro || (action_i_ == ac_exception_resume && !ev.fatal_quit))
	{
		key_led.setMode(LED_STEADY, LED_ORANGE);
		brush.slowOperate();
		vacuum.setTmpMode(Vac_Normal);
		return ac_go_to_charger;
	}
	return ac_null;
}

void ModeGoToCharger::keyClean(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: key clean.", __FUNCTION__, __LINE__);

	beeper.beepForCommand(VALID);
	wheel.stop();

	// Wait for key released.
	while (key.getPressStatus())
		usleep(20000);
	ev.key_clean_pressed = true;
	ROS_WARN("%s %d: Key clean is released.", __FUNCTION__, __LINE__);

	key.resetTriggerStatus();
}

void ModeGoToCharger::overCurrentWheelLeft(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Left wheel oc.", __FUNCTION__, __LINE__);
	ev.oc_wheel_left = true;
}

void ModeGoToCharger::overCurrentWheelRight(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Right wheel oc.", __FUNCTION__, __LINE__);
	ev.oc_wheel_right = true;
}

void ModeGoToCharger::remoteClean(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: remote clean.", __FUNCTION__, __LINE__);

	beeper.beepForCommand(VALID);
	wheel.stop();
	ev.key_clean_pressed = true;
	remote.reset();
}

void ModeGoToCharger::cliffAll(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Cliff all.", __FUNCTION__, __LINE__);

	ev.cliff_all_triggered = true;
}

void ModeGoToCharger::chargeDetect(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Charge detect!.", __FUNCTION__, __LINE__);
	if (charger.getChargeStatus() >= 1)
	{
		ROS_WARN("%s %d: Set ev.chargeDetect.", __FUNCTION__, __LINE__);
		ev.charge_detect = charger.getChargeStatus();
	}

}

