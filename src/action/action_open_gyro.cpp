//
// Created by lsy563193 on 11/29/17.
//

#include "pp.h"
#include "mode/mode.hpp"

//IAction::IAction(Mode* p_mode) {
//	sp_mode_ = p_mode;
//}

ActionOpenGyro::ActionOpenGyro(Mode* p_mode){
	IAction::sp_mode_ = p_mode;
	// Set for LEDs.
		if (ev.remote_home || g_go_home_by_remote)
			led.set_mode(LED_FLASH, LED_ORANGE, 1000);
		else
			led.set_mode(LED_FLASH, LED_GREEN, 1000);

		// Operate on gyro.
		gyro.setOff();
		usleep(30000);
		gyro.reOpen();

		// Reset for keys.
		key.resetPressStatus();

		// Playing speakers.
		// Can't register until the status has been checked. because if register too early, the handler may affect the pause status, so it will play the wrong speaker.
		if (g_resume_cleaning)
		{
			speaker.play(SPEAKER_CLEANING_CONTINUE);
		}
		else if (cs_is_paused())
		{
			speaker.play(SPEAKER_CLEANING_CONTINUE);
//			if (cs.is_going_home())
//			{
//				speaker.play(SPEAKER_BACK_TO_CHARGER);
//			}
		}
		else if(g_plan_activated)
		{
			g_plan_activated = false;
		}
		else{
			speaker.play(SPEAKER_CLEANING_START);
		}
}

bool ActionOpenGyro::isFinish(){
	ROS_INFO("%s,%d", __FUNCTION__, __LINE__);
	return (gyro.isOn());
}

void ActionOpenGyro::run() {
	ROS_INFO("%s,%d", __FUNCTION__, __LINE__);
	wheel.setPidTargetSpeed(0, 0);
	gyro.waitForOn();
}

//ActionOpenGyroNav::ActionOpenGyroNav() {

//}
