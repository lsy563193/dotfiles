//
// Created by austin on 17-12-3.
//

#include <pp.h>
#include "dev.h"
#include "action_new/action_open_gyro_new.hpp"

OpenGyroAction::OpenGyroAction(){

}

bool OpenGyroAction::isFinishAndUpdateAction()
{
	return gyro.isOn();
}

void OpenGyroAction::action()
{
	wheel.setPidTargetSpeed(0, 0);
	gyro.waitForOn();
}

NavOpenGyroAction::NavOpenGyroAction()
{
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
	key.resetTriggerStatus();

	// Playing wavs.
	// Can't register until the status has been checked. because if register too early, the handler may affect the pause status, so it will play the wrong wav.
	cm_register_events();
	if (g_resume_cleaning) {
		ROS_WARN("Restore from low battery pause");
		speaker.play(SPEAKER_CLEANING_CONTINUE);
	}
	else if (cs_is_paused()) {
		ROS_WARN("Restore from manual pause");
		speaker.play(SPEAKER_CLEANING_CONTINUE);
		if (cs.is_going_home()) {
			speaker.play(SPEAKER_BACK_TO_CHARGER);
		}
	}
	else if (g_plan_activated == true) {
		g_plan_activated = false;
	}
	else {
		speaker.play(SPEAKER_CLEANING_START);
	}
}
Action* NavOpenGyroAction::getNextAction()
{
	Action* new_action = new NavOpenGyroAction();
//	if (charger.isOnStub())
//		new_action = new NavBackFromChargerAction();
//	else
//		new_action = new NavOpenLaserAction();

	return new_action;
}

