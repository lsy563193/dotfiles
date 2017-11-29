//
// Created by root on 11/29/17.
//

#include "pp.h"

EventOpenGyro::EventOpenGyro() {
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
	key.reset();

	// Playing wavs.
	// Can't register until the status has been checked. because if register too early, the handler may affect the pause status, so it will play the wrong wav.
	cm_register_events();
	if (g_resume_cleaning) {
		ROS_WARN("Restore from low battery pause");
		wav.play(WAV_CLEANING_CONTINUE);
	}
	else if (cs_is_paused()) {
		ROS_WARN("Restore from manual pause");
		wav.play(WAV_CLEANING_CONTINUE);
		if (cs.is_going_home()) {
			wav.play(WAV_BACK_TO_CHARGER);
		}
	}
	else if (g_plan_activated == true) {
		g_plan_activated = false;
	}
	else {
		wav.play(WAV_CLEANING_START);
	}
}

bool EventOpenGyro::isStop() {
	return gyro.isOn();
}

bool EventOpenGyro::setNext() {
	if (charger.is_on_stub()) {
		cs.setNext(CS_BACK_FROM_CHARGER);
		charger_pose.setX(odom.getX());
		charger_pose.setY(odom.getY());
	}
	else
		cs.setNext(CS_OPEN_LASER);
}

void EventOpenGyro::doSomething() {
	wheel.setPidTargetSpeed(0, 0);
	gyro.waitForOn();

}

