//
// Created by austin on 17-12-5.
//

#include "dev.h"
#include <error.h>
#include "action.hpp"

#define ENABLE_LOW_POWER_CONSUMPTION 1

ActionSleep::ActionSleep()
{
	ROS_INFO("%s %d: Start sleep action.", __FUNCTION__, __LINE__);
	// Ensure the previous voice is finished before sleep.
	wifi_led.set(false);
	s_wifi.replyRobotStatus(0xc8,0x00);
	s_wifi.sleep();

	speaker.play(VOICE_NULL, false);
	beeper.beep(1, 80, 0, 1);
	usleep(100000);
	beeper.beep(2, 80, 0, 1);
	usleep(100000);
	beeper.beep(3, 80, 0, 1);
	usleep(100000);
	beeper.beep(4, 80, 0, 1);
	usleep(100000);
	key_led.setMode(LED_STEADY, LED_OFF);
	if (charger.getChargeStatus())
	{
		ROS_INFO("%s %d: Finish beeping, enter from charge mode.", __FUNCTION__, __LINE__);
		serial.setWorkMode(BATTERY_FULL_SLEEP_MODE);
	}
	else
	{
		ROS_INFO("%s %d: Finish beeping.", __FUNCTION__, __LINE__);
		serial.setWorkMode(NORMAL_SLEEP_MODE);
	}
#if ENABLE_LOW_POWER_CONSUMPTION
	// Sleep for 30ms to make sure the power byte has been sent.
	usleep(30000);
	system("/bin/echo standby > /sys/power/state");
//	sleep(1);
#else
	usleep(25000);
#endif
}

ActionSleep::~ActionSleep()
{

	beeper.beep(4, 80, 0, 1);
	usleep(100000);
	beeper.beep(3, 80, 0, 1);
	usleep(100000);
	beeper.beep(2, 80, 0, 1);
	usleep(100000);
	beeper.beep(1, 80, 4, 1);

	ROS_INFO("%s %d: End sleep action.", __FUNCTION__, __LINE__);
}

bool ActionSleep::isFinish()
{
	// It will be a long long sleep...
	return false;
}

void ActionSleep::run()
{
	// Just sleep...
}

