//
// Created by austin on 17-12-5.
//

#include "dev.h"
#include "robotbase.h"
#include <error.h>
#include "action.hpp"

#define ENABLE_LOW_POWER_CONSUMPTION 1

ActionSleep::ActionSleep()
{
	ROS_INFO("%s %d: Start sleep action.", __FUNCTION__, __LINE__);
	beeper.play(1, 80, 0, 1);
	usleep(100000);
	beeper.play(2, 80, 0, 1);
	usleep(100000);
	beeper.play(3, 80, 0, 1);
	usleep(100000);
	beeper.play(4, 80, 0, 1);
	usleep(100000);
	led.set_mode(LED_STEADY, LED_OFF);
	serial.setCleanMode(POWER_DOWN);
	ROS_INFO("%s %d: Finish beeping.", __FUNCTION__, __LINE__);
#if ENABLE_LOW_POWER_CONSUMPTION
	system("/bin/echo standby > /sys/power/state");
//	sleep(1);
#else
	usleep(25000);
#endif
}

ActionSleep::~ActionSleep()
{

	beeper.play(4, 80, 0, 1);
	usleep(100000);
	beeper.play(3, 80, 0, 1);
	usleep(100000);
	beeper.play(2, 80, 0, 1);
	usleep(100000);
	beeper.play(1, 80, 4, 1);

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

