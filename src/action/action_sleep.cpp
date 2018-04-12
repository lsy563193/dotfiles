//
// Created by austin on 17-12-5.
//

#include "dev.h"
#include <error.h>
#include "action.hpp"

ActionSleep::ActionSleep(bool fake_sleep)
{
	ROS_INFO("%s %d: Start sleep action.", __FUNCTION__, __LINE__);
	// Ensure the previous voice is finished before sleep.
	wifi_led.set(false);
	s_wifi.uploadStatus(0xc8,0x00);
//	s_wifi.sleep();

	gyro.setOff();
	gyro.resetStatus();

	speaker.play(VOICE_NULL, false);
	beeper.beep(1, 80, 0, 1);
	usleep(100000);
	beeper.beep(2, 80, 0, 1);
	usleep(100000);
	beeper.beep(3, 80, 0, 1);
	usleep(100000);
	beeper.beep(4, 80, 0, 1);
	usleep(100000);

	if (fake_sleep)
		ROS_INFO("%s %d: Shhhhhh.... Entering fake sleep ;)", __FUNCTION__, __LINE__);
	else
		lowPowerSleep();
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

void ActionSleep::lowPowerSleep()
{
	if (charger.getChargeStatus())
	{
		ROS_INFO("%s %d: Enter from charge mode.", __FUNCTION__, __LINE__);
		serial.setWorkMode(BATTERY_FULL_SLEEP_MODE);
	} else
		serial.setWorkMode(NORMAL_SLEEP_MODE);
	// Sleep for 30ms to make sure the power byte has been sent.
	usleep(30000);
	ROS_INFO("%s %d: Good night buddy.", __FUNCTION__, __LINE__);
	system("/bin/echo standby > /sys/power/state");
}
