//
// Created by austin on 17-12-5.
//

#include "dev.h"
#include <error.h>
#include "action.hpp"

ActionSleep::ActionSleep(bool fake_sleep)
{
	ROS_WARN("%s %d: Start sleep action.", __FUNCTION__, __LINE__);
	// Ensure the previous voice is finished before sleep.
	wifi_led.disable();
	//s_wifi.uploadStatus(0xc8,0x00);
	//s_wifi.sleep();

	gyro.setOff();

	speaker.play(VOICE_NULL, false);
	speaker.play(VOICE_SLEEP_UNOFFICIAL, false);
	speaker.play(VOICE_NULL, false);
	/*beeper.beep(1, 80, 0, 1);
	usleep(100000);
	beeper.beep(2, 80, 0, 1);
	usleep(100000);
	beeper.beep(3, 80, 0, 1);
	usleep(100000);
	beeper.beep(4, 80, 0, 1);
	usleep(100000);*/

	if (fake_sleep)
		ROS_WARN("%s %d: Shhhhhh.... Entering fake sleep ;)", __FUNCTION__, __LINE__);
	else
		lowPowerSleep();
}

ActionSleep::~ActionSleep()
{

	speaker.play(VOICE_WAKE_UP_UNOFFICIAL);
	speaker.play(VOICE_NULL, false);
	/*beeper.beep(4, 80, 0, 1);
	usleep(100000);
	beeper.beep(3, 80, 0, 1);
	usleep(100000);
	beeper.beep(2, 80, 0, 1);
	usleep(100000);
	beeper.beep(1, 80, 4, 1);*/

	wifi_led.enable();
	ROS_WARN("%s %d: End sleep action.", __FUNCTION__, __LINE__);
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
	system("turbo_cpu.sh");
	if (charger.getChargeStatus())
	{
		ROS_WARN("%s %d: Enter from charge mode.", __FUNCTION__, __LINE__);
		serial.setWorkMode(BATTERY_FULL_SLEEP_MODE);
	} else
		serial.setWorkMode(NORMAL_SLEEP_MODE);
	// Sleep for 30ms to make sure the power byte has been sent.
	usleep(30000);
	ROS_WARN("%s %d: Good night buddy.", __FUNCTION__, __LINE__);
	system("/bin/echo standby > /sys/power/state");
}
