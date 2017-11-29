#include <stdint.h>
#include <unistd.h>
#include <ros/ros.h>
#include <cliff.h>
#include <pp.h>
#include <clean_timer.h>
#include <remote.h>

#include "sleep.h"
#include "key.h"
#include "movement.h"
#include "wav.h"
#include "event_manager.h"
#include "clean_mode.h"
#include "beep.h"
#include "error.h"

uint8_t sleep_plan_reject_reason = 0; // 1 for error exist, 2 for robot lifted up, 3 for battery low, 4 for key clean clear the error.
bool sleep_rcon_triggered = false;
static Sleep_EventHandle eh;

uint8_t g_sleep_mode_flag = 0;
uint8_t get_sleep_mode_flag()
{
	return g_sleep_mode_flag;
}

void set_sleep_mode_flag()
{
	g_sleep_mode_flag = 1;
}

void reset_sleep_mode_flag()
{
	g_sleep_mode_flag = 0;
}

/*----------------------------------------------------------------Sleep mode_---------------------------*/
void sleep_mode(void)
{
	time_t check_battery_time = time(NULL);
	bool eh_status_now=false, eh_status_last=false;
	sleep_plan_reject_reason = 0;
	sleep_rcon_triggered = false;

	beeper.play(1, 80, 0, 1);
	usleep(100000);
	beeper.play(2, 80, 0, 1);
	usleep(100000);
	beeper.play(3, 80, 0, 1);
	usleep(100000);
	beeper.play(4, 80, 0, 1);
	usleep(100000);
	led.set_mode(LED_STEADY, LED_OFF);

	cs_disable_motors();
	serial.setCleanMode(POWER_DOWN);
	usleep(20000);
	ROS_INFO("%s %d,power status %u ",__FUNCTION__,__LINE__, serial.getCleanMode());

	key.reset();
	c_rcon.resetStatus();
	remote.reset();
	key.reset();
	timer.set_status(0);

	event_manager_reset_status();
	sleep_register_events();
	while(ros::ok())
	{
		usleep(20000);

		if (event_manager_check_event(&eh_status_now, &eh_status_last) == 1) {
			continue;
		}

		switch (sleep_plan_reject_reason)
		{
			case 1:
				error.alarm();
				wav.play(WAV_CANCEL_APPOINTMENT);
				sleep_plan_reject_reason = 0;
				break;
			case 2:
				wav.play(WAV_ERROR_LIFT_UP);
				wav.play(WAV_CANCEL_APPOINTMENT);
				sleep_plan_reject_reason = 0;
				break;
			case 3:
				wav.play(WAV_BATTERY_LOW);
				wav.play(WAV_CANCEL_APPOINTMENT);
				sleep_plan_reject_reason = 0;
				break;
		}
		/*--- Wake up events---*/
		if(ev.key_clean_pressed)
			cm_set(Clean_Mode_Idle);
		else if(ev.charge_detect)
			cm_set(Clean_Mode_Charging);
		else if(g_plan_activated)
			cm_set(Clean_Mode_Navigation);
		else if(sleep_rcon_triggered)
			cm_set(Clean_Mode_Go_Charger);

		if (cm_get() != Clean_Mode_Sleep)
			break;

		if (time(NULL) - check_battery_time > 30)
		{
			// Check the battery for every 30s. If battery below 12.5v, power of core board will be cut off.
			reset_sleep_mode_flag();
			ROS_WARN("Wake up robotbase to check if battery too low(<12.5v).");
			// Wait for 40ms to make sure base board has finish checking.
			usleep(40000);
			check_battery_time = time(NULL);
		}
		else if(!get_sleep_mode_flag())
			set_sleep_mode_flag();
	}

	sleep_unregister_events();

	beeper.play(4, 80, 0, 1);
	usleep(100000);
	beeper.play(3, 80, 0, 1);
	usleep(100000);
	beeper.play(2, 80, 0, 1);
	usleep(100000);
	beeper.play(1, 80, 4, 1);

	// Wait 1.5s to avoid gyro can't open if switch to navigation mode_ too soon after waking up.
	usleep(1500000);

	c_rcon.resetStatus();
	remote.reset();
	key.reset();
	timer.set_status(0);
}

void sleep_register_events(void)
{
	event_manager_register_handler(&eh);
	event_manager_set_enable(true);
}

void sleep_unregister_events(void)
{
	event_manager_set_enable(false);
}

void Sleep_EventHandle::rcon(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Waked up by rcon signal.", __FUNCTION__, __LINE__);
	if (error.get() == ERROR_CODE_NONE)
	{
		serial.setCleanMode(Clean_Mode_Go_Charger);
		sleep_rcon_triggered = true;
	}
	reset_sleep_mode_flag();
}

void Sleep_EventHandle::remote_clean(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Waked up by remote key clean.", __FUNCTION__, __LINE__);
	serial.setCleanMode(Clean_Mode_Idle);
	ev.key_clean_pressed = true;
	reset_sleep_mode_flag();
}

void Sleep_EventHandle::remote_plan(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Waked up by plan.", __FUNCTION__, __LINE__);
	if (timer.get_status() == 3)
	{
		if (error.get() != ERROR_CODE_NONE)
		{
			ROS_WARN("%s %d: Error exists, so cancel the appointment.", __FUNCTION__, __LINE__);
			sleep_plan_reject_reason = 1;
		}
		else if(cliff.get_status() & (BLOCK_LEFT|BLOCK_FRONT|BLOCK_RIGHT))
		{
			ROS_WARN("%s %d: Plan not activated not valid because of robot lifted up.", __FUNCTION__, __LINE__);
			sleep_plan_reject_reason = 2;
		}
		else
		{
			serial.setCleanMode(Clean_Mode_Navigation);
			g_plan_activated = true;
		}
	}
	reset_sleep_mode_flag();
	timer.set_status(0);
}

void Sleep_EventHandle::key_clean(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Waked up by key clean.", __FUNCTION__, __LINE__);
	ev.key_clean_pressed = true;
	serial.setCleanMode(Clean_Mode_Idle);
	reset_sleep_mode_flag();
	usleep(20000);

	beeper.play_for_command(VALID);

	while (key.get_press() & KEY_CLEAN)
		usleep(20000);

	ROS_WARN("%s %d: Key clean is released.", __FUNCTION__, __LINE__);
	key.reset();
}

void Sleep_EventHandle::charge_detect(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Detect charge!", __FUNCTION__, __LINE__);
	serial.setCleanMode(Clean_Mode_Charging);
	ev.charge_detect = true;
	reset_sleep_mode_flag();
}
