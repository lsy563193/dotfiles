//
// Created by austin on 17-12-21.
//

#include <event_manager.h>
#include "arch.hpp"
#include "dev.h"
#include "global.h"

ActionCheckBumper::ActionCheckBumper()
{
	ROS_INFO("%s %d: Starting action check bumper." , __FUNCTION__, __LINE__);
	led.set_mode(LED_STEADY, LED_OFF);
}

bool ActionCheckBumper::isFinish()
{
	if (ev.key_clean_pressed)
	{
		ev.key_clean_pressed = false;
		return true;
	}

	return false;
}

bool ActionCheckBumper::isExit()
{
	return false;
}

void ActionCheckBumper::run()
{
	if (bumper.getLeft() && bumper.getRight())
	{
		beeper.play(1, 50, 0, 1);
		led.set_mode(LED_STEADY, LED_ORANGE);
	}
	else if (bumper.getLeft())
	{
		beeper.play(3, 50, 0, 1);
		led.set_mode(LED_STEADY, LED_GREEN);
	}
	else if (bumper.getRight())
	{
		beeper.play(7, 50, 0, 1);
		led.set_mode(LED_STEADY, LED_RED);
	}
	else
	{
		beeper.play(1, 0, 0, 1);
		led.set_mode(LED_STEADY, LED_OFF);
	}
}
