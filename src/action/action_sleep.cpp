//
// Created by austin on 17-12-5.
//

#include <pp.h>
#include <error.h>
#include "dev.h"
#include "mode/mode.hpp"

ActionSleep::ActionSleep()
{
	beeper.play(1, 80, 0, 1);
	usleep(100000);
	beeper.play(2, 80, 0, 1);
	usleep(100000);
	beeper.play(3, 80, 0, 1);
	usleep(100000);
	beeper.play(4, 80, 0, 1);
	usleep(100000);
	led.set_mode(LED_STEADY, LED_OFF);
}

bool ActionSleep::isFinish()
{
	return false;
}

void ActionSleep::run()
{
	//Just sleep...
}

