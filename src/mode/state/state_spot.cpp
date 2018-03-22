//
// Created by lsy563193 on 12/4/17.
//


#include <mode.hpp>
#include <water_tank.hpp>

#include "vacuum.h"
#include "brush.h"
#include "key_led.h"
#include "speaker.h"

void StateSpot::init() {
	if (!water_tank.isEquipped())
		vacuum.setTmpMode(Vac_Max);
	brush.fullOperate();
	key_led.setMode(LED_STEADY, LED_GREEN);
	if(getMode()->isNavMode())
		speaker.play(VOICE_CLEANING_SPOT,false);
	ROS_INFO(" IS NAV MODE %d ",getMode()->isNavMode());
}
