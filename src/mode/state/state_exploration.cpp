//
// Created by lsy563193 on 12/4/17.
//

#include <mode.hpp>
#include <brush.h>
#include <water_tank.hpp>

#include "key_led.h"

void StateExploration::init() {
	key_led.setMode(LED_STEADY, LED_ORANGE);
	brush.slowOperate();
	water_tank.checkEquipment(false) ? water_tank.open(WaterTank::water_tank) : vacuum.bldcSpeed(Vac_Speed_NormalL);
}


