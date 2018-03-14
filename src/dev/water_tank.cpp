//
// Created by austin on 18-3-13.
//

#include "water_tank.hpp"
#include "serial.h"

WaterTank water_tank;

bool WaterTank::checkEquipment()
{
	serial.setSendData(CTL_WATER_TANK, 0x0A);
	usleep(200000);
	if (getStatus())
		is_equipped_ = true;

//	printf("watertank%d\n", is_equipped_);
	serial.setSendData(CTL_WATER_TANK, 0x00);

	return is_equipped_;
}
