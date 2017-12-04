//
// Created by lsy563193 on 11/30/17.
//

#include "pp.h"

bool ActionClean::isReach() {

	if (slam.isMapReady() && robot::instance()->isTfReady())
	{
//		next_ = CS_CLEAN;
//		next_ = 1;
		return true;
	}
	return false;
}

void ActionClean::doSomething() {
//	printf("\n gyro.waitForOn()\n");
	wheel.setPidTargetSpeed(0, 0);
}