//
// Created by root on 11/29/17.
//

#include "pp.h"

bool EventBackFromCharger::isStop() {
	return (two_points_distance_double(charger_pose.getX(), charger_pose.getY(), odom.getX(), odom.getY()) > 0.5);
}

bool EventBackFromCharger::setNext() {
	cs.setNext(CS_OPEN_LASER);
}

void EventBackFromCharger::doSomething() {
	wheel.setPidTargetSpeed(20, 20);
}

