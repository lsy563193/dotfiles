//
// Created by lsy563193 on 12/4/17.
//

#include <mode.hpp>
#include <water_tank.hpp>
#include "key_led.h"
#include "vacuum.h"
#include "brush.h"

void StateClean::init() {

	key_led.setMode(LED_STEADY, LED_GREEN);
	water_tank.checkEquipment(false) ? water_tank.open(WaterTank::tank_pump) : vacuum.setCleanState();
	brush.normalOperate();
}

//bool StateClean::isFinish() {
//	return path_next_nav(sp_action_->, path);
//}

//State *StateClean::setNextState() {
////	auto path = sp_mode_->generatePath(nav_map,nav_map.getCurrCell(),g_old_dir);
////	if(path.empty()) {
////		ROS_INFO("%s%d:", __FUNCTION__, __LINE__);
////		if (sp_mode_->checkTrapped(nav_map, nav_map.getCurrCell())) {
////			return new StateTrapped;
////		}
////		else {
////			return new StateGoHomePoint;
////		}
////	}
//	return this;
//
//}

