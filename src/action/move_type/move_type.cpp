//
// Created by lsy563193 on 12/4/17.
//

#include <mathematics.h>
#include "pp.h"
#include "arch.hpp"

boost::shared_ptr<IAction> IMoveType::sp_movement_ = nullptr;
boost::shared_ptr<ACleanMode> IMoveType::sp_cm_ = nullptr;
int IMoveType::movement_i_ = mm_null;

//bool IMoveType::isFinish(int& action_i) {
//	PP_INFO();
//	action_i = setNextAction();
//	if (action_i == Mode::ac_null)
//		return true;
//	PP_INFO();
//	return false;
//}

IMoveType::IMoveType() {

	g_wall_distance = WALL_DISTANCE_HIGH_LIMIT;
	bumper_turn_factor = 0.85;
	g_bumper_cnt = g_cliff_cnt = 0;
	g_slip_cnt = 0;
	g_slip_backward = false;
	c_rcon.resetStatus();
	robot::instance()->obsAdjustCount(20);
}

void IMoveType::resetTriggeredValue() {
{
	ev.lidar_triggered = 0;
	ev.rcon_triggered = 0;
	ev.bumper_triggered = 0;
	ev.obs_triggered = 0;
	ev.cliff_triggered = 0;
	ev.tilt_triggered = 0;
}
}

bool IMoveType::isFinish() {

}

void IMoveType::run() {
//	PP_INFO();
	sp_movement_->run();
}

