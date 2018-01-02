//
// Created by lsy563193 on 12/4/17.
//

#include <mathematics.h>
#include "pp.h"
#include "arch.hpp"
boost::shared_ptr<IMovement> IMoveType::sp_movement_ = nullptr;
Mode* IMoveType::sp_mode_ = nullptr;
int IMoveType::movement_i_ = mm_null;

bool IMoveType::shouldMoveBack()
{
	// Robot should move back for these cases.
	ev.bumper_triggered = bumper.get_status();
	ev.cliff_triggered = cliff.getStatus();
	ev.tilt_triggered = gyro.getTiltCheckingStatus();

	if (ev.bumper_triggered || ev.cliff_triggered || ev.tilt_triggered || g_robot_slip)
	{
		ROS_WARN("%s, %d,ev.bumper_triggered(%d) ev.cliff_triggered(%d) ev.tilt_triggered(%d) g_robot_slip(%d)."
				, __FUNCTION__, __LINE__,ev.bumper_triggered,ev.cliff_triggered,ev.tilt_triggered,g_robot_slip);
		return true;
	}

	return false;
}

bool IMoveType::isOBSStop()
{
	// Now OBS sensor is just for slowing down.
//	PP_INFO();
	return false;
/*
	ev.obs_triggered = obs.getStatus(200, 1700, 200);
	if(ev.obs_triggered)
	{
		turn_angle = obs_turn_angle();
		ROS_INFO("%s, %d: ev.obs_triggered(%d), turn for (%d).", __FUNCTION__, __LINE__, ev.obs_triggered, turn_angle);
		return true;
	}

	return false;*/
}

bool IMoveType::isLidarStop()
{
//	PP_INFO();
	ev.lidar_triggered = lidar_get_status();
	if (ev.lidar_triggered)
	{
		// Temporary use OBS to get angle.
//		ev.obs_triggered = ev.lidar_triggered;
//		g_turn_angle = obs_turn_angle();
//		ROS_WARN("%s, %d: ev.lidar_triggered(%d), turn for (%d).", __FUNCTION__, __LINE__, ev.lidar_triggered, g_turn_angle);
		return true;
	}

	return false;
}

bool IMoveType::shouldTurn()
{
	ev.lidar_triggered = lidar_get_status();
//	if (ev.lidar_triggered)
//	{
//		// Temporary use bumper as lidar triggered.
//		ev.bumper_triggered = ev.lidar_triggered;
//		g_turn_angle = bumper_turn_angle();
//		ev.bumper_triggered = 0;
//		ROS_WARN("%s %d: Lidar triggered, turn_angle: %d.", __FUNCTION__, __LINE__, g_turn_angle);
//		return true;
//	}

	ev.obs_triggered = (obs.getFront() > obs.getFrontTrigValue() + 1700);
	if (ev.obs_triggered)
	{
//		ev.obs_triggered = BLOCK_FRONT;
//		g_turn_angle = obs_turn_angle();
		ROS_WARN("%s %d: OBS triggered.", __FUNCTION__, __LINE__);
		return true;
	}

	return false;
}

IMoveType::IMoveType() {
	start_point_ = getPosition();
	g_slip_cnt = 0;
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
	return sp_mode_->isExceptionTriggered();
}

void IMoveType::run() {
//	PP_INFO();
	sp_movement_->run();
}
