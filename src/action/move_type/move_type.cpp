//
// Created by lsy563193 on 12/4/17.
//

#include <mathematics.h>
#include <event_manager.h>
#include "dev.h"
#include "robot.hpp"

#include <move_type.hpp>
#include <state.hpp>
#include <mode.hpp>

boost::shared_ptr<IMovement> IMoveType::sp_movement_ = nullptr;
Mode* IMoveType::sp_mode_ = nullptr;
int IMoveType::movement_i_ = mm_null;

IMoveType::IMoveType() {
//	resetTriggeredValue();
	last_ = start_point_ = getPosition();
	c_rcon.resetStatus();
	robot::instance()->obsAdjustCount(20);
}

bool IMoveType::shouldMoveBack()
{
	// Robot should move back for these cases.
	ev.bumper_triggered = bumper.getStatus();
	ev.cliff_triggered = cliff.getStatus();
	ev.tilt_triggered = gyro.getTiltCheckingStatus();
	ev.robot_slip = lidar.isRobotSlip();

	if (ev.bumper_triggered || ev.cliff_triggered || ev.tilt_triggered || ev.robot_slip)
	{
		ROS_WARN("%s, %d,ev.bumper_triggered(%d) ev.cliff_triggered(%d) ev.tilt_triggered(%d) ev.robot_slip(%d)."
				, __FUNCTION__, __LINE__, ev.bumper_triggered, ev.cliff_triggered, ev.tilt_triggered, ev.robot_slip);
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
		turn_angle = obsTurnAngle();
		ROS_INFO("%s, %d: ev.obs_triggered(%d), turn for (%d).", __FUNCTION__, __LINE__, ev.obs_triggered, turn_angle);
		return true;
	}

	return false;*/
}

bool IMoveType::isLidarStop()
{
//	PP_INFO();
	ev.lidar_triggered = lidar.getObstacleDistance(0,0.056) < 0.04 ? BLOCK_FRONT : 0;
	if (ev.lidar_triggered)
	{
		// Temporary use OBS to get angle.
		ROS_WARN("%s, %d: ev.lidar_triggered(%d).", __FUNCTION__, __LINE__, ev.lidar_triggered);
		return true;
	}

	return false;
}

bool IMoveType::shouldTurn()
{
//	ev.lidar_triggered = lidar_get_status();
//	if (ev.lidar_triggered)
//	{
//		// Temporary use bumper as lidar triggered.
//		ev.bumper_triggered = ev.lidar_triggered;
//		g_turn_angle = bumperTurnAngle();
//		ev.bumper_triggered = 0;
//		ROS_WARN("%s %d: Lidar triggered, turn_angle: %d.", __FUNCTION__, __LINE__, g_turn_angle);
//		return true;
//	}

//	ev.obs_triggered = (obs.getFront() > obs.getFrontTrigValue() + 1700);
//	if (ev.obs_triggered)
//	{
////		ev.obs_triggered = BLOCK_FRONT;
////		g_turn_angle = obsTurnAngle();
//		ROS_WARN("%s %d: OBS triggered.", __FUNCTION__, __LINE__);
//		return true;
//	}

	return false;
}

bool IMoveType::RconTrigger()
{
	ev.rcon_triggered = c_rcon.getWFRcon();
	if (ev.rcon_triggered) {
		ROS_WARN("%s, %d: ev.rcon_triggered(%d).", __FUNCTION__, __LINE__, ev.lidar_triggered);
		return true;
	}
	return false;
}

void IMoveType::resetTriggeredValue()
{
	ev.lidar_triggered = 0;
//	ev.rcon_triggered = 0;
	ev.bumper_triggered = 0;
	ev.obs_triggered = 0;
	ev.cliff_triggered = 0;
	ev.tilt_triggered = 0;
	ev.robot_slip = false;
}

bool IMoveType::isFinish()
{
	updatePosition();
	auto curr = getPosition();
	auto p_cm = dynamic_cast<ACleanMode*> (sp_mode_);
	if (!curr.isCellAndAngleEqual(last_))
	{
		last_ = curr;
		if(p_cm->moveTypeNewCellIsFinish(this))
			return true;
	}
	if(p_cm->moveTypeRealTimeIsFinish(this))
		return true;

	return false;
}

void IMoveType::run()
{
	sp_movement_->run();
}

int IMoveType::countRconTriggered(uint32_t rcon_value)
{
	if(rcon_value == 0)
		return 0;

	int MAX_CNT = 1;
	if ( rcon_value& RconL_HomeT)
		rcon_cnt[left]++;
	if ( rcon_value& RconFL_HomeT)
		rcon_cnt[fl1]++;
	if ( rcon_value& RconFL2_HomeT)
		rcon_cnt[fl2]++;
	if ( rcon_value& RconFR2_HomeT)
		rcon_cnt[fr2]++;
	if ( rcon_value& RconFR_HomeT)
		rcon_cnt[fr1]++;
	if ( rcon_value& RconR_HomeT)
		rcon_cnt[right]++;
	auto ret = 0;
	for (int i = 0; i < 6; i++)
		if (rcon_cnt[i] > MAX_CNT) {
			rcon_cnt[left] = rcon_cnt[fl1] = rcon_cnt[fl2] = rcon_cnt[fr2] = rcon_cnt[fr1] = rcon_cnt[right] = 0;
			ret = i + 1;
			break;
		}
	return ret;
}

bool IMoveType::isRconStop()
{
	ev.rcon_triggered = countRconTriggered(c_rcon.getForwardTop());

	bool ret = false;
	if(ev.rcon_triggered)
	{
		ROS_WARN("%s %d: Rcon triggered and stop.", __FUNCTION__, __LINE__);
		ret = true;
	}

	return ret;
}

bool IMoveType::isBlockCleared(GridMap &map, Points &passed_path)
{
	if (!passed_path.empty())
	{
//		ROS_INFO("%s %d: passed_path.back(%d %d)", __FUNCTION__, __LINE__, passed_path.back().x, passed_path.back().y);
		return !map.isBlockAccessible(passed_path.back().toCell().x, passed_path.back().toCell().y);
	}

	return false;
}

