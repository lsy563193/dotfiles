//
// Created by austin on 17-12-8.
//

#include <movement.hpp>
#include <move_type.hpp>
#include <event_manager.h>
#include <mode.hpp>
#include "dev.h"

MovementStay::MovementStay(double stay_time_sec)
{
	ROS_INFO("%s %d: Start movement stay.", __FUNCTION__, __LINE__);
	wheel.stop();
	start_timer_ = ros::Time::now().toSec();
	timeout_interval_ = stay_time_sec;
	bumper_status_in_stay_ = 0;
	cliff_status_in_stay_ = 0;
	tilt_status_in_stay_ = 0;
}

MovementStay::~MovementStay()
{
	robot::instance()->unlockScanCtrl();
	ROS_INFO("%s %d: End movement stay.", __FUNCTION__, __LINE__);
}


bool MovementStay::isFinish()
{
	bumper_status_in_stay_ = bumper.getStatus();
//	ROS_ERROR("%s,%d, movementStay",__FUNCTION__, __LINE__);
	if(sp_mt_->sp_mode_->action_i_ != sp_mt_->sp_mode_->ac_linear)
	{
		cliff_status_in_stay_ = cliff.getStatus();
		tilt_status_in_stay_ = gyro.getTiltCheckingStatus();
	}
	ev.bumper_triggered = static_cast<uint8_t>(bumper_status_in_stay_ ? bumper_status_in_stay_ : ev.bumper_triggered);
	ev.cliff_triggered = static_cast<uint8_t>(cliff_status_in_stay_ ? cliff_status_in_stay_: ev.cliff_triggered);
	ev.tilt_triggered = static_cast<uint8_t>(tilt_status_in_stay_ ? tilt_status_in_stay_ : ev.tilt_triggered);
	robot::instance()->lockScanCtrl();
//	ROS_ERROR("%s,%d, movementStay",__FUNCTION__, __LINE__);
	robot::instance()->pubScanCtrl(true, true);
	return isTimeUp() || bumper_status_in_stay_ || cliff_status_in_stay_ || tilt_status_in_stay_;
}

void MovementStay::adjustSpeed(int32_t &left_speed, int32_t &right_speed)
{
	left_speed = right_speed = 0;
//	ROS_WARN("staying...");
}

MovementStayRemote::MovementStayRemote(double stay_time_sec) : MovementStay(stay_time_sec)
{
	ROS_INFO("%s %d: Start movement stay remote.", __FUNCTION__, __LINE__);
}

bool MovementStayRemote::isFinish()
{
//	ev.bumper_triggered = bumper.getStatus();
	ev.cliff_triggered = cliff.getStatus();
//
	return ev.remote_direction_forward ||
		   ev.remote_direction_left ||
		   ev.remote_direction_right ||
//		   ev.bumper_triggered ||
		   ev.cliff_triggered ||
					isTimeUp();
}
