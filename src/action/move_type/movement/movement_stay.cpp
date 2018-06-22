//
// Created by austin on 17-12-8.
//

#include <event_manager.h>
#include <movement.hpp>
#include <move_type.hpp>
#include <mode.hpp>
#include <robot.hpp>
#include <wheel.hpp>
#include <bumper.h>
#include <cliff.h>
#include <gyro.h>

MovementStay::MovementStay(double stay_time_sec)
{
	ROS_WARN("%s %d: Start movement stay(%fs).", __FUNCTION__, __LINE__, stay_time_sec);
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
	ROS_WARN("%s %d: End movement stay.", __FUNCTION__, __LINE__);
}


bool MovementStay::isFinish()
{
	bumper_status_in_stay_ = bumper.getStatus();
	tilt_status_in_stay_ = gyro.getTiltCheckingStatus();
//	ROS_ERROR("%s,%d, movementStay",__FUNCTION__, __LINE__);
	if (sp_mt_->sp_mode_->action_i_ != sp_mt_->sp_mode_->ac_linear
		 && sp_mt_->sp_mode_->action_i_ != sp_mt_->sp_mode_->ac_follow_wall_left
		 && sp_mt_->sp_mode_->action_i_ != sp_mt_->sp_mode_->ac_follow_wall_right)
	{
		cliff_status_in_stay_ = cliff.getStatus();
//		ROS_INFO("%s,%d,cliff_status_in_stay_ = cliff.getStatus()", __FUNCTION__,__LINE__);
	}
	ev.bumper_triggered = static_cast<uint8_t>(bumper_status_in_stay_ ? bumper_status_in_stay_ : ev.bumper_triggered);
	ev.cliff_triggered = static_cast<uint8_t>(cliff_status_in_stay_ ? cliff_status_in_stay_: ev.cliff_triggered);
//	ROS_INFO("%s, %d: ev.cliff_triggered(%d).", __FUNCTION__, __LINE__, ev.cliff_triggered);
	ev.tilt_triggered = static_cast<uint8_t>(tilt_status_in_stay_ ? tilt_status_in_stay_ : ev.tilt_triggered);
	if(bumper_status_in_stay_)
		ROS_WARN("%s,%d,bumper trigger",__FUNCTION__,__LINE__);
	if(cliff_status_in_stay_)
		ROS_WARN("%s,%d,cliff trigger",__FUNCTION__,__LINE__);
	if(tilt_status_in_stay_)
		ROS_WARN("%s,%d,tilt trigger",__FUNCTION__,__LINE__);
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
	ROS_WARN("%s %d: Start movement stay remote.", __FUNCTION__, __LINE__);
}

bool MovementStayRemote::isFinish()
{
//	ev.bumper_triggered = bumper.getStatus();
	ev.cliff_triggered = cliff.getStatus();
//	ROS_INFO("%s, %d: ev.cliff_triggered(%d).", __FUNCTION__, __LINE__, ev.cliff_triggered);
//
	return ev.remote_direction_forward ||
		   ev.remote_direction_left ||
		   ev.remote_direction_right ||
//		   ev.bumper_triggered ||
		   ev.cliff_triggered ||
					isTimeUp();
}
