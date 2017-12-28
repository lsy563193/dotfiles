//
// Created by lsy563193 on 12/4/17.
//
#include <event_manager.h>
#include "pp.h"
#include "arch.hpp"


ActionFollowWall::ActionFollowWall(bool is_left, bool is_trapped)
{

	auto p_clean_mode = (ACleanMode*)sp_mode_;
	target_point_ = p_clean_mode->plan_path_.front();
	is_left_ = is_left;
	int16_t turn_angle;
	PP_INFO();
	if (!is_trapped)
		turn_angle = get_turn_angle(true);
	else
		turn_angle = 0;
	PP_INFO();
	turn_target_angle_ = ranged_angle(robot::instance()->getWorldPoseAngle() + turn_angle);
	movement_i_ = mm_turn;
	PP_INFO();
	sp_movement_.reset(new MovementTurn(turn_target_angle_, ROTATE_TOP_SPEED));
	PP_INFO();
	IMovement::sp_mt_ = this;
	PP_INFO();

//	if (action_i_ == ac_back) {
//		PP_INFO();
//		TIME_STRAIGHT = 0.2;
//		g_wall_distance = WALL_DISTANCE_HIGH_LIMIT;
//	}
//	else if (action_i_ == ac_turn) {
//		PP_INFO();
//		TIME_STRAIGHT = 0;
//		g_wall_distance = WALL_DISTANCE_HIGH_LIMIT;
//	}
}

ActionFollowWall::~ActionFollowWall()
{
	PP_WARN();
	ROS_ERROR("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");
	wheel.stop();
}

bool ActionFollowWall::isFinish()
{
	auto p_clean_mode = (ACleanMode*)sp_mode_;

	if(p_clean_mode->ActionFollowWallisFinish())
		return true;

	if (sp_movement_->isFinish()) {
		PP_WARN();
		if (movement_i_ == mm_turn) {
			resetTriggeredValue();
			movement_i_ = mm_straight;
			sp_movement_.reset(new MovementStraight());
		}
		else if (movement_i_ == mm_straight) {
			resetTriggeredValue();
			movement_i_ = mm_forward;
			sp_movement_.reset(new MovementFollowWallLidar(is_left_));
		}
		else if (movement_i_ == mm_forward) {
			if (ev.bumper_triggered || ev.cliff_triggered || ev.tilt_triggered || g_robot_slip) {
				PP_INFO();
//				resetTriggeredValue();
				movement_i_ = mm_back;
				sp_movement_.reset(new MovementBack(0.01, BACK_MAX_SPEED));
			}
			else if (ev.lidar_triggered || ev.obs_triggered) {
				PP_INFO();
				int16_t turn_angle =get_turn_angle(false);
				turn_target_angle_ = ranged_angle(robot::instance()->getWorldPoseAngle() + turn_angle);
				movement_i_ = mm_turn;
				sp_movement_.reset(new MovementTurn(turn_target_angle_, ROTATE_TOP_SPEED));
				resetTriggeredValue();
			}
			else if(tmp_plan_path_.empty())
			{
				sp_movement_.reset(new MovementForwardTurn(is_left_));
			}
		}
		else if (movement_i_ == mm_back) {
			movement_i_ = mm_turn;
			int16_t turn_angle =get_turn_angle(false);
			turn_target_angle_ = ranged_angle(robot::instance()->getWorldPoseAngle() + turn_angle);
			sp_movement_.reset(new MovementTurn(turn_target_angle_, ROTATE_TOP_SPEED));
			resetTriggeredValue();
		}
	}
	return false;
}

int16_t ActionFollowWall::bumper_turn_angle(bool is_left)
{
	int16_t turn_angle{};
//	static int bumper_jam_cnt_ = 0;
	auto get_wheel_step = is_left_ ? &Wheel::getRightStep : &Wheel::getLeftStep;
	auto get_obs = (is_left_) ? &Obs::getLeft : &Obs::getRight;
	auto get_obs_value = (is_left_) ? &Obs::getLeftTrigValue : &Obs::getRightTrigValue;
	auto status = ev.bumper_triggered;
	auto diff_side = (is_left_) ? BLOCK_RIGHT : BLOCK_LEFT;
	auto same_side = (is_left_) ? BLOCK_LEFT : BLOCK_RIGHT;

	if (status == BLOCK_ALL)
	{
		turn_angle = -600;
//		bumper_jam_cnt_ = (wheel.*get_wheel_step)() < 2000 ? ++bumper_jam_cnt_ : 0;
//		g_wall_distance = WALL_DISTANCE_HIGH_LIMIT;
	} else if (status == diff_side)
	{
		turn_angle = -850;
//		g_wall_distance = WALL_DISTANCE_HIGH_LIMIT;
	} else if (status == same_side)
	{
//		g_wall_distance = bumper_turn_factor * g_wall_distance;
//		if(g_wall_distance < 330)
//			g_wall_distance = WALL_DISTANCE_LOW_LIMIT;
		turn_angle = -300;
//		if (!cs.is_trapped()) {
//			turn_angle = (bumper_jam_cnt_ >= 3 || (obs.*get_obs)() <= (obs.*get_obs_value)()) ? -180 : -280;
//		} else {
//			turn_angle = (bumper_jam_cnt_ >= 3 || (obs.*get_obs)() <= (obs.*get_obs_value)()) ? -100 : -200;
//		}
		//ROS_INFO("%s, %d: turn_angle(%d)",__FUNCTION__,__LINE__, turn_angle);

//		bumper_jam_cnt_ = (wheel.*get_wheel_step)() < 2000 ? ++bumper_jam_cnt_ : 0;
	}
	//ROS_INFO("%s %d: g_wall_distance in bumper_turn_angular: %d", __FUNCTION__, __LINE__, g_wall_distance);
	wheel.resetStep();
	if(!is_left)
		turn_angle = -turn_angle;
	return turn_angle;
}

int16_t ActionFollowWall::cliff_turn_angle()
{
	int16_t turn_angle = -750;
	if(!is_left_)
		turn_angle = -turn_angle;
	return turn_angle;
}

int16_t ActionFollowWall::tilt_turn_angle()
{
	int16_t turn_angle{};
	auto tmp_status = gyro.getTiltCheckingStatus();
	if (is_left_)
	{
		if (tmp_status | TILT_LEFT)
			turn_angle = -600;
		if (tmp_status | TILT_FRONT)
			turn_angle = -850;
		if (tmp_status | TILT_RIGHT)
			turn_angle = -1100;
	}
	else
	{
		if (tmp_status | TILT_RIGHT)
			turn_angle = 600;
		if (tmp_status | TILT_FRONT)
			turn_angle = 850;
		if (tmp_status | TILT_LEFT)
			turn_angle = 1100;
	}
	return turn_angle;
}

int16_t ActionFollowWall::obs_turn_angle()
{
	int16_t turn_angle{};
	auto diff_side = (is_left_) ? BLOCK_RIGHT : BLOCK_LEFT;
	auto same_side = (is_left_) ? BLOCK_LEFT : BLOCK_RIGHT;
	if(ev.obs_triggered == BLOCK_FRONT)
		turn_angle = -850;
	else if(ev.obs_triggered == diff_side)
		turn_angle = -920;
	else if(ev.obs_triggered == same_side)
		turn_angle = -300;

	if(!is_left_)
		turn_angle = -turn_angle;
//	ROS_WARN("turn_angle(%d)",turn_angle);
	return turn_angle;
}

int16_t ActionFollowWall::rcon_turn_angle()
{
	int16_t turn_angle{};
	enum {left,fl2,fl,fr,fr2,right};
	int16_t left_angle[] =   {-300,-600,-850,-850,-950,-1100};
	int16_t right_angle[] =  {1100, 950, 850, 850, 600, 300};
	if(is_left_)
		turn_angle = left_angle[ev.rcon_triggered-1];
	else if(!is_left_)
		turn_angle = right_angle[ev.rcon_triggered-1];

	return turn_angle;
}

int ActionFollowWall::double_scale_10(double line_angle)
{
	int angle;
	if (line_angle > 0)
	{
		angle = int((180 - line_angle) * 10);
	} else
	{
		angle = int(fabs(line_angle) * 10);
	}
	return angle;
}

bool ActionFollowWall::_lidar_turn_angle(bool is_left, int16_t& turn_angle, int lidar_min, int lidar_max, int angle_min,int angle_max,double dis_limit)
{
//	ROS_INFO("%s,%d,bumper (\033[32m%d\033[0m)!",__FUNCTION__,__LINE__,bumper.get_status());
	double line_angle;
	double distance;
//	auto RESET_WALL_DIS = 100;
	auto line_is_found = lidar.lidarGetFitLine(lidar_min, lidar_max, -1.0, dis_limit, &line_angle, &distance,is_left_);
//	RESET_WALL_DIS = int(distance * 1000);

//	ROS_INFO("line_distance = %lf", distance);
//	ROS_INFO("line_angle_raw = %lf", line_angle);
	auto angle = double_scale_10(line_angle);

	if (is_left_)
		angle  = 1800-angle;

//	ROS_INFO("line_angle = %d", angle);
	if (line_is_found && angle >= angle_min && angle < angle_max)
	{
//		ROS_ERROR("distance: %f",(distance*100.0-16.7));
		line_angle=fabs(line_angle);
		if(line_angle < 90)
			robot_to_wall_distance=g_back_distance*100*sin(line_angle*3.1415/180.0);
		else
			robot_to_wall_distance=g_back_distance*100*sin((180-line_angle)*3.1415/180.0);
//		ROS_ERROR("left_x= %f  left_angle= %lf",x,line_angle);
		turn_angle = !is_left ? angle : -angle;
//		ROS_INFO("lidar generate turn angle(%d)!",turn_angle);
		return true;
	}
	return false;
}

bool ActionFollowWall::lidar_turn_angle(int16_t& turn_angle)
{
//	ROS_INFO("%s,%d: mt.is_fw",__FUNCTION__, __LINE__);
	wheel.stop();

	if (ev.obs_triggered != 0)
	{
//		ROS_INFO("%s %d: \033[32mfront obs trigger.\033[0m", __FUNCTION__, __LINE__);
		return _lidar_turn_angle(is_left_, turn_angle, 90, 270, 450, 1800, 0.25);
	}
	else if(ev.bumper_triggered != 0)
	{
		int angle_min, angle_max;
		if (is_left_ ^ (ev.bumper_triggered == BLOCK_LEFT))
		{
			angle_min = 600;
			angle_max = 1800;
		}
		else
		{
			angle_min = 180;
			angle_max = 900;
		}

		if (ev.bumper_triggered == BLOCK_ALL) {
//			ROS_INFO("%s %d: AllBumper trigger.", __FUNCTION__, __LINE__);
			return _lidar_turn_angle(is_left_, turn_angle, 90, 270, 900, 1800);
		}
		else if (ev.bumper_triggered == BLOCK_RIGHT) {
//			ROS_INFO("%s %d: RightBumper trigger.", __FUNCTION__, __LINE__);
			return _lidar_turn_angle(is_left_, turn_angle, 90, 180, angle_min, angle_max);
		}
		else if (ev.bumper_triggered == BLOCK_LEFT) {
//			ROS_INFO("%s %d: LeftBumper trigger.", __FUNCTION__, __LINE__);
			return _lidar_turn_angle(is_left_, turn_angle, 180, 270, angle_min, angle_max);
		}
	}
	return false;
}

int16_t ActionFollowWall::get_turn_angle_by_ev()
{
	int16_t turn_angle{};
	if (ev.bumper_triggered) {
		turn_angle = bumper_turn_angle(is_left_);
		ROS_WARN("%s %d: Bumper triggered, turn_angle: %d.", __FUNCTION__, __LINE__, turn_angle);
	}
	if (ev.cliff_triggered) {
		turn_angle = cliff_turn_angle();
		ROS_WARN("%s %d: Cliff triggered, turn_angle: %d.", __FUNCTION__, __LINE__, turn_angle);
	}
	if (ev.tilt_triggered) {
		turn_angle = tilt_turn_angle();
		ROS_WARN("%s %d: Tilt triggered, turn_angle: %d.", __FUNCTION__, __LINE__, turn_angle);
	}
	if (ev.obs_triggered) {
		turn_angle = obs_turn_angle();
		ROS_WARN("%s %d: OBS triggered, turn_angle: %d.", __FUNCTION__, __LINE__, turn_angle);
	}
	if (ev.lidar_triggered)
	{
		// Temporary use obs as lidar triggered.
		ev.obs_triggered = ev.lidar_triggered;
		turn_angle = obs_turn_angle();
		ROS_WARN("%s %d: Lidar triggered, turn_angle: %d.", __FUNCTION__, __LINE__, turn_angle);
	}
	if (ev.rcon_triggered)
	{
		turn_angle = rcon_turn_angle();
		ROS_WARN("%s %d: Rcon triggered, turn_angle: %d.", __FUNCTION__, __LINE__, turn_angle);
	}

	if(g_robot_slip)
	{
		// Temporary use obs as lidar triggered.
		ev.obs_triggered = BLOCK_FRONT;
		turn_angle = obs_turn_angle();
		ROS_WARN("%s %d: slip triggered, turn_angle: %d.", __FUNCTION__, __LINE__, turn_angle);
	}
	return turn_angle;
}

int16_t ActionFollowWall::get_turn_angle(bool use_target_angle)
{
	int16_t  turn_angle{};
	if (LIDAR_FOLLOW_WALL && lidar_turn_angle(turn_angle)) {
		ROS_INFO("%s %d: lidar_turn_angle(%d)", __FUNCTION__, __LINE__, turn_angle);
	}
	else {
		auto ev_turn_angle = get_turn_angle_by_ev();
		ROS_INFO("%s %d: event_turn_angle(%d)", __FUNCTION__, __LINE__, ev_turn_angle);
		if(use_target_angle) {
			auto cur = getPosition();
			auto tg_turn_angle = ranged_angle(course_to_dest(cur, target_point_) - robot::instance()->getWorldPoseAngle());
			ROS_INFO("%s %d: target_turn_angle(%d)", __FUNCTION__, __LINE__, tg_turn_angle);
			turn_angle = (std::abs(ev_turn_angle) > std::abs(tg_turn_angle)) ? ev_turn_angle : tg_turn_angle;
			ROS_INFO("%s %d: choose the big one(%d)", __FUNCTION__, __LINE__, turn_angle);
		}
		else
			turn_angle = ev_turn_angle;
	}
	ROS_INFO("turn_angle(%d)", turn_angle);
	resetTriggeredValue();
	return turn_angle;
}

