//
// Created by lsy563193 on 12/4/17.
//
#include <event_manager.h>
#include "pp.h"
#include "arch.hpp"


MoveTypeFollowWall::MoveTypeFollowWall(bool is_left)
{
	ROS_INFO("%s %d: Entering move type %s follow wall.", __FUNCTION__, __LINE__,
			 is_left ? "left" : "right");


	auto p_mode = dynamic_cast<ACleanMode*> (sp_mode_);
//	if(! p_clean_mode->plan_path_.empty())
//		p_clean_mode.target_point_ = p_clean_mode->plan_path_.front();
	is_left_ = is_left;
	int16_t turn_angle = getTurnAngle(!p_mode->plan_path_.empty());
	turn_target_angle_ = getPosition().addAngle(turn_angle).th;
	movement_i_ = mm_turn;
	sp_movement_.reset(new MovementTurn(turn_target_angle_, ROTATE_TOP_SPEED));
	IMovement::sp_mt_ = this;

	resetTriggeredValue();
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

MoveTypeFollowWall::~MoveTypeFollowWall()
{
	ROS_INFO("%s %d: Exit move type follow wall.", __FUNCTION__, __LINE__);
	wheel.stop();
}

bool MoveTypeFollowWall::isFinish()
{
	if (IMoveType::isFinish())
	{
		ROS_INFO("%s %d: Move type aborted.", __FUNCTION__, __LINE__);
		return true;
	}

	auto p_clean_mode = dynamic_cast<ACleanMode*> (sp_mode_);

	if(p_clean_mode->MoveTypeFollowWallIsFinish(this))
		return true;

	if (sp_movement_->isFinish()) {
		if (movement_i_ == mm_turn)
		{
			if (!handleMoveBackEvent(p_clean_mode))
			{
				resetTriggeredValue();// is it necessary?
				movement_i_ = mm_straight;
				sp_movement_.reset(new MovementStraight());
			}
		}
		else if (movement_i_ == mm_straight)
		{
			if (!handleMoveBackEvent(p_clean_mode))
			{
				resetTriggeredValue();// is it necessary?
				movement_i_ = mm_forward;
				sp_movement_.reset(new MovementFollowWallLidar(is_left_));
			}
		}
		else if (movement_i_ == mm_forward)
		{
			if (!handleMoveBackEvent(p_clean_mode))
			{
				p_clean_mode->actionFollowWallSaveBlocks();
				int16_t turn_angle = getTurnAngle(false);
				turn_target_angle_ = getPosition().addAngle(turn_angle).th;
				movement_i_ = mm_turn;
				sp_movement_.reset(new MovementTurn(turn_target_angle_, ROTATE_TOP_SPEED));
				resetTriggeredValue();
			}
		}
		else if (movement_i_ == mm_back) {
			movement_i_ = mm_turn;
			int16_t turn_angle = getTurnAngle(false);
			turn_target_angle_ = getPosition().addAngle(turn_angle).th;
			sp_movement_.reset(new MovementTurn(turn_target_angle_, ROTATE_TOP_SPEED));
			resetTriggeredValue();
		}
	}
	return false;
}

int16_t MoveTypeFollowWall::bumperTurnAngle()
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
	if(!is_left_)
		turn_angle = -turn_angle;
	return turn_angle;
}

int16_t MoveTypeFollowWall::cliffTurnAngle()
{
	int16_t turn_angle = -750;
	if(!is_left_)
		turn_angle = -turn_angle;
	return turn_angle;
}

int16_t MoveTypeFollowWall::tiltTurnAngle()
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


int16_t MoveTypeFollowWall::obsTurnAngle()
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

int16_t MoveTypeFollowWall::rconTurnAngle()
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

int MoveTypeFollowWall::double_scale_10(double line_angle)
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

bool MoveTypeFollowWall::_lidarTurnAngle(bool is_left, int16_t &turn_angle, int lidar_min, int lidar_max, int angle_min,
										 int angle_max, double dis_limit)
{
//	ROS_INFO("%s,%d,bumper (\033[32m%d\033[0m)!",__FUNCTION__,__LINE__,bumper.getStatus());
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

bool MoveTypeFollowWall::lidarTurnAngle(int16_t &turn_angle)
{
//	ROS_INFO("%s,%d: mt.is_fw",__FUNCTION__, __LINE__);
	wheel.stop();

	if (ev.obs_triggered != 0)
	{
//		ROS_INFO("%s %d: \033[32mfront obs trigger.\033[0m", __FUNCTION__, __LINE__);
		return _lidarTurnAngle(is_left_, turn_angle, 90, 270, 450, 1800, 0.25);
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
			return _lidarTurnAngle(is_left_, turn_angle, 90, 270, 900, 1800);
		}
		else if (ev.bumper_triggered == BLOCK_RIGHT) {
//			ROS_INFO("%s %d: RightBumper trigger.", __FUNCTION__, __LINE__);
			return _lidarTurnAngle(is_left_, turn_angle, 90, 180, angle_min, angle_max);
		}
		else if (ev.bumper_triggered == BLOCK_LEFT) {
//			ROS_INFO("%s %d: LeftBumper trigger.", __FUNCTION__, __LINE__);
			return _lidarTurnAngle(is_left_, turn_angle, 180, 270, angle_min, angle_max);
		}
	}
	return false;
}

int16_t MoveTypeFollowWall::getTurnAngleByEvent()
{
	int16_t turn_angle{};
	if (ev.bumper_triggered) {
		turn_angle = bumperTurnAngle();
		ROS_INFO("%s %d: Bumper triggered, turn_angle: %d.", __FUNCTION__, __LINE__, turn_angle);
	}
	if (ev.cliff_triggered) {
		turn_angle = cliffTurnAngle();
		ROS_INFO("%s %d: Cliff triggered, turn_angle: %d.", __FUNCTION__, __LINE__, turn_angle);
	}
	if (ev.tilt_triggered) {
		turn_angle = tiltTurnAngle();
		ROS_INFO("%s %d: Tilt triggered, turn_angle: %d.", __FUNCTION__, __LINE__, turn_angle);
	}
	if (ev.obs_triggered) {
		turn_angle = obsTurnAngle();
		ROS_INFO("%s %d: OBS triggered, turn_angle: %d.", __FUNCTION__, __LINE__, turn_angle);
	}
	if (ev.lidar_triggered)
	{
		// Temporary use obs as lidar triggered.
		ev.obs_triggered = ev.lidar_triggered;
		turn_angle = obsTurnAngle();
		ROS_INFO("%s %d: Lidar triggered, turn_angle: %d.", __FUNCTION__, __LINE__, turn_angle);
	}
	if (ev.rcon_triggered)
	{
		turn_angle = rconTurnAngle();
		ROS_INFO("%s %d: Rcon triggered, turn_angle: %d.", __FUNCTION__, __LINE__, turn_angle);
	}

	if(ev.robot_slip)
	{
		// Temporary use obs as lidar triggered.
		ev.obs_triggered = BLOCK_FRONT;
		turn_angle = obsTurnAngle();
		ROS_INFO("%s %d: slip triggered, turn_angle: %d.", __FUNCTION__, __LINE__, turn_angle);
	}
	return turn_angle;
}

int16_t MoveTypeFollowWall::getTurnAngle(bool use_target_angle)
{
	int16_t  turn_angle{};
	if(state_turn){
		state_turn = false;
		INFO_RED("getTurnAngle");
		auto diff = boost::dynamic_pointer_cast<AMovementFollowPoint>(sp_movement_)->angle_diff;
		ROS_INFO("angle_diff(%d)",diff);
		return diff;
	}
	if (LIDAR_FOLLOW_WALL && lidarTurnAngle(turn_angle)) {
		ROS_INFO("%s %d: Use lidarTurnAngle(%d)", __FUNCTION__, __LINE__, turn_angle);
	}
	else {
		auto ev_turn_angle = getTurnAngleByEvent();
		if(use_target_angle) {
			auto target_point_ = dynamic_cast<ACleanMode*> (sp_mode_)->plan_path_.front();
			auto tg_turn_angle = getPosition().angleDiffPoint(target_point_);;
			turn_angle = (std::abs(ev_turn_angle) > std::abs(tg_turn_angle)) ? ev_turn_angle : tg_turn_angle;
			ROS_INFO("%s %d: target_turn_angle(%d), event_turn_angle(%d), choose the big one(%d)",
					 __FUNCTION__, __LINE__, tg_turn_angle, ev_turn_angle, turn_angle);
		}
		else
		{
			turn_angle = ev_turn_angle;
			ROS_INFO("%s %d: Use event_turn_angle(%d)", __FUNCTION__, __LINE__, turn_angle);
		}
	}
	resetTriggeredValue();
	return turn_angle;
}

bool MoveTypeFollowWall::isOverOriginLine(GridMap &map)
{
	auto curr = getPosition();
	auto target_point_ = dynamic_cast<ACleanMode*>(sp_mode_)->plan_path_.front();
	if ((target_point_.y > start_point_.y && (start_point_.y - curr.y) > 120)
		|| (target_point_.y < start_point_.y && (curr.y - start_point_.y) > 120))
	{
//		ROS_WARN("origin(%d,%d) curr_p(%d, %d), target_point__(%d, %d)",start_point_.x, start_point_.y,  curr.x, curr.y, target_point_.x, target_point_.y);
//		auto target_angle = (target_point_.y > start_point_.y) ? -900 : 900;
//		if (std::abs(ranged_angle(robot::instance()->getWorldPoseAngle() - target_angle)) < 50) // If robot is directly heading to the opposite side of target line, stop.
//		{
//			ROS_WARN("%s %d: Opposite to target angle. curr(%d, %d), target_point_(%d, %d), gyro(%d), target_angle(%d)", __FUNCTION__, __LINE__, curr.x, curr.y, target_point_.x, target_point_.y,
//					 robot::instance()->getWorldPoseAngle(), target_angle);
//			return true;
//		}
//		else if (map.isBlockCleaned(curr.toCell().x, curr.toCell().y)) // If robot covers a big block, stop.
//		{
//			ROS_WARN("%s %d: Back to cleaned place, current(%d, %d), curr(%d, %d), target_point_(%d, %d).",
//					 __FUNCTION__, __LINE__, curr.x, curr.y, curr.x, curr.y, target_point_.x, target_point_.y);
			return true;
//		}
//		else{
//			ROS_WARN("%s %d: Dynamic adjust the origin line and target line, so it can smoothly follow the wall to clean..",__FUNCTION__,__LINE__);
//			target_point_.y += curr.y - start_point_.y;
//			start_point_.y = curr.y;
//		}
	}

	return false;
}

bool MoveTypeFollowWall::isNewLineReach(GridMap &map)
{
	auto target_point_ = dynamic_cast<ACleanMode*>(sp_mode_)->plan_path_.front();
	auto s_curr_p = getPosition();
	auto ret = false;
	auto is_pos_dir = target_point_.y - start_point_.y > 0;
	// The limit is CELL_COUNT_MUL / 8 * 3 further than target line center.
//	auto target_limit = target_point_.y + CELL_COUNT_MUL / 8 * 3 * is_pos_dir;
	auto target_limit = target_point_.y;
//	ROS_WARN("~~~~~~~~~~~~~~~~~%s %d: start_p.y(%d), target.y(%d),curr_y(%d)",
//					 __FUNCTION__, __LINE__, countToCell(s_curr_p.y), countToCell(target_point_.y),
//					 countToCell(s_curr_p.y));
	if (is_pos_dir ^ s_curr_p.y < target_limit) // Robot has reached the target line limit.
	{
		ROS_WARN("%s %d: Reach the target limit, start_p.y(%d), target.y(%d),curr_y(%d)",
				 __FUNCTION__, __LINE__, start_point_.y, target_point_.y,
				 s_curr_p.y);
		ret = true;
	}
	else if (is_pos_dir ^ s_curr_p.y < target_point_.y)
	{
		// Robot has reached the target line center but still not reach target line limit.
		// Check if the wall side has blocks on the costmap.
		auto dx = (is_pos_dir ^ is_left_) ? +2 : -2;
		if (map.isBlocksAtY(s_curr_p.toCell().x + dx, s_curr_p.toCell().y)) {
			ROS_WARN("%s %d: Already has block at the wall side, start_p.y(%d), target.y(%d),curr_y(%d)",
					 __FUNCTION__, __LINE__, start_point_.toCell().y, target_point_.toCell().y,
					 s_curr_p.toCell().y);
			ret = true;
		}
	}

	return ret;
}

bool MoveTypeFollowWall::isBlockCleared(GridMap &map, Points &passed_path)
{
	if (!passed_path.empty())
	{
//		ROS_INFO("%s %d: passed_path.back(%d %d)", __FUNCTION__, __LINE__, passed_path.back().x, passed_path.back().y);
		return !map.isBlockAccessible(passed_path.back().toCell().x, passed_path.back().toCell().y);
	}

	return false;
}

bool MoveTypeFollowWall::handleMoveBackEvent(ACleanMode* p_clean_mode)
{
	if (ev.bumper_triggered || ev.cliff_triggered)
	{
		p_clean_mode->actionFollowWallSaveBlocks();
		movement_i_ = mm_back;
		sp_movement_.reset(new MovementBack(0.01, BACK_MAX_SPEED));
		return true;
	}
	else if(ev.tilt_triggered)
	{
		p_clean_mode->actionFollowWallSaveBlocks();
		movement_i_ = mm_back;
		sp_movement_.reset(new MovementBack(0.3, BACK_MAX_SPEED));
		return true;
	}
	else if (ev.robot_slip)
	{
		p_clean_mode->actionFollowWallSaveBlocks();
		movement_i_ = mm_back;
		sp_movement_.reset(new MovementBack(0.3, BACK_MIN_SPEED));
		return true;
	}

	return false;
}

