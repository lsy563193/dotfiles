//
// Created by lsy563193 on 12/4/17.
//
#include "ros/ros.h"
#include <event_manager.h>
#include "dev.h"
#include "robot.hpp"
#include <move_type.hpp>
#include <state.hpp>
#include <mode.hpp>

#define STAY_SEC_AFTER_BACK (double)0.33

int g_follow_last_follow_wall_dir=0;
MoveTypeFollowWall::MoveTypeFollowWall(Points remain_path, bool is_left)
{
	IMovement::sp_mt_ = this;
	ROS_WARN("%s %d: Entering move type %s follow wall.", __FUNCTION__, __LINE__,
			 is_left ? "left" : "right");
	remain_path.pop_front();
	remain_path_ = remain_path;
	auto p_mode = dynamic_cast<ACleanMode*> (sp_mode_);
	is_left_ = is_left;
	auto turn_radian = getTurnRadian(!remain_path_.empty());
	turn_target_radian_ = getPosition().addRadian(turn_radian).th;

	movement_i_ = p_mode->isGyroDynamic() ? mm_dynamic : mm_turn;
	if(movement_i_ == mm_dynamic)
		sp_movement_.reset(new MovementGyroDynamic());
	else
		sp_movement_.reset(new MovementTurn(turn_target_radian_, ROTATE_TOP_SPEED));

	resetTriggeredValue();
}

MoveTypeFollowWall::~MoveTypeFollowWall()
{
	wheel.stop();
	if(sp_mode_ != nullptr){
		auto p_mode = dynamic_cast<ACleanMode*> (sp_mode_);
		p_mode->saveBlocks();
		p_mode->mapMark();
	}
	ROS_WARN("%s %d: Exit move type follow wall.", __FUNCTION__, __LINE__);
}

bool MoveTypeFollowWall::isFinish()
{
	if (IMoveType::isFinish() && isNotHandleEvent())
	{
		ROS_WARN("%s %d: Move type aborted.", __FUNCTION__, __LINE__);
		return true;
	}

	auto p_cm = dynamic_cast<ACleanMode*> (sp_mode_);

	auto is_trapped = p_cm->is_trapped_;
	int dijkstra_cleaned_count = 0;
	if(is_trapped) {//check if trapped in a small area
//		int count;
		p_cm->clean_map_.count_if(getPosition().toCell(), [&](Cell_t c_it) {
			return (p_cm->clean_map_.getCell(CLEAN_MAP, c_it.x, c_it.y) == CLEANED);
		},dijkstra_cleaned_count);
		if (dijkstra_cleaned_count < TRAP_IN_SMALL_AREA_COUNT)
			is_trapped_in_small_area_ = true;
		else
			is_trapped_in_small_area_ = false;
	} else {
		is_trapped_in_small_area_ = false;
	}
	if(is_trapped_in_small_area_)
		beeper.debugBeep(INVALID);

	if (sp_movement_->isFinish()) {
		if(movement_i_ == mm_dynamic){
			movement_i_ = mm_turn;
			sp_movement_.reset(new MovementTurn(turn_target_radian_, ROTATE_TOP_SPEED));
			resetTriggeredValue();
		}
		else if (movement_i_ == mm_turn)
		{
			if (!handleMoveBackEvent(p_cm))
			{
				resetTriggeredValue();// is it necessary?
				movement_i_ = mm_straight;
				sp_movement_.reset(new MovementStraight());
			}
		}
		else if (movement_i_ == mm_straight)
		{
			move_forward_time_ +=  ros::Time::now().toSec() - sp_movement_->start_timer_;
			if (!handleMoveBackEvent(p_cm))
			{
				resetTriggeredValue();// is it necessary?
				movement_i_ = mm_forward;
				sp_movement_.reset(new MovementFollowWallLidar(is_left_));
			}
		}
		else if (movement_i_ == mm_forward)
		{
			move_forward_time_ +=  ros::Time::now().toSec() - sp_movement_->start_timer_;
			if (!handleMoveBackEvent(p_cm))
			{
				if(ev.rcon_status) {
					if (p_cm->go_home_path_algorithm_ != nullptr)
						p_cm->go_home_path_algorithm_->setHomePoint(getPosition());
					p_cm->saveBlocks();
					movement_i_ = mm_rcon;
					sp_movement_.reset(new MovementRcon(is_left_));
				}
				else{
					p_cm->saveBlocks();
					auto turn_angle = getTurnRadian(false);
					turn_target_radian_ = getPosition().addRadian(turn_angle).th;

					auto p_mode = dynamic_cast<ACleanMode*>(sp_mode_);
					movement_i_ = p_mode->isGyroDynamic() ? mm_dynamic : mm_turn;
					if(movement_i_ == mm_dynamic)
						sp_movement_.reset(new MovementGyroDynamic());
					else
						sp_movement_.reset(new MovementTurn(turn_target_radian_, ROTATE_TOP_SPEED));
				}
				resetTriggeredValue();
			}else{
				if(ev.tilt_triggered)
					is_stop_follow_wall_after_tilt_ = true;
			}
			state_turn = false;
		}
		else if (movement_i_ == mm_rcon)
		{
			if (!handleMoveBackEvent(p_cm))
			{
				resetTriggeredValue();// is it necessary?
				movement_i_ = mm_straight;
				sp_movement_.reset(new MovementStraight());
			}
		}
		else if(movement_i_ == mm_back)
		{
			movement_i_ = mm_stay;
			sp_movement_.reset(new MovementStay(STAY_SEC_AFTER_BACK));
			//resetTriggeredValue();
		}
		else if (movement_i_ == mm_stay) {
			ROS_INFO("%s,%d, mt_fw",__FUNCTION__, __LINE__);
			if(!handleMoveBackEventRealTime(p_cm)){
				ROS_INFO("%s,%d, mt_fw",__FUNCTION__, __LINE__);
				auto turn_angle = getTurnRadian(false);
				turn_target_radian_ = getPosition().addRadian(turn_angle).th;
				resetTriggeredValue();
				if(is_stop_follow_wall_after_tilt_)
				{
					ROS_INFO("%s,%d, mt_fw",__FUNCTION__, __LINE__);
					is_stop_follow_wall_after_tilt_ = false;
					return true;
				}

				ROS_INFO("%s,%d, mt_fw",__FUNCTION__, __LINE__);
				auto p_mode = dynamic_cast<ACleanMode*>(sp_mode_);
				movement_i_ = p_mode->isGyroDynamic() ? mm_dynamic : mm_turn;
				if(movement_i_ == mm_dynamic)
					sp_movement_.reset(new MovementGyroDynamic());
				else
					sp_movement_.reset(new MovementTurn(turn_target_radian_, ROTATE_TOP_SPEED));
			}
		}
	}
	return false;
}

int16_t MoveTypeFollowWall::bumperTurnAngle()
{
	int16_t turn_angle{};
	auto p_mode = dynamic_cast<ACleanMode*>(sp_mode_);
	auto is_trapped = p_mode->is_trapped_;
	int dijkstra_cleaned_count = 0;
	auto status = ev.bumper_triggered;
	auto get_obs = (is_left_) ? obs.getLeft() : obs.getRight();
	auto diff_side = (is_left_) ? BLOCK_RIGHT : BLOCK_LEFT;
	auto same_side = (is_left_) ? BLOCK_LEFT : BLOCK_RIGHT;
	std::set<Cell_t> c_cleans;

	if (status == BLOCK_ALL || status == BLOCK_LIDAR_BUMPER)
	{
		if(is_trapped)
			turn_angle = is_trapped_in_small_area_ ? -50 : -55;
		else
			turn_angle = -60;
	} else if (status == diff_side)
	{
		if(is_trapped)
			turn_angle = is_trapped_in_small_area_ ? -75 : -80;
		else
			turn_angle = -85;
	} else if (status == same_side)
	{
		if(is_trapped){
			if (is_trapped_in_small_area_)
				turn_angle = -10;
			else
				turn_angle = get_obs > (obs.getLeftTrigValue() + 250) ? -10 : -20;
		}else{
			turn_angle = get_obs > (obs.getLeftTrigValue() + 250) ? -18 : -28;
		}
	}
	if(!is_left_)
		turn_angle = -turn_angle;
	return turn_angle;
}

int16_t MoveTypeFollowWall::cliffTurnAngle()
{
	int16_t turn_angle = -75;
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
/*		if (tmp_status | TILT_LEFT)
			turn_angle = -60;
		if (tmp_status | TILT_FRONT)
			turn_angle = -85;
		if (tmp_status | TILT_RIGHT)
			turn_angle = -110;*/
		turn_angle = -60;
	}
	else
	{
/*		if (tmp_status | TILT_RIGHT)
			turn_angle = 60;
		if (tmp_status | TILT_FRONT)
			turn_angle = 85;
		if (tmp_status | TILT_LEFT)
			turn_angle = 110;*/
		turn_angle = 60;
	}
	return turn_angle;
}


int16_t MoveTypeFollowWall::obsTurnAngle()
{
	int16_t turn_angle{};
	auto diff_side = (is_left_) ? BLOCK_RIGHT : BLOCK_LEFT;
	auto same_side = (is_left_) ? BLOCK_LEFT : BLOCK_RIGHT;
	if(ev.obs_triggered == BLOCK_FRONT)
		turn_angle = -85;
	else if(ev.obs_triggered == diff_side)
		turn_angle = -92;
	else if(ev.obs_triggered == same_side)
		turn_angle = -30;

	if(!is_left_)
		turn_angle = -turn_angle;
//	ROS_WARN("turn_angle(%d)",turn_angle);
	return turn_angle;
}

bool MoveTypeFollowWall::_lidarTurnRadian(bool is_left, double &turn_radian, double lidar_min, double lidar_max,
										  double radian_min, double radian_max, bool is_moving,
																					double dis_limit)
{
	double line_radian;
	double distance;
	bool line_is_found;
//	while (ros::ok()) {
//		wheel.stop();
//		sleep(1);
	std::vector<LineABC> fit_line_group;
		line_is_found = lidar.getFitLine(&fit_line_group, lidar_min, lidar_max, -1.0, dis_limit, &line_radian, &distance, is_left_, 0.10);
//	}

	auto p_mode = dynamic_cast<ACleanMode*> (sp_mode_);

//	p_mode->wall_distance = is_moving ? distance - 0.022 : distance;
	p_mode->wall_distance = is_moving ? distance - 0.03 : distance;

	auto radian = line_radian;

	ROS_INFO("line_angle_raw = %lf, line_is_found = %d, distance = %lf", radian_to_degree(line_radian), line_is_found, distance);

/*	if (!is_left_)
		radian  = PI - line_radian;*/
	if (is_left_) {
		if (radian >= 0) {
			radian = PI - radian;
		} else {
			radian = 0 - radian;
		}
	} else {
		if (radian < 0)
			radian = radian + PI;
	}

	ROS_INFO("line_angle = %lf", radian_to_degree(radian));
	ROS_INFO("angle_range(%lf, %lf)", radian_to_degree(radian_min), radian_to_degree(radian_max));
	radian = fabs(radian);
	ROS_INFO("line_angle after fabs() = %lf", radian_to_degree(radian));
	if (line_is_found && radian >= radian_min && radian < radian_max)
	{
/*		ROS_ERROR("distance: %f",(distance*100.0-16.7));
		line_radian=std::abs(line_radian);
		if(line_radian < PI/4)
			robot_to_wall_distance=g_back_distance*100*sin(line_radian);
		else
			robot_to_wall_distance=g_back_distance*100*sin(PI-line_radian);
		ROS_ERROR("left_x= %f  left_angle= %lf",x,line_radian);*/
		turn_radian = is_left ? -radian : radian;
		ROS_INFO("lidar generate turn angle(%lf)! is_left(%d)",radian_to_degree(turn_radian), is_left);
		return true;
	} else {
		turn_radian = 0;
		ROS_INFO("lidar generate turn angle(%lf) failed! is_left(%d)",radian_to_degree(turn_radian), is_left);
		return false;
	}
}

bool MoveTypeFollowWall::lidarTurnRadian(double &turn_radian)
{
	bool is_moving{false};
	wheel.stop();
	lidar_angle_param_ param;
	if (ev.bumper_triggered) {
		if (is_left_ ^ (ev.bumper_triggered == BLOCK_LEFT)) {//hit the different side, turn angle limit
			param.radian_min_ = degree_to_radian(45);
			param.radian_max_ = degree_to_radian(180);
		}
		else {//hit the same side, turn angle limit
			param.radian_min_ = degree_to_radian(18);
			param.radian_max_ = degree_to_radian(100);
		}

		if (ev.bumper_triggered == BLOCK_ALL || ev.bumper_triggered == BLOCK_LIDAR_BUMPER) {
			param.lidar_min_ = degree_to_radian(90);
			param.lidar_max_ = degree_to_radian(270);
		}
		else if (ev.bumper_triggered == BLOCK_RIGHT) {
			param.lidar_min_ = degree_to_radian(90);
			param.lidar_max_ = degree_to_radian(180);
		}
		else if (ev.bumper_triggered == BLOCK_LEFT) {
			param.lidar_min_ = degree_to_radian(180);
			param.lidar_max_ = degree_to_radian(270);
		}
	}
	else if (ev.lidar_triggered) {
		param.lidar_min_ = degree_to_radian(90);
		param.lidar_max_ = degree_to_radian(270);
		param.radian_min_ = degree_to_radian(18);
		param.radian_max_ = degree_to_radian(180);
		is_moving = true;
	}
/*	while (ros::ok()) {
		_lidarTurnRadian(is_left_, turn_radian, param.lidar_min_, param.lidar_max_, param.radian_min_,
														param.radian_max_);
		sleep(2);
	}*/
	return _lidarTurnRadian(is_left_, turn_radian, param.lidar_min_, param.lidar_max_, param.radian_min_,
													param.radian_max_, is_moving);
}

double MoveTypeFollowWall::getTurnRadianByEvent()
{
	int16_t turn_angle{};
	if (ev.bumper_triggered) {
		turn_angle = bumperTurnAngle();
		ROS_WARN("%s %d: Bumper triggered, turn_angle: %d.", __FUNCTION__, __LINE__, turn_angle);
	}
	if (ev.cliff_triggered) {
		turn_angle = cliffTurnAngle();
		ROS_WARN("%s %d: Cliff triggered, turn_angle: %d.", __FUNCTION__, __LINE__, turn_angle);
	}
	if (ev.tilt_triggered) {
		turn_angle = tiltTurnAngle();
		ROS_WARN("%s %d: Tilt triggered, turn_angle: %d.", __FUNCTION__, __LINE__, turn_angle);
	}
	if (ev.obs_triggered) {
		turn_angle = obsTurnAngle();
		ROS_WARN("%s %d: OBS triggered, turn_angle: %d.", __FUNCTION__, __LINE__, turn_angle);
	}
	if (ev.lidar_triggered)
	{
		// Temporary use obs as lidar triggered.
		ev.obs_triggered = ev.lidar_triggered;
		turn_angle = obsTurnAngle();
		ROS_WARN("%s %d: Lidar triggered, turn_angle: %d.", __FUNCTION__, __LINE__, turn_angle);
	}
	if(ev.robot_slip)
	{
		// Temporary use obs as lidar triggered.
		ev.obs_triggered = BLOCK_FRONT;
		turn_angle = obsTurnAngle();
		ROS_WARN("%s %d: slip triggered, turn_angle: %d.", __FUNCTION__, __LINE__, turn_angle);
	}
	return degree_to_radian(turn_angle);
}

double MoveTypeFollowWall::getTurnRadian(bool use_target_radian)
{
	double  turn_radian{};
	auto p_mode = dynamic_cast<ACleanMode*>(sp_mode_);
	auto is_trapped = p_mode->is_trapped_;
	if(state_turn){
		state_turn = false;
		auto diff = boost::dynamic_pointer_cast<AMovementFollowPoint>(sp_movement_)->radian_diff;
		ROS_INFO("%s %d: Use radian_diff(%f)", __FUNCTION__, __LINE__, diff);
		return diff;
	}

	if (lidarTurnRadian(turn_radian) && !is_trapped) {
		ROS_WARN("%s %d: Use fit line angle!(%f in degree)", __FUNCTION__, __LINE__, radian_to_degree(turn_radian));
	}
	else {
		ROS_INFO("%s %d: Not use fit line angle!", __FUNCTION__, __LINE__);
		auto ev_turn_radian = getTurnRadianByEvent();
		if(ev_turn_radian == 0 && use_target_radian) { //		if(/*use_target_radian*/ 0 )
			auto target_point_ = remain_path_.back();
//			auto target_turn_radian = getPosition().courseToDest(target_point_);
			turn_radian = getPosition().courseToDest(target_point_);
//			turn_radian = std::abs(ev_turn_radian) > std::abs(target_turn_radian) ? ev_turn_radian : target_turn_radian;
//			ROS_INFO("%s %d: target_turn_radian(%f in degree), event_turn_radian(%f in degree), choose the big one(%f in degree)",
//					 __FUNCTION__, __LINE__, radian_to_degree(target_turn_radian),
//					 radian_to_degree(ev_turn_radian), radian_to_degree(turn_radian));
		}
		else
		{
			turn_radian = ev_turn_radian;
			ROS_INFO("%s %d: Use event_turn_radian(%f in degree)", __FUNCTION__, __LINE__, radian_to_degree(turn_radian));
		}
	}
	resetTriggeredValue();
	return turn_radian;
}

bool MoveTypeFollowWall::isOverOriginLine(GridMap &map)
{
	if(remain_path_.empty())
		return true;
	auto curr = getPosition();
	auto target_point_ = remain_path_.back();
	auto p_mode = dynamic_cast<ACleanMode*>(sp_mode_);
	auto start_point_ = p_mode->iterate_point_;
//	ROS_WARN("movement_i_ == mm_forward(%d), ros::Time::now().toSec() - sp_movement_->start_timer_ + move_forward_time_(%lf)",
//					 movement_i_ == mm_forward || movement_i_ == mm_straight, ros::Time::now().toSec() - sp_movement_->start_timer_ + move_forward_time_);
	double const WF_TIME_LIMIT = 1.5;//force wall follow time
	if (((target_point_.y > start_point_.y && start_point_.y - curr.y > CELL_SIZE / 6)
		|| ((curr.y - start_point_.y > CELL_SIZE / 6) && target_point_.y < start_point_.y)) &&
			 ( (ros::Time::now().toSec() - sp_movement_->start_timer_ + move_forward_time_) > WF_TIME_LIMIT))
	{
//		ROS_WARN("origin(%d,%d) curr_p(%d, %d), target_point__(%d, %d)",start_point_.x, start_point_.y,  curr.x, curr.y, target_point_.x, target_point_.y);
//		auto target_angle = (target_point_.y > start_point_.y) ? -900 : 900;
//		if (std::abs(ranged_radian(robot::instance()->getWorldPoseRadian() - target_angle)) < 50) // If robot is directly heading to the opposite side of target line, stop.
//		{
//			ROS_WARN("%s %d: Opposite to target angle. curr(%d, %d), target_point_(%d, %d), gyro(%d), target_angle(%d)", __FUNCTION__, __LINE__, curr.x, curr.y, target_point_.x, target_point_.y,
//					 robot::instance()->getWorldPoseRadian(), target_angle);
//			return true;
//		}
//		else if (map.isNotBlockAndCleaned(curr.toCell().x, curr.toCell().y)) // If robot covers a big block, stop.
//		{
			ROS_WARN("%s %d: Back to cleaned place, current(%f, %f), curr(%f, %f), target_point_(%f, %f).",
					 __FUNCTION__, __LINE__, curr.x, curr.y, curr.x, curr.y, target_point_.x, target_point_.y);
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
	auto ret = false;
	if(remain_path_.empty())
		return true;
	auto target_point_ = remain_path_.back();
	auto s_curr_p = getPosition();
	auto p_mode = dynamic_cast<ACleanMode*>(sp_mode_);
	auto start_point_ = p_mode->iterate_point_;
	auto is_pos_dir = target_point_.y - start_point_.y > 0;
	// The limit is CELL_COUNT_MUL / 8 * 3 further than target line center.
//	auto target_limit = target_point_.y + CELL_COUNT_MUL / 8 * 3 * is_pos_dir;
	auto target_limit = target_point_.y;
//	ROS_WARN("~~~~~~~~~~~~~~~~~%s %d: start_p.y(%d), target.y(%d),curr_y(%d)",
//					 __FUNCTION__, __LINE__, countToCell(s_curr_p.y), countToCell(target_point_.y),
//					 countToCell(s_curr_p.y));
	if (is_pos_dir ^ s_curr_p.y < target_limit) // Robot has reached the target line limit.
	{
		g_follow_last_follow_wall_dir = (is_left_ ^ (target_point_.y<start_point_.y)) ? 1 : 2;
		ROS_WARN("%s %d: Reach the target limit, start_p.y(%f), target.y(%f), curr_y(%f), last follow wall dir(%d)",
				 __FUNCTION__, __LINE__, start_point_.y, target_point_.y, s_curr_p.y, g_follow_last_follow_wall_dir);
		ret = true;
	}
	return ret;
}

bool MoveTypeFollowWall::handleMoveBackEventRealTime(ACleanMode *p_clean_mode)
{
	if (movement_i_ != mm_stay)
	{
		ROS_ERROR("%s,%d, This function is just for movement stay!!!!!", __FUNCTION__, __LINE__);
		return false;
	}

	auto bumper_status = boost::dynamic_pointer_cast<MovementStay>(sp_movement_)->bumper_status_in_stay_;
	auto cliff_status = boost::dynamic_pointer_cast<MovementStay>(sp_movement_)->cliff_status_in_stay_;
	auto tilt_status = boost::dynamic_pointer_cast<MovementStay>(sp_movement_)->tilt_status_in_stay_;
	ROS_INFO("%s,%d, mt_fw", __FUNCTION__, __LINE__);
	if (bumper_status || cliff_status || tilt_status)
	{
		ROS_WARN("%s,%d, bumper_status(%d), cliff_status(%d), tilt_status(%d)", __FUNCTION__, __LINE__, bumper_status, cliff_status, tilt_status);
		p_clean_mode->saveBlocks();
		movement_i_ = mm_back;
		float back_distance = static_cast<float>(bumper_status ? 0.01 : 0.05);
		back_distance = static_cast<float>(tilt_status ? TILT_BACK_DISTANCE : back_distance);
		sp_movement_.reset(new MovementBack(back_distance, BACK_MAX_SPEED));
		return true;
	}
	return false;
}

