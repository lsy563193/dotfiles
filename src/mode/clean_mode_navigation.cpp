//
// Created by austin on 17-12-3.
//

#include <event_manager.h>
#include <pp.h>
#include "arch.hpp"

#define NAV_INFO() ROS_INFO("st(%d),mt(%d),ac(%d)", state_i_, move_type_i_, action_i_)

CleanModeNav::CleanModeNav()
{
	event_manager_register_handler(this);
	event_manager_set_enable(true);
	IMovement::sp_cm_ = this;
	// TODO: Remove these old checking.
	if (g_resume_cleaning || cs_is_paused())
	{
		speaker.play(SPEAKER_CLEANING_CONTINUE);
	}
	else if(g_plan_activated)
	{
		g_plan_activated = false;
	}
	else{
		speaker.play(SPEAKER_CLEANING_START);
	}

	paused_ = false;
	has_aligned_and_open_slam = false;
	paused_odom_angle_ = 0;
	moved_during_pause_ = false;
	clean_path_algorithm_ = new NavCleanPathAlgorithm();
	go_home_path_algorithm_ = nullptr;
}

CleanModeNav::~CleanModeNav()
{
	wheel.stop();
	brush.stop();
	vacuum.stop();
	lidar.motorCtrl(OFF);
	lidar.setScanOriginalReady(0);

	robot::instance()->setBaselinkFrameType(Odom_Position_Odom_Angle);
	odom.setAngleOffset(0);

	if (moved_during_pause_)
	{
		speaker.play(SPEAKER_CLEANING_STOP);
		ROS_WARN("%s %d: Moved during pause. Stop cleaning.", __FUNCTION__, __LINE__);
	}
	else if (ev.cliff_all_triggered)
	{
		speaker.play(SPEAKER_ERROR_LIFT_UP, false);
		speaker.play(SPEAKER_CLEANING_STOP);
		ROS_WARN("%s %d: Cliff all triggered. Stop cleaning.", __FUNCTION__, __LINE__);
	}
	else
	{
		speaker.play(SPEAKER_CLEANING_FINISHED);
		ROS_WARN("%s %d: Finish cleaning.", __FUNCTION__, __LINE__);
	}

	auto cleaned_count = nav_map.getCleanedArea();
	auto map_area = cleaned_count * (CELL_SIZE * 0.001) * (CELL_SIZE * 0.001);
	ROS_INFO("%s %d: Cleaned area = \033[32m%.2fm2\033[0m, cleaning time: \033[32m%d(s) %.2f(min)\033[0m, cleaning speed: \033[32m%.2f(m2/min)\033[0m.",
			 __FUNCTION__, __LINE__, map_area, robot_timer.getWorkTime(),
			 static_cast<float>(robot_timer.getWorkTime()) / 60, map_area / (static_cast<float>(robot_timer.getWorkTime()) / 60));
}

uint8_t CleanModeNav::setFollowWall()
{
	uint8_t block_count = 0;
	if (!passed_path_.empty())
	{
		std::string msg = "cell:";
		Cell_t block_cell;
		auto dy = move_type_i_ == mt_follow_wall_left ? 2 : -2;
		for(auto& cell : passed_path_){
			if(nav_map.getCell(CLEAN_MAP,cell.X,cell.Y) != BLOCKED_RCON){
				GridMap::robotToCell(GridMap::cellToPoint(cell), dy * CELL_SIZE, 0, block_cell.X, block_cell.Y);
				msg += "(" + std::to_string(block_cell.X) + "," + std::to_string(block_cell.Y) + ")";
				nav_map.setCell(CLEAN_MAP, GridMap::cellToCount(block_cell.X), GridMap::cellToCount(block_cell.Y), BLOCKED_CLIFF);
				block_count++;
			}
		}
		ROS_INFO("%s,%d: Current(%d, %d), \033[32m map_mark CLEAN_MAP %s\033[0m",__FUNCTION__, __LINE__, nav_map.getXCell(), nav_map.getYCell(), msg.c_str());
	}
}

bool CleanModeNav::map_mark()
{
	PathAlgorithmBase::displayPath(passed_path_);
	if (move_type_i_ == mt_linear) {
		PP_INFO()
		nav_map.setCleaned(passed_path_);
	}

	if (state_i_ == st_trapped)
		nav_map.markRobot(CLEAN_MAP);

	nav_map.setBlocks();
	if (move_type_i_ == mt_follow_wall_left || move_type_i_ == mt_follow_wall_right)
		setFollowWall();
	if (state_i_ == st_trapped)
		fw_map.setFollowWall();

	PP_INFO()
	nav_map.print(CLEAN_MAP, nav_map.getXCell(), nav_map.getYCell());

	passed_path_.clear();
	return false;
}


bool CleanModeNav::isFinish()
{
	if (action_i_ == ac_pause)
	{
		// For pausing case, only key or remote clean will wake it up.
		if (ev.key_clean_pressed)
		{
			ev.key_clean_pressed = false;
			speaker.play(SPEAKER_CLEANING_CONTINUE);
			ROS_INFO("%s %d: Resume cleaning.", __FUNCTION__, __LINE__);
			action_i_ = ac_open_gyro;
			genMoveAction();
			return ACleanMode::isFinish();
		}
		else if (ev.remote_home)
		{
			state_i_ = st_go_home_point;
			st_init(state_i_);

			map_mark();
			if(!setNextState())
			{
				ROS_WARN("%s %d:.", __FUNCTION__, __LINE__);
				setNextMode(md_idle);
				return true;
			}
			move_type_i_ = mt_null;
			setNextMoveType();
			action_i_ = ac_null;
			setNextAction();
			return ACleanMode::isFinish();
		}
		else
			return false;
	}
	// else if (action_i_ == ac_self_check)
	// {}
	else
	{
		if (ev.key_clean_pressed)
		{
			ev.key_clean_pressed = false;
			speaker.play(SPEAKER_CLEANING_PAUSE);
			ROS_INFO("%s %d: Key clean pressed, pause cleaning.", __FUNCTION__, __LINE__);
			paused_ = true;
			paused_odom_angle_ = odom.getAngle();
			action_i_ = ac_pause;
			genMoveAction();
			return ACleanMode::isFinish();
		}
		else if (ev.remote_home)
		{
			state_i_ = st_go_home_point;
			st_init(state_i_);

			map_mark();
			if(!setNextState())
			{
				ROS_WARN("%s %d:.", __FUNCTION__, __LINE__);
				setNextMode(md_idle);
				return true;
			}
			move_type_i_ = mt_null;
			setNextMoveType();
			action_i_ = ac_null;
			setNextAction();
			return ACleanMode::isFinish();
		}
		else
			return ACleanMode::isFinish();
	}

	return false;
}

bool CleanModeNav::isExit()
{
	if (action_i_ == ac_pause && sp_action_->isTimeUp())
	{
		ROS_WARN("%s %d:.", __FUNCTION__, __LINE__);
		setNextMode(md_sleep);
		return true;
	}

	if (action_i_ == ac_pause && sp_action_->isExit())
	{
		ROS_WARN("%s %d:.", __FUNCTION__, __LINE__);
		moved_during_pause_ = true;
		setNextMode(md_idle);
		return true;
	}

	if (ev.key_long_pressed || ev.cliff_all_triggered || sp_action_->isExit())
	{
		ROS_WARN("%s %d:.", __FUNCTION__, __LINE__);
		setNextMode(md_idle);
		return true;
	}

	if (ev.charge_detect >= 3)
	{
		ROS_WARN("%s %d: Exit for directly charge.", __FUNCTION__, __LINE__);
		setNextMode(md_charge);
		return true;
	}

	return false;
}

bool CleanModeNav::setNextAction() {
	if (isInitState() || action_i_ == ac_open_slam) {
		PP_INFO();NAV_INFO();
		if (action_i_ == ac_open_gyro) {
			PP_INFO();
			if (paused_)
				odom.setAngleOffset(paused_odom_angle_);
			if (charger.isOnStub())
				action_i_ = ac_back_form_charger;
			else
				action_i_ = ac_open_lidar;
		}
		else if (action_i_ == ac_back_form_charger)
			action_i_ = ac_open_lidar;
		else if (action_i_ == ac_open_lidar) {
			if (!has_aligned_and_open_slam) {
				action_i_ = ac_align;
			}
			else if (paused_) {
				move_type_i_ = mt_null;
				action_i_ = ac_null;
				// Clear the pause status.
				paused_ = false;
			}
			else
				action_i_ = ac_null;
		}
		else if (action_i_ == ac_align)
			action_i_ = ac_open_slam;
		else if (action_i_ == ac_open_slam) {
			has_aligned_and_open_slam = true;
			move_type_i_ = mt_null;
			action_i_ = ac_null;
			if (paused_)
				// If paused before align and open slam, reset pause status here.
				paused_ = false;
		}
		genMoveAction();
	}
	else {
		ACleanMode::setNextAction();
	}
	PP_INFO();
	NAV_INFO();
	return action_i_ != ac_null;
}

void CleanModeNav::key_clean(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: key clean.", __FUNCTION__, __LINE__);

	beeper.play_for_command(VALID);

	// Wait for key released.
	while (key.getPressStatus())
		usleep(20000);
	ev.key_clean_pressed = true;
	ROS_WARN("%s %d: Key clean is released.", __FUNCTION__, __LINE__);

	key.resetTriggerStatus();
}

void CleanModeNav::remote_clean(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: remote clean.", __FUNCTION__, __LINE__);

	beeper.play_for_command(VALID);
	ev.key_clean_pressed = true;
	remote.reset();
}

void CleanModeNav::remote_home(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: remote home.", __FUNCTION__, __LINE__);

	beeper.play_for_command(VALID);
	ev.remote_home = true;
	remote.reset();
}

void CleanModeNav::cliff_all(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Cliff all.", __FUNCTION__, __LINE__);

	ev.cliff_all_triggered = true;
}

bool CleanModeNav::setNextMoveType() {
	if (state_i_ == st_clean) {
		auto start = nav_map.getCurrCell();
		auto dir = old_dir_;
		auto delta_y = plan_path_.back().Y - start.Y;
		ROS_INFO( "%s,%d: path size(%u), dir(%d), g_check_path_in_advance(%d), bumper(%d), cliff(%d), lidar(%d), delta_y(%d)",
						__FUNCTION__, __LINE__, plan_path_.size(), dir, g_check_path_in_advance, ev.bumper_triggered, ev.cliff_triggered,
						ev.lidar_triggered, delta_y);

		if (!GridMap::isXDirection(dir) // If last movement is not x axis linear movement, should not follow wall.
				|| plan_path_.size() > 2 ||
				(!g_check_path_in_advance && !ev.bumper_triggered && !ev.cliff_triggered && !ev.lidar_triggered)
				|| delta_y == 0 || std::abs(delta_y) > 2) {
			move_type_i_ = mt_linear;
		}
		else {
			delta_y = plan_path_.back().Y - start.Y;
			bool is_left = GridMap::isPositiveDirection(old_dir_) ^ delta_y > 0;
			ROS_INFO("\033[31m""%s,%d: target:, 0_left_1_right(%d=%d ^ %d)""\033[0m", __FUNCTION__, __LINE__, is_left, GridMap::isPositiveDirection(old_dir_), delta_y);
			move_type_i_ = is_left ? mt_follow_wall_left : mt_follow_wall_right;
		}
	}
	else if (state_i_ == st_go_home_point)
		move_type_i_ = mt_linear;
	return ACleanMode::setNextMoveType();
}

bool CleanModeNav::MovementFollowWallisFinish() {
	return isNewLineReach() || isOverOriginLine();
}

bool CleanModeNav::isOverOriginLine()
{
	auto s_curr_p = nav_map.getCurrPoint();
//	ROS_INFO("origin(%d,%d) curr_p(%d, %d), target_p_(%d, %d)",cm_origin_p_.X, cm_origin_p_.Y,  s_curr_p.X, s_curr_p.Y, cm_target_p_.X, cm_target_p_.Y);
	if ((cm_target_p_.Y > cm_origin_p_.Y && (cm_origin_p_.Y - s_curr_p.Y) > 120)
		|| (cm_target_p_.Y < cm_origin_p_.Y && (s_curr_p.Y - cm_origin_p_.Y) > 120))
	{
		ROS_WARN("origin(%d,%d) curr_p(%d, %d), target_p_(%d, %d)",cm_origin_p_.X, cm_origin_p_.Y,  s_curr_p.X, s_curr_p.Y, cm_target_p_.X, cm_target_p_.Y);
		auto curr = nav_map.pointToCell(s_curr_p);
		auto target_angle = (cm_target_p_.Y > cm_origin_p_.Y) ? -900 : 900;
		if (std::abs(ranged_angle(robot::instance()->getPoseAngle() - target_angle)) < 50) // If robot is directly heading to the opposite side of target line, stop.
		{
			ROS_WARN("%s %d: Opposite to target angle. s_curr_p(%d, %d), cm_target_p_(%d, %d), gyro(%d), target_angle(%d)", __FUNCTION__, __LINE__, s_curr_p.X, s_curr_p.Y, cm_target_p_.X, cm_target_p_.Y, robot::instance()->getPoseAngle(), target_angle);
			return true;
		}
		else if (nav_map.isBlockCleaned(curr.X, curr.Y)) // If robot covers a big block, stop.
		{
			ROS_WARN("%s %d: Back to cleaned place, current(%d, %d), s_curr_p(%d, %d), cm_target_p_(%d, %d).",
					 __FUNCTION__, __LINE__, curr.X, curr.Y, s_curr_p.X, s_curr_p.Y, cm_target_p_.X, cm_target_p_.Y);
			return true;
		}
		else{
			ROS_WARN("%s %d: Dynamic adjust the origin line and target line, so it can smoothly follow the wall to clean..",__FUNCTION__,__LINE__);
			cm_target_p_.Y += s_curr_p.Y - cm_origin_p_.Y;
			cm_origin_p_.Y = s_curr_p.Y;
		}
	}

	return false;
}

bool CleanModeNav::isNewLineReach()
{
	auto s_curr_p = nav_map.getCurrPoint();
	auto ret = false;
	auto is_pos_dir = cm_target_p_.Y - cm_origin_p_.Y > 0;
	// The limit is CELL_COUNT_MUL / 8 * 3 further than target line center.
	auto target_limit = cm_target_p_.Y + CELL_COUNT_MUL / 8 * 3 * is_pos_dir;
//	ROS_WARN("~~~~~~~~~~~~~~~~~%s %d: cm_origin_p_.Y(%d), target.Y(%d),curr_y(%d)",
//					 __FUNCTION__, __LINE__, nav_map.countToCell(cm_origin_p_.Y), nav_map.countToCell(cm_target_p_.Y),
//					 nav_map.countToCell(s_curr_p.Y));
	if (is_pos_dir ^ s_curr_p.Y < target_limit) // Robot has reached the target line limit.
	{
		ROS_WARN("%s %d: Reach the target limit, cm_origin_p_.Y(%d), target.Y(%d),curr_y(%d)",
				 __FUNCTION__, __LINE__, nav_map.countToCell(cm_origin_p_.Y), nav_map.countToCell(cm_target_p_.Y),
				 nav_map.countToCell(s_curr_p.Y));
		ret = true;
	}
	else if (is_pos_dir ^ s_curr_p.Y < cm_target_p_.Y)
	{
		// Robot has reached the target line center but still not reach target line limit.
		// Check if the wall side has blocks on the costmap.
		auto dx = (is_pos_dir ^ move_type_i_ == mt_follow_wall_left) ? +2 : -2;
		if (nav_map.isBlocksAtY(nav_map.countToCell(s_curr_p.X) + dx, nav_map.countToCell(s_curr_p.Y))) {
			ROS_WARN("%s %d: Already has block at the wall side, cm_origin_p_.Y(%d), target.Y(%d),curr_y(%d)",
					 __FUNCTION__, __LINE__, nav_map.countToCell(cm_origin_p_.Y), nav_map.countToCell(cm_target_p_.Y),
					 nav_map.countToCell(s_curr_p.Y));
			ret = true;
		}
	}

	return ret;
}
