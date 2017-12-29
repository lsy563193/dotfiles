//
// Created by austin on 17-12-3.
//

#include <event_manager.h>
#include <pp.h>
#include <error.h>
#include "arch.hpp"

#define NAV_INFO() ROS_INFO("st(%d),ac(%d)", state_i_, action_i_)

CleanModeNav::CleanModeNav()
{
	event_manager_register_handler(this);
	event_manager_set_enable(true);
	ROS_INFO("%s %d: Entering Navigation mode\n=========================" , __FUNCTION__, __LINE__);
	IMoveType::sp_mode_ = this;
	if(g_plan_activated)
		g_plan_activated = false;
	else
		speaker.play(VOICE_CLEANING_START);

	has_aligned_and_open_slam_ = false;
	paused_odom_angle_ = 0;
	moved_during_pause_ = false;
	clean_path_algorithm_.reset(new NavCleanPathAlgorithm());
	go_home_path_algorithm_.reset();
	map_ = &nav_map;
	map_->reset(CLEAN_MAP);
}

CleanModeNav::~CleanModeNav()
{
	IMoveType::sp_mode_ = nullptr;
	wheel.stop();
	brush.stop();
	vacuum.stop();
	lidar.motorCtrl(OFF);
	lidar.setScanOriginalReady(0);

	robot::instance()->setBaselinkFrameType(ODOM_POSITION_ODOM_ANGLE);
	slam.stop();
	odom.setAngleOffset(0);

	if (moved_during_pause_)
	{
		speaker.play(VOICE_CLEANING_STOP, false);
		ROS_WARN("%s %d: Moved during pause. Stop cleaning.", __FUNCTION__, __LINE__);
	}
	else if (ev.cliff_all_triggered)
	{
		speaker.play(VOICE_ERROR_LIFT_UP, false);
		speaker.play(VOICE_CLEANING_STOP);
		ROS_WARN("%s %d: Cliff all triggered. Stop cleaning.", __FUNCTION__, __LINE__);
	}
	else if (ev.fatal_quit)
	{
		speaker.play(VOICE_CLEANING_STOP, false);
		error.alarm();
	}
	else
	{
		speaker.play(VOICE_CLEANING_FINISHED, false);
		ROS_WARN("%s %d: Finish cleaning.", __FUNCTION__, __LINE__);
	}

	auto cleaned_count = nav_map.getCleanedArea();
	auto map_area = cleaned_count * (CELL_SIZE * 0.001) * (CELL_SIZE * 0.001);
	ROS_INFO("%s %d: Cleaned area = \033[32m%.2fm2\033[0m, cleaning time: \033[32m%d(s) %.2f(min)\033[0m, cleaning speed: \033[32m%.2f(m2/min)\033[0m.",
			 __FUNCTION__, __LINE__, map_area, robot_timer.getWorkTime(),
			 static_cast<float>(robot_timer.getWorkTime()) / 60, map_area / (static_cast<float>(robot_timer.getWorkTime()) / 60));
}

bool CleanModeNav::mapMark()
{
	clean_path_algorithm_->displayCellPath(pointsGenerateCells(passed_path_));
	robot::instance()->pubCleanMapMarkers(nav_map, pointsGenerateCells(plan_path_));
//	if (action_i_ == ac_linear) {
	PP_WARN();
		nav_map.setCleaned(pointsGenerateCells(passed_path_));
//	}

	nav_map.setBlocks();
	if (action_i_ == ac_follow_wall_left || action_i_ == ac_follow_wall_right)
	{
		ROS_ERROR("-------------------------------------------------------");
		auto start = *passed_path_.begin();
		passed_path_.erase(std::remove_if(passed_path_.begin(),passed_path_.end(),[&start](Point32_t& it){
			return it.toCell() == start.toCell();
		}),passed_path_.end());
		clean_path_algorithm_->displayCellPath(pointsGenerateCells(passed_path_));
		ROS_ERROR("-------------------------------------------------------");
		setFollowWall(passed_path_);
	}
	if (state_i_ == st_trapped)
		fw_map.setFollowWall(action_i_ == ac_follow_wall_left,plan_path_);
	else if (state_i_ == st_clean)
	{
		// Set home cell.
		if (ev.rcon_triggered)
		{
			home_points_.push_front(getPosition());
			ROS_INFO("%s %d: Set home cell(%d, %d).", __FUNCTION__, __LINE__, home_points_.front().X, home_points_.front().Y);
		}
	}

	nav_map.markRobot(CLEAN_MAP);
	PP_INFO();
	nav_map.print(CLEAN_MAP, getPosition().toCell().X, getPosition().toCell().Y);

	passed_path_.clear();
	return false;
}

bool CleanModeNav::isFinish()
{
	if (state_i_ == st_pause)
	{
		// For pausing case, only key or remote clean will wake it up.
		if (ev.key_clean_pressed)
		{
			resumePause();
			setNextAction();
		}
		else if (ev.remote_home)
		{
			switchToGoHomePointState();
			setNextAction();
		}
	}
	else if (state_i_ == st_charge)
	{
		if (ev.key_clean_pressed)
		{
			resumeLowBatteryCharge();
			setNextAction();
		}
	}
	else // For any else state.
	{
		if (ev.key_clean_pressed)
		{
			enterPause();
			setNextAction();
		}
		else if (ev.remote_home || ev.battery_home)
		{
			switchToGoHomePointState();
			setNextAction();
		}
	}
	return ACleanMode::isFinish();
}

bool CleanModeNav::isExit()
{
	if (state_i_ == st_pause)
	{
		if (sp_action_->isTimeUp())
		{
			ROS_WARN("%s %d:.", __FUNCTION__, __LINE__);
			setNextMode(md_sleep);
			return true;
		}
		else if (sp_action_->isExit())
		{
			ROS_WARN("%s %d:.", __FUNCTION__, __LINE__);
			moved_during_pause_ = true;
			setNextMode(md_idle);
			return true;
		}
	}

	if (ev.fatal_quit || ev.key_long_pressed || ev.cliff_all_triggered || sp_action_->isExit())
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

bool CleanModeNav::setNextAction()
{
	if (state_i_ == st_init)
	{
		if (action_i_ == ac_null)
			action_i_ = ac_open_gyro;
		else if (action_i_ == ac_open_gyro)
		{
			// If it is the starting of navigation mode, paused_odom_angle_ will be zero.
			odom.setAngleOffset(paused_odom_angle_);
			if (charger.isOnStub())
				action_i_ = ac_back_form_charger;
			else
				action_i_ = ac_open_lidar;

			vacuum.setMode(Vac_Save);
			brush.normalOperate();
		}
		else if (action_i_ == ac_back_form_charger)
		{
			action_i_ = ac_open_lidar;
			// Init odom position here.
			robot::instance()->initOdomPosition();
		}
		else if (action_i_ == ac_open_lidar)
		{
			if (!has_aligned_and_open_slam_)
				action_i_ = ac_align;
			else
				action_i_ = ac_null;
		}
		else if (action_i_ == ac_align)
			action_i_ = ac_open_slam;
		else if (action_i_ == ac_open_slam)
			action_i_ = ac_null;
	}
	else if (isExceptionTriggered())
		action_i_ = ac_exception_resume;
	else if (state_i_ == st_clean)
	{
		auto start = getPosition().toCell();
		auto delta_y = plan_path_.back().Y - start.Y;
		ROS_INFO("%s,%d: path size(%u), old_dir_(%d), bumper(%d), cliff(%d), lidar(%d), delta_y(%d)",
						__FUNCTION__, __LINE__, plan_path_.size(), old_dir_, ev.bumper_triggered,
						ev.cliff_triggered, ev.lidar_triggered, delta_y);
		if (!isXAxis(old_dir_) // If last movement is not x axis linear movement, should not follow wall.
				|| plan_path_.size() > 2 ||
				(!ev.bumper_triggered && !ev.cliff_triggered && !ev.lidar_triggered)
				|| delta_y == 0 || std::abs(delta_y) > 2) {
			action_i_ = ac_linear;
		}
		else
		{
			delta_y = plan_path_.back().Y - start.Y;
			bool is_left = isPos(old_dir_) ^delta_y > 0;
			ROS_INFO("\033[31m""%s,%d: target:, 0_left_1_right(%d=%d ^ %d)""\033[0m",
					 __FUNCTION__, __LINE__, is_left, isPos(old_dir_), delta_y);
			action_i_ = is_left ? ac_follow_wall_left : ac_follow_wall_right;
		}
	}
	else if (state_i_ == st_trapped)
		action_i_ = ac_follow_wall_left;
	else if (state_i_ == st_go_home_point || state_i_ == st_resume_low_battery_charge)
		action_i_ = ac_linear;
	else if (state_i_ == st_go_to_charger)
		action_i_ = ac_go_to_charger;
	else if (state_i_ == st_charge)
		action_i_ = ac_charge;
	else if (state_i_ == st_pause)
		action_i_ = ac_pause;

	genNextAction();
	PP_INFO(); NAV_INFO();
	return action_i_ != ac_null;
}

bool CleanModeNav::setNextState()
{
	PP_INFO();

	bool state_confirm = false;
	while (ros::ok() && !state_confirm)
	{
		if (state_i_ == st_init)
		{
			if (action_i_ == ac_open_slam)
			{
				has_aligned_and_open_slam_ = true;

				auto curr = updatePosition();
				passed_path_.push_back(curr);

				home_points_.back().TH = robot::instance()->getWorldPoseAngle();
				PP_INFO();

				state_i_ = st_clean;
				stateInit(state_i_);
			}
			else if (action_i_ == ac_open_lidar && has_aligned_and_open_slam_)
			{
				if (low_battery_charge_)
				{
					state_i_ = st_resume_low_battery_charge;
					low_battery_charge_ = false;
				}
				else // Resume from pause, because slam is not opened for the first time that open lidar action finished.
					state_i_ = saved_state_i_before_pause;
				stateInit(state_i_);
			}
			else
				state_confirm = true;
		}
		else if (isExceptionTriggered())
		{
			ROS_INFO("%s %d: Pass this state switching for exception cases.", __FUNCTION__, __LINE__);
			// Apply for all states.
			// If all these exception cases happens, directly set next action to exception resume action.
			// BUT DO NOT CHANGE THE STATE!!! Because after exception resume it should restore the state.
			state_confirm = true;
		}
		else if(state_i_ == st_clean)
		{
			PP_INFO();
			old_dir_ = new_dir_;
			ROS_ERROR("old_dir_(%d)", old_dir_);
			if (clean_path_algorithm_->generatePath(nav_map, getPosition(), old_dir_, plan_path_))
			{
				new_dir_ = (MapDirection)plan_path_.front().TH;
				ROS_ERROR("new_dir_(%d)", new_dir_);
				plan_path_.pop_front();
				clean_path_algorithm_->displayCellPath(pointsGenerateCells(plan_path_));
				state_confirm = true;
			}
			else
			{
				if (clean_path_algorithm_->checkTrapped(nav_map, getPosition().toCell()))
				{
					// Robot trapped.
					state_i_ = st_trapped;
					stateInit(state_i_);
				}
				else
					// Robot should go home.
					switchToGoHomePointState();
			}
		}
		else if (state_i_ == st_trapped)
		{
			PP_INFO();
			if (robot_timer.trapTimeout(ESCAPE_TRAPPED_TIME))
			{
				ROS_WARN("%s %d: Escape trapped timeout!(%d)", __FUNCTION__, __LINE__, ESCAPE_TRAPPED_TIME);
				state_i_ = st_null;
				state_confirm = true;
			}
			else if (!clean_path_algorithm_->checkTrapped(nav_map, getPosition().toCell()))
			{
				ROS_WARN("%s %d: Escape trapped !", __FUNCTION__, __LINE__);
				state_i_ = st_clean;
				stateInit(state_i_);
			}
			else
				// Still trapped.
				state_confirm = true;
		}
		else if (state_i_ == st_go_home_point)
		{
			PP_INFO();
			state_confirm = setNextStateForGoHomePoint(nav_map);
		}
		else if (state_i_ == st_resume_low_battery_charge)
		{
			PP_INFO();
			if (getPosition().toCell() == plan_path_.back().toCell())
			{
				// Reach continue point.
				state_i_ = st_clean;
				stateInit(state_i_);
			}
			else
			{
				old_dir_ = new_dir_;
				ROS_ERROR("old_dir_(%d)", old_dir_);
				clean_path_algorithm_->generateShortestPath(nav_map, getPosition(), continue_point_, old_dir_, plan_path_);
				if (!plan_path_.empty())
				{
					new_dir_ = (MapDirection)plan_path_.front().TH;
					ROS_ERROR("new_dir_(%d)", new_dir_);
					plan_path_.pop_front();
					clean_path_algorithm_->displayCellPath(pointsGenerateCells(plan_path_));
					state_confirm = true;
				}
				else
				{
					state_i_ = st_clean;
					stateInit(state_i_);
				}
			}
		}
		else if (state_i_ == st_go_to_charger)
		{
			PP_INFO();
			if (ev.charge_detect && charger.isOnStub())
			{
				if (go_home_for_low_battery_)
				{
					// If it is during low battery go home, it should not leave the clean mode, it should just charge.
					ROS_INFO("%s %d: Enter low battery charge.", __FUNCTION__, __LINE__);
					state_i_ = st_charge;
					stateInit(state_i_);
					paused_odom_angle_ = odom.getAngle();
					go_home_for_low_battery_ = false;
					go_home_path_algorithm_.reset();
				}
				else
					state_i_ = st_null;

				state_confirm = true;
			}
			else
				state_i_ = st_go_home_point;
		}
		else if (state_i_ == st_charge)
		{
			// For low battery charge case.
			if (battery.isFull() || !charger.getChargeStatus())
				resumeLowBatteryCharge();
			else
				// Still charging.
				state_confirm = true;
		}
	}

	return state_i_ != st_null;
}

void CleanModeNav::keyClean(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: key clean.", __FUNCTION__, __LINE__);

	beeper.play_for_command(VALID);
	wheel.stop();

	// Wait for key released.
	bool long_press = false;
	while (key.getPressStatus())
	{
		if (!long_press && key.getPressTime() > 3)
		{
			ROS_WARN("%s %d: key clean long pressed.", __FUNCTION__, __LINE__);
			beeper.play_for_command(VALID);
			long_press = true;
		}
		usleep(20000);
	}

	if (long_press)
		ev.key_long_pressed = true;
	else
		ev.key_clean_pressed = true;
	ROS_WARN("%s %d: Key clean is released.", __FUNCTION__, __LINE__);

	key.resetTriggerStatus();
}

void CleanModeNav::overCurrentWheelLeft(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Left wheel oc.", __FUNCTION__, __LINE__);
	ev.oc_wheel_left = true;
}

void CleanModeNav::overCurrentWheelRight(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Right wheel oc.", __FUNCTION__, __LINE__);
	ev.oc_wheel_right = true;
}

void CleanModeNav::remoteClean(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: remote clean.", __FUNCTION__, __LINE__);

	beeper.play_for_command(VALID);
	wheel.stop();
	ev.key_clean_pressed = true;
	remote.reset();
}

void CleanModeNav::remoteHome(bool state_now, bool state_last)
{
	if (state_i_ == st_clean || action_i_ == ac_pause)
	{
		ROS_WARN("%s %d: remote home.", __FUNCTION__, __LINE__);
		beeper.play_for_command(VALID);
		ev.remote_home = true;
	}
	else
	{
		ROS_WARN("%s %d: remote home but not valid.", __FUNCTION__, __LINE__);
		beeper.play_for_command(INVALID);
	}
	remote.reset();
}

void CleanModeNav::remoteDirectionLeft(bool state_now, bool state_last)
{
	//todo: Just for debug
	if (state_i_ == st_clean)
	{
		beeper.play_for_command(VALID);
		continue_point_ = getPosition();
		ROS_INFO("%s %d: low battery, battery =\033[33m %dmv \033[0m, continue cell(%d, %d)", __FUNCTION__, __LINE__,
				 battery.getVoltage(), continue_point_.X, continue_point_.Y);
		ev.battery_home = true;
		go_home_for_low_battery_ = true;
	}
	else
		beeper.play_for_command(INVALID);

	remote.reset();
}

void CleanModeNav::cliffAll(bool state_now, bool state_last)
{
	if (!ev.cliff_all_triggered)
	{
		ROS_WARN("%s %d: Cliff all.", __FUNCTION__, __LINE__);
		ev.cliff_all_triggered = true;
	}
}

void CleanModeNav::batteryHome(bool state_now, bool state_last)
{
	if (state_i_ == st_clean)
	{
		continue_point_ = getPosition();
		ROS_INFO("%s %d: low battery, battery =\033[33m %dmv \033[0m, continue cell(%d, %d)", __FUNCTION__, __LINE__,
				 battery.getVoltage(), continue_point_.X, continue_point_.Y);
		ev.battery_home = true;
		go_home_for_low_battery_ = true;
	}
}

void CleanModeNav::chargeDetect(bool state_now, bool state_last)
{
	if (!ev.charge_detect)
	{
		ROS_WARN("%s %d: Charge detect!.", __FUNCTION__, __LINE__);
		ev.charge_detect = charger.getChargeStatus();
	}

}
// End event handlers.

bool CleanModeNav::ActionFollowWallisFinish()
{
	if (state_i_ == st_trapped)
		return isBlockCleared();
	else
		return isNewLineReach() || isOverOriginLine();

	return false;
}

bool CleanModeNav::isOverOriginLine()
{
	auto curr = getPosition();
	auto p_mt = boost::dynamic_pointer_cast<IMoveType>(sp_action_);
	if ((p_mt->target_point_.Y > p_mt->start_point_.Y && (p_mt->start_point_.Y - curr.Y) > 120)
		|| (p_mt->target_point_.Y < p_mt->start_point_.Y && (curr.Y - p_mt->start_point_.Y) > 120))
	{
		ROS_WARN("origin(%d,%d) curr_p(%d, %d), p_mt->target_point__(%d, %d)",p_mt->start_point_.X, p_mt->start_point_.Y,  curr.X, curr.Y, p_mt->target_point_.X, p_mt->target_point_.Y);
		auto target_angle = (p_mt->target_point_.Y > p_mt->start_point_.Y) ? -900 : 900;
		if (std::abs(ranged_angle(robot::instance()->getWorldPoseAngle() - target_angle)) < 50) // If robot is directly heading to the opposite side of target line, stop.
		{
			ROS_WARN("%s %d: Opposite to target angle. curr(%d, %d), p_mt->target_point_(%d, %d), gyro(%d), target_angle(%d)", __FUNCTION__, __LINE__, curr.X, curr.Y, p_mt->target_point_.X, p_mt->target_point_.Y,
					 robot::instance()->getWorldPoseAngle(), target_angle);
			return true;
		}
		else if (nav_map.isBlockCleaned(curr.toCell().X, curr.toCell().Y)) // If robot covers a big block, stop.
		{
			ROS_WARN("%s %d: Back to cleaned place, current(%d, %d), curr(%d, %d), p_mt->target_point_(%d, %d).",
					 __FUNCTION__, __LINE__, curr.X, curr.Y, curr.X, curr.Y, p_mt->target_point_.X, p_mt->target_point_.Y);
			return true;
		}
		else{
			ROS_WARN("%s %d: Dynamic adjust the origin line and target line, so it can smoothly follow the wall to clean..",__FUNCTION__,__LINE__);
			p_mt->target_point_.Y += curr.Y - p_mt->start_point_.Y;
			p_mt->start_point_.Y = curr.Y;
		}
	}

	return false;
}

bool CleanModeNav::isNewLineReach()
{
	auto s_curr_p = getPosition();
	auto ret = false;
	auto p_mt = boost::dynamic_pointer_cast<IMoveType>(sp_action_);
	auto is_pos_dir = p_mt->target_point_.Y - p_mt->start_point_.Y > 0;
	// The limit is CELL_COUNT_MUL / 8 * 3 further than target line center.
	auto target_limit = p_mt->target_point_.Y + CELL_COUNT_MUL / 8 * 3 * is_pos_dir;
//	ROS_WARN("~~~~~~~~~~~~~~~~~%s %d: start_p.Y(%d), target.Y(%d),curr_y(%d)",
//					 __FUNCTION__, __LINE__, countToCell(s_curr_p.Y), countToCell(p_mt->target_point_.Y),
//					 countToCell(s_curr_p.Y));
	if (is_pos_dir ^ s_curr_p.Y < target_limit) // Robot has reached the target line limit.
	{
		ROS_WARN("%s %d: Reach the target limit, start_p.Y(%d), target.Y(%d),curr_y(%d)",
				 __FUNCTION__, __LINE__, p_mt->start_point_.Y, p_mt->target_point_.Y,
				 s_curr_p.Y);
		ret = true;
	}
	else if (is_pos_dir ^ s_curr_p.Y < p_mt->target_point_.Y)
	{
		// Robot has reached the target line center but still not reach target line limit.
		// Check if the wall side has blocks on the costmap.
		auto dx = (is_pos_dir ^ action_i_ == ac_follow_wall_left) ? +2 : -2;
		if (nav_map.isBlocksAtY(s_curr_p.toCell().X + dx, s_curr_p.toCell().Y)) {
			ROS_WARN("%s %d: Already has block at the wall side, start_p.Y(%d), target.Y(%d),curr_y(%d)",
					 __FUNCTION__, __LINE__, p_mt->start_point_.toCell().Y, p_mt->target_point_.toCell().Y,
					 s_curr_p.toCell().Y);
			ret = true;
		}
	}

	return ret;
}

bool CleanModeNav::isBlockCleared()
{
	if (!passed_path_.empty())
	{
//		ROS_INFO("%s %d: passed_path_.back(%d %d)", __FUNCTION__, __LINE__, passed_path_.back().X, passed_path_.back().Y);
		return !nav_map.isBlockAccessible(passed_path_.back().X, passed_path_.back().Y);
	}

	return false;
}

void CleanModeNav::resumePause()
{
	ev.key_clean_pressed = false;
	speaker.play(VOICE_CLEANING_CONTINUE);
	ROS_INFO("%s %d: Resume cleaning.", __FUNCTION__, __LINE__);
	// It will NOT change the state.
	state_i_ = st_init;
	stateInit(state_i_);
}

void CleanModeNav::resumeLowBatteryCharge()
{
	// For key clean force continue cleaning.
	if (ev.key_clean_pressed)
		ev.key_clean_pressed = false;

	// Resume from low battery charge.
	speaker.play(VOICE_CLEANING_CONTINUE, false);
	ROS_INFO("%s %d: Resume low battery charge.", __FUNCTION__, __LINE__);
	state_i_ = st_init;
	stateInit(state_i_);
}

void CleanModeNav::switchToGoHomePointState()
{
	state_i_ = st_go_home_point;
	stateInit(state_i_);
	mapMark();
}

void CleanModeNav::enterPause()
{
	ev.key_clean_pressed = false;
	speaker.play(VOICE_CLEANING_PAUSE);
	ROS_INFO("%s %d: Key clean pressed, pause cleaning.", __FUNCTION__, __LINE__);
	paused_odom_angle_ = odom.getAngle();
	saved_state_i_before_pause = state_i_;
	state_i_ = st_pause;
	mapMark();
}

uint8_t CleanModeNav::setFollowWall(const Points& path)
{
	uint8_t block_count = 0;
	if (!path.empty())
	{
		std::string msg = "cell:";
		Cell_t block_cell;
		auto dy = action_i_ == ac_follow_wall_left ? 2 : -2;
		for(auto& point : path){
			if(nav_map.getCell(CLEAN_MAP,point.toCell().X,point.toCell().Y) != BLOCKED_RCON){
				GridMap::robotToCell(point, dy * CELL_SIZE, 0, block_cell.X, block_cell.Y);
				msg += "(" + std::to_string(block_cell.X) + "," + std::to_string(block_cell.Y) + ")";
				nav_map.setCell(CLEAN_MAP, block_cell.X, block_cell.Y, BLOCKED_CLIFF);
				block_count++;
			}
		}
		ROS_INFO("%s,%d: Current(%d, %d), \033[32m mapMark CLEAN_MAP %s\033[0m",__FUNCTION__, __LINE__, getPosition().toCell().X, getPosition().toCell().Y, msg.c_str());
	}
}


