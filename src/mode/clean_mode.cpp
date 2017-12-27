//
// Created by lsy563193 on 17-12-3.
//

#include <mathematics.h>
#include <pp.h>
#include "arch.hpp"

#define NAV_INFO() ROS_INFO("st(%d),ac(%d)", state_i_, action_i_)

Path_t ACleanMode::passed_path_ = {};
Path_t ACleanMode::plan_path_ = {};
Cell_t ACleanMode::last_ = {};
//boost::shared_ptr<IMovement> ACleanMode::sp_movement_ = nullptr;

ACleanMode::ACleanMode()
{
	led.set_mode(LED_FLASH, LED_GREEN, 1000);
	ev.key_clean_pressed = false;
	sp_action_.reset(new ActionOpenGyro());
	action_i_ = ac_open_gyro;
	state_i_ = st_null;
	isInitFinished_ = false;
	robot_timer.initWorkTimer();
	key.resetPressStatus();

	c_rcon.resetStatus();

	home_cells_.resize(1,g_zero_home);
	clean_path_algorithm_.reset();
	go_home_path_algorithm_.reset();
}

bool ACleanMode::setNextAction()
{
	if (!isInitFinished_)
	{
		if(action_i_ == ac_open_gyro)
		{
			vacuum.setMode(Vac_Save);
			brush.normalOperate();
			action_i_ = ac_open_lidar;
		}
		else if(action_i_ == ac_open_lidar)
			action_i_ = ac_open_slam;
		else
		{
			isInitFinished_ = true;
			action_i_ = ac_null;
		}
	}
	else if (isExceptionTriggered())
		action_i_ = ac_exception_resume;
	else
		action_i_ = ac_null;

	genNextAction();
	PP_INFO(); NAV_INFO();
	return action_i_ != ac_null;
}

void ACleanMode::setNextMode(int next)
{
	if (ev.charge_detect && charger.isOnStub()) {
		ROS_WARN("%s %d:.", __FUNCTION__, __LINE__);
		Mode::setNextMode(md_charge);
	}
	else {
		ROS_WARN("%s %d:.", __FUNCTION__, __LINE__);
		Mode::setNextMode(next);
	}
}

bool ACleanMode::isFinish()
{
	if (isInitFinished_)
		updatePath(*clean_map_);

	if (!sp_action_->isFinish())
		return false;

	sp_action_.reset();//for call ~constitution;
	PP_INFO();

	if (isInitFinished_)
		mapMark();

	do
	{
		if (!setNextState())
		{
			setNextMode(0);
			return true;
		}
	} while (!setNextAction());

	return false;
}

bool is_equal_with_angle_(const Cell_t &l, const Cell_t &r)
{
	return  l == r && std::abs(ranged_angle(l.TH - r.TH)) < 200;
}

Cell_t ACleanMode::updatePath(GridMap& map)
{
	auto curr = map.updatePosition();
//	auto point = map.getCurrPoint();
//	robot::instance()->pubCleanMapMarkers(nav_map, tmp_plan_path_);
//	PP_INFO();
//	ROS_INFO("point(%d,%d,%d)",point.X, point.Y,point.TH);
//	ROS_INFO("last(%d,%d,%d)",last_.X, last_.Y, last_.TH);
	if (passed_path_.empty())
	{
		passed_path_.push_back(curr);
//		ROS_INFO("curr(%d,%d,%d)",curr.X, curr.Y, curr.TH);
	}
	else if (!is_equal_with_angle_(curr, last_))
	{
		last_ = curr;
		auto loc = std::find_if(passed_path_.begin(), passed_path_.end(), [&](Cell_t it) {
				return is_equal_with_angle_(curr, it);
		});
		auto distance = std::distance(loc, passed_path_.end());
		if (distance == 0) {
			ROS_INFO("curr(%d,%d,%d)",curr.X, curr.Y, curr.TH);
			passed_path_.push_back(curr);
		}
		if (distance > 5) {
			passed_path_.clear();
			g_wf_reach_count++;
		}
		map.saveBlocks(action_i_ == ac_linear);
//		displayPath(passed_path_);
	}
	return curr;
}

void ACleanMode::genNextAction()
{

	PP_INFO();
	if(action_i_ == ac_open_gyro)
		sp_action_.reset(new ActionOpenGyro);
	else if(action_i_ == ac_back_form_charger)
		sp_action_.reset(new ActionBackFromCharger);
	else if(action_i_ == ac_open_lidar)
		sp_action_.reset(new ActionOpenLidar);
	else if(action_i_ == ac_align)
		sp_action_.reset(new ActionAlign);
	else if(action_i_ == ac_open_slam)
		sp_action_.reset(new ActionOpenSlam);
	else if (action_i_ == ac_pause)
		sp_action_.reset(new ActionPause);
	else if (action_i_ == ac_linear)
		sp_action_.reset(new ActionLinear);
	else if (action_i_ == ac_follow_wall_left || action_i_ == ac_follow_wall_right)
		sp_action_.reset(new ActionFollowWall(action_i_ == ac_follow_wall_left, state_i_ == st_trapped));
	else if (action_i_ == ac_go_to_charger)
		sp_action_.reset(new MoveTypeGoToCharger);
	else if (action_i_ == ac_exception_resume)
		sp_action_.reset(new MovementExceptionResume);
	else if (action_i_ == ac_check_bumper)
		sp_action_.reset(new ActionCheckBumper);
	else if (action_i_ == ac_bumper_hit_test)
		sp_action_.reset(new MoveTypeBumperHitTest);
	else if (action_i_ == ac_check_vacuum)
		sp_action_.reset(new ActionCheckVacuum);
	else if (action_i_ == ac_movement_direct_go)
		sp_action_.reset(new MovementDirectGo);
	else if(action_i_ == ac_null)
		sp_action_.reset();
	PP_INFO();
}

void ACleanMode::resetTriggeredValue(void)
{
	ev.lidar_triggered = 0;
	ev.rcon_triggered = 0;
	ev.bumper_triggered = 0;
	ev.obs_triggered = 0;
	ev.cliff_triggered = 0;
	ev.tilt_triggered = 0;
}

void ACleanMode::stateInit(int next)
{
	if (next == st_clean) {
		g_wf_reach_count = 0;
		led.set_mode(LED_STEADY, LED_GREEN);
		PP_INFO();
	}
	if (next == st_go_home_point)
	{
		vacuum.setMode(Vac_Normal, false);
		wheel.stop();

		wheel.setPidTargetSpeed(0, 0, REG_TYPE_LINEAR);
		if (ev.remote_home)
			led.set_mode(LED_STEADY, LED_ORANGE);

		// Play wavs.
		if (ev.battery_home)
			speaker.play(VOICE_BATTERY_LOW, true);

		speaker.play(VOICE_BACK_TO_CHARGER, true);

		ev.remote_home = false;
		ev.battery_home = false;

		if (go_home_path_algorithm_ == nullptr)
			go_home_path_algorithm_.reset(new GoHomePathAlgorithm(nav_map, home_cells_));
		ROS_INFO("%s %d: home_cells_.size(%lu)", __FUNCTION__, __LINE__, home_cells_.size());

	}
	if (next == st_tmp_spot) {
//		if (SpotMovement::instance()->getSpotType() == NO_SPOT) {
//			ROS_INFO("%s %d: Entering temp spot during navigation.", __FUNCTION__, __LINE__);
//			Cell_t curr_cell = nav_map.getCurrCell();
//			ROS_WARN("%s %d: current cell(%d, %d).", __FUNCTION__, __LINE__, curr_cell.X, curr_cell.Y);
//			SpotMovement::instance()->setSpotType(CLEAN_SPOT);
//			wheel.stop();
//		}
//		else if (SpotMovement::instance()->getSpotType() == CLEAN_SPOT) {
//			ROS_INFO("%s %d: Exiting temp spot.", __FUNCTION__, __LINE__);
//			SpotMovement::instance()->spotDeinit();
//			wheel.stop();
//			speaker.play(VOICE_CLEANING_CONTINUE);
//		}
//		ev.remote_spot = false;
	}
	if (next == st_trapped) {
		robot_timer.initTrapTimer();
		led.set_mode(LED_FLASH, LED_GREEN, 300);
	}
	if (next == st_exploration) {
		g_wf_reach_count = 0;
		led.set_mode(LED_STEADY, LED_ORANGE);
	}
	if (next == st_go_to_charger) {
		gyro.TiltCheckingEnable(false); //disable tilt detect
		led.set_mode(LED_STEADY, LED_ORANGE);
	}
	if (next == st_self_check) {
		led.set_mode(LED_STEADY, LED_GREEN);
	}
}

uint8_t ACleanMode::saveFollowWall(bool is_left)
{
	auto dy = is_left ? 2 : -2;
	int16_t x, y;
	//int32_t	x2, y2;
	std::string msg = "cell:";
	GridMap::robotToCell(GridMap::getCurrPoint(), dy * CELL_SIZE, 0, x, y);
	//robot_to_point(robot::instance()->getPoseAngle(), dy * CELL_SIZE, 0, &x2, &y2);
	//ROS_WARN("%s %d: d_cell(0, %d), angle(%d). Old method ->point(%d, %d)(cell(%d, %d)). New method ->cell(%d, %d)."
	//			, __FUNCTION__, __LINE__, dy, robot::instance()->getPoseAngle(), x2, y2, count_to_cell(x2), count_to_cell(y2), x, y);
//	bool should_save_for_MAP = !(cm_is_navigation() && mt.is_follow_wall() && Movement::getMoveDistance() < 0.1);
	temp_fw_cells.push_back({x, y});
	msg += "[0," + std::to_string(dy) + "](" + std::to_string(x) + "," + std::to_string(y) + ")";
	//ROS_INFO("%s,%d: Current(%d, %d), save \033[32m%s\033[0m",__FUNCTION__, __LINE__, get_x_cell(), get_y_cell(), msg.c_str());

	return 1;
}

bool ACleanMode::ActionFollowWallisFinish()
{
	return false;
}




