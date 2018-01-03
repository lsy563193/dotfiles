//
// Created by lsy563193 on 17-12-3.
//

#include <mathematics.h>
#include <pp.h>
#include <event_manager.h>
#include "arch.hpp"

//#define NAV_INFO() ROS_INFO("st(%d),ac(%d)", state_i_, action_i_)

State* ACleanMode::sp_state{};
State* ACleanMode::state_saved_state_before_pause{};
State* ACleanMode::state_init = new StateInit();
State* ACleanMode::state_clean = new StateClean();
State* ACleanMode::state_go_home_point = new StateGoHomePoint();
State* ACleanMode::state_go_to_charger = new StateGoCharger();
State* ACleanMode::state_charge = new StateCharge();
State* ACleanMode::state_trapped = new StateTrapped();
State* ACleanMode::state_tmp_spot = new StateTmpSpot();
State* ACleanMode::state_exception_resume = new StateSelfCheck();
State* ACleanMode::state_exploration = new StateExploration();
State* ACleanMode::state_resume_low_battery_charge = new StateResumeLowBatteryCharge();
State* ACleanMode::state_pause = new StatePause();
Points ACleanMode::passed_path_ = {};
Points ACleanMode::plan_path_ = {};
//Point32_t ACleanMode::last_ = {};
//boost::shared_ptr<IMovement> ACleanMode::sp_movement_ = nullptr;

ACleanMode::ACleanMode()
{
	sp_state->setMode(this);
	ev.key_clean_pressed = false;
	sp_state = state_init;
	ROS_ERROR("%d",sp_state);
	sp_state->update();
	setNextAction();
	robot_timer.initWorkTimer();
	key.resetPressStatus();

	c_rcon.resetStatus();

	home_points_.resize(1, {{0, 0, 0}, false});
	clean_path_algorithm_.reset();
	go_home_path_algorithm_.reset();

	// Init odom position here.
	robot::instance()->initOdomPosition();

	passed_path_.clear();
	plan_path_.clear();
}

bool ACleanMode::setNextAction()
{
	if (sp_state == state_init)
	{
		if (action_i_ == ac_null)
			action_i_ = ac_open_gyro;
		else if(action_i_ == ac_open_gyro)
		{
			vacuum.setMode(Vac_Save);
			brush.normalOperate();
			action_i_ = ac_open_lidar;
		}
		else if(action_i_ == ac_open_lidar)
			action_i_ = ac_open_slam;
		else // action_open_slam
			action_i_ = ac_null;
	}
	else if (isExceptionTriggered())
		action_i_ = ac_exception_resume;
	else
		action_i_ = ac_null;

	genNextAction();
	PP_INFO();
	return action_i_ != ac_null;
}

void ACleanMode::setNextModeDefault()
{
	if (ev.charge_detect && charger.isOnStub()) {
		ROS_WARN("%s %d:.", __FUNCTION__, __LINE__);
		Mode::setNextMode(md_charge);
	}
	else {
		ROS_WARN("%s %d:.", __FUNCTION__, __LINE__);
		Mode::setNextMode(md_idle);
	}
}

bool ACleanMode::isExit()
{
	if (sp_state == state_init)
	{
		if (action_i_ == ac_open_lidar && sp_action_->isTimeUp())
		{
			error.set(ERROR_CODE_LIDAR);
			setNextMode(md_idle);
			ev.fatal_quit = true;
			return true;
		}
	}

	return false;
}

bool ACleanMode::isFinish()
{
	if (sp_state != state_init)
		updatePath(clean_map_);

	if (!(sp_action_ == nullptr || sp_action_->isFinish()))
		return false;

	sp_action_.reset();//for call ~constitution;
	PP_INFO();

	if (sp_state != state_init)
	{
		clean_map_.saveBlocks(action_i_ == ac_linear, sp_state == state_clean);
		mapMark();

	}

	do
	{
		if (!setNextState())
		{
			setNextModeDefault();
			return true;
		}
	} while (ros::ok() && !setNextAction());

	return false;
}

Point32_t ACleanMode::updatePath(GridMap& map)
{
	auto curr = updatePosition();
//	auto point = getPosition();
//	robot::instance()->pubCleanMapMarkers(nav_map, tmp_plan_path_);
//	PP_INFO();
//	ROS_INFO("point(%d,%d,%d)",point.x, point.y,point.th);
//	ROS_INFO("last(%d,%d,%d)",last_.x, last_.y, last_.th);
	if (passed_path_.empty())
	{
		passed_path_.push_back(curr);
		last_ = curr;
	}
	else if (!curr.isCellAndAngleEqual(last_))
	{
		last_ = curr;
		auto loc = std::find_if(passed_path_.begin(), passed_path_.end(), [&](Point32_t it) {
				return curr.isCellAndAngleEqual(it);
		});
		auto distance = std::distance(loc, passed_path_.end());
		if (distance == 0) {
			ROS_INFO("curr(%d,%d,%d)",curr.toCell().x, curr.toCell().y, curr.th);
			passed_path_.push_back(curr);
		}
		if (distance > 5) {
		ROS_INFO("reach_cleaned_count_(%d)",reach_cleaned_count_);
			reach_cleaned_count_++;
		}
		map.saveBlocks(action_i_ == ac_linear, sp_state == state_clean);
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
		sp_action_.reset(new MoveTypeLinear);
	else if (action_i_ == ac_follow_wall_left || action_i_ == ac_follow_wall_right)
		sp_action_.reset(new MoveTypeFollowWall(action_i_ == ac_follow_wall_left, sp_state == state_trapped));
	else if (action_i_ == ac_go_to_charger)
		sp_action_.reset(new MoveTypeGoToCharger);
	else if (action_i_ == ac_exception_resume)
		sp_action_.reset(new MovementExceptionResume);
	else if (action_i_ == ac_charge)
		sp_action_.reset(new MovementCharge);
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

void ACleanMode::stateInit(State* next)
{
	if (next == state_init)
	{
		action_i_ = ac_null;
		led.set_mode(LED_FLASH, LED_GREEN, 1000);
		PP_INFO();
	}
	if (next == state_clean) {
		reach_cleaned_count_ = 0;
		led.set_mode(LED_STEADY, LED_GREEN);
		PP_INFO();
	}
	if (next == state_go_home_point)
	{
		vacuum.setMode(Vac_Normal, false);
		wheel.stop();

		wheel.setPidTargetSpeed(0, 0, REG_TYPE_LINEAR);
		if (ev.remote_home)
			led.set_mode(LED_STEADY, LED_ORANGE);

		// Play wavs.
		if (ev.battery_home)
			speaker.play(VOICE_BATTERY_LOW, false);

		speaker.play(VOICE_BACK_TO_CHARGER, true);

		ev.remote_home = false;
		ev.battery_home = false;

		if (go_home_path_algorithm_ == nullptr)
			go_home_path_algorithm_.reset(new GoHomePathAlgorithm(clean_map_, home_points_));
		ROS_INFO("%s %d: home_cells_.size(%lu)", __FUNCTION__, __LINE__, home_points_.size());

	}
	if (next == state_tmp_spot) {
	}
	if (next == state_trapped) {
		robot_timer.initTrapTimer();
		led.set_mode(LED_FLASH, LED_GREEN, 300);
	}
	if (next == state_exploration) {
		reach_cleaned_count_ = 0;
		led.set_mode(LED_STEADY, LED_ORANGE);
	}
	if (next == state_go_to_charger) {
		gyro.TiltCheckingEnable(false); //disable tilt detect
		led.set_mode(LED_STEADY, LED_ORANGE);
	}
	if (next == state_exception_resume) {
		led.set_mode(LED_STEADY, LED_GREEN);
	}
	if (next == state_charge)
	{
		// Nothing
	}
	if (next == state_resume_low_battery_charge)
	{
		led.set_mode(LED_STEADY, LED_GREEN);
		PP_INFO();
	}
}

//uint8_t ACleanMode::saveFollowWall(bool is_left)
//{
//	auto dy = is_left ? 2 : -2;
//	int16_t x, y;
//	//int32_t	x2, y2;
//	std::string msg = "cell:";
//	GridMap::robotToCell(getPosition(), dy * CELL_SIZE, 0, x, y);
//	//robot_to_point(robot::instance()->getWorldPoseAngle(), dy * CELL_SIZE, 0, &x2, &y2);
//	//ROS_WARN("%s %d: d_cell(0, %d), angle(%d). Old method ->point(%d, %d)(cell(%d, %d)). New method ->cell(%d, %d)."
//	//			, __FUNCTION__, __LINE__, dy, robot::instance()->getWorldPoseAngle(), x2, y2, count_to_cell(x2), count_to_cell(y2), x, y);
////	bool should_save_for_MAP = !(cm_is_navigation() && mt.is_follow_wall() && Movement::getMoveDistance() < 0.1);
//	temp_fw_cells.push_back({x, y});
//	msg += "[0," + std::to_string(dy) + "](" + std::to_string(x) + "," + std::to_string(y) + ")";
//	//ROS_INFO("%s,%d: Current(%d, %d), save \033[32m%s\033[0m",__FUNCTION__, __LINE__, get_x_cell(), get_y_cell(), msg.c_str());
//
//	return 1;
//}
void ACleanMode::setRconPos(Point32_t pos)
{
		charger_pos_ = pos;
}

bool ACleanMode::actionFollowWallisFinish()
{
	return false;
}

void ACleanMode::actionFollowWallSaveBlocks()
{
	return;
}

bool ACleanMode::setNextStateForGoHomePoint(GridMap &map)
{
	bool state_confirm = true;
	old_dir_ = new_dir_;
	if (ev.rcon_triggered)
	{
		ev.rcon_triggered = 0;
		sp_state = state_go_to_charger;
		stateInit(sp_state);
	}
	else if (!reach_home_point_ && getPosition().toCell() == go_home_path_algorithm_->getCurrentHomePoint().home_point.toCell())
	{
		// Reach home cell!!
		if (go_home_path_algorithm_->getCurrentHomePoint().have_seen_charger)
		{
			ROS_INFO("%s %d: Reach home cell (%d, %d).", __FUNCTION__, __LINE__,
					 go_home_path_algorithm_->getCurrentHomePoint().home_point.toCell().x,
					 go_home_path_algorithm_->getCurrentHomePoint().home_point.toCell().y);
			sp_state = state_go_to_charger;
			stateInit(sp_state);
		}
		else
		{
			ROS_INFO("%s %d: Reach home cell (%d, %d) but do not go to charger.", __FUNCTION__, __LINE__,
					 go_home_path_algorithm_->getCurrentHomePoint().home_point.toCell().x,
					 go_home_path_algorithm_->getCurrentHomePoint().home_point.toCell().y);
			sp_state = nullptr;
		}
		reach_home_point_ = true;
	}
	else if (go_home_path_algorithm_->generatePath(map, getPosition(),old_dir_, plan_path_))
	{
		// New path to home cell is generated.
		new_dir_ = (MapDirection)plan_path_.front().th;
		plan_path_.pop_front();
		go_home_path_algorithm_->displayCellPath(pointsGenerateCells(plan_path_));
		robot::instance()->pubCleanMapMarkers(clean_map_, pointsGenerateCells(plan_path_));
		reach_home_point_ = false;
	}
	else
	{
		// No more paths to home cells.
		PP_INFO();
		sp_state = nullptr;
	}
	return state_confirm;
}

void ACleanMode::setRconPos(float cd,float dist)
{
//	float yaw = robot::instance()->getWorldPoseAngle()/10.0;
//	float wpx = cosf( (float)ranged_angle((yaw+cd)*10)/10.0 * PI/180.0 )*dist+ robot::instance()->getWorldPoseX();
//	float wpy = sinf( (float)ranged_angle((yaw+cd)*10)/10.0 * PI/180.0 )*dist+ robot::instance()->getWorldPoseY();
//	charger_pos_ = {(int32_t)(wpx*1000/CELL_SIZE), (int32_t)(wpy*1000/CELL_SIZE),(int16_t)0};
//	if(found_charger_)
//		g_homes.push_back(charger_pos_);
//	ROS_INFO("%s,%d:rcon value \033[32m0x%x\033[0m,charger direction \033[32m%f\033[0m,cureent direction \033[32m%f\033[0m,distance \033[32m%f\033[0m,world pos(\033[32m%f,%f\033[0m), cell pos(\033[32m%hd,%hd\033[0m)",__FUNCTION__,__LINE__,rcon_status_&RconAll_Home_T,cd,yaw,dist,wpx,wpy,charger_pos_.x,charger_pos_.y);

}

bool ACleanMode::estimateChargerPos(uint32_t rcon_value)
{
	if(!(rcon_value & RconAll_Home_T)){
		return false;
	}
	enum {flfr,frfr2,flfl2,fl2l,fr2r,bll,brr,fl2,fr2,l,r,bl,br};
	static int8_t cnt[13]={0,0,0,0,0,0,0,0,0,0,0,0,0};
	float cd = 0.0;//charger direction corrding to robot,in degrees
	float dist = 0.0;
	float len = 0.0;
	const int MAX_CNT = 2;
	const float DETECT_RANGE_MAX = 0.63;
	const float DETECT_RANGE_MIN = 0.33;

	/*-- here we only detect top signal from charge stub --*/
	//ROS_INFO("%s,%d,rcon_value 0x%x",__FUNCTION__,__LINE__,rcon_value & RconAll_Home_T);
	if( (rcon_value & RconFR_HomeT) || (rcon_value & RconFL_HomeT) ){ //fl & fr sensor
		if((cnt[flfr]++) >= MAX_CNT){
			cnt[flfr] = 0;
			cd = 0;
		}
		else{
			return false;
		}
	}
	else if( (rcon_value & RconFL_HomeT ) && (rcon_value & RconFL_HomeT)  && !(rcon_value & RconAll_R_HomeT)){//fl & fl2 sensor
		if((cnt[flfl2]++) >= MAX_CNT){
			cnt[flfl2] = 0;
			cd = 20;
		}
		else{
			return false;
		}
	}
	else if( (rcon_value & RconFR_HomeT ) && (rcon_value & RconFR2_HomeT) && !(rcon_value & RconAll_L_HomeT)){//fr & fr2 sensor
		if((cnt[frfr2]++) >= MAX_CNT){
			cnt[frfr2] = 0;
			cd = -20;
		}
		else{
			return false;
		}
	}
	else if( (rcon_value & RconFL2_HomeT) && !(rcon_value & RconAll_R_HomeT) //fl2 sensor
				&& !(rcon_value & RconBL_HomeT) )//to avoid charger signal reflection from other flat
	{
		if((cnt[fl2]++) >= MAX_CNT){
			cnt[fl2] = 0;
			cd = 43.0;
		}
		else{
			return false;
		}
	}
	else if( (rcon_value & RconFR2_HomeT) && !(rcon_value & RconAll_L_HomeT) //fr2 sensor
				&& !(rcon_value & RconBR_HomeT) ){//to avoid charger signal reflection from other flat
		if((cnt[fr2]++) >= MAX_CNT){
			cnt[fr2] = 0;
			cd = -43.0;
		}
		else{
			return false;
		}
	}

	else if( (rcon_value & RconL_HomeT) && (rcon_value & RconFL2_HomeT) && !(rcon_value & RconAll_R_HomeT) ){//fl2 & l sensor
		if((cnt[fl2l]++) >= MAX_CNT){
			cnt[fl2l] = 0;
			cd = 65;
		}
		else{
			return false;
		}
	}
	else if( (rcon_value & RconR_HomeT) && (rcon_value & RconR_HomeT) && !(rcon_value & RconAll_L_HomeT) ){//fr2 & r sensor
		if((cnt[fr2r]++) >= MAX_CNT){
			cnt[fr2r]=0;
			cd = -65;
		}
		else{
			return false;
		}
	}
	else if( (rcon_value & RconL_HomeT) && !(rcon_value & RconAll_R_HomeT) ){//l sensor
		if((cnt[l]++) >= MAX_CNT){
			cnt[l] = 0;
			cd = 85;
		}
		else{
			return false;
		}
	}
	else if( (rcon_value & RconR_HomeT) && !(rcon_value & RconAll_L_HomeT) ){//r sensor
		if((cnt[r]++) >= MAX_CNT){
			cnt[r]=0;
			cd = -85;
		}
		else{
			return false;
		}
	}
	else if( (rcon_value & RconBL_HomeT) && (rcon_value & RconL_HomeT) && !(rcon_value & RconAll_R_HomeT)){//l & bl sensor
		if((cnt[bll]++) >= MAX_CNT){
			cnt[bll] = 0;
			cd = 110;
		}
		else{
			return false;
		}
	}
	else if( (rcon_value & RconBR_HomeT) && (rcon_value & RconR_HomeT) && !(rcon_value & RconAll_L_HomeT)){//r & br sensor
		if((cnt[brr]++) >= MAX_CNT){
			cnt[brr] = 0;
			cd = -110;
		}
		else{
			return false;
		}
	}
	else if( (rcon_value & RconBL_HomeT) && !(rcon_value & RconAll_R_HomeT) ){//bl sensor
		if((cnt[bl]++) >= MAX_CNT){
			cnt[bl] = 0;
			cd = 133;
		}
		else{
			return false;
		}
	}
	else if( (rcon_value & RconBR_HomeT) && !(rcon_value & RconAll_L_HomeT) ){//br sensor
		if((cnt[br]++) >= MAX_CNT){
			cnt[br] = 0;
			cd = -133;
		}
		else{
			return false;
		}
	}
	else{
		return false;
	}
	memset(cnt,0,sizeof(int8_t)*13);
	bool scan_allow = robot::instance()->isScanAllow();
	bool lidar_new = lidar.lidarCheckFresh(0.300,1);
	if( scan_allow && lidar_new ){
		int count = 0;
		double sum = 0.0;
		for(int i = cd -1;i<=cd +1;i++){//for calculate avarage distance
			dist = lidar.getLidarDistance(180+i);
			if(dist <= DETECT_RANGE_MAX && dist >= DETECT_RANGE_MIN){
				sum += dist;
				count++;
			}
		}
		if(count != 0){
			dist = sum /(count*1.0);
			if(fabs(cd)  == 43.0){//it's easy to make mistake in this angle ,which robot was almost paralled to the charger station ,so we add 0.1 to avoid mistake.
				if( dist >= (DETECT_RANGE_MIN + 0.15) ){
					setRconPos(cd,dist);
					ROS_INFO("\033[42;37m%s,%d,cd %f, rcon_state = 0x%x,dist = %f\033[0m",__FUNCTION__,__LINE__,cd,rcon_value,dist);
					should_mark_temp_charger_ = true;
					found_temp_charger_ = true;
					return false;
				}
			}
			else{
				if( dist > DETECT_RANGE_MAX || dist < DETECT_RANGE_MIN){
					ROS_INFO("\033[42;37m%s,%d,cd %f, rcon_state = 0x%x, distrance too far or too near\033[0m",__FUNCTION__,__LINE__,cd,rcon_value);
					return false;
				}
			}
		}
		else
			return false;
	} else{
		ROS_INFO("\033[42;37m%s,%d,cd %f, rcon_state = 0x%x,scan_allow = %d ,lidar_new = %d\033[0m",__FUNCTION__,__LINE__,cd,rcon_value,scan_allow,lidar_new);
		return false;
	}
	found_charger_ = true;
	found_temp_charger_ = false;
	should_mark_charger_ = true;
	setRconPos(cd,dist);
	return true;
}

Cells ACleanMode::pointsGenerateCells(Points &targets)
{
//	displayCellPath(targets);
	Cells path{};
	for(const Point32_t& point : targets) {
		path.push_back(point.toCell());
	}
	return path;
}

