//
// Created by lsy563193 on 17-12-3.
//

#include <mode.hpp>
#include <mathematics.h>
#include <event_manager.h>
#include <map.h>
#include "dev.h"
#include <robot_timer.h>
#include <robot.hpp>
#include <slam.h>
#include "error.h"

ACleanMode::ACleanMode()
{

	scanLinear_sub_ = clean_nh_.subscribe("scanLinear", 1, &Lidar::scanLinearCb, &lidar);
	scanCompensate_sub_ = clean_nh_.subscribe("scanCompensate", 1, &Lidar::scanCompensateCb, &lidar);
	lidarPoint_sub_ = clean_nh_.subscribe("lidarPoint", 1, &Lidar::lidarPointCb, &lidar);
	map_sub_ = clean_nh_.subscribe("/map", 1, &Slam::mapCb, &slam);


	send_clean_map_marker_pub_ = clean_nh_.advertise<visualization_msgs::Marker>("clean_map_markers", 1);
	fit_line_marker_pub_ = clean_nh_.advertise<visualization_msgs::Marker>("fit_line_marker", 1);
	line_marker_pub_ = clean_nh_.advertise<visualization_msgs::Marker>("line_marker", 1);
	line_marker_pub2_ = clean_nh_.advertise<visualization_msgs::Marker>("line_marker2", 1);

	event_manager_register_handler(this);
	event_manager_set_enable(true);
	IMoveType::sp_mode_ = this;
	sp_state->setMode(this);
	ev.key_clean_pressed = false;
	sp_state = state_init;
	sp_state->init();
	action_i_ = ac_open_gyro;
	genNextAction();
	robot_timer.initWorkTimer();
	key.resetPressStatus();

	c_rcon.resetStatus();
	robot::instance()->initOdomPosition();

}

ACleanMode::~ACleanMode() {
	scanLinear_sub_.shutdown();
	scanCompensate_sub_.shutdown();
	lidarPoint_sub_.shutdown();
	map_sub_.shutdown();

	send_clean_map_marker_pub_.shutdown();
	fit_line_marker_pub_.shutdown();
	line_marker_pub_.shutdown();
	line_marker_pub2_.shutdown();
	clean_nh_.shutdown();

	IMoveType::sp_mode_ = nullptr;
	sp_state = nullptr;
	event_manager_set_enable(false);
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
		speaker.play(VOICE_ERROR_LIFT_UP_CLEANING_STOP, false);
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

	auto cleaned_count = clean_map_.getCleanedArea();
	auto map_area = cleaned_count * CELL_SIZE * CELL_SIZE;
	ROS_INFO("%s %d: Cleaned area = \033[32m%.2fm2\033[0m, cleaning time: \033[32m%d(s) %.2f(min)\033[0m, cleaning speed: \033[32m%.2f(m2/min)\033[0m.",
			 __FUNCTION__, __LINE__, map_area, robot_timer.getWorkTime(),
			 static_cast<float>(robot_timer.getWorkTime()) / 60, map_area / (static_cast<float>(robot_timer.getWorkTime()) / 60));
}

void ACleanMode::pubFitLineMarker(visualization_msgs::Marker fit_line_marker)
{
	fit_line_marker_pub_.publish(fit_line_marker);
}

void ACleanMode::visualizeMarkerInit()
{
	clean_markers_.points.clear();

	geometry_msgs::Point m_points_{};
	clean_markers_.points.push_back(m_points_);

	clean_map_markers_.ns = "cleaning_grid_map";
	clean_map_markers_.id = 1;
	clean_map_markers_.type = visualization_msgs::Marker::POINTS;
	clean_map_markers_.action= visualization_msgs::Marker::ADD;
	clean_map_markers_.lifetime=ros::Duration(0);
	clean_map_markers_.scale.x = 0.1;
	clean_map_markers_.scale.y = 0.1;
	clean_map_markers_.header.frame_id = "/map";
	clean_map_markers_.points.clear();
	clean_map_markers_.colors.clear();
}

void ACleanMode::setCleanMapMarkers(int16_t x, int16_t y, CellState type)
{
	geometry_msgs::Point m_points_;
	m_points_.x = 0.0;
	m_points_.y = 0.0;
	m_points_.z = 0.0;
	std_msgs::ColorRGBA color_;
	color_.a = 0.7;
	m_points_.x = x * CELL_SIZE ;
	m_points_.y = y * CELL_SIZE ;
	m_points_.z = 0;
	if (type == CLEANED)
	{
		// Green
		if(y%2==0)
		{
			color_.r = 0.0;
			color_.g = 0.5;
			color_.b = 0.0;
		}
		else{
			color_.r = 0.0;
			color_.g = 1.0;
			color_.b = 0.0;
		}
	}
	else if (type == BLOCKED_FW)
	{
		color_.r = 0.2;
		color_.g = 0.1;
		color_.b = 0.5;
	}
	else if (type == BLOCKED_BUMPER)
	{
		// Red
		color_.r = 1.0;
		color_.g = 0.0;
		color_.b = 0.0;
	}
	else if (type == BLOCKED_CLIFF)
	{
		// Magenta
		color_.r = 1.0;
		color_.g = 0.0;
		color_.b = 1.0;
	}
	else if (type == BLOCKED_RCON)
	{
		// White
		color_.r = 1.0;
		color_.g = 1.0;
		color_.b = 1.0;
	}
	else if (type == BLOCKED_LIDAR)
	{
		//Blue
		color_.r = 0.0;
		color_.g = 0.0;
		color_.b = 1.0;
	}
	else if (type == BLOCKED_TILT)
	{
		// Gray
		color_.r = 0.5;
		color_.g = 0.5;
		color_.b = 0.5;
	}
	else if (type == SLAM_MAP_BLOCKED)
	{
		color_.r = 0.75;
		color_.g = 0.33;
		color_.b = 0.50;
	}
	else if (type == TARGET)// Next point
	{
		// Yellow
		color_.r = 1.0;
		color_.g = 1.0;
		color_.b = 0.0;
	}
	else if (type == TARGET_CLEAN)// Target point
	{
		// Cyan
		color_.r = 0.0;
		color_.g = 1.0;
		color_.b = 1.0;
	}
	else if (type == BLOCKED_TILT)
	{
		// Gray
		color_.r = 0.5;
		color_.g = 0.5;
		color_.b = 0.5;
	}
	else if (type == BLOCKED_SLIP)
	{
		// i dont know what color it is..
		color_.r = 0.7;
		color_.g = 0.7;
		color_.b = 0.2;
	}
	clean_map_markers_.points.push_back(m_points_);
	clean_map_markers_.colors.push_back(color_);
}

void ACleanMode::pubCleanMapMarkers(GridMap& map, const std::deque<Cell_t>& path)
{

	if (path.empty())
		return;

	visualizeMarkerInit();
	int16_t x, y, x_min, x_max, y_min, y_max;
	CellState cell_state;
	Cell_t next = path.front();
	Cell_t target = path.back();
	map.getMapRange(CLEAN_MAP, &x_min, &x_max, &y_min, &y_max);

	if (next.x == SHRT_MIN )
		next.x = x_min;
	else if (next.x == SHRT_MAX)
		next.x = x_max;

	for (x = x_min; x <= x_max; x++)
	{
		for (y = y_min; y <= y_max; y++)
		{
			if (x == target.x && y == target.y)
				setCleanMapMarkers(x, y, TARGET_CLEAN);
			else if (x == next.x && y == next.y)
				setCleanMapMarkers(x, y, TARGET);
			else
			{
				cell_state = map.getCell(CLEAN_MAP, x, y);
				if (cell_state > UNCLEAN && cell_state < BLOCKED_BOUNDARY )
					setCleanMapMarkers(x, y, cell_state);
			}
		}
	}
	if (!path.empty())
	{
		for (auto it = path.begin(); it->x != path.back().x || it->y != path.back().y; it++)
			setCleanMapMarkers(it->x, it->y, TARGET);

		setCleanMapMarkers(path.back().x, path.back().y, TARGET_CLEAN);
	}

	clean_map_markers_.header.stamp = ros::Time::now();
	send_clean_map_marker_pub_.publish(clean_map_markers_);
	clean_map_markers_.points.clear();
	clean_map_markers_.colors.clear();
}
void ACleanMode::pubLineMarker(const std::vector<LineABC> *lines)
{
	visualization_msgs::Marker line_marker;
	line_marker.ns = "line_marker_2";
	line_marker.id = 0;
	line_marker.type = visualization_msgs::Marker::LINE_LIST;
	line_marker.action= 0;//add
	line_marker.lifetime=ros::Duration(0);
	line_marker.scale.x = 0.05;
	//line_marker.scale.y = 0.05;
	//line_marker.scale.z = 0.05;
	line_marker.color.r = 0.5;
	line_marker.color.g = 1.0;
	line_marker.color.b = 0.2;
	line_marker.color.a = 1.0;
	line_marker.header.frame_id = "/map";
	line_marker.header.stamp = ros::Time::now();
	geometry_msgs::Point point1;
	point1.z = 0.0;
	geometry_msgs::Point point2;
	point2.z = 0.0;
	line_marker.points.clear();
	std::vector<LineABC>::const_iterator it;
	if(!lines->empty() && lines->size() >= 2){
		for(it = lines->cbegin(); it != lines->cend();it++){
			point1.x = it->x1;
			point1.y = it->y1;
			point2.x = it->x2;
			point2.y = it->y2;
			line_marker.points.push_back(point1);
			line_marker.points.push_back(point2);
		}
		line_marker_pub2_.publish(line_marker);
		line_marker.points.clear();
	}
	/*
	else{
		line_marker.points.clear();
		line_marker_pub2.publish(line_marker);
	}
	*/

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
//	INFO_BLUE("ACleanMode::isExit()");
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
	if (ev.fatal_quit || sp_action_->isExit())
	{
		ROS_WARN("%s %d: Exit for ev.fatal_quit or sp_action_->isExit()", __FUNCTION__, __LINE__);
		setNextMode(md_idle);
		return true;
	}

	if(ev.key_clean_pressed || ev.key_long_pressed){
		ev.key_clean_pressed = false;
		ROS_WARN("%s %d: Exit for remote key or clean key or long press clean key.", __FUNCTION__, __LINE__);
		setNextMode(md_idle);
		return true;
	}

	if(ev.cliff_all_triggered) {
		ev.cliff_all_triggered = false;
		ROS_WARN("%s %d: Exit for cliff all.", __FUNCTION__, __LINE__);
		setNextMode(md_idle);
		return true;
	}

	if (charger.isDirected())
	{
		ROS_WARN("%s %d: Exit for directly charge.", __FUNCTION__, __LINE__);
		setNextMode(md_charge);
		return true;
	}

	return false;
}

bool ACleanMode::isUpdateFinish() {
	if (sp_state->isSwitchByEvent())
		return sp_state == nullptr;

	if (sp_action_ != nullptr && !sp_action_->isFinish())
		return true;
//	sp_action_.reset();//for call ~constitution;

	if (!sp_state->updateAction()) {
		sp_state->switchState();
		return sp_state == nullptr;
	}
	return true;
}

State* ACleanMode::updateState()
{
	while (!isUpdateFinish() && ros::ok());
}

bool ACleanMode::isFinish()
{
//	printf("\033[1;40;32m\n====================================Start update state===============================\n\033[0m");
	updateState();
//	printf("\033[1;40;34m\n=====================================End update state================================\n\033[0m");

	if(sp_state == nullptr)
	{
		setNextModeDefault();
		return true;
	}
	return false;
}

void ACleanMode::genNextAction()
{
	INFO_GREEN(before genNextAction);

	switch (action_i_) {
		case ac_null :
//			ROS_INFO_FL();
			sp_action_.reset();
			break;
		case ac_open_gyro :
//			ROS_INFO_FL();
			sp_action_.reset(new ActionOpenGyro);
			break;
		case ac_back_form_charger :
//			ROS_INFO_FL();
			sp_action_.reset(new ActionBackFromCharger);
			break;
		case ac_open_lidar :
//			ROS_INFO_FL();
			sp_action_.reset(new ActionOpenLidar);
			break;
		case ac_align :
//			ROS_INFO_FL();
			sp_action_.reset(new ActionAlign);
			break;
		case ac_open_slam :
//			ROS_INFO_FL();
			sp_action_.reset(new ActionOpenSlam);
			break;
		case ac_pause :
//			ROS_INFO_FL();
			sp_action_.reset(new ActionPause);
			break;
		case ac_linear :
//			ROS_INFO_FL();
			sp_action_.reset(new MoveTypeLinear);
			break;
		case ac_follow_wall_left  :
		case ac_follow_wall_right :
			ROS_INFO_FL();
			sp_action_.reset(new MoveTypeFollowWall(action_i_ == ac_follow_wall_left));
			break;
		case ac_go_to_charger :
//			ROS_INFO_FL();
			sp_action_.reset(new MoveTypeGoToCharger);
			break;
		case ac_exception_resume :
//			ROS_INFO_FL();
			sp_action_.reset(new MovementExceptionResume);
			break;
		case ac_charge :
//			ROS_INFO_FL();
			sp_action_.reset(new MovementCharge);
			break;
		case ac_check_bumper :
//			ROS_INFO_FL();
			sp_action_.reset(new ActionCheckBumper);
			break;
		case ac_bumper_hit_test :
//			ROS_INFO_FL();
			sp_action_.reset(new MoveTypeBumperHitTest);
			break;
		case ac_check_vacuum :
//			ROS_INFO_FL();
			sp_action_.reset(new ActionCheckVacuum);
			break;
		case ac_movement_direct_go :
//			ROS_INFO_FL();
			sp_action_.reset(new MovementDirectGo);
			break;
	}
	INFO_GREEN(after genNextAction);
}

void ACleanMode::setRconPos(Point_t pos)
{
		charger_pos_ = pos;
}

bool ACleanMode::moveTypeFollowWallIsFinish(MoveTypeFollowWall *p_mt)
{
	return false;
}

void ACleanMode::moveTypeFollowWallSaveBlocks()
{
	clean_map_.saveBlocks(action_i_ == ac_linear, isStateClean());
}

bool ACleanMode::moveTypeLinearIsFinish(MoveTypeLinear *p_mt)
{
	return p_mt->isPoseReach() || p_mt->isPassTargetStop(new_dir_);
}

void ACleanMode::moveTypeLinearSaveBlocks()
{
	clean_map_.saveBlocks(action_i_ == ac_linear, sp_state == state_clean);
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
	for(const Point_t& point : targets) {
		path.push_back(point.toCell());
	}
	return path;
}

void ACleanMode::setHomePoint()
{
	// Set home cell.
	Points::iterator home_point_it = home_points_.begin();
	for (;home_point_it != home_points_.end(); home_point_it++)
	{
		if (home_point_it->toCell() == getPosition().toCell())
		{
			ROS_INFO("%s %d: Home point(%d, %d) exists.",
					 __FUNCTION__, __LINE__, home_point_it->toCell().x, home_point_it->toCell().y);
			return;
		}
	}

	if (home_points_.size() >= HOME_POINTS_SIZE)
		// Drop the oldest home point to keep the home_points_.size() is within HOME_POINTS_SIZE.
		home_points_.pop_back();

	home_points_.push_front(getPosition());
	std::string msg = "Update Home_points_: ";
	for (auto it : home_points_)
		msg += "(" + std::to_string(it.toCell().x) + ", " + std::to_string(it.toCell().y) + "),";
	ROS_INFO("%s %d: %s", __FUNCTION__, __LINE__, msg.c_str());
}

bool ACleanMode::isRemoteGoHomePoint()
{
	return remote_go_home_point;
}

// ------------------Handlers--------------------------
void ACleanMode::remoteHome(bool state_now, bool state_last)
{
	if (sp_state == state_clean || sp_state == state_pause)
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

void ACleanMode::cliffAll(bool state_now, bool state_last) {
	ROS_WARN("%s %d: Cliff all.", __FUNCTION__, __LINE__);
	ev.cliff_all_triggered = true;
}
// ------------------Handlers end--------------------------

bool ACleanMode::checkEnterExceptionResumeState()
{
	if (isExceptionTriggered()) {
		ROS_WARN("%s %d: Exception triggered!", __FUNCTION__, __LINE__);
		sp_action_.reset();
		sp_saved_states.push_back(sp_state);
		sp_state = state_exception_resume;
		sp_state->init();
		return true;
	}

	return false;
}

bool ACleanMode::checkEnterNullState()
{
	if (ev.key_clean_pressed)
	{
		ev.key_clean_pressed = false;
		action_i_ = ac_null;
		sp_action_.reset();
		sp_state = nullptr;
		return true;
	}

	return false;
}

// ------------------State init--------------------
bool ACleanMode::isSwitchByEventInStateInit() {
	return checkEnterNullState();
}

bool ACleanMode::updateActionInStateInit() {
	if (action_i_ == ac_null)
		action_i_ = ac_open_gyro;
	else if (action_i_ == ac_open_gyro) {
		vacuum.setLastMode();
		brush.normalOperate();
		action_i_ = ac_open_lidar;
	}
	else if (action_i_ == ac_open_lidar)
		action_i_ = ac_open_slam;
	else // action_open_slam
		return false;

	genNextAction();
	return true;
}

void ACleanMode::switchInStateInit() {
//	if(action_i_ == ac_open_slam)
	action_i_ = ac_null;
	sp_action_ = nullptr;
	sp_state = state_clean;
	sp_state->init();
}

// ------------------State clean--------------------
bool ACleanMode::isSwitchByEventInStateClean() {
	return checkEnterNullState() || checkEnterGoHomePointState();
}


void ACleanMode::switchInStateClean() {
//    checkEnterNullState()
//	if(action_i_ == ac_open_slam)
	action_i_ = ac_null;
	sp_action_.reset();
	sp_state = nullptr;
}

// ------------------State go home point--------------------
bool ACleanMode::checkEnterGoHomePointState()
{
	if (ev.remote_home || ev.battery_home)
	{
		if (ev.remote_home)
			remote_go_home_point = true;
		sp_action_.reset();
		sp_state = state_go_home_point;
		sp_state->init();
		speaker.play(VOICE_BACK_TO_CHARGER, true);
		if (go_home_path_algorithm_ == nullptr)
			go_home_path_algorithm_.reset(new GoHomePathAlgorithm(clean_map_, home_points_, start_point_));
		return true;
	}

	return false;
}

bool ACleanMode::isSwitchByEventInStateGoHomePoint()
{
	return checkEnterExceptionResumeState();
}

bool ACleanMode::updateActionInStateGoHomePoint()
{
	bool update_finish;
	sp_action_.reset();//to mark in destructor
	old_dir_ = new_dir_;

//	ROS_INFO("%s %d: curr(%d, %d), current home point(%d, %d).", __FUNCTION__, __LINE__,
//			 getPosition().toCell().x, getPosition().toCell().y,
//			 go_home_path_algorithm_->getCurrentHomePoint().toCell().x,
//			 go_home_path_algorithm_->getCurrentHomePoint().toCell().y);
	if (ev.rcon_triggered)
	{
		// Directly switch to state go to charger.
		ROS_INFO("%s %d: Rcon T signal triggered and switch to state go to charger.", __FUNCTION__, __LINE__);
		should_go_to_charger_ = true;
		ev.rcon_triggered = 0;
		update_finish = false;
	}
	else if (go_home_path_algorithm_->reachTarget(should_go_to_charger_))
	{
		update_finish = false;
		home_points_ = go_home_path_algorithm_->getRestHomePoints();
	}
	else if (go_home_path_algorithm_->generatePath(clean_map_, getPosition(),old_dir_, plan_path_))
	{
		// New path to home cell is generated.
		new_dir_ = plan_path_.front().th;
		plan_path_.pop_front();
		go_home_path_algorithm_->displayCellPath(pointsGenerateCells(plan_path_));
		pubCleanMapMarkers(clean_map_, pointsGenerateCells(plan_path_));
		should_go_to_charger_ = false;
		action_i_ = ac_linear;
		genNextAction();
		update_finish = true;
		home_points_ = go_home_path_algorithm_->getRestHomePoints();
	}else
		// path is empty.
		update_finish = false;


	return update_finish;
}

void ACleanMode::switchInStateGoHomePoint()
{
	if (should_go_to_charger_)
	{
		should_go_to_charger_ = false;
		sp_state = state_go_to_charger;
		sp_state->init();
		sp_action_.reset();
	}
	else // path is empty.
	{
		ROS_INFO("%s %d, No more home point, finish cleaning.", __FUNCTION__, __LINE__);
		sp_state = nullptr;
	}
}

// ------------------State go to charger--------------------
bool ACleanMode::checkEnterGoCharger()
{
	ev.rcon_triggered = c_rcon.getForwardTop();
	if (ev.rcon_triggered) {
		ev.rcon_triggered= false;
		ROS_WARN("%s,%d:find charge success,convert to go to charge state", __func__, __LINE__);
		sp_state = state_go_to_charger;
		sp_state->init();
		action_i_ = ac_go_to_charger;
		genNextAction();
		return true;
	}
	return false;
}

bool ACleanMode::updateActionInStateGoToCharger()
{
	if (sp_action_ == nullptr)
	{
		action_i_ = ac_go_to_charger;
		genNextAction();
		return true;
	}

	// Go to charger finish, succeeded or failed.
	return false;
}

void ACleanMode::switchInStateGoToCharger() {
	if (charger.isOnStub()) {
		// Reach charger and exit clean mode.
		sp_state = nullptr;
	} else {
		ROS_INFO("%s %d: Failed to go to charger, resume state go home point.", __FUNCTION__, __LINE__);
		sp_state = state_go_home_point;
		sp_state->init();
	}
}

// ------------------State spot--------------------
bool ACleanMode::updateActionInStateSpot() {
	old_dir_ = new_dir_;
	ROS_ERROR("old_dir_(%d)", old_dir_);
	auto cur_point = getPosition();
	ROS_INFO("\033[32m plan_path size(%d), front (%d,%d),cur point:(%d,%d)\033[0m",plan_path_.size(),
				plan_path_.front().toCell().x,plan_path_.front().toCell().y,cur_point.toCell().x,cur_point.toCell().y);
	sp_action_.reset();// to mark in destructor
	if (clean_path_algorithm_->generatePath(clean_map_, cur_point, old_dir_, plan_path_)) {
		new_dir_ = plan_path_.front().th;
		ROS_ERROR("new_dir_(%d)", new_dir_);
//		PP_INFO();
		clean_path_algorithm_->displayCellPath(pointsGenerateCells(plan_path_));
		plan_path_.pop_front();
		action_i_ = ac_linear;
		genNextAction();
		return true;
	}
	else {
		return false;
	}
}

bool ACleanMode::isSwitchByEventInStateSpot()
{
	return false;
}

// ------------------State exception resume--------------
bool ACleanMode::isSwitchByEventInStateExceptionResume()
{
	return checkEnterNullState();
}

bool ACleanMode::updateActionInStateExceptionResume()
{
	if (isExceptionTriggered())
	{
		sp_action_.reset();
		action_i_ = ac_exception_resume;
		genNextAction();
		return true;
	}
	return false;
}

void ACleanMode::switchInStateExceptionResume()
{
	if (!isExceptionTriggered())
	{
		ROS_INFO("%s %d: Resume to previous state", __FUNCTION__, __LINE__);
		sp_action_.reset();
		sp_state = sp_saved_states.back();
		sp_saved_states.pop_back();
	}
}

// ------------------State exploration--------------
bool ACleanMode::isSwitchByEventInStateExploration() {
	return checkEnterGoCharger();
}

bool ACleanMode::updateActionInStateExploration() {
	PP_INFO();
	old_dir_ = new_dir_;
	ROS_WARN("old_dir_(%d)", old_dir_);
	plan_path_.clear();
	sp_action_.reset();//to mark in constructor
	if (clean_path_algorithm_->generatePath(clean_map_, getPosition(), old_dir_, plan_path_)) {
		action_i_ = ac_linear;
		new_dir_ = plan_path_.front().th;
		ROS_WARN("new_dir_(%d)", new_dir_);
		plan_path_.pop_front();
		clean_path_algorithm_->displayCellPath(pointsGenerateCells(plan_path_));
		pubCleanMapMarkers(clean_map_, pointsGenerateCells(plan_path_));
		genNextAction();
		return true;
	}
	else {
		ROS_WARN("%s,%d:exploration finish,did not find charge", __func__, __LINE__);
		action_i_ = ac_null;
	}
	return false;
}

void ACleanMode::switchInStateExploration() {
	PP_INFO();
	if (clean_path_algorithm_->checkTrapped(clean_map_, getPosition().toCell())) {
		ROS_WARN("%s,%d: enter state trapped",__FUNCTION__,__LINE__);
		sp_saved_states.push_back(sp_state);
		sp_state = state_trapped;
	}
	else{
		auto curr = getPosition();
		start_point_.th = curr.th;
		sp_state = state_go_home_point;
		speaker.play(VOICE_BACK_TO_CHARGER, true);
		if (go_home_path_algorithm_ == nullptr)
			go_home_path_algorithm_.reset(new GoHomePathAlgorithm(clean_map_, home_points_, start_point_));
	}
	action_i_ = ac_null;
	sp_state->init();
	genNextAction();
}

// ------------------State trapped------------------

bool ACleanMode::isSwitchByEventInStateTrapped()
{
	return checkEnterExceptionResumeState();
}

bool ACleanMode::updateActionInStateTrapped()
{
	PP_INFO();

	sp_action_.reset();// to mark in destructor
	action_i_ = ac_follow_wall_left;
	genNextAction();

	if (robot_timer.trapTimeout(ESCAPE_TRAPPED_TIME)) {
		action_i_ = ac_null;
		genNextAction();
		trapped_time_out_ = true;
		return false;
	}
	else if (!clean_path_algorithm_->checkTrapped(clean_map_, getPosition().toCell())) {
		action_i_ = ac_null;
		genNextAction();
		return false;
	}

	return true;
}

void ACleanMode::switchInStateTrapped()
{
	PP_INFO();
	if (trapped_time_out_) {
		trapped_time_out_ = false;
		ROS_WARN("%s %d: Escape trapped timeout!(%d)", __FUNCTION__, __LINE__, ESCAPE_TRAPPED_TIME);
		reach_cleaned_count_ = 0;
		sp_state = nullptr;
	}
	else/* if (escape_trapped_)*/ {
		ROS_WARN("%s %d: Escape trapped !", __FUNCTION__, __LINE__);
		reach_cleaned_count_ = 0;
//		sp_state = (sp_tmp_state == state_clean) ? state_clean : state_exploration;
		sp_state = sp_saved_states.back();
		sp_saved_states.pop_back();
		sp_state->init();
	}
}

