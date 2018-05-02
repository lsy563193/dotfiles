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
#include <infrared_display.hpp>
#include "error.h"

const double CHASE_X = 0.107;

extern bool g_pp_shutdown;
ros::Publisher ACleanMode::point_marker_pub_={};
ros::Publisher ACleanMode::line_marker_pub2_={};
bool ACleanMode::plan_activation_={};

ACleanMode::ACleanMode()
{
	lidar.init();
	scanLinear_sub_ = clean_nh_.subscribe("scanLinear", 1, &Lidar::scanLinearCb, &lidar);
	scanCompensate_sub_ = clean_nh_.subscribe("scanCompensate", 1, &Lidar::scanCompensateCb, &lidar);
	scanOriginal_sub_ = clean_nh_.subscribe("scanOriginal", 1, &ACleanMode::scanOriginalCb, this);
	lidarPoint_sub_ = clean_nh_.subscribe("lidarPoint", 1, &Lidar::lidarXYPointCb, &lidar);
	map_sub_ = clean_nh_.subscribe("/map", 1, &Slam::mapCb, &slam);

	tmp_target_pub_ = clean_nh_.advertise<visualization_msgs::Marker>("tmp_target", 1);
	point_marker_pub_ = clean_nh_.advertise<visualization_msgs::Marker>("point_marker", 1);
	send_clean_map_marker_pub_ = clean_nh_.advertise<visualization_msgs::Marker>("clean_map_markers", 1);
	fit_line_marker_pub_ = clean_nh_.advertise<visualization_msgs::Marker>("fit_line_marker", 1);
	line_marker_pub_ = clean_nh_.advertise<visualization_msgs::Marker>("line_marker", 1);
	line_marker_pub2_ = clean_nh_.advertise<visualization_msgs::Marker>("line_marker2", 1);

	lidar.slipCheckingCtrl(ON);
	event_manager_register_handler(this);
	event_manager_reset_status();
	event_manager_set_enable(true);
	serial.setWorkMode(WORK_MODE);
	IMoveType::sp_mode_ = this;
	APathAlgorithm::p_cm_ = this;
	State::sp_cm_ = this;
	if (robot::instance()->getR16WorkMode() == WORK_MODE || robot::instance()->getR16WorkMode() == IDLE_MODE ||
			robot::instance()->getR16WorkMode() == CHARGE_MODE)
	{
		sp_state = state_init.get();
		sp_state->init();
		action_i_ = ac_open_gyro;
		genNextAction();
	}
	robot_timer.initWorkTimer();
	key.resetPressStatus();
	time_gyro_dynamic_ = ros::Time::now().toSec();

	resetPosition();
	charger_pose_.clear();
	tmp_charger_pose_.clear();
	c_rcon.resetStatus();
	robot::instance()->initOdomPosition();
	s_wifi.resetReceivedWorkMode();
	if (error.get())
		error.clear(error.get(), true);
//	fw_map.reset(CLEAN_MAP);

//	// todo:debug
//	infrared_display.displayNormalMsg(8, 5555);
}

ACleanMode::~ACleanMode()
{
	tmp_target_pub_.shutdown();
	scanLinear_sub_.shutdown();
	scanCompensate_sub_.shutdown();
	scanOriginal_sub_.shutdown();
	lidarPoint_sub_.shutdown();
	map_sub_.shutdown();

	send_clean_map_marker_pub_.shutdown();
	point_marker_pub_.shutdown();
	fit_line_marker_pub_.shutdown();
	line_marker_pub_.shutdown();
	line_marker_pub2_.shutdown();
	clean_nh_.shutdown();

	IMoveType::sp_mode_ = nullptr;
	sp_state = nullptr;
	sp_action_.reset();
	event_manager_set_enable(false);
	if(!g_pp_shutdown){
		wheel.stop();
		brush.stop();
		vacuum.stop();
		water_tank.stop(WaterTank::operate_option::swing_motor_and_pump);
		lidar.motorCtrl(OFF);
		lidar.setScanOriginalReady(0);
		lidar.slipCheckingCtrl(OFF);

		robot::instance()->setBaselinkFrameType(ODOM_POSITION_ODOM_ANGLE);
		slam.stop();
		odom.setRadianOffset(0);

		if (next_mode_i_ == md_idle)
		{
			if (ev.fatal_quit)
			{
				if (switch_is_off_)
				{
					speaker.play(VOICE_CHECK_SWITCH, false);
					ROS_WARN("%s %d: Switch is not on. Stop cleaning.", __FUNCTION__, __LINE__);
				} else if (ev.cliff_all_triggered)
				{
					speaker.play(VOICE_ERROR_LIFT_UP, false);
					ROS_WARN("%s %d: Cliff all triggered. Stop cleaning.", __FUNCTION__, __LINE__);
				} else
					ROS_WARN("%s %d: fatal_quit is true. Stop cleaning.", __FUNCTION__, __LINE__);
			} else if (ev.key_clean_pressed || ev.key_long_pressed)
			{
				if (mode_i_ != cm_exploration && mode_i_ != cm_navigation)
					speaker.play(VOICE_CLEANING_FINISHED, false);
				ROS_WARN("%s %d: Finish cleaning for key_clean_pressed or key_long_pressed.", __FUNCTION__, __LINE__);
			} else if (mode_i_ == cm_exploration ||
					((mode_i_ == cm_navigation || mode_i_ == cm_wall_follow) && seen_charger_during_cleaning_))
			{
				speaker.play(VOICE_BACK_TO_CHARGER_FAILED, false);
				ROS_WARN("%s %d: Finish cleaning but failed to go to charger.", __FUNCTION__, __LINE__);
			} else if (mode_i_ == cm_navigation && (trapped_closed_or_isolate || trapped_time_out_))
			{
				speaker.play(VOICE_ROBOT_TRAPPED, false);
				trapped_closed_or_isolate = false;
				trapped_time_out_ = false;
				ROS_WARN("%s %d: Robot is trapped.Stop cleaning.", __FUNCTION__, __LINE__);
			} else if (mode_i_ == cm_navigation && moved_during_pause_)
			{
				speaker.play(VOICE_CLEANING_FINISHED, false);
				ROS_WARN("%s %d: Moved during pause. Stop cleaning.", __FUNCTION__, __LINE__);
			} else if (mode_i_ != cm_exploration && !seen_charger_during_cleaning_)
			{
				speaker.play(VOICE_CLEANING_FINISHED, false);
				ROS_WARN("%s %d: Finish cleaning.", __FUNCTION__, __LINE__);
			}
		}
		else if (next_mode_i_ == md_charge)
			ROS_WARN("%s %d: Charge detect. Stop cleaning.", __FUNCTION__, __LINE__);
		else if (next_mode_i_ == md_sleep)
		{
//			speaker.play(VOICE_CLEANING_FINISHED, false);
			ROS_WARN("%s %d: Pause timeout or long press in pause.", __FUNCTION__, __LINE__);
		}
		else
			ROS_WARN("%s %d: Entering other clean mode(%d), so do not play the finish voice.",
					 __FUNCTION__, __LINE__, next_mode_i_);
	}
	auto cleaned_count = clean_map_.getCleanedArea();
	auto map_area = cleaned_count * CELL_SIZE * CELL_SIZE;
	ROS_INFO("%s %d: Cleaned area = \033[32m%.2fm2\033[0m, cleaning time: \033[32m%d(s) %.2f(min)\033[0m, cleaning speed: \033[32m%.2f(m2/min)\033[0m.",
			 __FUNCTION__, __LINE__, map_area, robot_timer.getWorkTime(),
			 static_cast<float>(robot_timer.getWorkTime()) / 60, map_area / (static_cast<float>(robot_timer.getWorkTime()) / 60));
	brush.updateSideBrushTime(robot_timer.getWorkTime());
	brush.updateMainBrushTime(robot_timer.getWorkTime());
	if (isUsingDustBox())
		vacuum.updateFilterTime(robot_timer.getWorkTime());

	if (mode_i_ == cm_navigation)
	{
		// Upload clean record takes sometime, so upload the work mode first for better service on app.
		s_wifi.setWorkMode(next_mode_i_);
		s_wifi.taskPushBack(S_Wifi::ACT::ACT_UPLOAD_STATUS);

		// Place here to get map_area.
		time_t real_calendar_time;
		robot_timer.getRealCalendarTime(real_calendar_time);
		ROS_INFO("%s %d seconds from 1970' : %d, current calendar time: %s ", __FUNCTION__, __LINE__,
				 real_calendar_time, ctime(&real_calendar_time));
		robot::instance()->updateCleanRecord(static_cast<const uint32_t &>(real_calendar_time - robot_timer.getWorkTime())
											 , static_cast<const uint16_t &>(robot_timer.getWorkTime())
											 , static_cast<const uint16_t &>(map_area)
											 , clean_map_);
		s_wifi.taskPushBack(S_Wifi::ACT::ACT_UPLOAD_LAST_CLEANMAP);

	}
}

void ACleanMode::saveBlock(int block, int dir, std::function<Cells()> get_list)
{
	printf("curr(%d,%d),block(%d):", getPosition().toCell().x, getPosition().toCell().y, block);
	for(auto& d_cell : get_list())
	{
		Cell_t cell;
//		if(dir == MAP_ANY)
			cell = getPosition().getCenterRelative(d_cell.x * CELL_SIZE, d_cell.y * CELL_SIZE).toCell();
		printf("{%d,%d}->{%d,%d}\n", d_cell.x, d_cell.y, cell.x, cell.y);
//		else {
//			auto x = d_cell * cell_direction_[dir].x;
//			auto y = d_cell * cell_direction_[(dir+2)%4].y;
//			cell = getPosition().toCell() +  x + y;
//		}
		c_blocks.insert({block, cell});
	}
	printf("\n");
}

void ACleanMode::saveBlocks() {
//	PP_INFO();
	bool is_linear = action_i_== ac_linear;
	auto is_save_rcon = sp_state == state_clean.get();
	if (action_i_== ac_linear && is_save_rcon)
		saveBlock(BLOCKED_TMP_RCON, iterate_point_.dir, [&]() {
			auto rcon_trig = ev.rcon_status/*rcon_get_trig()*/;
			Cells d_cells;
			switch (c_rcon.convertToEnum(rcon_trig)) {
				case Rcon::left:
					d_cells = {{1, 2}, {1, 3}, {1, 4}, {2, 2}, {2, 3}, {2, 4}, {3, 2}, {3, 3}, {3, 4}};
					break;
				case Rcon::fl2:
					d_cells = {{2, 1}, {2, 2}, {2, 3}, {3, 1}, {3, 2}, {3, 3}, {4, 1}, {4, 2}, {4, 3}};
					break;
				case Rcon::fl:
				case Rcon::fr:
					d_cells = {{2, 0}, {3, 0}, {4, 0}, {2, -1}, {3, -1}, {4, -1}, {2, 1}, {3, 1}, {4, 1}};
					break;
				case Rcon::fr2:
//			dx = 1, dy = -2;
//			dx2 = 2, dy2 = -1;
					d_cells = {{2, -1}, {2, -2}, {2, -3}, {3, -1}, {3, -2}, {3, -3}, {4, -1}, {4, -2}, {4, -3}};
					break;
				case Rcon::right:
					d_cells = {{1, -2}, {1, -3}, {1, -4}, {2, -2}, {2, -3}, {2, -4}, {3, -2}, {3, -3}, {3, -4}};
					break;
			}
			return d_cells;
		});

	saveBlock(BLOCKED_BUMPER,iterate_point_.dir, [&]() {
		auto bumper_trig = ev.bumper_triggered/*bumper.getStatus()*/;
		Cells d_cells; // Direction indicator cells.
//		if ((bumper_trig & BLOCK_RIGHT) && (bumper_trig & BLOCK_LEFT))
		if (bumper_trig == BLOCK_ALL || bumper_trig == BLOCK_LIDAR_BUMPER)
			d_cells = {/*{2,-1},*/ {2, 0}/*, {2,1}*/};
		else if (bumper_trig & BLOCK_LEFT) {
			if (is_linear)
				d_cells = {{2, 1}/*, {2,2},{1,2}*/};
			else
				d_cells = {/*{2, 1},*//* {2,2}, */{1, 2}};
		}
		else if (bumper_trig & BLOCK_RIGHT) {
			if (is_linear)
				d_cells = {{2, -1}/*,{2,-2},{1,-2}*/};
			else
				d_cells = {/*{2,-1},*//*{2,-1},*/{1, -2}};
		}
		return d_cells;
	});

	saveBlock(BLOCKED_CLIFF,iterate_point_.dir, [&]() {
		auto cliff_trig = ev.cliff_triggered;
		Cells d_cells;
		if (cliff_trig & BLOCK_FRONT) {
			d_cells = {{2, -1}, {2, 0}, {2, 1}};
		}
		if (cliff_trig & BLOCK_LEFT) {
			d_cells = {{2, 1}, {2, 2}};
		}
		if (cliff_trig & BLOCK_RIGHT) {
			d_cells = {{2, -1}, {2, -2}};
		}
		return d_cells;
	});

	//save block for wheel cliff, but in case of incresing the map cost, it is same as the BOCKED_CLIFF, please check the log if it was triggered
	saveBlock(BLOCKED_CLIFF,iterate_point_.dir, [&]() {
//		auto wheel_cliff_trig = ev.left_wheel_cliff || ev.right_wheel_cliff;
		auto wheel_cliff_trig = is_wheel_cliff_triggered;
		Cells d_cells;
		if (wheel_cliff_trig) {
			d_cells = {{2, -1}, {2, 0}, {2, 1}, {2, 2}, {2, -2}};
		}
		return d_cells;
	});

	//save block for oc_brush_main, but in case of incresing the map cost, it is same as the BOCKED_CLIFF, please check the log if it was triggered
	saveBlock(BLOCKED_CLIFF,iterate_point_.dir, [&]() {
		Cells d_cells{};
		if (ev.oc_brush_main)
			d_cells = {{1,  1}, {1,  0}, {1,  -1}, {0,  1}, {0,  0}, {0,  -1}, {-1, 1}, {-1, 0}, {-1, -1}};
		return d_cells;

	});

	saveBlock(BLOCKED_SLIP,iterate_point_.dir, [&]() {
		Cells d_cells{};
		if (ev.robot_slip)
			d_cells = {{1,  1}, {1,  0}, {1,  -1}, {0,  1}, {0,  0}, {0,  -1}, {-1, 1}, {-1, 0}, {-1, -1}};
		return d_cells;

	});

	saveBlock(BLOCKED_TILT,iterate_point_.dir, [&]() {
		Cells d_cells;
		auto tilt_trig = ev.tilt_triggered;
/*		if (tilt_trig & TILT_LEFT)
			d_cells = {{2, 2}, {2, 1}, {2, 0}, {1, 2}, {1, 1}, {1, 0}, {0, 2}, {0, 1}, {0, 0}};
		else if (tilt_trig & TILT_RIGHT)
			d_cells = {{2, -2}, {2, -1}, {2, 0}, {1, -2}, {1, -1}, {1, 0}, {0, -2}, {0, -1}, {0, 0}};
		else if (tilt_trig & TILT_FRONT)
			d_cells = {{2, 1}, {2, 0}, {2, -1}, {1, 1}, {1, 0}, {1, -1}, {0, 1}, {0, 0}, {0, -1}};*/
		if (tilt_trig) {
			d_cells = {{1,  1}, {1,  0}, {1,  -1}, {0,  1}, {0,  0}, {0,  -1}, {-1, 1}, {-1, 0}, {-1, -1}};
		}

		return d_cells;
	});

	saveBlock(BLOCKED_LIDAR,iterate_point_.dir, [&]() {
		auto lidar_trig = ev.lidar_triggered;
		Cells d_cells{};
		if (lidar_trig & BLOCK_FRONT) {
			d_cells.push_back({2, 0});
		}
		if (lidar_trig & BLOCK_LEFT) {
			d_cells.push_back({2, 1});
		}
		if (lidar_trig & BLOCK_RIGHT) {
			d_cells.push_back({2, -1});
		}

		return d_cells;
	});
}

uint8_t ACleanMode::setBlocks(Dir_t dir)
{

	if(passed_path_.empty())
		passed_path_.push_back(getPosition());
	uint8_t block_count = 0;
	printf("before filter:");
	for(auto && cost_block: c_blocks)
	{
		printf("{%d, {%d,%d}} ",cost_block.first, cost_block.second.x, cost_block.second.y);
	}
	printf("\n");
	if(!isAny(dir)) {
		auto p_end = passed_path_.back();
		auto c_end_next = p_end.toCell() + cell_direction_[dir];
		auto dir_switch = (dir+2)%4;
		for(auto i = -1; i<=1; i++)
		{
			auto c_it = c_end_next + cell_direction_[dir_switch]*i;
			if(std::find_if(c_blocks.begin(), c_blocks.end(),[&c_it](const PairCell_t& p_cell){ return p_cell.second == c_it;}) != c_blocks.end())
			{
				auto c_it_next = c_it + cell_direction_[dir];
				auto result = std::find_if(c_blocks.begin(), c_blocks.end(),[&c_it_next](const PairCell_t& p_cell){ return p_cell.second == c_it_next;});
				if(result != c_blocks.end())
				{
					ROS_WARN("remove block(%d,%d) after block(%d,%d) in dir(%d)",result->second.x,result->second.y,c_it.x, c_it.y, dir);
					c_blocks.erase(result);
				}
			}
		}
	}

	printf("after filter:");
	for (auto &&cost_block  : c_blocks) {
		printf("{%d, {%d,%d}} ",cost_block.first, cost_block.second.x, cost_block.second.y);
		clean_map_.setCell(CLEAN_MAP, cost_block.second.x, cost_block.second.y, cost_block.first);
	}
	printf("\n");
	c_blocks.clear();
	return block_count;
}

void ACleanMode::pubTmpTarget(const Point_t &point, bool is_virtual) {
	visualization_msgs::Marker point_markers;
	point_markers.ns = "tmp_target";
	point_markers.id = 0;
	point_markers.type = visualization_msgs::Marker::SPHERE_LIST;
	point_markers.action = 0;//add
	point_markers.lifetime = ros::Duration(0), "base_link";
	point_markers.scale.x = 0.07;
	point_markers.scale.y = 0.07;
	point_markers.scale.z = 0.10;
	if(!is_virtual)
	{
		point_markers.color.r = 1.0;
		point_markers.color.g = 0.5;
		point_markers.color.b = 0.5;
	}else{
		point_markers.color.r = 0.3;
		point_markers.color.g = 0.3;
		point_markers.color.b = 0.4;
	}

	point_markers.color.a = 1.0;
	point_markers.header.frame_id = "/map";
	point_markers.header.stamp = ros::Time::now();

//	for(const auto & point : points)
	{
		geometry_msgs::Point point_marker;
		point_marker.x = point.x;
		point_marker.y = point.y;
		point_marker.z = 0;
		point_markers.points.push_back(point_marker);
	}
	tmp_target_pub_.publish(point_markers);
	//ROS_INFO("%s,%d,points size:%u,points %s",__FUNCTION__,__LINE__,points->size(),msg.c_str());
	point_markers.points.clear();
	//ROS_INFO("pub points!!");
}

void ACleanMode::pubPointMarkers(const std::deque<Vector2<double>> *points, std::string frame_id, std::string name)
{
	visualization_msgs::Marker point_marker;
	point_marker.ns = name;
	point_marker.id = 0;
	point_marker.type = visualization_msgs::Marker::SPHERE_LIST;
	point_marker.action= 0;//add
	point_marker.lifetime=ros::Duration(0);
	point_marker.scale.x = 0.05;
	point_marker.scale.y = 0.05;
	point_marker.scale.z = 0.05;
	point_marker.color.r = 0.0;
	point_marker.color.g = 1.0;
	point_marker.color.b = 0.0;
	point_marker.color.a = 1.0;
	point_marker.header.frame_id = frame_id;
	point_marker.header.stamp = ros::Time::now();

	geometry_msgs::Point lidar_points;
	lidar_points.z = 0;
	if (!points->empty()) {
		std::string msg("");
		for (auto iter = points->cbegin(); iter != points->cend(); ++iter) {
			lidar_points.x = iter->x;
			lidar_points.y = iter->y;
			point_marker.points.push_back(lidar_points);
			msg+="("+std::to_string(iter->x)+","+std::to_string(iter->y)+"),";
		}
		point_marker_pub_.publish(point_marker);
		//ROS_INFO("%s,%d,points size:%u,points %s",__FUNCTION__,__LINE__,points->size(),msg.c_str());
		point_marker.points.clear();
		//ROS_INFO("pub point!!");
	}
	else {
		point_marker.points.clear();
		point_marker_pub_.publish(point_marker);
	}
}

bool ACleanMode::checkCorner(const sensor_msgs::LaserScan::ConstPtr &scan, const PointSelector &para) {
	int forward_wall_count = 0;
	int side_wall_count = 0;
	for (int i = 359; i > 0; i--) {
		if (scan->ranges[i] < 4) {
			auto point = polarToCartesian(scan->ranges[i], i);
			if (sqrt(pow(point.x, 2) + pow(point.y, 2)) <= para.corner_front_trig_lim) {
				if (para.inForwardRange(point)) {
					forward_wall_count++;
//			ROS_INFO("point(%f,%f)",point.x,point.y);
				}
			}
			if (para.inSidedRange(point)) {
				side_wall_count++;
//			ROS_INFO("point(%f,%f)",point.x,point.y);
//				ROS_INFO("side_wall_count(%d)",side_wall_count);
			}
		}
	}
//	ROS_INFO("forward_wall_count(%d), side_wall_count(%d)",forward_wall_count, side_wall_count);
	return forward_wall_count > para.forward_count_lim && side_wall_count > para.side_count_lim;
}

/*Vector2<double> ACleanMode::polarToCartesian(double polar, int i)
{
	Vector2<double> point{cos(degree_to_radian(i * 1.0 + 180.0)) * polar,
					sin(degree_to_radian(i * 1.0 + 180.0)) * polar };

	coordinate_transform(&point.x, &point.y, LIDAR_THETA, LIDAR_OFFSET_X, LIDAR_OFFSET_Y);
	return point;

}*/

Vector2<double> ACleanMode::getTargetPoint(const Vector2<double> &p1, const Vector2<double> &p2, const PointSelector &para) {
	auto p3 = (p1 + p2) / 2;
	Vector2<double> target{};

//	ROS_INFO("p1(%f,%f)", p1.x, p1.y);
//	ROS_INFO("p2(%f,%f)", p2.x, p2.y);
//	ROS_INFO("p3 (%f,%f)", p3.x, p3.y);

//	auto x4 = para.narrow / (sqrt(1 + p1.SquaredDistance(p2))) + p3.x;
//	auto y4 = ((x4 - p3.x) * (p1.x - p2.x) / (p2.y - p1.y)) + p3.y;
	auto dx = para.narrow / (sqrt(1 + ((p1.x - p2.x) / (p2.y - p1.y)) * ((p1.x - p2.x) / (p2.y - p1.y))));
	auto x4 = dx + p3.x;

	auto dy = (x4 - p3.x) * (p1.x - p2.x) / (p2.y - p1.y);
	auto y4 = dy + p3.y;

//	ROS_INFO("x4,y4(%f,%f)", x4, y4);

	if (((p1.x - x4) * (p2.y - y4) - (p1.y - y4) * (p2.x - x4)) < 0) {
		target = {x4,y4};
//		ROS_INFO_FL();
//		ROS_INFO("target(%f,%f)", target.x, target.y);
	}
	else {
		x4 =  -dx + p3.x;
		y4 = (x4 - p3.x) * (p1.x - p2.x) / (p2.y - p1.y) + p3.y;
		target = {x4,y4};
//		ROS_INFO_FL();
//		ROS_INFO("target(%f,%f)", target.x, target.y);
	}
	return target;
}

bool ACleanMode::removeCrossingPoint(const Vector2<double> &target_point, PointSelector &para,
																		 const sensor_msgs::LaserScan::ConstPtr &scan) {
	for (int i = 359; i >= 0; i--) {
		auto laser_point = polarToCartesian(scan->ranges[i], i);
//		ROS_WARN("laser_point.Distance(%lf)", laser_point.Distance(zero_point));
/*		if (laser_point.Distance({0, 0}) <= ROBOT_RADIUS) {
			beeper.beepForCommand(VALID);
			ROS_ERROR("laser_point.Distance(%lf)", laser_point.Distance({0, 0}));
			continue;
		}*/
		auto distance = target_point.Distance(laser_point);
		//ROS_INFO("distance =  %lf", distance);
		if (distance < para.narrow - para.narrow_minuend) {
			return false;
		}
	}
	return true;
}

bool ACleanMode::calcLidarPath(const sensor_msgs::LaserScan::ConstPtr & scan,bool is_left, std::deque<Vector2<double>>& points, double wall_distance) {
	PointSelector para{is_left, wall_distance};
//	ROS_INFO("is_left(%d)",is_left);
	auto is_corner = checkCorner(scan, para);
	if(is_corner)
	{
//		beeper.beepForCommand(VALID);
		ROS_WARN("is_corner = %d", is_corner);
	}
	for (int i = 359; i >= 0; i--) {
//		ROS_INFO("laser point(%d, %lf)", i, scan->ranges[i]);
		if (scan->ranges[i] < 4 && scan->ranges[i - 1] < 4) {
			auto point1 = polarToCartesian(scan->ranges[i], i);
//			ROS_INFO("point1(%lf, %lf)", point1.x, point1.y);

			if (!para.LaserPointRange(point1, is_corner)) { //				INFO_BLUE("1");
				continue;
			}


			auto point2 = polarToCartesian(scan->ranges[i - 1], i - 1);

			if (!para.LaserPointRange(point2, is_corner)) {
//				INFO_BLUE("1");
				continue;
			}

			if (point2.Distance(point1) > 0.05) {
				//ROS_INFO("two points distance is too large");
//				INFO_BLUE("2");
				continue;
			}
			auto target = getTargetPoint(point1, point2, para);

			if (!para.TargetPointRange(target)) {
//				INFO_BLUE("3");
				continue;
			}

			if (target.Distance({0, 0}) > 0.4) {
//				INFO_BLUE("4");
				continue;
			}

			if (is_corner) {
				if (!removeCrossingPoint(target, para, scan)) {
//				INFO_BLUE("5");
					continue;
				}
			}

//			ROS_WARN("PUSH! points(%d):target(%lf,%lf),dis(%f)", points.size(), target.x, target.y, target.Distance({CHASE_X, 0}));
			points.push_back(target);
		}
	}
//	ROS_INFO("points.size(%d)", points.size());
	if (points.empty()) {
		ROS_WARN("no laser wf target!");
		return false;
	}
	if (!is_left) {
		std::reverse(points.begin(), points.end());//for the right wall follow
	}
	auto min = std::min_element(points.rbegin(), points.rend(), [](Vector2<double>& a, Vector2<double>& b) {
		return a.Distance({CHASE_X, 0}) < b.Distance({CHASE_X, 0});
	});
//	ROS_INFO("min(%f,%f)",min->x, min->y);

	auto size = points.size();
	std::copy(points.rbegin(), min+1, std::front_inserter(points));
	points.resize(size);
//	for (const auto &target :points)
//			ROS_WARN("points(%d):target(%lf,%lf),dis(%f)", points.size(), target.x, target.y, target.Distance({CHASE_X, 0}));
//	ROS_WARN("points(%d):target(%lf,%lf)", points.size(), points.front().x, points.front().y);
	pubPointMarkers(&points, "base_link", "point marker");

	return true;
}

void ACleanMode::scanOriginalCb(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	lidar.scanOriginalCb(scan);
	lidar.checkRobotSlip();
	if (lidar.isScanOriginalReady()
		&& (action_i_ == ac_follow_wall_left || action_i_ == ac_follow_wall_right)) {
		std::deque<Vector2<double>> points{};
		calcLidarPath(scan, action_i_ == ac_follow_wall_left, points, wall_distance);
		setTempTarget(points, scan->header.seq);
	}
}

void ACleanMode::pubFitLineMarker(visualization_msgs::Marker fit_line_marker)
{
	fit_line_marker_pub_.publish(fit_line_marker);
}

uint8_t ACleanMode::setFollowWall(GridMap& map, bool is_left,const Points& passed_path)
{
	uint8_t block_count = 0;
	if (!passed_path.empty() && !c_blocks.empty())
	{
		std::string msg = "cell:";
		auto dy = is_left ? 2 : -2;
		for(auto& point : passed_path){
			if(map.getCell(CLEAN_MAP,point.toCell().x,point.toCell().y) != BLOCKED_RCON){
				auto relative_cell = point.getRelative(0, dy * CELL_SIZE);
				auto block_cell = relative_cell.toCell();
				msg += "(" + std::to_string(block_cell.x) + "," + std::to_string(block_cell.y) + ")";
				map.setCell(CLEAN_MAP,block_cell.x,block_cell.y, BLOCKED_FW);
				block_count++;
			}
		}
		ROS_INFO("%s,%d: Current(%d, %d), \033[32m mapMark CLEAN_MAP %s\033[0m",__FUNCTION__, __LINE__, getPosition().toCell().x, getPosition().toCell().y, msg.c_str());
	}
}

void ACleanMode::setLinearCleaned()
{
	ROS_INFO("setLinearCleaned cells:");
	// start-1
	auto p_start = passed_path_.front();
	auto c_start_last = p_start.toCell() - cell_direction_[p_start.dir];
	auto c_diff_start_switch = cell_direction_[(passed_path_.front().dir + 2)%4];
	ROS_INFO("{%d,%d}",c_start_last.x, c_start_last.y);
	for(auto i =-1; i<=1; i++)
	{
		auto c_it = c_start_last + c_diff_start_switch*i;
		auto c_val = clean_map_.getCell(CLEAN_MAP, c_it.x, c_it.y);
		if(c_val >=BLOCKED && c_val<=BLOCKED_BOUNDARY)
		{
			auto c_it_shift = c_it - cell_direction_[p_start.dir];
			ROS_WARN("!!!!!!start_point -1 dir is in block,move back 1 cell c_it(%d,%d)->c_it_shift(%d,%d)",c_it.x, c_it.y,c_it_shift.x,c_it_shift.y);
			c_blocks.insert({c_val, c_it_shift});
		}
	}
	// end+1 point opt
	auto p_end = passed_path_.back();
	auto c_end_next = p_end.toCell() + cell_direction_[p_end.dir];
	auto c_end_diff_switch = cell_direction_[(p_end.dir + 2)%4];
	ROS_INFO("{%d,%d}",c_end_next.x, c_end_next.y);
	for(auto i =-1; i<=1; i++)
	{
		auto c_it = c_end_next + c_end_diff_switch*i;
		auto c_val = clean_map_.getCell(CLEAN_MAP, c_it.x, c_it.y);
		if(c_val >=BLOCKED && c_val<=BLOCKED_BOUNDARY)
		{
			auto c_it_shift = c_it + cell_direction_[p_end.dir];
			ROS_WARN("!!!!!!map end_point +1 dir is in block,move front 1 cell c_it(%d,%d)->c_it_shift(%d,%d)",c_it.x, c_it.y,c_it_shift.x,c_it_shift.y);
			c_blocks.insert({c_val, c_it_shift});
		}
		for(auto && c_block:  c_blocks)
		{
			if(c_block.second == c_it)
			{
				auto c_it_shift = c_it + cell_direction_[p_end.dir];
				ROS_WARN("!!!!!!block end_point +1 dir is in block,move front 1 cell c_it(%d,%d)->c_it_shift(%d,%d)",c_it.x, c_it.y,c_it_shift.x,c_it_shift.y);
				c_blocks.insert({c_block.first, c_it_shift});
			}
		}
	}
}

void ACleanMode::setCleaned(std::deque<Cell_t> cells)
{
	if(cells.empty())
		return;

	//while robot turn finish and going to a new direction
	//may cost location change and cover the block cells
	//so we append a cell in front of the cell list
	//to avoid robot clean in the same line again.
	auto is_x_pos = cells.front().x <= cells.back().x;
	auto x_offset = is_x_pos? -1 : 1;
	Cell_t cell_front = {int16_t(cells.front().x + x_offset),cells.front().y};
	cells.push_front(cell_front);

	//in the first cell of cells ,we just mark 6 cells
	for (uint32_t i = 0; i< cells.size(); i++)
	{
		Cell_t cell = cells.at(i);
		for( int dx = -ROBOT_SIZE_1_2;dx <=ROBOT_SIZE_1_2;dx++)
		{
			if( i == 0)
				if(is_x_pos && dx == -ROBOT_SIZE_1_2)
					continue;
				else if(!is_x_pos && dx == ROBOT_SIZE_1_2)
					continue;
			for(int dy = -ROBOT_SIZE_1_2; dy <= ROBOT_SIZE_1_2; dy++)
			{
				CellState status = clean_map_.getCell(CLEAN_MAP, cell.x+dx, cell.y+dy);
				if (status != BLOCKED_TILT && status != BLOCKED_SLIP && status != BLOCKED_RCON)
					clean_map_.setCell(CLEAN_MAP,cell.x+dx,cell.y+dy, CLEANED);
			}
		}
	}
}

void ACleanMode::setCleanMapMarkers(int16_t x, int16_t y, CellState type, visualization_msgs::Marker& clean_map_markers_)
{
	geometry_msgs::Point m_points_;
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
	else if (type == BLOCKED_FW)//Follow Wall
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
		// Black
		color_.r = 0.0;
		color_.g = 0.0;
		color_.b = 0.0;
	}
	else if (type == BLOCKED_RCON || type == BLOCKED_TMP_RCON)
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
		// Purple 
		color_.r = 1.0;
		color_.g = 0.0;
		color_.b = 1.0;
	}
	clean_map_markers_.points.push_back(m_points_);
	clean_map_markers_.colors.push_back(color_);
}

void ACleanMode::pubCleanMapMarkers(GridMap& map, const std::deque<Cell_t>& path)
{

	if (path.empty())
		return;
	visualization_msgs::Marker clean_markers_;
	clean_markers_.points.clear();
	geometry_msgs::Point m_points_{};
	clean_markers_.points.push_back(m_points_);
	visualization_msgs::Marker clean_map_markers_;
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
	int16_t x, y, x_min, x_max, y_min, y_max;
	CellState cell_state;
//	Cell_t next = path.front();
	Cell_t target = path.back();
	map.getMapRange(CLEAN_MAP, &x_min, &x_max, &y_min, &y_max);
/*

	if (next.x == SHRT_MIN )
		next.x = x_min;
	else if (next.x == SHRT_MAX)
		next.x = x_max;
*/

	for (x = x_min; x <= x_max; x++) {
		for (y = y_min; y <= y_max; y++) {
			cell_state = map.getCell(CLEAN_MAP, x, y);
			if (cell_state > UNCLEAN && cell_state < BLOCKED_BOUNDARY)
				setCleanMapMarkers(x, y, cell_state, clean_map_markers_);
		}
	}
	for (auto &&cell : path) {
		setCleanMapMarkers(cell.x, cell.y, TARGET, clean_map_markers_);
	}
	if (!path.empty())
		setCleanMapMarkers(path.back().x, path.back().y, TARGET_CLEAN, clean_map_markers_);

	clean_map_markers_.header.stamp = ros::Time::now();
	send_clean_map_marker_pub_.publish(clean_map_markers_);
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
	line_marker.color.r = 0.0;
	line_marker.color.g = 0.0;
	line_marker.color.b = 1.0;
	line_marker.color.a = 1.0;
	line_marker.header.frame_id = "/base_link";
	line_marker.header.stamp = ros::Time::now();
	geometry_msgs::Point point1;
	point1.z = 0.0;
	geometry_msgs::Point point2;
	point2.z = 0.0;
	line_marker.points.clear();
	std::vector<LineABC>::const_iterator it;
	if(!lines->empty() && lines->size() >= 1){
		for(it = lines->cbegin(); it != lines->cend();it++){
			point1.x = it->x1;
			point2.x = it->x2;
			if (it->B != 0) {
				point1.y = (-it->C - it->A * point1.x) / it->B;
				point2.y = (-it->C - it->A * point2.x) / it->B;
			} else {
				point1.y = it->y1;
				point2.y = it->y2;
			}
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
	if (sp_state == state_init.get())
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

	if (ev.key_clean_pressed || ev.key_long_pressed || s_wifi.receiveIdle()){
		ROS_WARN("%s %d: Exit for remote key or clean key or long press clean key or wifi idle.", __FUNCTION__, __LINE__);
		setNextMode(md_idle);
		return true;
	}

	if (charger.isDirected())
	{
		ROS_WARN("%s %d: Exit for directly charge.", __FUNCTION__, __LINE__);
		setNextMode(md_charge);
		return true;
	}

	if (s_wifi.receivePlan1())
	{
		ROS_WARN("%s %d: Exit for wifi plan1.", __FUNCTION__, __LINE__);
		setNextMode(cm_navigation);
		return true;
	}
	return false;
}

bool ACleanMode::moveTypeNewCellIsFinish(IMoveType *p_mt) {
	auto curr = getPosition();
	auto loc = std::find_if(passed_path_.begin(), passed_path_.end(), [&](Point_t it) {
		return curr.isCellAndAngleEqual(it);
	});
	auto distance = std::distance(loc, passed_path_.end());
	if (distance == 0) {
		curr.dir = iterate_point_.dir;
		ROS_INFO("curr(%d,%d,%d,%d)", curr.toCell().x, curr.toCell().y, static_cast<int>(radian_to_degree(curr.th)),curr.dir);
		passed_path_.push_back(curr);
	}

	markMapInNewCell();//real time mark to exploration

	if (isStateFollowWall()) {
//			auto p_mt = dynamic_cast<MoveTypeFollowWall *>(p_mt);
		if (p_mt->isBlockCleared(clean_map_, passed_path_)) {
			clean_map_.markRobot(CLEAN_MAP);
			std::vector<Vector2<int>> markers{};
			if (lidar.isScanCompensateReady())
				lidar.lidarMarker(markers, p_mt->movement_i_, action_i_);
			ROS_ERROR("markers.size() = %d", markers.size());
			std::vector<Vector2<int>> left_marks{{0,2}, {1,2},{-1,2}};
//			ROS_ERROR("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
			for (const auto &marker : markers) {
//				ROS_ERROR("marker(%d, %d)", marker.x, marker.y);
				if(std::any_of(left_marks.begin(), left_marks.end(), [&](const Vector2<int> & mark_it){
//					ROS_ERROR("any_of:marker(%d,%d),mark_it(%d,%d)", marker.x, marker.y, mark_it.x, mark_it.y);
					return marker == mark_it;
				})){
//					beeper.debugBeep(VALID);
					auto cell = getPosition().getCenterRelative(marker.x * CELL_SIZE, marker.y * CELL_SIZE).toCell();
					ROS_ERROR("follow wall find lidar obs(%d, %d)", cell.x, cell.y);
					clean_map_.setCell(CLEAN_MAP, cell.x, cell.y, BLOCKED_LIDAR);
				}
			}
			if (!clean_path_algorithm_->checkTrapped(clean_map_, getPosition().toCell())) {
				out_of_trapped = true;
				ROS_ERROR("OUT OF TRAPPED");
				return true;
			}
		}
	}

	if (distance > 5 && getNextMode() != cm_spot) {// closed
		ROS_INFO("next_mode_i_(%d)",getNextMode());
		ROS_INFO("mode_i_(%d)",mode_i_);
		is_closed = true;
		is_isolate = isIsolate();
		if(is_isolate)
			isolate_count_++;
		else
		{
			passed_path_.clear();
			closed_count_++;
			if(closed_count_<closed_count_limit_)
				return false;
		}
		ROS_ERROR("distance > 5,limit %d, closed %d",closed_count_limit_,is_closed);
		return true;

	}

	return false;
}

bool ACleanMode::moveTypeRealTimeIsFinish(IMoveType *p_move_type)
{
	markRealTime();
	if(action_i_ == ac_linear) {
		auto p_mt = dynamic_cast<MoveTypeLinear *>(p_move_type);
		if(p_mt->movement_i_ == p_mt->mm_forward && (p_mt->isPoseReach() || p_mt->isPassTargetStop(iterate_point_.dir)))
			return true;

		if (p_mt->isLinearForward()){
			if(checkChargerPos())
				return false;
			else
				return p_mt->isRconStop();
		}
	}
	else//rounding
	{
		if(!isStateFollowWall() && !isStateTest())
		{
			auto p_mt = dynamic_cast<MoveTypeFollowWall *>(p_move_type);
			if(p_mt->movement_i_ == p_mt->mm_forward ||p_mt->movement_i_ == p_mt->mm_straight)
				return p_mt->isNewLineReach(clean_map_) || p_mt->isOverOriginLine(clean_map_);
		}
	}
	return false;
}

bool ACleanMode::isStateUpdateFinish() {
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
	while (!isStateUpdateFinish() && ros::ok());
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

bool ACleanMode::checkChargerPos()
{
	const int16_t DETECT_RANGE = 20;//cells
	if(!isStateGoHomePoint()){
		if(c_rcon.getStatus())
		{
			if(found_charger_)
			{
				int counter=0;
				for(Point_t charger_position:charger_pose_)
				{
					if(getPosition().toCell().Distance(charger_position.toCell()) > DETECT_RANGE )
						counter++;
					else{
						c_rcon.resetStatus();
						return true;
					}
					if(counter >= charger_pose_.size())
					{
						if(estimateChargerPos(c_rcon.getStatus()))
						{
							INFO_CYAN("FOUND CHARGER");
							c_rcon.resetStatus();
							setHomePoint();
							return true;
						}
						break;
					}
				}
			}
			else if(!found_charger_){
				if(estimateChargerPos(c_rcon.getStatus())){
					INFO_CYAN("FOUND CHARGER");
					c_rcon.resetStatus();
					setHomePoint();
					return true;
				}
			}
		}
	}
	return false;

}

bool ACleanMode::estimateChargerPos(uint32_t rcon_value)
{
	if(!(rcon_value & RconAll_Home_T)){
		return false;
	}
	enum {flfr,frfr2,flfl2,fl2l,fr2r,bll,brr,fl2,fr2,l,r,bl,br};
	static int8_t cnt[13]={0,0,0,0,0,0,0,0,0,0,0,0,0};
	float direction = 0.0;//charger direction acorrding to robot,in degrees
	float dist = 0.0;
	float len = 0.0;
	const int STABLE_CNT = 2;
	const float TMP_RCON_MARKER_DIST = 1.5;//in meters
	const float DETECT_RANGE_MAX = CELL_SIZE*6;
	const float DETECT_RANGE_MIN = CELL_SIZE*2;
	const float RCON_1 = 0,RCON_2 = 22.0,RCON_3 = 40.0 ,RCON_4 = 78.0 ,RCON_5 = 130.0;
	bool rcon_trig = 0;
	//-- here we only detect top signal from charge stub ----
	//ROS_INFO("%s,%d,rcon_value 0x%x",__FUNCTION__,__LINE__,rcon_value & RconAll_Home_T);
	if(1)
	{
		if( (rcon_value & RconFR_HomeT) || (rcon_value & RconFL_HomeT)  && !(rcon_value & 0x22200222) ){ //fl & fr sensor
			if((cnt[flfr]++) >= STABLE_CNT){
				cnt[flfr] = 0;
				direction = RCON_1;
				rcon_trig = 1;
			}
			else{
				return false;
			}
		}
		else if( (rcon_value & RconFL_HomeT) && (rcon_value & RconFL2_HomeT)  && !(rcon_value & RconAll_R_HomeT)){//fl & fl2 sensor
			if((cnt[flfl2]++) >= STABLE_CNT){
				cnt[flfl2] = 0;
				direction = RCON_2;
				rcon_trig = 1;
			}
			else{
				return false;
			}
		}
		else if( (rcon_value & RconFR_HomeT) && (rcon_value & RconFR2_HomeT) && !(rcon_value & RconAll_L_HomeT)){//fr & fr2 sensor
			if((cnt[frfr2]++) >= STABLE_CNT){
				cnt[frfr2] = 0;
				direction = -RCON_2;
				rcon_trig = 1;
			}
			else{
				return false;
			}
		}
		if( (rcon_value & RconFL2_HomeT) && !(rcon_value & RconL_HomeT) && !(rcon_value & RconAll_R_HomeT) //fl2 sensor
					&& !(rcon_value & RconBL_HomeT) )//to avoid charger signal reflection from other flat
		{
			if((cnt[fl2]++) >= STABLE_CNT){
				cnt[fl2] = 0;
				direction = RCON_3;
				rcon_trig = 1;
			}
			else
				return false;
		}
		else if( (rcon_value & RconFR2_HomeT) && !(rcon_value & RconR_HomeT) && !(rcon_value & RconAll_L_HomeT) //fr2 sensor
					&& !(rcon_value & RconBR_HomeT) ){//to avoid charger signal reflection from other direction
			if((cnt[fr2]++) >= STABLE_CNT){
				cnt[fr2] = 0;
				direction = -RCON_3;
				rcon_trig = 1;
			}
			else
				return false;
		}
		/*
		else if( (rcon_value & RconL_HomeT) && (rcon_value & RconFL2_HomeT) && !(rcon_value & RconAll_R_HomeT) ){//fl2 & l sensor
			if((cnt[fl2l]++) >= STABLE_CNT){
				cnt[fl2l] = 0;
				direction = 60;
				rcon_trig = 1;
			}
			else
				return false;
		}
		else if( (rcon_value & RconR_HomeT) && (rcon_value & RconFR2_HomeT) && !(rcon_value & RconAll_L_HomeT) ){//fr2 & r sensor
			if((cnt[fr2r]++) >= STABLE_CNT){
				cnt[fr2r]=0;
				direction = -60;
				rcon_trig = 1;
			}
			else
				return false;
		}
		*/
		else if( (rcon_value & RconL_HomeT) && !(rcon_value & RconFL2_HomeT) && !(rcon_value & RconAll_R_HomeT) //l sensor
					&& !(rcon_value & RconBL_HomeT) ){//to avoid charger signal reflection from other flat

			if((cnt[l]++) >= STABLE_CNT){
				cnt[l] = 0;
				direction = RCON_4;
				rcon_trig = 1;
			}
			else
				return false;
		}
		else if( (rcon_value & RconR_HomeT ) && !(rcon_value & RconFR2_HomeT) && !(rcon_value & RconAll_L_HomeT) //r sensor
					&& !(rcon_value & RconBR_HomeT) ){//to avoid charger signal reflection from other direction
			if((cnt[r]++) >= STABLE_CNT){
				cnt[r]=0;
				direction = -RCON_4;
				rcon_trig = 1;
			}
			else
				return false;
		}
		/*
		else if( (rcon_value & RconBL_HomeT) && (rcon_value & RconL_HomeT) && !(rcon_value & RconAll_R_HomeT)){//l & bl sensor
			if((cnt[bll]++) >= STABLE_CNT){
				cnt[bll] = 0;
				direction = 110;
				rcon_trig = 1;
			}
			else
				return false;
		}
		else if( (rcon_value & RconBR_HomeT) && (rcon_value & RconR_HomeT) && !(rcon_value & RconAll_L_HomeT)){//r & br sensor
			if((cnt[brr]++) >= STABLE_CNT){
				cnt[brr] = 0;
				direction = -110;
				rcon_trig = 1;
			}
			else
				return false;
		}
		*/
		else if( (rcon_value & RconBL_HomeT) && !(rcon_value & RconL_HomeT) && !(rcon_value & RconAll_R_HomeT) ){//bl sensor
			if((cnt[bl]++) >= STABLE_CNT){
				cnt[bl] = 0;
				direction = RCON_5;
				rcon_trig = 1;
			}
			else
				return false;
		}
		else if( (rcon_value & RconBR_HomeT) && !(rcon_value & RconR_HomeT) && !(rcon_value & RconAll_L_HomeT) ){//br sensor
			if((cnt[br]++) >= STABLE_CNT){
				cnt[br] = 0;
				direction = -RCON_5;
				rcon_trig = 1;
			}
			else
				return false;
		}
		else
			return false;
	}
	memset(cnt,0,sizeof(int8_t)*13);
	// set charger using lidar and rcon
	if( lidar.lidarCheckFresh(0.1,3))
	{
		dist = lidar.getLidarDistance(direction,DETECT_RANGE_MAX,DETECT_RANGE_MIN);
		if (dist <= DETECT_RANGE_MAX && dist >= DETECT_RANGE_MIN && fabsf(direction) != RCON_1 && fabsf(direction) != RCON_2 )
		{
			double angle_offset = ranged_radian( degree_to_radian(direction) + getPosition().th);
			Point_t c_pose_;
			c_pose_.SetX( cos(angle_offset)* dist  +  getPosition().GetX());//set pos x
			c_pose_.SetY( sin(angle_offset)* dist +  getPosition().GetY() );//set pos y
			//c_pose_.th =  ranged_radian(getPosition().th + degree_to_radian( direction ));//set pos th
			c_pose_.th =  ranged_radian(degree_to_radian( direction ));//set pos th

			c_pose_.dir = iterate_point_.dir;
			int16_t cell_distance = getPosition().toCell().Distance(c_pose_.toCell());
			charger_pose_.push_back( c_pose_ );
			setChargerArea( c_pose_ );
			ROS_INFO("\033[1;40;32m%s,%d,FOUND CHARGER direction %f,cur_angle = %f,angle_offset= %f, rcon_state = 0x%x, \033[0m",__FUNCTION__,__LINE__,direction,radian_to_degree(ranged_radian(getPosition().th)),radian_to_degree(angle_offset),rcon_value);
			found_charger_ = true;
			return true;
		}
		else
			ROS_INFO("%s,%d,  distance out of range %f , direction %f, rcon_state = 0x%x",__FUNCTION__,__LINE__,dist,direction,rcon_value);
	}
	else
		ROS_INFO("%s,%d,  lidar data not update, direction %f, rcon_state = 0x%x",__FUNCTION__,__LINE__,direction,rcon_value);
	/*
	//set charger only using with rcon 
	if( rcon_trig = 1 && fabsf(direction) != RCON_5 )
	{
		ROS_INFO("\033[42;37m%s,%d, set charger with rcon direction %f, rcon_state = 0x%x, map direction = %d \033[0m",__FUNCTION__,__LINE__,direction,rcon_value,iterate_point_.dir);
		//on x axies
		if(isXAxis(iterate_point_.dir))
		{
			Point_t tmp_c_pose;
			tmp_c_pose.SetX(getPosition().GetX());
			tmp_c_pose.SetY(getPosition().GetY());
			tmp_c_pose.th =  ranged_radian(degree_to_radian( direction ));//set pos th
			tmp_c_pose.dir = iterate_point_.dir;
			tmp_charger_pose_.push_back( tmp_c_pose );
			float diff_th = fabsf(tmp_c_pose.th - tmp_charger_pose_.front().th);
			ROS_INFO("\033[42;37m%s,%d,tmp_charger_pose size = %d ,diff_th = %f\033[0m",__FUNCTION__,__LINE__,tmp_charger_pose_.size(),radian_to_degree(diff_th));
			if(tmp_charger_pose_.size() >=2 && diff_th >= M_PI/18)
			{
				Point_t poseA = tmp_charger_pose_.front();
				Point_t poseB = tmp_charger_pose_.back();
				float tmp_dist = poseA.Distance(poseB);
				if( tmp_dist <= TMP_RCON_MARKER_DIST)
				{
					float A,B = 0; //angle A,B for temporary use
					A = fabsf(poseA.th);
					if(poseB.dir == MAP_NEG_X )
						B = -fabsf(poseB.th);
					else
						B =  M_PI - fabsf(poseB.th);
					if(A+B < M_PI){
						float real_dist = tmp_dist*sin(B)/sin(M_PI - (A + B));
						if(real_dist <= TMP_RCON_MARKER_DIST)
						{
							Point_t c_pose;
							c_pose.SetY(poseA.y + sin(A)*real_dist);
							c_pose.SetX(poseA.x + cos(A)*real_dist);
							c_pose.th = poseA.th;
							c_pose.dir = poseA.dir;
							ROS_INFO("charger pose (%d,%d,%f) tmp_dist = %f,real_dist = %f",c_pose.toCell().x,c_pose.toCell().y,radian_to_degree(c_pose.th),tmp_dist,real_dist);
							charger_pose_.push_back(c_pose);
							found_charger_ = true;
							INFO_CYAN("found charger with rcon only");
							setChargerArea( c_pose );
							tmp_charger_pose_.clear();
							return true;
						}
						else
							ROS_INFO("charger pos distance (%f)",real_dist);
					}
				}
				else{
					ROS_INFO("tmp charger pos distance (%f)",tmp_dist);
				}
			}
		}
	}
	*/
	return false;
}

void ACleanMode::checkShouldMarkCharger(float angle_offset,float distance)
{
	if(found_charger_)
	{
		Point_t pose;
		pose.SetX( cos(angle_offset)* distance  +  getPosition().GetX() );
		pose.SetY( sin(angle_offset)* distance  +  getPosition().GetY() );
		pose.th = ranged_radian( getPosition().th - (M_PI - angle_offset));
		pose.dir = iterate_point_.dir;
		charger_pose_.push_back(pose);
		ROS_INFO("%s,%d, offset angle (%f),charger pose (%d,%d),th = %f ,dir = %d",__FUNCTION__,__LINE__, angle_offset,pose.toCell().GetX(),pose.toCell().GetY(),pose.th,pose.dir);
		setChargerArea(pose);
	}
}

void ACleanMode::setChargerArea(const Point_t charger_pos)
{
	//before set BLOCKED_RCON, clean BLOCKED_TMP_RCON first.
	int16_t x_min,x_max,y_min,y_max;
	clean_map_.getMapRange(CLEAN_MAP, &x_min, &x_max, &y_min, &y_max);
	for(int16_t i = x_min;i<=x_max;i++){
		for(int16_t j = y_min;j<=y_max;j++){
			if(clean_map_.getCell(CLEAN_MAP, i, j) == BLOCKED_TMP_RCON)
				clean_map_.setCell(CLEAN_MAP,i,j, UNCLEAN);
		}
	}

	Cell_t charger_pos_cell  = charger_pos.toCell();
	ROS_INFO("%s,%d,charger position(%d,%d),th= %f,dir=%d",__FUNCTION__,__LINE__,charger_pos_cell.x,charger_pos_cell.y,charger_pos.th,charger_pos.dir);

	for(int j = 3;j>-3;j--){
		for( int16_t i = 3;i>-3;i--){
			ROS_INFO("%s,%d,(%d,%d)",__FUNCTION__,__LINE__,charger_pos_cell.x+i,charger_pos_cell.y+j);
			clean_map_.setCell(CLEAN_MAP,charger_pos_cell.x +i ,charger_pos_cell.y+j,BLOCKED_RCON);
		}
	}
	//const int RADIAN= 4;//cells
	//clean_map_.setCircleMarkers(charger_pos,true,RADIAN,BLOCKED_RCON);

	/*	
	int16_t y_init;
	int16_t x_init;
	
	if((charger_pos.th >= 0 && charger_pos.dir == MAP_POS_X)|| (charger_pos.th<0 && charger_pos.dir == MAP_NEG_X) )
	{
		x_init = 3;
		y_init = -3;
		for(int16_t j = y_init;j< -y_init;j++)
		{
			for(int16_t i = x_init;i>-x_init;i--)
			{
				if( slam_grid_map.getCell(CLEAN_MAP,charger_pos_cell.x+i,charger_pos_cell.y+j) < SLAM_MAP_BLOCKED ){
					ROS_INFO("%s,%d,(%d,%d)",__FUNCTION__,__LINE__,charger_pos_cell.x+i,charger_pos_cell.y+j);
					clean_map_.setCell(CLEAN_MAP,charger_pos_cell.x + i, charger_pos_cell.y+j, BLOCKED_RCON);
				}
				else
					break;
			}
			if((slam_grid_map.getCell(CLEAN_MAP,charger_pos_cell.x+i,charger_pos_cell.y+j) >= SLAM_MAP_BLOCKED) )
				break;
		}
		y_init = 3;
		for(int16_t i = x_init;i> -x_init;i--)
		{
			for(int16_t j = -y_init;j> y_init; j++)
			{
				if( slam_grid_map.getCell(CLEAN_MAP,charger_pos_cell.x+i,charger_pos_cell.y+j) < SLAM_MAP_BLOCKED ){
					ROS_INFO("%s,%d,(%d,%d)",__FUNCTION__,__LINE__,charger_pos_cell.x+i,charger_pos_cell.y+j);
					clean_map_.setCell(CLEAN_MAP,charger_pos_cell.x + i, charger_pos_cell.y+j, BLOCKED_RCON);
				}
				else
					break;
			}
		}

	}
	else if((charger_pos.th < 0 && charger_pos.dir == MAP_POS_X) || (charger_pos.th >=0 && charger_pos.dir == MAP_NEG_X))
	{
		x_init = 3;
		y_init = 3;
		for(int16_t j = y_init;j> -y_init; j--)
		{
			for(int16_t i = x_init;i>-x_init;i--)
			{
				if( slam_grid_map.getCell(CLEAN_MAP,charger_pos_cell.x+i,charger_pos_cell.y+j) < SLAM_MAP_BLOCKED ){
					ROS_INFO("%s,%d,(%d,%d)",__FUNCTION__,__LINE__,charger_pos_cell.x+i,charger_pos_cell.y+j);
					clean_map_.setCell(CLEAN_MAP,charger_pos_cell.x + i, charger_pos_cell.y+j, BLOCKED_RCON);
				}
				else
					break;
			}
			if((slam_grid_map.getCell(CLEAN_MAP,charger_pos_cell.x+i,charger_pos_cell.y+j) >= SLAM_MAP_BLOCKED) )
				break;

		}
		x_init = -3;
		for(int16_t i = x_init;i< -x_init; i++)
		{
			for(int16_t j = y_init;j> -y_init; j--)
			{
				if(slam_grid_map.getCell(CLEAN_MAP,charger_pos_cell.x+i,charger_pos_cell.y+j) < SLAM_MAP_BLOCKED ){
					ROS_INFO("%s,%d,(%d,%d)",__FUNCTION__,__LINE__,charger_pos_cell.x+i,charger_pos_cell.y+j);
					clean_map_.setCell(CLEAN_MAP,charger_pos_cell.x + i, charger_pos_cell.y+j, BLOCKED_RCON);
				}
				else
					break;
			}
		}
	}
	else if((charger_pos.th >= 0 && charger_pos.dir == MAP_POS_Y) || (charger_pos.th <0 && charger_pos.dir == MAP_NEG_Y) )
	{
		x_init = 3;
		y_init = 3;
		for(int16_t i = x_init;i> -x_init;i--)
		{
			for(int16_t j = y_init;j> -y_init; j--)
			{
				if( slam_grid_map.getCell(CLEAN_MAP,charger_pos_cell.x+i,charger_pos_cell.y+j) < SLAM_MAP_BLOCKED ){
					ROS_INFO("%s,%d,(%d,%d)",__FUNCTION__,__LINE__,charger_pos_cell.x+i,charger_pos_cell.y+j);
					clean_map_.setCell(CLEAN_MAP,charger_pos_cell.x + i, charger_pos_cell.y+j, BLOCKED_RCON);
				}
				else
					break;
			}
			if( slam_grid_map.getCell(CLEAN_MAP,charger_pos_cell.x+i,charger_pos_cell.y+j) < SLAM_MAP_BLOCKED )
				break;
		}
		for(int16_t j = y_init;j> -y_init; j--)
		{
			for(int16_t i = x_init;i>-x_init;i--)
			{
				if( slam_grid_map.getCell(CLEAN_MAP,charger_pos_cell.x+i,charger_pos_cell.y+j) < SLAM_MAP_BLOCKED ){
					ROS_INFO("%s,%d,(%d,%d)",__FUNCTION__,__LINE__,charger_pos_cell.x+i,charger_pos_cell.y+j);
					clean_map_.setCell(CLEAN_MAP,charger_pos_cell.x + i, charger_pos_cell.y+j, BLOCKED_RCON);
				}
				else
					break;
			}
		}

	}
	else if((charger_pos.th < 0 && charger_pos.dir == MAP_POS_Y) || (charger_pos.th >=0 && charger_pos.dir == MAP_NEG_Y) )
	{
		x_init = -3;
		y_init = 3;
		for(int16_t i = x_init;i< -x_init; i++)
		{
			for(int16_t j = y_init;j> -y_init; j--)
			{
				if( (slam_grid_map.getCell(CLEAN_MAP,charger_pos_cell.x+i,charger_pos_cell.y+j) < SLAM_MAP_BLOCKED) ){
					ROS_INFO("%s,%d,(%d,%d)",__FUNCTION__,__LINE__,charger_pos_cell.x+i,charger_pos_cell.y+j);
					clean_map_.setCell(CLEAN_MAP,charger_pos_cell.x + i, charger_pos_cell.y+j, BLOCKED_RCON);
				}
				else
					break;
			}
			if( slam_grid_map.getCell(CLEAN_MAP,charger_pos_cell.x+i,charger_pos_cell.y+j) < SLAM_MAP_BLOCKED )
				break;

		}
		for(int16_t j = y_init;j> -y_init; j--)
		{
			for(int16_t i = x_init;i>-x_init;i--)
			{
				if( slam_grid_map.getCell(CLEAN_MAP,charger_pos_cell.x+i,charger_pos_cell.y+j) < SLAM_MAP_BLOCKED ){
					ROS_INFO("%s,%d,(%d,%d)",__FUNCTION__,__LINE__,charger_pos_cell.x+i,charger_pos_cell.y+j);
					clean_map_.setCell(CLEAN_MAP,charger_pos_cell.x + i, charger_pos_cell.y+j, BLOCKED_RCON);
				}
				else
					break;
			}
			if((slam_grid_map.getCell(CLEAN_MAP,charger_pos_cell.x+i,charger_pos_cell.y+j) >= SLAM_MAP_BLOCKED) )
				break;

		}

	}
	*/	

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

	while(ros::ok() && home_points_.size() >= (uint32_t)HOME_POINTS_SIZE && (home_points_.size() >= 1))
		// Drop the oldest home point to keep the home_points_.size() is within HOME_POINTS_SIZE.
		home_points_.pop_back();

	home_points_.push_front(getPosition());
	std::string msg = "Update Home_points_: ";
	for (auto it : home_points_)
		msg += "(" + std::to_string(it.toCell().x) + ", " + std::to_string(it.toCell().y) + "),";
	ROS_INFO("%s %d: %s", __FUNCTION__, __LINE__, msg.c_str());

	if (!seen_charger_during_cleaning_)
		seen_charger_during_cleaning_ = true;
}

// ------------------Handlers--------------------------

void ACleanMode::cliffAll(bool state_now, bool state_last)
{
	if (!charger.getChargeStatus() && !ev.cliff_all_triggered)
	{
		ROS_WARN("%s %d: Cliff all.", __FUNCTION__, __LINE__);
		ev.cliff_all_triggered = true;
	}
}

void ACleanMode::robotSlip(bool state_now, bool state_last){
	if(!ev.robot_slip)
	{
		ROS_WARN("%s %d: Robot slip.", __FUNCTION__, __LINE__);
		ev.robot_slip= true;
	}
}

void ACleanMode::overCurrentBrushMain(bool state_now, bool state_last)
{
	if (!ev.oc_brush_main)
	{
		INFO_YELLOW("MAIN BRUSH OVER CURRENT");
		ev.oc_brush_main = true;
		brush.stop();
	}
}

void ACleanMode::overCurrentVacuum(bool state_now, bool state_last)
{
	if (!ev.oc_vacuum)
	{
		ROS_WARN("%s %d: Vacuum over current.", __FUNCTION__, __LINE__);
		ev.oc_vacuum = true;
	}
}
// ------------------Handlers end--------------------------

// ------------------State init--------------------
bool ACleanMode::isSwitchByEventInStateInit() {
	return checkEnterExceptionResumeState();
}

bool ACleanMode::updateActionInStateInit() {
	if (action_i_ == ac_null)
		action_i_ = ac_open_gyro;
	else if (action_i_ == ac_open_gyro) {
		boost::dynamic_pointer_cast<StateInit>(state_init)->initOpenLidar();
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
	sp_state = state_clean.get();
	sp_state->init();
}

// ------------------State clean--------------------
bool ACleanMode::isSwitchByEventInStateClean() {
	return checkEnterGoHomePointState() || checkEnterExceptionResumeState();
}

void ACleanMode::switchInStateClean() {
	action_i_ = ac_null;
	sp_action_.reset();
	sp_state = nullptr;
}

// ------------------State go home point--------------------
bool ACleanMode::checkEnterGoHomePointState()
{
	if (ev.remote_home || ev.battery_home || s_wifi.receiveHome())
	{
		if (ev.remote_home)
			remote_go_home_point = true;
		if (s_wifi.receiveHome())
		{
			wifi_go_home_point = true;
			s_wifi.resetReceivedWorkMode();
		}
		if (ev.battery_home)
			go_home_for_low_battery_ = true;
		sp_action_.reset();
		sp_state = state_go_home_point.get();
		sp_state->init();
		speaker.play(VOICE_GO_HOME_MODE);
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
	old_dir_ = iterate_point_.dir;

	ROS_INFO("%s %d: curr(%d, %d), current home point(%d, %d).", __FUNCTION__, __LINE__,
			 getPosition().toCell().x, getPosition().toCell().y,
			 go_home_path_algorithm_->getCurrentHomePoint().toCell().x,
			 go_home_path_algorithm_->getCurrentHomePoint().toCell().y);
	if (ev.rcon_status)
	{
		// Directly switch to state go to charger.
		ROS_INFO("%s %d: Rcon T signal triggered and switch to state go to charger.", __FUNCTION__, __LINE__);
		should_go_to_charger_ = true;
		ev.rcon_status = 0;
		update_finish = false;
	}
	else if (go_home_path_algorithm_->reachTarget(should_go_to_charger_))
	{
		update_finish = false;
		home_points_ = go_home_path_algorithm_->getRestHomePoints();
	}
	else if (home_points_.empty() && getPosition().toCell() == start_point_.toCell())
	{
		ROS_INFO("Reach start point but angle not equal,start_point_(%d,%d,%f,%d)",start_point_.toCell().x, start_point_.toCell().y, radian_to_degree(start_point_.th), start_point_.dir);
//		beeper.beepForCommand(VALID);
		iterate_point_ = getPosition();
		iterate_point_.th = start_point_.th;
		plan_path_.clear();
		plan_path_.push_back(iterate_point_) ;
		plan_path_.push_back(start_point_) ;
		action_i_ = ac_linear;
		genNextAction();
		update_finish = true;
		home_points_ = go_home_path_algorithm_->getRestHomePoints();
	}
	else if (go_home_path_algorithm_->generatePath(clean_map_, getPosition(),old_dir_, plan_path_))
	{
		// New path to home cell is generated.
		iterate_point_ = plan_path_.front();
//		plan_path_.pop_front();
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
		sp_state = state_go_to_charger.get();
		sp_state->init();
		sp_action_.reset();
		if (isFirstTimeGoHomePoint())
		{
			if (!isRemoteGoHomePoint() && !isGoHomePointForLowBattery())
			{
				if (seen_charger_during_cleaning_)
					speaker.play(VOICE_CLEANING_FINISH_BACK_TO_CHARGER);
			}
			setFirstTimeGoHomePoint(false);
		}
	}
	else // path is empty.
	{
		ROS_INFO("%s %d, No more home point, finish cleaning.", __FUNCTION__, __LINE__);
		sp_state = nullptr;
	}
}

// ------------------State go to charger--------------------
bool ACleanMode::checkEnterGoToCharger()
{
	ev.rcon_status = c_rcon.getStatus() & (RconAll_Home_T);
	c_rcon.resetStatus();
	if (ev.rcon_status) {
		ev.rcon_status= false;
		ROS_WARN("%s,%d:find charge success,convert to go to charge state", __func__, __LINE__);
		sp_state = state_go_to_charger.get();
		sp_state->init();
		action_i_ = ac_go_to_charger;
		genNextAction();
		return true;
	}
	return false;
}

bool ACleanMode::isSwitchByEventInStateGoToCharger()
{
	return checkEnterExceptionResumeState();
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
		sp_state = state_go_home_point.get();
		sp_state->init();
	}
}

// ------------------State spot--------------------
bool ACleanMode::updateActionInStateSpot() {
	old_dir_ = iterate_point_.dir;
	if(!plan_path_.empty())
	{
		auto result = std::find_if(plan_path_.begin(), plan_path_.end(), [&](const Point_t& c_it){
			return c_it.toCell() == iterate_point_.toCell();
		});
		if(result != plan_path_.end())
			plan_path_.erase(plan_path_.begin(), result+1);
	}
	ROS_ERROR("old_dir_(%d)", old_dir_);
	auto cur_point = getPosition();
	ROS_INFO("\033[32m plan_path size(%d), front (%d,%d),cur point:(%d,%d)\033[0m",plan_path_.size(),
				plan_path_.front().toCell().x,plan_path_.front().toCell().y,cur_point.toCell().x,cur_point.toCell().y);
	sp_action_.reset();// to mark in destructor
	if (clean_path_algorithm_->generatePath(clean_map_, cur_point, old_dir_, plan_path_)) {
		iterate_point_ = plan_path_.front();
		ROS_ERROR("%s,%d,iterate_point_.dir(%d)", __FUNCTION__,__LINE__,iterate_point_.dir);
//		PP_INFO();
		clean_path_algorithm_->displayCellPath(pointsGenerateCells(plan_path_));
//		plan_path_.pop_front();
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
	return checkEnterExceptionResumeState();
}

// ------------------State exception resume--------------
bool ACleanMode::checkEnterExceptionResumeState()
{
	if (isExceptionTriggered()) {
		ROS_WARN("%s %d: Exception triggered!", __FUNCTION__, __LINE__);
		sp_action_.reset();
		sp_saved_states.push_back(sp_state);
		sp_state = state_exception_resume.get();
		sp_state->init();
		return true;
	}

	return false;
}

bool ACleanMode::isSwitchByEventInStateExceptionResume()
{
	return false;
}

bool ACleanMode::updateActionInStateExceptionResume()
{
	if (!ev.fatal_quit && isExceptionTriggered())
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
	if (!ev.fatal_quit)
	{
		ROS_INFO("%s %d: Resume to previous state", __FUNCTION__, __LINE__);
		action_i_ = ac_null;
		genNextAction();
		sp_state = sp_saved_states.back();
		sp_saved_states.pop_back();
		sp_state->init();
	}
	else
		sp_state = nullptr;
}

// ------------------State exploration--------------
bool ACleanMode::isSwitchByEventInStateExploration() {
	return checkEnterGoToCharger() || checkEnterExceptionResumeState();
}

bool ACleanMode::updateActionInStateExploration() {
	PP_INFO();
	old_dir_ = iterate_point_.dir;
	ROS_WARN("old_dir_(%d)", old_dir_);
	plan_path_.clear();
	sp_action_.reset();//to mark in constructor
	if (clean_path_algorithm_->generatePath(clean_map_, getPosition(), old_dir_, plan_path_)) {
		action_i_ = ac_linear;
		iterate_point_ = plan_path_.front();
		ROS_WARN("start_point_.dir(%d)", iterate_point_.dir);
//		plan_path_.pop_front();
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
	old_dir_ = iterate_point_.dir;
	Cells tmp_path =  clean_path_algorithm_->findShortestPath(clean_map_,getPosition().toCell(),Cell_t{0,0},old_dir_,false,false,Cell_t{0,0},Cell_t{0,0});
	if (tmp_path.empty()) {
		ROS_WARN("%s,%d: enter state trapped",__FUNCTION__,__LINE__);
		sp_saved_states.push_back(sp_state);
		is_trapped_ = true;
		sp_state = state_folllow_wall.get();
		is_isolate = true;
		is_closed = true;
		closed_count_ = 0;
		isolate_count_ = 0;
	}
	else{
		auto curr = getPosition();
		start_point_.th = curr.th;
		sp_state = state_go_home_point.get();
		if (go_home_path_algorithm_ == nullptr)
			go_home_path_algorithm_.reset(new GoHomePathAlgorithm(clean_map_, home_points_, start_point_));
	}
	action_i_ = ac_null;
	sp_state->init();
	genNextAction();
}

// ------------------State follow wall------------------

bool ACleanMode::isSwitchByEventInStateFollowWall()
{
	return checkEnterExceptionResumeState()||checkEnterGoHomePointState();
}

bool ACleanMode::updateActionInStateFollowWall()
{
	passed_path_.clear();
	auto ret = true;
	if(is_closed) {
		is_closed = false;
		if (is_isolate) {
			ROS_INFO_FL();
			ROS_ERROR("is_isolate");
			is_isolate = false;
			plan_path_.clear();
			auto angle = (isolate_count_ == 0) ? 0 : -900;
			auto point = getPosition().addRadian(angle);
			plan_path_.push_back(point);
			point = point.getRelative(8, 0);
			plan_path_.push_back(point);
			iterate_point_ = plan_path_.front();
			iterate_point_.dir = MAP_ANY;// note: fix bug follow isPassPosition
			clean_path_algorithm_->displayPointPath(plan_path_);
			action_i_ = ac_linear;
		}

		if (closed_count_ >= closed_count_limit_ || isolate_count_ >= isolate_count_limit_) {
			ROS_INFO("cc(%d),ccl(%d),ic(%d),icl(%d)",closed_count_, closed_count_limit_, isolate_count_, isolate_count_limit_);
			ROS_ERROR("p_mt->closed_count_ >= closed_count_limit_");
			trapped_closed_or_isolate = true;
			action_i_ = ac_null;
//			genNextAction();
			ROS_WARN("%s,%d:follow clean finish", __func__, __LINE__);
		}
	}
	else if(out_of_trapped) {
//		out_of_trapped = false;
		ROS_ERROR("out_of_trapped");
		action_i_ = ac_null;
//		genNextAction();
	}
	else if (robot_timer.trapTimeout(ESCAPE_TRAPPED_TIME)) {
			action_i_ = ac_null;
			trapped_time_out_ = true;
	}else{
		action_i_ = ac_follow_wall_left;//Set the left wall follow in the wall follow mode
//		action_i_ = ac_follow_wall_right;
		ROS_WARN("%s,%d: mt_follow_wall_left", __FUNCTION__, __LINE__);
	}

	if(action_i_ == ac_null)
		ret = false;

	genNextAction();
	return ret;
}

void ACleanMode::switchInStateFollowWall()
{
	PP_INFO();
	is_trapped_ = false;
	if(trapped_closed_or_isolate)
	{
		ROS_WARN("%s %d: closed_count_ >= closed_count_limit_!", __FUNCTION__, __LINE__);
		sp_state = nullptr;
	}
	else if (trapped_time_out_) {
		ROS_WARN("%s %d: Escape trapped timeout!(%d)", __FUNCTION__, __LINE__, ESCAPE_TRAPPED_TIME);
		sp_state = nullptr;
	}
	else/* if (escape_trapped_)*/ {//out_of_trapped = false
		ROS_WARN("%s %d:escape_trapped_ restore state from trapped !", __FUNCTION__, __LINE__);
//		sp_state = (sp_tmp_state == state_clean) ? state_clean : state_exploration;
		out_of_trapped = false;
		sp_state = sp_saved_states.back();
		sp_saved_states.pop_back();
		sp_state->init();
	}
}

// ------------------State resume low battery charge------------------
bool ACleanMode::isSwitchByEventInStateResumeLowBatteryCharge()
{
	return checkEnterExceptionResumeState();
}

void ACleanMode::setTempTarget(std::deque<Vector2<double>>& points, uint32_t  seq) {
	boost::mutex::scoped_lock lock(temp_target_mutex_);
	path_head_ = {};
	path_head_.tmp_plan_path_.clear();

//	ROS_ERROR("curr_point(%d,%d)",getPosition().x,getPosition().y);
	for (const auto &iter : points) {
		auto target = getPosition(ODOM_POSITION_ODOM_ANGLE).getRelative(static_cast<float>(iter.x), static_cast<float>(iter.y));
		path_head_.tmp_plan_path_.push_back(target);
//		printf("temp_target(%f, %f)\n",target.x,target.y);
	}
	path_head_.seq = seq;
}

PathHead ACleanMode::getTempTarget()
{
	boost::mutex::scoped_lock lock(temp_target_mutex_);

//	auto tmp = tmp_plan_path_;
//	tmp_plan_path_.clear();
//	return tmp;
	return path_head_;
}

bool ACleanMode::isIsolate() {
	BoundingBox2 bound{};
	GridMap fw_tmp_map;
	setFollowWall(fw_tmp_map, action_i_ == ac_follow_wall_left,passed_path_);

	fw_tmp_map.getMapRange(CLEAN_MAP, &bound.min.x, &bound.max.x, &bound.min.y, &bound.max.y);

	auto target = bound.max + Cell_t{1, 1};
	bound.SetMinimum(bound.min - Cell_t{8, 8});
	bound.SetMaximum(bound.max + Cell_t{8, 8});
	ROS_ERROR("ISOLATE MAP");
	fw_tmp_map.print(getPosition().toCell(), CLEAN_MAP,Cells{target});
	ROS_ERROR("ISOLATE MAP");
	ROS_ERROR("minx(%d),miny(%d),maxx(%d),maxy(%d)",bound.min.x, bound.min.y,bound.max.x, bound.max.y);

	auto path = clean_path_algorithm_->findShortestPath(fw_tmp_map, getPosition().toCell(), target, MAP_POS_X, true, true,
																											bound.min, bound.max);

	return !path.empty();
}

bool ACleanMode::generatePath(GridMap &map, const Point_t &curr, const int &last_dir, Points &targets)
{
	if (targets.empty()) {//fw ->linear
		auto curr = getPosition();

		ROS_WARN("%s,%d: empty! point(%d, %d, %d)", __FUNCTION__, __LINE__,targets.back().x, targets.back().y, targets.back().th);
	}
	else//linear->fw
	{
		targets.clear();
		targets.push_back(getPosition());
		ROS_WARN("%s,%d: not empty! point(%d, %d, %d)", __FUNCTION__, __LINE__,targets.back().x, targets.back().y, targets.back().th);
	}
	return true;
}

bool ACleanMode::isGyroDynamic() {
	return ros::Time::now().toSec() - time_gyro_dynamic_ > robot::instance()->getGyroDynamicInterval();
}

void ACleanMode::genNextAction() {
	if(action_i_ == ac_linear || action_i_ == ac_follow_wall_right ||action_i_ == ac_follow_wall_left) {
		switch (action_i_) {
			case ac_linear :
				sp_action_.reset(new MoveTypeLinear(plan_path_));
				break;
			case ac_follow_wall_left  :
			case ac_follow_wall_right :
				sp_action_.reset(new MoveTypeFollowWall(plan_path_,action_i_ == ac_follow_wall_left));
				break;
		}
	}
	else
		Mode::genNextAction();
}

void ACleanMode::wifiSetWaterTank()
{
	if (!water_tank.getStatus(WaterTank::operate_option::swing_motor))
		return;

	if ((isStateInit() && action_i_ > ac_open_gyro)
		|| isStateClean()
		|| isStateFollowWall())
	{
		auto user_set_swing_motor_mode = water_tank.getUserSetSwingMotorMode();
		if (water_tank.getCurrentSwingMotorMode() != user_set_swing_motor_mode)
			water_tank.setCurrentSwingMotorMode(user_set_swing_motor_mode);

		auto user_set_pump_mode = water_tank.getUserSetPumpMode();
		if (water_tank.getStatus(WaterTank::operate_option::pump) &&
			water_tank.getCurrentPumpMode() != user_set_pump_mode)
			water_tank.setCurrentPumpMode(user_set_pump_mode);
	}
}

void ACleanMode::setVacuum()
{
	if (water_tank.getStatus(WaterTank::operate_option::swing_motor))
		return;

	if ((isStateInit() && action_i_ > ac_open_gyro)
		|| isStateClean()
		|| isStateFollowWall())
	{
		auto user_set_max_mode = vacuum.isUserSetMaxMode();
		if (vacuum.isCurrentMaxMode() != user_set_max_mode)
		{
			vacuum.setSpeedByUserSetMode();
			speaker.play(vacuum.isCurrentMaxMode() ? VOICE_VACCUM_MAX : VOICE_VACUUM_NORMAL);
		}
	}
}
