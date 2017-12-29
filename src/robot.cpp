#include "pp.h"

#include <stdio.h>
#include <time.h>
#include <math.h>
#include <nav_msgs/OccupancyGrid.h>
#include <global.h>
#include <std_srvs/SetBool.h>
#include <pp.h>
#include <pp/SetLidar.h>
#include <odom.h>
#include "lidar.hpp"
#include "robot.hpp"

#include "std_srvs/Empty.h"

#define RAD2DEG(rad) ((rad)*57.29578)

bool	g_is_low_bat_pause=false;
bool g_is_manual_pause=false;
time_t	start_time;

int16_t slam_error_count;

// For obs dynamic adjustment
int OBS_adjust_count = 50;



//extern pp::x900sensor sensor;
robot::robot(std::string serial_port, int baudrate, std::string lidar_bumper_dev)/*:offset_angle_(0),saved_offset_angle_(0)*/
{
	// Subscribers.
	sensor_sub_ = robot_nh_.subscribe("/robot_sensor", 10, &robot::sensorCb, this);
	odom_sub_ = robot_nh_.subscribe("/odom", 1, &robot::robotOdomCb, this);
	map_sub_ = robot_nh_.subscribe("/map", 1, &robot::mapCb, this);
	scanLinear_sub_ = robot_nh_.subscribe("scanLinear", 1, &robot::scanLinearCb, this);
	scanOriginal_sub_ = robot_nh_.subscribe("scanOriginal", 1, &robot::scanOriginalCb, this);
	scanCompensate_sub_ = robot_nh_.subscribe("scanCompensate", 1, &robot::scanCompensateCb, this);
	lidarPoint_sub_ = robot_nh_.subscribe("lidarPoint", 1, &robot::lidarPointCb, this);
	/*map subscriber for exploration*/
	//map_metadata_sub = robot_nh_.subscribe("/map_metadata", 1, &robot::robot_map_metadata_cb, this);

	// Service clients.
	lidar_motor_cli_ = robot_nh_.serviceClient<pp::SetLidar>("lidar_motor_ctrl");
	end_slam_cli_ = robot_nh_.serviceClient<std_srvs::Empty>("End_Slam");
	start_slam_cli_ = robot_nh_.serviceClient<std_srvs::Empty>("Start_Slam");
	robot_tf_ = new tf::TransformListener(robot_nh_, ros::Duration(0.1), true);

	// Publishers.
	send_clean_marker_pub_ = robot_nh_.advertise<visualization_msgs::Marker>("clean_markers", 1);
	send_clean_map_marker_pub_ = robot_nh_.advertise<visualization_msgs::Marker>("clean_map_markers", 1);
	odom_pub_ = robot_nh_.advertise<nav_msgs::Odometry>("robot_odom", 1);
	scan_ctrl_pub_ = robot_nh_.advertise<pp::scan_ctrl>("scan_ctrl", 1);
	line_marker_pub_ = robot_nh_.advertise<visualization_msgs::Marker>("line_marker", 1);
	line_marker_pub2_ = robot_nh_.advertise<visualization_msgs::Marker>("line_marker2", 1);
	point_marker_pub_ = robot_nh_.advertise<visualization_msgs::Marker>("point_marker", 1);
	fit_line_marker_pub_ = robot_nh_.advertise<visualization_msgs::Marker>("fit_line_marker", 1);

	visualizeMarkerInit();
	is_sensor_ready_ = false;
	is_tf_ready_ = false;

	temp_spot_set_ = false;

	resetCorrection();

	start_time = time(NULL);

	// Initialize the low battery pause variable.
	// Initialize the key press count.
	// Initialize the manual pause variable.
	g_is_manual_pause = false;

	setBaselinkFrameType(ODOM_POSITION_ODOM_ANGLE);

	// Init for serial.
	if (!serial.init(serial_port.c_str(), baudrate))
	{
		ROS_ERROR("%s %d: Serial init failed!!", __FUNCTION__, __LINE__);
		return;
	}


#if VERIFY_CPU_ID
	if (verify_cpu_id() < 0) {
		verify_ok = false;
	}
#endif

#if VERIFY_KEY
	if (verify_ok == true && verify_key() == 0) {
		verify_ok = false;
	}
#endif

	// Init for lidar bumper.
	if (bumper.lidarBumperInit(lidar_bumper_dev.c_str()) == -1)
		ROS_ERROR(" lidar bumper open fail!");

	// Init for robotbase.
	robotbase_init();

	// Init for event manager.
	event_manager_init();
	auto event_manager_thread = new boost::thread(event_manager_thread_cb);
	event_manager_thread->detach();
	auto event_handler_thread = new boost::thread(event_handler_thread_cb);
	event_handler_thread->detach();

	// Init for core thread.
	auto core_thread = new boost::thread(core_thread_cb);
	core_thread->detach();
	ROS_INFO("%s %d: robot init done!", __FUNCTION__, __LINE__);
}

robot::~robot()
{
	bumper.lidarBumperDeinit();
	robotbase_deinit();
	delete robot_tf_;
}

robot *robot::instance()
{
	extern robot* robot_instance;
	return robot_instance;
}

void robot::sensorCb(const pp::x900sensor::ConstPtr &msg)
{
	is_sensor_ready_ = true;

	// Dynamic adjust obs
	obs.DynamicAdjust(OBS_adjust_count);

	// Check for whether robot should publish this frame of scan.
		scan_ctrl_.allow_publishing =
						!(fabs(wheel.getLeftWheelActualSpeed() - wheel.getRightWheelActualSpeed()) > 0.1
					|| (wheel.getLeftWheelActualSpeed() * wheel.getRightWheelActualSpeed() < 0)
					|| bumper.get_status()
					|| gyro.getTiltCheckingStatus()
					|| abs(wheel.getLeftSpeedAfterPid() - wheel.getRightSpeedAfterPid()) > 100
					|| wheel.getLeftSpeedAfterPid() * wheel.getRightSpeedAfterPid() < 0);

	scan_ctrl_pub_.publish(scan_ctrl_);
}

void robot::robotOdomCb(const nav_msgs::Odometry::ConstPtr &msg)
{
	tf::StampedTransform		transform;
	float tmp_x = 0, tmp_y = 0;
	double tmp_yaw = 0;

	float	odom_pose_x_;
	float	odom_pose_y_;
	double	odom_pose_yaw_;

	if (getBaselinkFrameType() == SLAM_POSITION_SLAM_ANGLE)
	{
		if(slam.isMapReady()/* && !ev.slam_error*/)
		{
			try {
				robot_tf_->lookupTransform("/map", "/base_link", ros::Time(0), transform);
				tmp_x = transform.getOrigin().x();
				tmp_y = transform.getOrigin().y();
				tmp_yaw = tf::getYaw(transform.getRotation());
				robot_tf_->lookupTransform("/odom", "/base_link", ros::Time(0), transform);
				odom_pose_x_ = transform.getOrigin().x();
				odom_pose_y_ = transform.getOrigin().y();
				odom_pose_yaw_ = tf::getYaw(transform.getRotation());
				slam_correction_x_ = tmp_x - odom_pose_x_;
				slam_correction_y_ = tmp_y - odom_pose_y_;
				slam_correction_yaw_ = tmp_yaw - odom_pose_yaw_;
			} catch(tf::TransformException e) {
				ROS_WARN("%s %d: Failed to compute map transform, skipping scan (%s)", __FUNCTION__, __LINE__, e.what());
				setTfReady(false);
				slam_error_count++;
				if (slam_error_count > 1)
				{
//					ev.slam_error = true;
					slam_error_count = 0;
				}
//				return;
			}

			if (!isTfReady())
			{
				ROS_INFO("%s %d: Set is_tf_ready_ to true.", __FUNCTION__, __LINE__);
				setTfReady(true);
				slam_error_count = 0;
			}
		}
//		if(! is_turn())
//			cm_update_map();
//		cm_update_position();
	}
	else if (getBaselinkFrameType() == ODOM_POSITION_ODOM_ANGLE)
	{
		//ROS_INFO("SLAM = 0");
		odom_pose_x_ = odom.getX();
		odom_pose_y_ = odom.getY();
		odom_pose_yaw_ = odom.getAngle() * M_PI / 180;
	}
	else if (getBaselinkFrameType() == SLAM_POSITION_ODOM_ANGLE)
	{//Wall_Follow_Mode
		//ROS_INFO("SLAM = 2");
		if(slam.isMapReady()/* && !ev.slam_error*/)
		{
			tf::Stamped<tf::Pose> ident;
			ident.setIdentity();
			ident.frame_id_ = "base_link";
			ident.stamp_ = msg->header.stamp;
			try {
				robot_tf_->lookupTransform("/map", "/base_link", ros::Time(0), transform);
				//robot_tf_->waitForTransform("/map", ros::Time::now(), ident.frame_id_, msg->header.stamp, ident.frame_id_, ros::Duration(0.5));
				//robot_tf_->lookupTransform("/map", "/base_link", ros::Time(0), transform);
				tmp_x = transform.getOrigin().x();
				tmp_y = transform.getOrigin().y();
				//robot_tf_->waitForTransform("/odom", ros::Time::now(), ident.frame_id_, msg->header.stamp, ident.frame_id_, ros::Duration(0.5));
				robot_tf_->lookupTransform("/odom", "/base_link", ros::Time(0), transform);
				tmp_yaw = tf::getYaw(transform.getRotation());
				odom_pose_x_ = transform.getOrigin().x();
				odom_pose_y_ = transform.getOrigin().y();
				odom_pose_yaw_ = tf::getYaw(transform.getRotation());
				slam_correction_x_ = tmp_x - odom_pose_x_;
				slam_correction_y_ = tmp_y - odom_pose_y_;
				slam_correction_yaw_ = tmp_yaw - odom_pose_yaw_;
			} catch(tf::TransformException e) {
				ROS_WARN("%s %d: Failed to compute map transform, skipping scan (%s)", __FUNCTION__, __LINE__, e.what());
				setTfReady(false);
				slam_error_count++;
				if (slam_error_count > 1)
				{
//					ev.slam_error = true;
					slam_error_count = 0;
				}
//				return;
			}

			if (!isTfReady())
			{
				ROS_INFO("%s %d: Set is_tf_ready_ to true.", __FUNCTION__, __LINE__);
				setTfReady(true);
				slam_error_count = 0;
			}
		}

//		cm_update_map();
	}

#if USE_ROBOT_TF
	updateRobotPose(odom_pose_x_, odom_pose_y_, odom_pose_yaw_,
					slam_correction_x_, slam_correction_y_, slam_correction_yaw_,
					robot_correction_x_, robot_correction_y_, robot_correction_yaw_,
					robot_x_, robot_y_, robot_yaw_);
	//robot_x = (tmp_x + robot_x) / 2;
	//robot_y = (tmp_y + robot_y) / 2;
	//robot_yaw = tmp_yaw;

	// Publish the robot tf.
	ros::Time cur_time;
	//robot_trans.header.stamp = cur_time;
	//robot_trans.header.frame_id = "map";
	//robot_trans.child_frame_id = "robot";
	//geometry_msgs::Quaternion	robot_quat;
	//robot_trans.transform.translation.x = robot_x_;
	//robot_trans.transform.translation.y = robot_y_;
	//robot_trans.transform.translation.z = 0.0;
	//robot_quat = tf::createQuaternionMsgFromYaw(robot_yaw_);
	//robot_trans.transform.rotation = robot_quat;
	//robot_broad.sendTransform(robot_trans);
	//ROS_WARN("%s %d: World position (%f, %f), yaw: %f.", __FUNCTION__, __LINE__, tmp_x, tmp_y, tmp_yaw);
	nav_msgs::Odometry robot_pose;
	robot_pose.header.stamp = cur_time;
	robot_pose.header.frame_id = "map";
	robot_pose.child_frame_id = "robot";
	robot_pose.pose.pose.position.x = robot_x_;
	robot_pose.pose.pose.position.y = robot_y_;
	robot_pose.pose.pose.position.z = 0.0;
	robot_pose.pose.pose.orientation = tf::createQuaternionMsgFromYaw(robot_yaw_);
	robot_pose.twist.twist.linear.x = 0.0;
	robot_pose.twist.twist.linear.y = 0.0;
	robot_pose.twist.twist.angular.z = 0.0;
	odom_pub_.publish(robot_pose);
	//printf("Map->base(%f, %f, %f). Map->robot (%f, %f, %f)\n", tmp_x, tmp_y, RAD2DEG(tmp_yaw), robot_x_, robot_y_, RAD2DEG(robot_yaw_));
	world_pose_.setX(robot_x_);
	world_pose_.setY(robot_y_);
	world_pose_.setAngle(ranged_angle(robot_yaw_ * 1800 / M_PI));
#else
	pose.setX(tmp_x_);
	pose.setY(tmp_y_);
	pose.setAngle(tmp_yaw_);
#endif
	//ROS_WARN("%s %d: Position (%f, %f), yaw: %f. Odom position(%f, %f), yaw: %f.", __FUNCTION__, __LINE__, tmp_x, tmp_y, tmp_yaw, odom_pose_x_, odom_pose_y_, odom_pose_yaw_);
	//ROS_WARN("%s %d: Position diff(%f, %f), yaw diff: %f.", __FUNCTION__, __LINE__, tmp_x - odom_pose_x_, tmp_y - odom_pose_y_, tmp_yaw - odom_pose_yaw_);
	//ROS_WARN("%s %d: Odom diff(%f, %f).", __FUNCTION__, __LINE__, odom_pose_x_ - msg->pose.pose.position.x, odom_pose_y_ - msg->pose.pose.position.y);
	//ROS_WARN("%s %d: Correct  diff(%f, %f), yaw diff: %f.", __FUNCTION__, __LINE__, correct_x, correct_y, correct_yaw);
//	ROS_WARN("Position (%f, %f), angle: %d.", odom_pose_x_, odom_pose_y_, gyro.get_angle());
}

void robot::mapCb(const nav_msgs::OccupancyGrid::ConstPtr &map)
{
	slam.mapCb(map);
}

void robot::scanLinearCb(const sensor_msgs::LaserScan::ConstPtr &msg)
{
	lidar.scanLinearCb(msg);
}

void robot::scanOriginalCb(const sensor_msgs::LaserScan::ConstPtr &scan)
{
	lidar.scanOriginalCb(scan);
}

void robot::scanCompensateCb(const sensor_msgs::LaserScan::ConstPtr &msg)
{
	lidar.scanCompensateCb(msg);
}

void robot::lidarPointCb(const visualization_msgs::Marker &point_marker)
{
	if (lidar.isScanOriginalReady())
		lidar.lidarPointCb(point_marker);
}

void robot::visualizeMarkerInit()
{
	clean_markers_.ns = "waypoints";
	clean_markers_.id = 0;
	clean_markers_.type = visualization_msgs::Marker::LINE_STRIP;
	clean_markers_.action= 0;//add
	clean_markers_.lifetime=ros::Duration(0);

	clean_markers_.scale.x = 0.33;
//	clean_markers_.scale.y = 0.31;
	clean_markers_.color.r = 0.0;
	clean_markers_.color.g = 1.0;
	clean_markers_.color.b = 0.0;
	clean_markers_.color.a = 0.4;
	clean_markers_.header.frame_id = "/map";
	clean_markers_.header.stamp = ros::Time::now();
	m_points_.x = 0.0;
	m_points_.y = 0.0;
	m_points_.z = 0.0;
	clean_markers_.points.clear();
	clean_markers_.points.push_back(m_points_);

	clean_map_markers_.ns = "cleaning_grid_map";
	clean_map_markers_.id = 1;
	clean_map_markers_.type = visualization_msgs::Marker::POINTS;
	clean_map_markers_.action= visualization_msgs::Marker::ADD;
	clean_map_markers_.lifetime=ros::Duration(0);
	clean_map_markers_.scale.x = 0.1;
	clean_map_markers_.scale.y = 0.1;
	color_.a = 0.7;
	clean_map_markers_.header.frame_id = "/map";
	clean_map_markers_.points.clear();
	clean_map_markers_.colors.clear();
}

void robot::setCleanMapMarkers(int8_t x, int8_t y, CellState type)
{
	m_points_.x = x * (float)CELL_SIZE / 1000;
	m_points_.y = y * (float)CELL_SIZE / 1000;
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
	else if (type == BLOCKED_OBS)
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

void robot::pubCleanMapMarkers(GridMap& map, const std::deque<Cell_t>& path, Cell_t* cell_p)
{
	// temp_target is valid if only path is not empty.
	if (path.empty())
		return;

	int16_t x, y, x_min, x_max, y_min, y_max;
	CellState cell_state;
	Cell_t next = path.front();
	Cell_t target = path.back();
	map.getMapRange(CLEAN_MAP, &x_min, &x_max, &y_min, &y_max);

	if (next.X == SHRT_MIN )
		next.X = x_min;
	else if (next.X == SHRT_MAX)
		next.X = x_max;

	for (x = x_min; x <= x_max; x++)
	{
		for (y = y_min; y <= y_max; y++)
		{
			if (x == target.X && y == target.Y)
				robot::instance()->setCleanMapMarkers(x, y, TARGET_CLEAN);
			else if (x == next.X && y == next.Y)
				robot::instance()->setCleanMapMarkers(x, y, TARGET);
			else if (cell_p != nullptr && x == (*cell_p).X && y == (*cell_p).Y)
				robot::instance()->setCleanMapMarkers(x, y, TARGET_CLEAN);
			else
			{
				cell_state = map.getCell(CLEAN_MAP, x, y);
				if (cell_state > UNCLEAN && cell_state < BLOCKED_BOUNDARY )
					robot::instance()->setCleanMapMarkers(x, y, cell_state);
			}
		}
	}
	if (!path.empty())
	{
		for (auto it = path.begin(); it->X != path.back().X || it->Y != path.back().Y; it++)
			robot::instance()->setCleanMapMarkers(it->X, it->Y, TARGET);

		robot::instance()->setCleanMapMarkers(path.back().X, path.back().Y, TARGET_CLEAN);
	}

	clean_map_markers_.header.stamp = ros::Time::now();
	send_clean_map_marker_pub_.publish(clean_map_markers_);
	clean_map_markers_.points.clear();
	clean_map_markers_.colors.clear();
}

void robot::pubLineMarker(const std::vector<LineABC> *lines)
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

void robot::pubLineMarker(std::vector<std::vector<Vector2<double>> > *groups)
{
	int points_size;
	visualization_msgs::Marker line_marker;
	line_marker.ns = "line_marker";
	line_marker.id = 0;
	line_marker.type = visualization_msgs::Marker::SPHERE_LIST;
	line_marker.action= 0;//add
	line_marker.lifetime=ros::Duration(0);
	line_marker.scale.x = 0.03;
	line_marker.scale.y = 0.03;
	line_marker.scale.z = 0.03;
	line_marker.color.r = 0.0;
	line_marker.color.g = 1.0;
	line_marker.color.b = 0.0;
	line_marker.color.a = 1.0;
	line_marker.header.frame_id = "/base_link";
	line_marker.header.stamp = ros::Time::now();
	geometry_msgs::Point lidar_points_;
	lidar_points_.x = 0.0;
	lidar_points_.y = 0.0;
	lidar_points_.z = 0.0;

	/*line_marker.pose.position.x = 0.0;
	line_marker.pose.position.y = 0.0;
	line_marker.pose.position.z = 0.0;
	line_marker.pose.orientation.x = 0.0;
	line_marker.pose.orientation.y = 0.0;
	line_marker.pose.orientation.z = 0.0;
	line_marker.pose.orientation.w = 1.0;*/
	if (!(*groups).empty()) {
		for (auto iter = (*groups).begin(); iter != (*groups).end(); ++iter) {
			/*x1 = iter->begin()->x;
			y1 = iter->begin()->y;
			x2 = (iter->end() - 1)->x;
			y2 = (iter->end() - 1)->y;*/
			points_size = iter->size();
			for (int j = 0; j < points_size; j++) {
				//line_marker.pose.position.x = (iter->begin() + j)->x;
				//line_marker.pose.position.y = (iter->begin() + j)->y;
				lidar_points_.x = (iter->begin() + j)->X;
				lidar_points_.y = (iter->begin() + j)->Y;
				//ROS_INFO("lidar_points_.x = %lf lidar_points_.y = %lf",lidar_points_.x, lidar_points_.y);
				line_marker.points.push_back(lidar_points_);
			}
		}
		line_marker_pub_.publish(line_marker);
		line_marker.points.clear();
	} else {
		line_marker.points.clear();
		line_marker_pub_.publish(line_marker);
	}
}

void robot::pubFitLineMarker(visualization_msgs::Marker fit_line_marker)
{
	fit_line_marker_pub_.publish(fit_line_marker);
}

void robot::pubPointMarkers(const std::vector<Vector2<double>> *points, std::string frame_id)
{
	visualization_msgs::Marker point_marker;
	point_marker.ns = "point_marker";
	point_marker.id = 0;
	point_marker.type = visualization_msgs::Marker::POINTS;
	point_marker.action= 0;//add
	point_marker.lifetime=ros::Duration(0),"base_link";
	point_marker.scale.x = 0.02;
	point_marker.scale.y = 0.02;
	point_marker.scale.z = 0.0;
	point_marker.color.r = 1.0;
	point_marker.color.g = 1.0;
	point_marker.color.b = 1.0;
	point_marker.color.a = 1.0;
	point_marker.header.frame_id = frame_id;
	point_marker.header.stamp = ros::Time::now();

	geometry_msgs::Point lidar_points;
	lidar_points.z = 0;
	if (!points->empty()) {
		std::string msg("");
		for (auto iter = points->cbegin(); iter != points->cend(); ++iter) {
			lidar_points.x = iter->X;
			lidar_points.y = iter->Y;
			point_marker.points.push_back(lidar_points);
			msg+="("+std::to_string(iter->X)+","+std::to_string(iter->Y)+"),";
		}
		point_marker_pub_.publish(point_marker);
		//ROS_INFO("%s,%d,points size:%u,points %s",__FUNCTION__,__LINE__,points->size(),msg.c_str());
		point_marker.points.clear();
	}
	/*
	else {
		point_marker.points.clear();
		point_marker_pub.publish(point_marker);
	}
	*/
}

bool robot::lidarMotorCtrl(bool switch_)
{
	pp::SetLidar ctrl_message;
	if(switch_){
/*		ctrl_message.request.x_acc_init= gyro.getInitXAcc();
		ctrl_message.request.y_acc_init= gyro.getInitYAcc();
		ctrl_message.request.z_acc_init= gyro.getInitZAcc();*/
		ctrl_message.request.x_acc_init= 0;
		ctrl_message.request.y_acc_init= 0;
		ctrl_message.request.z_acc_init= 0;
	}
	ctrl_message.request.data = switch_;

	if (lidar_motor_cli_.call(ctrl_message))
	{
		ROS_INFO("\033[35m" "%s %d: Service response: %s" "\033[0m", __FUNCTION__, __LINE__, ctrl_message.response.message.c_str());
		return true;
	}
	return false;
}

bool robot::slamStart(void)
{
	std_srvs::Empty empty;
	ROS_INFO("%s %d: Call start slam service.", __FUNCTION__, __LINE__);
	return start_slam_cli_.call(empty);
}

bool robot::slamStop(void)
{
	std_srvs::Empty empty;
	return end_slam_cli_.call(empty);
}

void robot::initOdomPosition()
{
	//is_tf_ready_ = false;
//	position_x_=0;
//	position_y_=0;
//	position_z_=0;
	robotbase_reset_odom_pose();
	visualizeMarkerInit();
}

void robot::updateRobotPose(const float& odom_x, const float& odom_y, const double& odom_yaw,
					const float& slam_correction_x, const float& slam_correction_y, const double& slam_correction_yaw,
					float& robot_correction_x, float& robot_correction_y, double& robot_correction_yaw,
					float& robot_x, float& robot_y, double& robot_yaw)
{
	if (wheel.getLeftSpeedAfterPid() * wheel.getRightSpeedAfterPid() > 0)
	{
		float scale;
		scale = fabs(slam_correction_x - robot_correction_x) > 0.05 ? 0.1 * fabs(slam_correction_x - robot_correction_x) / 0.05 : 0.03;
		scale = scale > 1.0 ? 1.0 : scale;
		robot_correction_x += (slam_correction_x - robot_correction_x) * scale;
		scale = fabs(slam_correction_y - robot_correction_y) > 0.05 ? 0.1 * fabs(slam_correction_y - robot_correction_y) / 0.05 : 0.03;
		scale = scale > 1.0 ? 1.0 : scale;
		robot_correction_y += (slam_correction_y - robot_correction_y) * scale;
		double yaw = slam_correction_yaw - robot_correction_yaw;
		while (yaw < -3.141592)
			yaw += 6.283184;
		while (yaw > 3.141592)
			yaw -= 6.283184;
		robot_correction_yaw += (yaw) * 0.8;
//		printf("Slam (%f, %f, %f). Adjust (%f, %f, %f)\n", slam_correction_x, slam_correction_y, RAD2DEG(slam_correction_yaw), robot_correction_x, robot_correction_y, RAD2DEG(robot_correction_yaw));
	}

	robot_x = odom_x + robot_correction_x;
	robot_y = odom_y + robot_correction_y;
	robot_yaw = odom_yaw + robot_correction_yaw;
}

void robot::resetCorrection()
{
	slam_correction_x_ = 0;
	slam_correction_y_ = 0;
	slam_correction_yaw_ = 0;
	robot_correction_x_ = 0;
	robot_correction_y_ = 0;
	robot_correction_yaw_ = 0;
	robot_x_ = 0;
	robot_y_ = 0;
	robot_yaw_ = 0;
}

void robot::obsAdjustCount(int count)
{
#ifdef OBS_DYNAMIC
	OBS_adjust_count = count;
#endif
}

//--------------------
static int32_t xCount{}, yCount{};

Point32_t getPosition()
{
	return {(int32_t)round(xCount), (int32_t)round(yCount), robot::instance()->getWorldPoseAngle()};
}

int32_t cellToCount(int16_t i) {
	return i * CELL_COUNT_MUL;
}

void setPosition(int32_t x, int32_t y) {
	xCount = x;
	yCount = y;
}

bool isPos(MapDirection dir)
{
	return (dir == MAP_POS_X || dir == MAP_POS_Y || dir == MAP_NONE);
}

bool isXAxis(MapDirection dir)
{
	return dir == MAP_POS_X || dir == MAP_NEG_X || dir == MAP_NONE;
}

Point32_t updatePosition()
{
	auto pos_x = robot::instance()->getWorldPoseX() * 1000 * CELL_COUNT_MUL / CELL_SIZE;
	auto pos_y = robot::instance()->getWorldPoseY() * 1000 * CELL_COUNT_MUL / CELL_SIZE;
	setPosition(pos_x, pos_y);
//	ROS_INFO("%s %d:", __FUNCTION__, __LINE__);
	return getPosition();
}

uint16_t relative_theta = 3600;
Point32_t getRelative(Point32_t point, int16_t dy, int16_t dx, bool using_point_pos) {
	double relative_sin, relative_cos;
	if(point.TH != relative_theta) {
		if(point.TH == 0) {
			relative_sin = 0;
			relative_cos = 1;
		} else if(point.TH == 900) {
			relative_sin = 1;
			relative_cos = 0;
		} else if(point.TH == 1800) {
			relative_sin = 0;
			relative_cos = -1;
		} else if(point.TH == -900) {
			relative_sin = -1;
			relative_cos = 0;
		} else {
			relative_sin = sin(deg_to_rad(point.TH, 10));
			relative_cos = cos(deg_to_rad(point.TH, 10));
		}
	}

	if (using_point_pos)
	{
		point.X += (int32_t)( ( ((double)dx * relative_cos * CELL_COUNT_MUL) - ((double)dy	* relative_sin * CELL_COUNT_MUL) ) / CELL_SIZE );
		point.Y += (int32_t)( ( ((double)dx * relative_sin * CELL_COUNT_MUL) + ((double)dy	* relative_cos * CELL_COUNT_MUL) ) / CELL_SIZE );
	}
	else
	{
		point.X = cellToCount(point.toCell().X) + (int32_t)( ( ((double)dx * relative_cos * CELL_COUNT_MUL) - ((double)dy	* relative_sin * CELL_COUNT_MUL) ) / CELL_SIZE );
		point.Y = cellToCount(point.toCell().Y) + (int32_t)( ( ((double)dx * relative_sin * CELL_COUNT_MUL) + ((double)dy	* relative_cos * CELL_COUNT_MUL) ) / CELL_SIZE );
//		ROS_ERROR("piont.x:%d  point:y:%d",point.X,point.Y,point.TH);
	}
	return point;
}

